# agri_navigation

`agri_navigation` owns the robot-side mapping, localization, and navigation
configuration. The stage 3 entry point provides standalone synchronous SLAM
Toolbox mapping. It consumes the filtered MID-360 `sensor_msgs/LaserScan` on
`/scan`, the wheel odometry TF (`odom -> base_footprint`), and publishes the
mapping TF (`map -> odom`) plus `/map`.

## Mapping baseline

Start the simulation and scan pipeline first, then in another terminal run:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=93
export ROS_LOG_DIR=/tmp/agri_ros_logs

ros2 launch agri_navigation mapping.launch.py
```

The launch file does not start Gazebo, AMCL, `map_server`, or Nav2.  This
keeps mapping and saved-map navigation mutually exclusive and makes the same
entry point usable with hardware.  The default map resolution is 0.05 m and
the expected frame chain is:

```text
map --(SLAM Toolbox)--> odom --(wheel odometry)--> base_footprint
```

Drive slowly through one row and close the loop before saving.  Save an
occupancy map after checking it in RViz:

```bash
MAP_DIR="artifacts/maps/field_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/tomato_field" \
  --ros-args -p use_sim_time:=true \
    -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
```

The command writes `tomato_field.yaml` and `tomato_field.pgm`.  The transient
local option is important because `/map` is published with a latched QoS; the
timeout avoids an indefinite wait when the mapping node is not active.  Preserve
the SLAM Toolbox pose graph alongside them when a resumable map is required:

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_DIR/tomato_field'}"
```

The map, pose graph, generated field SDF, fruit seed, scan configuration, and a
short validation report form one map version; do not reuse a map after changing
row spacing or plant dimensions.

## Saved-map navigation baseline

Stop `mapping.launch.py`, then load the saved map with AMCL and the Nav2 stage 4
baseline:

```bash
ros2 launch agri_navigation navigation.launch.py \
  map:="/absolute/path/to/tomato_field.yaml" \
  use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

Omit `map:=...` to load the packaged `tomato_field_default.yaml`, which is the
validated 206 x 76 central-corridor map for the default tomato field. Pass an
absolute map path only when using another field or map version.

One lifecycle manager activates nine managed nodes: map server, AMCL, both
keepout publishers, controller, planner, behaviors, BT navigator, and collision
monitor. The default planner is NavFn in Dijkstra mode. `FollowPath` is a
Rotation Shim controller wrapping Regulated Pure Pursuit (RPP): RPP tracks the
path in forward or reverse, while the shim closes the small remaining goal-yaw
error after RPP reaches the station position. Commands flow through
`/cmd_vel_nav_raw`, the collision monitor, `/cmd_vel_nav`, and the base velocity
gate before reaching `/cmd_vel_safe`.

The default launch also activates `filter_mask_server` and
`costmap_filter_info_server`. Both global and local costmaps consume the
transient-local `/keepout_filter_mask` through `KeepoutFilter`; the mask is
version-bound to the packaged map and is generated from the plant collision
bands. The default profile uses a 0.40 m inflation radius and 0.035 m costmap
footprint padding, while the hard collision-monitor polygon retains its own
0.04 m safety padding. Verify the filter publishers before starting a route:

```bash
ros2 lifecycle get /filter_mask_server
ros2 lifecycle get /costmap_filter_info_server
ros2 topic echo /costmap_filter_info --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /keepout_filter_mask --once \
  --qos-reliability reliable --qos-durability transient_local
```

The filter-info message should report `type: 0`, `base: 0.0`, and
`multiplier: 1.0`; the mask should be in `map` at 0.05 m resolution and match
the map dimensions. A route executor must reject a different map or mask.

### Separate localization and collision inputs

The default continues to use `/map` and `/scan` for both localization and
navigation. For a simulation whose rendered leaves have no collision geometry,
a separate collision map and obstacle scan can be selected explicitly:

```bash
ros2 launch agri_navigation navigation.launch.py \
  map:="/absolute/path/to/localization.yaml" \
  navigation_map:="/absolute/path/to/navigation.yaml" \
  keepout_mask:="/absolute/path/to/keepout.yaml" \
  obstacle_scan:=/scan_obstacles \
  use_sim_time:=true
```

`map_server` and AMCL retain the localization map on `/map`; AMCL continues to
read `/scan`. The additional `navigation_map_server` publishes a transient-local
`/navigation_map` in the same `map` frame and has its own lifecycle manager.
Both costmaps read it through a static layer, which is added to the local
costmap only in this mode. Their obstacle layers and the collision monitor read
`obstacle_scan`. The footprint, inflation parameters, stop polygon and keepout
filters retain their configured values.

The two map YAMLs must share the world-to-map alignment, and the navigation
map, obstacle scan and keepout must describe the same collision geometry. This
launch does not create `/scan_obstacles` or a collision map; start the matching
obstacle source first. If it is missing, the collision monitor retains its
normal source timeout and stops motion. A route using these inputs also needs
the corresponding navigation-map and scene binding checks. An empty
`navigation_map` omits the extra map server and keeps the previous costmap plugin
layout; `obstacle_scan` can also be selected independently.

The default controller profile is specialized for the validated central
corridor. NavFn keeps the requested goal yaw with
`use_final_approach_orientation: false`; the shim consumes path orientations
with `use_path_orientations: true`; and RPP uses `allow_reversing: true` with
`use_rotate_to_heading: false`. Use yaw 0 for outbound, harvest, and return
goals. This coupling lets RPP reverse along the return path instead of turning
the robot around beside the plants, while the shim may make a small terminal
yaw correction. Do not change one of these parameters in isolation. After a
planner or controller change, repeat the outbound/return and full-route tests.
The accepted final absolute yaw error is at most 0.0873 rad.

The shim uses `closed_loop: false` because the four-wheel-steering joints need
time to settle. With closed-loop shim acceleration, the initially near-zero
measured yaw rate repeatedly held the command at 0.025 rad/s, below the useful
steering range. Open-loop acceleration uses the previous command to ramp toward
0.20 rad/s; the collision monitor still checks the live footprint and stops
unsafe motion. This setting changes only the shim's angular acceleration
feedback and does not disable odometry, TF, AMCL, or collision feedback.

`rotate_to_heading_once: true` keeps repeated plans for the same goal from
interrupting RPP's curved reverse approach with another heading alignment.
Terminal heading correction remains enabled through `rotate_to_goal_heading`.

`localization.launch.py` starts map server and AMCL without the navigation
servers. The packaged `stage3_baseline.yaml` remains an explicit legacy test
fixture; the launch defaults use `tomato_field_default.yaml`. Layout changes
require regenerating the field, re-running SLAM, saving a new map version, and
restarting navigation with that YAML. See `docs/navigation_stage4.md` at the
repository root for commands.

## Important boundaries

Only one node may publish each TF edge.  In mapping mode SLAM Toolbox owns
`map -> odom`; AMCL must be stopped.  The wheel-odometry adapter currently owns
`odom -> base_footprint`; do not start a second EKF publisher until that
ownership is explicitly switched.  `/scan` must use `mid360_scan_frame` and
the scan timestamps must be in the same clock domain as the odometry.
