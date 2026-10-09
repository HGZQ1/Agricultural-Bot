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

One lifecycle manager activates map server and AMCL before the controller,
planner, behaviors, BT navigator, and collision monitor. The default planner is
NavFn in Dijkstra mode and the controller is Regulated Pure Pursuit. Commands
flow through `/cmd_vel_nav_raw`, the collision monitor, `/cmd_vel_nav`, and the
base velocity gate before reaching `/cmd_vel_safe`.

`localization.launch.py` starts map server and AMCL without the navigation
servers. The packaged `stage3_baseline.yaml` is a startup fixture; use a complete
saved field map for driving. Layout changes require regenerating the field,
re-running SLAM, saving a new map version, and restarting navigation with that
YAML. See `docs/navigation_stage4.md` at the repository root for commands.

## Important boundaries

Only one node may publish each TF edge.  In mapping mode SLAM Toolbox owns
`map -> odom`; AMCL must be stopped.  The wheel-odometry adapter currently owns
`odom -> base_footprint`; do not start a second EKF publisher until that
ownership is explicitly switched.  `/scan` must use `mid360_scan_frame` and
the scan timestamps must be in the same clock domain as the odometry.
