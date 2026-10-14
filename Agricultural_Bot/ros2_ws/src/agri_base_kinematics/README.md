# agri_base_kinematics

Python ROS 2 node for four-wheel-steering inverse kinematics and wheel odometry.
In the simulation launch it consumes the guarded `/cmd_vel_safe` stream from
`agri_base_adapter`; when launched alone it retains `/cmd_vel` as its input.

## Interfaces

- Input: `/cmd_vel_safe` (`geometry_msgs/msg/TwistStamped`) from the simulation
  launch; standalone default is `/cmd_vel`
- Input: `/joint_states` (`sensor_msgs/msg/JointState`)
- Output: `/steering_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- Output: `/wheel_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- Output: `/wheel/odom` (`nav_msgs/msg/Odometry`)
- Optional TF: `odom -> base_footprint`

The node requires a current `TwistStamped` in `base_footprint` when using the
stamped input. It rejects stale, future or non-finite commands, stops after a
0.5 s command timeout, and holds wheel speed at zero until the steering error
is within the configured alignment thresholds. Four steering and four wheel
feedback values must be present and finite in one `JointState` message before
odometry is updated. The configured steering offsets account for the CAD
module zero headings; change them only after a new calibration.

`max_wheel_acceleration` limits normal changes of the final wheel commands after
the steering alignment scale. Its default is 0.8 rad/s², corresponding to about
0.10 m/s² when driving straight with the 0.127 m wheel radius. This also ramps
drive when steering alignment recovers from a zero-speed hold. The limiter uses
ROS time (`/clock` in simulation); a slower real-time factor does not increase
the permitted acceleration. It does not extend the command watchdog.
The current permitted speed of each wheel is a hard upper bound: decreasing
the alignment scale or reducing a commanded speed can immediately reduce the
output. The ramp never postpones such a reduction for smoother braking.

A full zero command, including the velocity gate's locked output, bypasses the
ramp and stops immediately. Expired commands, stale joint feedback, backwards
time, zero elapsed time, or a timer gap exceeding the shorter of the command
and joint feedback timeouts also stop and clear the ramp history. The next
valid command therefore starts from zero. Odometry still uses measured wheel
states. These changes address drive-command steps; the arm stow check remains
unchanged and must be verified in Gazebo after rebuilding.

Normal steering position references are also smoothed using ROS/SIM time.
`max_steering_velocity` defaults to 0.5 rad/s and
`max_steering_acceleration` to 1.0 rad/s². Initialization uses the first fresh
measured angles. Velocity changes are limited before the bounded position
reference is published; a changed target decelerates before reversing. Wheel
alignment continues to compare the raw inverse-kinematics target with actual
measured steering, so a slowly moving reference cannot falsely release drive.
Zero or expired commands hold fresh measured steering and clear planned
motion. Missing feedback or discontinuous time clears reference history and
stops drive; steering publishing resumes from fresh measured angles. These
safety transitions deliberately bypass normal steering reference continuity.
Measured joint states and wheel odometry are never replaced with references.
This reduces steering target steps; its effect on arm hold during turning
still requires an actual Gazebo test.

Steering uses bounded revolute joint coordinates. Equivalent wheel headings
are selected by their direct travel within the limits; a heading shifted by
pi also reverses that wheel's speed. Calibration and alignment checks preserve
these coordinates across the +/-pi boundary. `steering_joint_limits` must match
the URDF hard limits (currently `[-3.1416, 3.1416]`); the node transforms them
through each module's sign and offset and intersects them with
`steering_limits`. If any wheel cannot realize its requested rolling direction,
all wheel drive stops instead of moving with a clipped steering angle.

`/wheel/odom` is the source-specific odometry stream. In the integrated
simulation `agri_base_adapter/odom_adapter` validates it and publishes the
navigation-facing `/odom`; the four-wheel node remains the sole baseline TF
publisher. When an EKF is enabled, set `publish_tf:=false` here and remove the
adapter publisher so the EKF owns the transform.

The four entries in each command array use this fixed order:
`f1, f2, r1, r2`.

## Run

```bash
ros2 launch agri_base_kinematics four_wheel_steering.launch.py
```

The wheel positions in `config/four_wheel_steering.yaml` are converted from the
current URDF into the ROS base plane. Joint signs must be confirmed in Gazebo at
low speed after ros2_control is connected.
