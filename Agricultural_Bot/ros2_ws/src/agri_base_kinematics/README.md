# agri_base_kinematics

Python ROS 2 node for four-wheel-steering inverse kinematics and wheel odometry.

## Interfaces

- Input: `/cmd_vel` (`geometry_msgs/msg/TwistStamped` by default)
- Input: `/joint_states` (`sensor_msgs/msg/JointState`)
- Output: `/steering_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- Output: `/wheel_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- Output: `/wheel/odom` (`nav_msgs/msg/Odometry`)
- Optional TF: `odom -> base_footprint`

The four entries in each command array use this fixed order:
`f1, f2, r1, r2`.

## Run

```bash
ros2 launch agri_base_kinematics four_wheel_steering.launch.py
```

The wheel positions in `config/four_wheel_steering.yaml` are converted from the
current URDF into the ROS base plane. Joint signs must be confirmed in Gazebo at
low speed after ros2_control is connected.
