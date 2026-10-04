# agri_sim_description

Initial Gazebo Harmonic overlay for the Agricultural_Bot model.

Current milestone:

- starts Gazebo Harmonic;
- publishes the robot description;
- spawns the normalized URDF through `ros_gz_sim create`;
- provides an empty test world.

The four-wheel steering controllers, RGB-D/LiDAR sensors, bridge YAML, and
`gz_ros2_control` configuration are intentionally separate follow-up work.
