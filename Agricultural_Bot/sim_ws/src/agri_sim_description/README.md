# agri_sim_description

Gazebo Harmonic overlay for the Agricultural_Bot model.

Current milestone:

- starts Gazebo Harmonic;
- expands the Gazebo-specific Xacro and publishes `robot_description`;
- spawns the normalized model through `ros_gz_sim create`;
- provides an empty test world.
- optionally injects all `gz_ros2_control` joint interfaces and starts the
  configured controllers with `use_control:=true`.

The default launch enables `use_control:=true`; use `use_control:=false` for a
model-only smoke test.

RGB-D/LiDAR sensors and bridge configuration remain follow-up work.
