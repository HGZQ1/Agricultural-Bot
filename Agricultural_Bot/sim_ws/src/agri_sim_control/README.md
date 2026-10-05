# agri_sim_control

`agri_sim_control` contains the Gazebo Harmonic `ros2_control` controller
configuration for the canonical `agri_robot` model.

## Model interface

`agri_sim_description/urdf/agri_robot.gazebo.urdf.xacro` supplies the
`gz_ros2_control` plugin and exports these interfaces when `use_control` is
enabled:

- `f1_steer_joint`, `f2_steer_joint`, `r1_steer_joint`, `r2_steer_joint`:
  `position` command and `position` state.
- `f1_wheel_joint`, `f2_wheel_joint`, `r1_wheel_joint`, `r2_wheel_joint`:
  `velocity` command plus `position` and `velocity` state.
- `arm_joint1` through `arm_joint5` and `cr5_joint6`: `position` command plus
  `position` and `velocity` state.
- `gripper_left_finger_joint` and `gripper_right_finger_joint`: `position`
  command and `position` state.

The plugin loads `config/controllers.yaml` through the simulation Xacro's
`controller_config` argument. `gz_ros2_control` owns the controller manager in
simulation; this package only starts the controller spawners.

## Launch

Install the Gazebo hardware plugin if it is not already present:

```bash
sudo apt install ros-jazzy-gz-ros2-control
```

Then start the simulation with:

```bash
ros2 launch agri_sim_bringup simulation.launch.py use_control:=true
```

The default `use_control:=true` path starts the controllers and Python steering
kinematics. Use `use_control:=false` for a model-only smoke test.

The gripper intentionally lists both finger joints. Gazebo DART does not
enforce URDF `<mimic>` constraints, so the future Python gripper adapter should
send the same position value to both joints (their axes are already opposite),
rather than relying on the mimic tag.
