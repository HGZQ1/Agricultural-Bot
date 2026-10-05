# ros2_ws

机器人运行算法与硬件无关接口工作空间。

计划包含：

- `agri_interfaces`
- `agri_robot_description`
- `agri_robot_moveit_config`
- `agri_base_kinematics`
- `agri_base_adapter`
- `agri_lidar_adapter`
- `agri_camera_adapter`
- `agri_arm_adapter`
- `agri_gripper_adapter`
- `agri_navigation`
- `agri_perception`
- `agri_manipulation`
- `agri_task_manager`
- `agri_bringup`
- `agri_tests`

当前已初始化 `agri_robot_description` 组件化 Xacro 和 `agri_base_kinematics` Python 包。
感知、导航、MoveIt 和任务算法包保持骨架，等待控制接口冻结后逐个实现。

## 模型检查

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
check_urdf install/agri_robot_description/share/agri_robot_description/urdf/robot_pick_robot11.urdf
gz sdf -p install/agri_robot_description/share/agri_robot_description/urdf/robot_pick_robot11.urdf >/tmp/agri_robot.sdf
ros2 launch agri_robot_description display.launch.py
```

无显示环境使用 `rviz:=false`。

当前 canonical 模型保留最新 URDF 的尺寸和安装位姿；四个转向 link 已修正为 15 kg，
右指轴为 `0 0 -1`，右后轮速度上限与左后轮一致。CR5 轴系、碰撞网格和控制器仍需
根据更新后的 CAD/实机参数定版。

四舵轮节点启动：

```bash
ros2 launch agri_base_kinematics four_wheel_steering.launch.py
```

默认输入 `/cmd_vel` 为 `geometry_msgs/msg/TwistStamped`，输出四个转向角、四个轮速数组
和 `/wheel/odom`；仿真控制模式会自动启动该节点。
