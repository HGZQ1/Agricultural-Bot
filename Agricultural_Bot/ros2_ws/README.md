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

当前已初始化 `agri_robot_description` 资源包，并导入 `robot_pick_robot11` 的规范化副本。
其余算法包保持空骨架，待 canonical TF、关节命名和控制接口冻结后逐个初始化。

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
