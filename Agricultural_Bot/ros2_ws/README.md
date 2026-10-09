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

当前已初始化 `agri_robot_description` 组件化 Xacro、`agri_base_kinematics` Python 包、
`agri_base_adapter` 底盘速度/里程计适配包、`agri_lidar_adapter` MID-360 导航扫描适配包，
以及 `agri_navigation` 的 SLAM Toolbox 建图、保存地图 AMCL 定位和 Nav2 单目标导航
基线。建图和定位模式互斥；参数化路线、MoveIt、感知和任务算法包继续分阶段实现。

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

番茄田仿真入口会同时启动速度门控和 `/wheel/odom → /odom` 适配：遥控使用 `/cmd_vel`，
后续 Nav2 使用 `/cmd_vel_nav`，四舵轮节点只接收 `/cmd_vel_safe`。门控默认限制
`0.30 m/s / 0 / 0.50 rad/s`，输入断流 0.35 s 后归零；底层四舵轮看门狗为 0.5 s。
`/base_motion/set_lock` 可在停车或机械臂作业前锁住底盘。

MID-360 导航扫描启动（需要 `sim_ws` 中的传感器或实机点云及 TF）：

```bash
ros2 launch agri_lidar_adapter lidar_to_scan.launch.py
```

输入 `/mid360/points`（`mid360_sensor_frame`）先变换/裁剪到
`mid360_scan_frame`，输出 `/mid360/navigation_points` 和 `/scan`。水平 frame 与雷达
光心同原点，默认切片 `-0.40..0.40 m`、量程 `0.10..40 m`；参数见
`src/agri_lidar_adapter/config/lidar_to_scan.yaml`。

SLAM Toolbox 建图基线（先由仿真或实机提供 `/scan` 和 `odom → base_footprint`）：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch agri_navigation mapping.launch.py
```

该入口只让 SLAM Toolbox 发布 `map → odom` 和 `/map`，不启动 AMCL、`map_server` 或
Nav2。番茄田仿真中的接口检查由 `sim_ws` 的 `check_mapping` 完成；地图保存建议使用
`map_subscribe_transient_local:=true`，并将 YAML/PGM、pose graph、田场景 seed 和布局
参数保存为同一版本。详细参数见
`src/agri_navigation/config/slam_toolbox.yaml`，启动参数见
`src/agri_navigation/launch/mapping.launch.py`。

保存地图定位与单目标导航（先由仿真或实机提供 `/scan`、`/odom` 与底盘速度门控）：

```bash
ros2 launch agri_navigation navigation.launch.py \
  map:="/absolute/path/to/tomato_field.yaml" \
  use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

该入口按顺序激活 map server、AMCL、RPP controller、NavFn planner、behaviors、
BT Navigator 和 collision monitor。导航速度链为
`/cmd_vel_nav_raw → /cmd_vel_nav → /cmd_vel_safe`。场景布局变化后的地图与代价地图
更新方法见 `../docs/navigation_stage4.md`。
