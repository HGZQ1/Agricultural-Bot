# Agricultural_Bot

ROS 2 Jazzy + Gazebo Harmonic 农业番茄采摘机器人项目工作空间。

当前阶段已建立 ROS 2/Gazebo 包骨架和资产审计基线。原始 SolidWorks、旧版 URDF、图纸
和第三方项目仍未直接纳入工作空间；canonical 模型是经过最小结构修正的独立副本。

## 工作空间

- `ros2_ws`：机器人运行算法、稳定接口、canonical 机器人描述、MoveIt 配置和
  Python `rclpy` 节点。
- `sim_ws`：Gazebo Harmonic 世界、仿真 overlay、仿真传感器、控制配置和
  headless 集成测试。

依赖方向固定为：

```text
/opt/ros/jazzy -> ros2_ws -> sim_ws
```

`ros2_ws` 不得依赖 `sim_ws`。仿真和实机通过相同 ROS Topic、Service、
Action 和 TF 契约替换底层 adapter。

## 当前资料

原始输入暂时保留在项目外层：

- `../农机/`：SolidWorks 导出的 ROS 1 URDF、STL、launch 和关节 CSV。
- `../番茄自主采摘机器人/`：SolidWorks 装配体、零件、STEP 和压缩资源。
- `../共享资源/`：机器人尺寸图、CR5 V1.8 使用手册和资源说明。

已确认参数、资料路径和迁移阻塞项见
[asset_inventory.md](docs/asset_inventory.md)。
URDF 颜色、父子组件位姿和当前夹爪安装修正方法见
[urdf_model_editing.md](docs/urdf_model_editing.md)。

## 当前约束

- 本轮未创建 `references/`。
- 所有我方自研 ROS 2 可执行节点使用 Python 3.12、`rclpy` 和
  `ament_python`。
- Nav2、MoveIt 2、SLAM Toolbox、Gazebo 和 ros2_control 使用 Jazzy 官方二进制。
- `agri_robot_description` 已拆分为组件化 Xacro，并通过 `check_urdf` 和 Gazebo SDF 转换。
- canonical URDF 已加入 SolidWorks CAD 坐标到 ROS/Gazebo 坐标的根轴变换，解决初始模型
  侧躺问题；惯性原点仍需后续按 link 局部坐标校准。
- 四舵轮运动学、`gz_ros2_control`、CR5/夹爪控制器已经完成阶段一；D405/MID-360 传感器、
  温室世界、MoveIt、Nav2、YOLOv8 和采摘任务逻辑属于后续阶段。

## 当前最小运行入口

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
ros2 launch agri_sim_bringup simulation.launch.py
```

服务器模式（适合 CI 或无显示环境）：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false
```

需要在重力积分前检查模型时，可使用 `paused:=true`。

仅加载模型、不启动控制器：

```bash
ros2 launch agri_sim_bringup simulation.launch.py use_control:=false
```
