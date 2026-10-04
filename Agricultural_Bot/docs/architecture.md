# 工作空间边界

## ros2_ws

保存机器人运行时算法和硬件无关接口：

- canonical URDF/Xacro 和 MoveIt 配置；
- 自定义 msg/srv/action；
- Python 四舵轮运动学与硬件 adapter；
- SLAM/Nav2、感知、抓放、任务状态机和验收节点。

所有自研可执行节点使用 Python 3.12、`rclpy` 和 `ament_python`。

## sim_ws

保存 Gazebo Harmonic 专用内容：

- 仿真 Xacro/SDF overlay；
- `gz_ros2_control` 和控制器参数；
- MID-360、D405、IMU 等效传感器与 `ros_gz_bridge`；
- 温室、土槽、植株、番茄、篮筐模型；
- 仿真启动和 headless 系统测试。

## 依赖规则

```text
/opt/ros/jazzy -> ros2_ws -> sim_ws
```

- `ros2_ws` 不得依赖 `sim_ws`。
- `sim_ws` 可以 overlay `ros2_ws`。
- Gazebo entity ID、真值位姿和仿真专用 Topic 不得进入算法接口。
- 本阶段不创建 `references/`。

