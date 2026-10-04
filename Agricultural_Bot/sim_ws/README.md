# sim_ws

Gazebo Harmonic 仿真环境与仿真 overlay 工作空间。

计划包含：

- `agri_sim_description`
- `agri_greenhouse_worlds`
- `agri_sim_sensors`
- `agri_sim_control`
- `agri_sim_bringup`
- `agri_sim_assets`
- `agri_sim_tests`

构建本工作空间前必须先 source：

```bash
source /opt/ros/jazzy/setup.bash
source ../ros2_ws/install/setup.bash
```

当前已初始化 `agri_sim_description` 和 `agri_sim_bringup`，包含空 Gazebo 世界、机器人
spawn 入口和资源路径配置。四舵轮控制器、CR5/夹爪控制、D405/MID-360 传感器插件和
温室资产仍未加入。

当前模型入口已经包含 SolidWorks Y-up 到 ROS Z-up 的根坐标变换，默认 `spawn_z=0.40`
使轮底接近地面；可以用 `spawn_z:=0.45` 等参数做高度微调。若模型仍出现姿态问题，先
确认没有残留的 Gazebo server，再检查启动日志和 `gz model -m agri_robot -p`。

## 最小验证

在工作空间根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
colcon build --symlink-install --base-paths sim_ws
source sim_ws/install/setup.bash
ros2 launch agri_sim_bringup simulation.launch.py
```

无界面测试使用 `gui:=false`：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false
```

启动后先暂停物理，检查初始坐标和网格姿态：

```bash
ros2 launch agri_sim_bringup simulation.launch.py paused:=true
```

该入口验证 URDF、mesh 资源、`robot_state_publisher` 和 Gazebo 实体生成；它还没有
提供可驱动的 ros2_control 控制器，也不会发布真实的 LiDAR/RGB-D 数据。
