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

当前已初始化 `agri_sim_description`、`agri_greenhouse_worlds`、
`agri_sim_control` 和 `agri_sim_bringup`。其中参数化番茄田位于
`agri_greenhouse_worlds`，默认布局为 10 行、每行 15 株；机器人 spawn、资源路径、
四舵轮、CR5、双指夹爪的 `ros2_control` 接口由其余仿真包负责。D405/MID-360
传感器插件仍未加入。

当前模型入口已经包含 SolidWorks Y-up 到 ROS Z-up 的根坐标变换，默认 `spawn_z=0.40`
使轮底接近地面；可以用 `spawn_z:=0.45` 等参数做高度微调。若模型仍出现姿态问题，先
确认没有残留的 Gazebo server，再检查启动日志和 `gz model -m agri_robot -p`。

## 最小验证

在工作空间根目录执行：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/sim_ws"
source /opt/ros/jazzy/setup.bash
source ../ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agri_sim_bringup simulation.launch.py
```

无界面测试使用 `gui:=false`：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false
```

安装 `ros-jazzy-gz-ros2-control` 后，可以启动控制器：

```bash
ros2 launch agri_sim_bringup simulation.launch.py use_control:=true
```

启动后先暂停物理，检查初始坐标和网格姿态：

```bash
ros2 launch agri_sim_bringup simulation.launch.py paused:=true
```

加载参数化番茄田并在行首安全区域生成机器人：

```bash
ros2 launch agri_sim_bringup tomato_field.launch.py
```

该入口默认暂停、关闭控制器，并将机器人放在
`x=0, y=-6, z=0.40, yaw=1.5708`。确认姿态与接地正常后，再分别启用物理和控制器。

默认入口验证 Xacro、mesh 资源、`robot_state_publisher` 和 Gazebo 实体生成；启用
`use_control` 后会加载可驱动的 ros2_control 控制器。当前仍不会发布 LiDAR/RGB-D
数据。
