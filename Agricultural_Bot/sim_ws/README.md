# sim_ws

Gazebo Harmonic 仿真环境与仿真 overlay 工作空间。

工作空间包：

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

截至 2026-10-06，`agri_sim_description`、`agri_sim_control` 和 `agri_sim_bringup`
已包含机器人 spawn、资源路径、四舵轮/CR5/双指夹爪控制接口与控制器。
`agri_sim_sensors` 已提供 MID-360 的 GPU LiDAR/RGL 双后端、D405 原生对齐 RGB-D、
桥接、RGL 数据规范化节点和 RViz；`agri_sim_tests` 提供两类传感器可执行验收。
`agri_greenhouse_worlds` 已提供 22×14 m 参数化番茄田；`tomato_field.launch.py` 默认加载
6 行×10 株、2 m 行距、0.7 m 株距的导航开发场景，旧 10 行×15 株场景仍作为兼容基线保留。
首次依赖准备、构建、只打开场地、导入机器人和排错见
[番茄田复现指南](../docs/tomato_field_reproduction.md)。完整温室结构和可采摘果实仍待开发。

当前模型入口已经包含 SolidWorks Y-up 到 ROS Z-up 的根坐标变换，默认 `spawn_z=0.40`
使轮底接近地面；可以用 `spawn_z:=0.45` 等参数做高度微调。若模型仍出现姿态问题，先
确认没有残留的 Gazebo server，再检查启动日志和 `gz model -m agri_robot -p`。

## 最小验证

在工作空间根目录执行：

```bash
cd Agricultural_Bot/sim_ws  # 从 Git 仓库根目录执行
source /opt/ros/jazzy/setup.bash
source ../ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agri_sim_bringup simulation.launch.py
```

无界面测试使用 `gui:=false`，默认同时传入 `--headless-rendering`，由 Ogre2/EGL
产生 GPU 传感器数据：

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

该入口默认暂停，关闭控制器、雷达和相机，并将机器人放在
`x=0, y=-6, z=0.40, yaw=1.5708`。确认姿态与接地正常后，再分别启用物理和控制器。

通用 `simulation.launch.py` 入口仍默认启用控制器、四舵轮运动学、MID-360 GPU LiDAR 和 D405 RGB-D。
`use_control:=false` 同时默认关闭运动学节点；`use_lidar:=false` 关闭雷达及点云桥接。
`use_camera:=false` 关闭相机和三路 RGB-D 桥接；`camera_config` 指定相机 YAML。

## MID-360 运行与验收

```bash
ros2 launch agri_sim_bringup simulation.launch.py rviz:=true
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=rgl rviz:=true
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=auto
```

显式 `rgl` 要求先运行项目根目录的 `scripts/setup_mid360_rgl.py`；`auto` 在依赖预检
不通过时回退到 GPU LiDAR。两种后端的公开 Topic、frame、仿真时间戳和量程/FOV
契约相同，原始 RGL 图样超出标称角度的点由仿真侧规范化节点过滤。

接口验收需在仿真运行后，从另一个已 source 两个工作空间的终端执行：

```bash
ros2 run agri_sim_tests check_mid360 --duration 5 --timeout 90
```

GPU 最终记录为连续 60 仿真秒、9.7 Hz、583 帧、FOV 违规 0；RGL 规范化公开链路为
60 秒、10 Hz、601 帧、FOV 违规 0；两种后端的四墙/地面场景检查均通过。
这只覆盖 MID-360 的传感器验收；相机结果见下节，完整 M3 仍需温室与番茄资产。
安装版本、独立场景验收命令、证据路径和 RGL 验收状态见
[mid360_simulation.md](../docs/mid360_simulation.md)。

## D405 运行与验收

原生 RGB-D 共用投影和 stamp，输出 848×480、30 Hz 的 RGB、对齐深度与 CameraInfo。
三路 frame 为 `camera_optical_frame`；`32FC1` 深度为光学 Z 米，范围 0.07–0.50 m。
彩色图允许显示 10 m 内物体，深度不会因此扩展到远处；算法使用 0.10–0.50 m。

```bash
ros2 launch agri_sim_bringup simulation.launch.py rviz:=true
```

第二个终端执行：

```bash
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120
```

相机启用时 RViz 显示 RGB、深度、雷达及 TF。相机配置新增 48 项测试、验收器新增
43 项边界测试通过，Xacro/SDF 几何验证通过；60 仿真秒接口及三组靶标场景验收通过。
CAD 中央 pinhole、裁剪、双编码及测试命令见
[d405_simulation.md](../docs/d405_simulation.md)。
