# Agricultural_Bot

ROS 2 Jazzy + Gazebo Harmonic 农业番茄采摘机器人项目工作空间。

截至 2026-10-09，整机组件化模型、四舵轮运动学、底盘/CR5/夹爪仿真控制、MID-360
和 D405 RGB-D 仿真接口已经接通。MID-360 默认使用 GPU LiDAR，可选 RGL 官方图样；
D405 使用 Harmonic 原生 RGB-D，连续接口与靶标场景验收通过。参数化温室、导航路线、
YOLO 番茄检测已经接入 `ros2_ws`，可读取 D405 RGB-D 图像并发布检测结果和三维位置；
采摘任务仍在分阶段开发。2026-10-07 集成了参数化番茄田（150 株）及入口生成机器人功能；
依赖、构建、打开场地与复现步骤见 [番茄田复现指南](docs/tomato_field_reproduction.md)。
开启底盘控制、行间测试路线及 Gazebo 真值量化评测见
[番茄田底盘运动测试](docs/chassis_field_motion.md)。
阶段一已增加速度门控、锁车、标准 `/odom` 适配和换舵稳态回归；阶段二已增加
`agri_lidar_adapter`、水平 `mid360_scan_frame` 和 `/scan`，并在番茄田无界面仿真中通过
8 秒接口验收。阶段三已增加 `agri_navigation` 的 SLAM Toolbox 建图入口、5 cm 地图
参数、地图/pose graph 保存流程和 `check_mapping` 验收器。阶段四已加入保存地图
`map_server`＋AMCL 定位、NavFn（Dijkstra）＋RPP 单目标导航、静态/实时障碍代价地图和
碰撞监控；参数化路线仍按[导航开发流程](docs/navigation_development_plan.md)继续开发。

原始 SolidWorks、旧版 URDF 和图纸保留原位；canonical 模型是独立修正的副本。
RGL 第三方依赖安装到忽略的 `.cache/rgl`，版本和下载校验值由安装脚本固定。

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
MID-360 双后端、安装、光心基准、运行与验收记录见
[mid360_simulation.md](docs/mid360_simulation.md)。
D405 光学坐标、对齐投影、深度范围与验收见
[d405_simulation.md](docs/d405_simulation.md)。
导航开发流程见 [navigation_development_plan.md](docs/navigation_development_plan.md)：SLAM Toolbox 建图、AMCL 定位、NavFn＋RPP 导航与停车作业状态机；FAST-LIO 已取消。
Panther-FR3 温室项目的导航、感知与夹取源码评估见 [robot_workspaces_review.md](docs/robot_workspaces_review.md)。

## 当前约束

- 本轮未创建 `references/`。
- 所有我方自研 ROS 2 可执行节点使用 Python 3.12、`rclpy` 和
  `ament_python`。
- Nav2、MoveIt 2、SLAM Toolbox、Gazebo 和 ros2_control 使用 Jazzy 官方二进制。
- `agri_robot_description` 已拆分为组件化 Xacro，并通过 `check_urdf` 和 Gazebo SDF 转换。
- canonical URDF 已加入 SolidWorks CAD 坐标到 ROS/Gazebo 坐标的根轴变换，解决初始模型
  侧躺问题；惯性原点仍需后续按 link 局部坐标校准。
- 四舵轮运动学、`gz_ros2_control`、CR5/夹爪控制器已经接通，用户已确认底盘移动测试通过。
- MID-360 已有光学 frame、GPU/RGL 双后端、点云桥接、RViz 配置与验收工具；两个后端
  均通过连续 60 仿真秒接口、FOV 和四墙/地面场景检查；`agri_lidar_adapter` 已将
  `/mid360/points` 转为 `/mid360/navigation_points` 和 `/scan`。
- MID-360 光心和 −15° 安装俯仰来自现有 CAD 的光学穹顶拟合，后续仍需实测外参标定。
- D405 已接入 848×480/30 Hz RGB、对齐 32FC1 米深度和 CameraInfo；60 仿真秒接口及
  三组靶标几何/裁剪验收通过。当前 canonical 模型为 27 个 link、26 个 joint（包含
  水平 `mid360_scan_frame`）。
- D405 中央虚拟 pinhole 来自 CAD 前玻璃中心向内 3.7 mm，不代表实机左眼或手眼标定。
  完整 M3 仍需温室与番茄资产。
- AMCL 与 Nav2 单目标基线已经接入；参数化田路线、停车作业状态机、MoveIt、YOLOv8
  和采摘任务逻辑仍待实现。`agri_navigation` 同时提供互斥的建图与保存地图导航入口。

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

`gui:=false` 默认启用 Ogre2 无窗口渲染，仍需要可用的 EGL 渲染设备。
需要在重力积分前检查模型时，可使用 `paused:=true`；传感器按仿真时间更新。

显示 MID-360 点云、D405 RGB/深度或切换雷达后端：

```bash
ros2 launch agri_sim_bringup simulation.launch.py rviz:=true
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=rgl rviz:=true
```

RGL 首次安装步骤和自动回退规则见 [MID-360 文档](docs/mid360_simulation.md)。
默认点云为 `/mid360/points`，类型 `sensor_msgs/msg/PointCloud2`，frame 为
`mid360_sensor_frame`，10 Hz，水平 360°、垂直 −7° 至 +52°，量程 0.1–40 m。
导航默认扫描为 `/scan`，frame 为 `mid360_scan_frame`，原点保持 MID-360 光心，默认
高度切片 `-0.40..0.40 m`、角分辨率 1°；启动时加 `use_scan:=true`。扫描验收命令：

```bash
ros2 run agri_sim_tests check_scan --duration 8 --timeout 60
```

阶段三建图基线（先启动带 `use_scan:=true` 的番茄田仿真，再在另一终端运行）：

```bash
ros2 launch agri_navigation mapping.launch.py
ros2 run agri_sim_tests check_mapping --duration 5 --timeout 90
```

`mapping.launch.py` 只启动 SLAM Toolbox，发布 `map → odom` 和 `/map`；它不会同时
启动 AMCL、`map_server` 或 Nav2。确认 RViz 中地图闭合后保存：

```bash
MAP_DIR="artifacts/maps/field_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/tomato_field" \
  --ros-args -p use_sim_time:=true \
    -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_DIR/tomato_field'}"
```

地图 YAML/PGM、pose graph、生成田场景的 seed 和布局参数应作为同一地图版本保存；
改变行距、冠幅或通道边界后需重新建图并重新标定停车点。

阶段四保存地图定位和 Nav2 单目标导航（不要同时运行 `mapping.launch.py`）：

```bash
ros2 launch agri_navigation navigation.launch.py \
  use_sim_time:=true initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

省略 `map` 参数时使用仓库内置的 `stage3_baseline.yaml`；自定义场景完成建图后，再将
`map` 指向该场景实际存在的 YAML 文件。

布局参数改变后的地图/代价地图重建、单目标命令和可选植株禁行遮罩见
[阶段四导航说明](docs/navigation_stage4.md)。

接口快速验收在另一个已加载环境的终端运行：

```bash
ros2 run agri_sim_tests check_mid360 --duration 5 --timeout 90
```

D405 默认启用，三路输出使用 `camera_optical_frame`，深度范围 0.07–0.50 m，
近距算法工作距离 0.10–0.50 m。相机验收命令：

```bash
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120
```

后续 YOLO 粗定位流程可切换 0.07–2 m 理想仿真 profile，RViz 深度显示同步量程：

```bash
ros2 launch agri_sim_bringup simulation.launch.py rviz:=true \
  camera_config:="$(ros2 pkg prefix --share agri_sim_sensors)/config/d405_extended_sim.yaml"
# 另一终端
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120 --max-depth 2
```

扩展配置不保证实机 D405 在 2 m 的精度。后续感知距离过滤需参数化，远处粗定位后
接近至 0.10–0.50 m 再估计抓取位姿。YOLO 启动、模型路径和话题验收见
[YOLO 番茄检测接入说明](docs/yolo_tomato_integration.md)。量程与靶标验收见
[D405 文档](docs/d405_simulation.md)。

公开 CameraInfo 主点为 `(423.5,239.5)`；仿真专用节点补偿原生 Gazebo 的半像素
采样差异，保持渲染主点 `(424,240)`。感知按标准整数像素反投影，实机数据不加此补偿。
2 m profile 已通过相机独立 60 秒验收：三路约 30.303 Hz，联合同步/及时 TF 覆盖
99.671%；1.6 m 靶标深度及 2.2 m 裁剪通过，详见
[实际报告](artifacts/d405_extended_20261006/interface_60s.json)。此前雷达共存结果为历史验收。

仅加载模型、不启动控制器：

```bash
ros2 launch agri_sim_bringup simulation.launch.py \
  use_control:=false use_lidar:=false use_camera:=false
```

番茄田支持逐株果实数量、高度、直径和成熟概率随机化，并可用固定种子复现。
随机果实默认显示原单果网格和红/青贴图，支持 `--fruit-visual sphere` 切回简化球体。
`--plant-height` 和 `--plant-width` 可独立设置枝叶株高和冠幅，同步调整简化茎杆碰撞体，
不会自动改变果实直径、世界高度或水平分布半径；共享资产保持原样。
未启用果实随机化时，原固定果实和花朵的大小、位置保持原样。功能更新后构建一次
`agri_greenhouse_worlds`；此后修改参数只需重新生成世界并重启对应 Gazebo。
实际温室尺寸待测，2 m 株高、1 m 冠幅及 0.80–1.60 m 果实中心高度仅为测试值。
生成与加载自定义世界见
[植株尺寸与果实参数化指南](docs/tomato_fruit_randomization.md)。独立果实当前为静态目标，采摘物理待接入。
