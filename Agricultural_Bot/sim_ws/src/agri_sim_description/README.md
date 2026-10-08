# agri_sim_description

农业机器人模型的 Gazebo Harmonic 仿真 overlay。

截至 2026-10-06，已提供：

- 展开 canonical 模型和 Gazebo 专用 Xacro，发布 `robot_description` 并生成机器人实体；
- `gz_ros2_control`、四舵轮运动学与控制器启动；
- 空世界 `empty_greenhouse.sdf`、雷达验收世界 `mid360_validation.sdf` 与相机验收世界
  `d405_validation.sdf`；
- MID-360 GPU LiDAR/RGL 后端选择、仿真时间、桥接、RGL 规范化节点与可选 RViz；
- MID-360 水平导航扫描：`agri_lidar_adapter` 点云过滤、`mid360_scan_frame` 和 `/scan`；
- D405 原生对齐 RGB-D、匹配 CameraInfo、独立彩色/深度裁剪与三路桥接；
- 运行时生成独立世界/桥接 YAML，并保留原世界目录及已有 Gazebo 资源路径。

推荐从 `agri_sim_bringup` 启动：

```bash
ros2 launch agri_sim_bringup simulation.launch.py
ros2 launch agri_sim_bringup simulation.launch.py gui:=false world:=mid360_validation.sdf
```

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `world` | `empty_greenhouse.sdf` | 本包 worlds 下文件名，或外部世界绝对路径 |
| `gui` | `true` | Gazebo 窗口；false 启动 server |
| `headless_rendering` | `true` | server 模式启用 Ogre2/EGL 无窗口渲染 |
| `paused` | `false` | 是否先暂停物理与传感器时间 |
| `spawn_z` | `0.40` | 初始实体高度，单位 m |
| `use_control` | `true` | 加载控制器 |
| `use_kinematics` | 跟随 `use_control` | 启动四舵轮命令/里程计节点 |
| `use_lidar` | `true` | 启动 MID-360 和公开点云输出 |
| `use_scan` | `false` | 将 MID-360 点云过滤并转换为 `/scan`；要求 `use_lidar:=true` |
| `scan_min_height`、`scan_max_height` | `-0.40`、`0.40` | `mid360_scan_frame` 中的导航高度切片 |
| `scan_range_min`、`scan_range_max` | `0.10`、`40.0` | 二维扫描量程（米） |
| `scan_angle_increment` | `0.0174533` | 二维扫描角分辨率（弧度） |
| `scan_voxel_size` | `0` | 过滤点云体素边长；0 表示不降采样 |
| `use_camera` | `true` | 启动 D405 RGB、对齐深度和 CameraInfo |
| `camera_config` | 包内 d405.yaml | 覆盖相机 YAML 路径 |
| `lidar_mode` | `gpu_lidar` | `gpu_lidar`、`rgl` 或 `auto` |
| `rviz` | `false` | 相机启用时显示 RGB/深度/雷达/TF，否则显示雷达/TF/模型 |
| `lidar_config` | 包内 mid360.yaml | 覆盖雷达 YAML 路径 |
| `rgl_install_prefix`、`rgl_patterns_dir` | 雷达 YAML 中配置 | 覆盖 RGL 本地缓存路径 |

仅加载模型使用 `use_control:=false use_lidar:=false use_camera:=false`。
MID-360 光心与双后端契约、安装及验收见
[mid360_simulation.md](../../../docs/mid360_simulation.md)。
D405 光学 frame、848×480/30 Hz、87°×58° 投影、0.07–0.50 m 深度及正式
运行验收见 [d405_simulation.md](../../../docs/d405_simulation.md)。
