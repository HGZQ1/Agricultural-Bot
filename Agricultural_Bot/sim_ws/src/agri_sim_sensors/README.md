# agri_sim_sensors

提供 MID-360 双后端与 D405 原生 RGB-D 仿真、参数校验、桥接和 RViz 配置。

## MID-360

基于 `rm_sim_26` 的双后端组织方式，适配农业机器人自己的光心、安装位姿和 ROS 接口。
默认 `gpu_lidar`；`lidar_mode:=rgl` 使用固定版本的 RGLGazeboPlugin 及官方 MID-360
40 段轮换预设。两种模式统一发布 `/mid360/points`，frame 为 `mid360_sensor_frame`。

参数位于 `config/mid360.yaml`：10 Hz，0.1–40 m，水平 360°，垂直 −7° 至 +52°。
垂直角以传感器自身坐标表达，整机安装倾角来自 canonical URDF。

GPU 模式的栅格样本数、`range_resolution` 和 `noise_stddev=0.005 m` 只作用于
GPU 后端。RGL 使用官方原版图样，目前没有接入距离噪声节点，产生理想几何回波。
公开点云链路如下：

- GPU：Gazebo `/mid360/points` 单向桥接到 ROS `/mid360/points`。
- RGL：Gazebo `/mid360/points` 桥接到 ROS `/mid360/rgl_raw`，Python
  `normalize_mid360_cloud` 按传感器局部 FOV、量程和有限值过滤后发布 `/mid360/points`，
  保持原 frame、stamp 和字段结构。

原版 RGL 预设共有 40×20,000 条射线，按帧轮换，不重复整份 800,000 射线。
其原始垂直射线实测范围约 −8.22° 至 +54.36°，约 1.32% 超出本项目标称 FOV；
图样保持原版，公开输出按 −7° 至 +52° 裁剪。

```bash
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=gpu_lidar rviz:=true
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=rgl rviz:=true
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=auto
```

首次使用 RGL，在 `Agricultural_Bot` 根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/setup_mid360_rgl.py
```

插件和预设保存在忽略的 `.cache/rgl`；可通过 `rgl_install_prefix` 和
`rgl_patterns_dir` 覆盖路径。显式 RGL 模式缺少依赖时会给出安装提示；auto
模式检查 NVIDIA 设备、插件依赖和预设文件，条件不满足时使用 gpu_lidar。

`gui:=false` 默认附加 `--headless-rendering`，仍需要可用的 Ogre2/EGL 渲染设备。
混合显卡机器可对单次启动设置 `__NV_PRIME_RENDER_OFFLOAD=1`。
`use_lidar:=false` 关闭传感器和点云桥接。
用 `rviz:=true` 查看点云。当前未装载 RGL GUI ray 可视化插件，Gazebo 自定义传感器
不作为 RGL 射线束显示入口。

RGL 光线忽略雷达外壳，但仍检查底盘、机械臂和篮筐遮挡：仿真 overlay 保留
有质量的 `mid360_link` 固定关节，避免上游 per-link 排除误伤整台机器人。
两种模式均使用同一个 sensor pose；点云 header 改名不承担坐标变换。

光心使用 CAD 光学穹顶拟合球心，扫描 frame 相对 `base_footprint` 为
`xyz=(-0.401439121228, 0.003437758801, 0.459044570728)` m、pitch=−15°。
这是现有 CAD 的仿真基准，后续可替换为实测外参。传感器修正保持网格外观不变。

GPU 后端为规则栅格功能等效；RGL 后端使用官方图样轮换。两者均承诺标准
`PointCloud2` 的 x/y/z/intensity 和整帧仿真时间戳，不承诺逐点时间、Livox
自定义消息或完整运动畸变模型。

验证世界启动和接口验收分别在两个已加载环境的终端执行：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false world:=mid360_validation.sdf
```

```bash
ros2 run agri_sim_tests check_mid360 --duration 60 --timeout 90
python3 scripts/check_mid360_scene.py --backend gpu_lidar
```

`check_mid360` 已默认启用仿真时钟；duration 是仿真数据跨度，timeout 是墙钟上限。
GPU 最终 60 秒接口验收为 583 帧、9.7 Hz、FOV 越界 0，场景 P95 误差 8.80 mm；RGL 插件编译和
CUDA/OptiX 探针、过滤后的公开链路 60 秒/601 帧/10 Hz/FOV 越界 0 与场景检查通过。
RGL 场景 36,770 个环境点全部距已知表面小于 30 mm，P95 0.007161 mm；此为未加
距离噪声的理想仿真误差，不是实机精度。传感器包 25 项、验收包 28 项测试通过。

开源版本见 [NOTICE](NOTICE)，安装与完整指标见
[mid360_simulation.md](../../../docs/mid360_simulation.md)。

## D405 RGB-D

`config/d405.yaml` 配置 Harmonic 原生 `rgbd_camera`：848×480、30 Hz、87°×58°。
`fx=446.802773119128`、`fy=432.971461265142`、渲染主点 `(424,240)` 进入实际投影。
公开 CameraInfo 主点为 `(423.5,239.5)`，补偿当前原生 Gazebo 半像素采样差异；
原生 RGB/深度共用投影与 stamp，无需额外配准节点。

| ROS 流 | 内容 |
| --- | --- |
| `/d405/color/image_raw` | `rgb8` 彩色；裁剪 0.01–10 m |
| `/d405/aligned_depth_to_color/image_raw` | `32FC1` 光学 Z 米；裁剪 0.07–0.50 m |
| `/d405/color/camera_info` | 同投影的 K/P/R/D 与 848×480 尺寸 |

三路均为 `camera_optical_frame`，单向 `GZ_TO_ROS`，SensorDataQoS。
RGB/深度使用官方 `ros_gz_image`，CameraInfo 经 `ros_gz_bridge` 进入内部 raw topic，
再由 `normalize_d405_camera_info` 唯一发布公共内参。该节点仅将 K/P 的 cx、cy
减去 `pixel_center_offset`（默认 0.5），保留 stamp、frame、焦距及其他字段；上游已
校正或版本变化时可设为 0。渲染和图像保持原值，标准整数像素反投影不另加偏移；
此适配仅用于仿真，不修改 RealSense 实机数据或算法。
相机和点云桥接进程采用 64 MiB Fast DDS SHM 缓冲，以容纳大帧；若已指定 DDS XML，
沿用用户配置。其他 RMW 不设置该 profile，订阅端无需额外配置。
Gazebo sensor 使用 `d405_sensor_frame` 的 X 前轴，ROS optical frame 为 X 右、Y 下、
Z 前。CAD 壳体保存在 `camera_lens_link`，独立中央虚拟光心位于前玻璃中心向内
3.7 mm；网格外观保持原位，虚拟光心不代表厂商标定的实机左眼中心。

范围外的 ±Inf、NaN、0 深度无效；默认近距算法工作距离为 0.10–0.50 m。彩色图可看到远处
物体，深度按所选 profile 裁剪。当前为理想无噪声模型，不实现 RealSense 立体匹配
或厂商后处理。不桥接原生 `/d405/points`，其 Gazebo X 前坐标与 optical header 存在
语义冲突；后续按 depth+CameraInfo 生成标准 optical 点云。

默认 `use_camera:=true`；`use_camera:=false` 关闭 sensor 与三路桥接，`camera_config`
可覆盖 YAML。`rviz:=true` 使用 `rviz/sensors.rviz` 显示 RGB、深度、雷达与 TF。

```bash
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120
```

可选 `config/d405_extended_sim.yaml` 将深度远裁剪设为 2 m，保留默认 profile 的
0.5 m；启动时 RViz 深度 Min/Max 跟随配置。启动/验收：

```bash
ros2 launch agri_sim_bringup simulation.launch.py rviz:=true \
  camera_config:="$(ros2 pkg prefix --share agri_sim_sensors)/config/d405_extended_sim.yaml"
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120 --max-depth 2
```

2 m 是供后续 YOLO 与粗定位流程使用的理想仿真量程，不承诺 D405 实机远距精度。
后续感知距离过滤需参数化，近距抓取继续使用 0.10–0.50 m 基线；YOLO 尚未实现。
扩展配置已通过相机独立 60 秒验收：三路约 30.303 Hz，联合同步/及时 TF 覆盖
99.671%，TF 逾时与深度违规均为 0。1.6 m 紫球内部深度有效率 100%，球面 P95
0.354 mm、边界 1 px 匹配 100%；4 cm/2.2 m 靶内部深度无效 100%。新公共
CameraInfo 的 K/P 主点为 `(423.5,239.5)`。报告见
[interface_60s.json](../../../artifacts/d405_extended_20261006/interface_60s.json) 和
[scene_report.json](../../../artifacts/d405_extended_20261006/scene_report.json)。
本次传感器 109、验收器 118、bringup 1，共 228 项测试通过；以下默认 profile 与
雷达共存指标保留为历史测试结果，本次未重新执行相机/雷达共存验收。

相机配置/overlay 新增 48 项、验收器新增 43 项边界测试通过；验收包含 MID-360 共
71 项通过，几何/SDF 回归通过。60 秒接口和三组靶标场景验收通过。
坐标、机械基准、规格来源和验收方法见
[d405_simulation.md](../../../docs/d405_simulation.md)。
