# D405 RGB-D 仿真与验收

更新时间：2026-10-06。腕部 D405 已使用 Gazebo Harmonic 原生 `rgbd_camera` 接入，
配置、光学坐标、三路桥接与可执行验收器已经实现。离线配置、编码边界、Xacro/SDF
几何检查及靶标场景通过；连续运行结果见下方验收表。

## 公开接口与投影

| 项目 | 当前契约 |
| --- | --- |
| 彩色图 | `/d405/color/image_raw`，`sensor_msgs/msg/Image`，`rgb8`；验收兼容 `bgr8` |
| 对齐深度 | `/d405/aligned_depth_to_color/image_raw`，`32FC1`，以米表示的光学 Z 深度 |
| 内参 | `/d405/color/camera_info`，`sensor_msgs/msg/CameraInfo` |
| 尺寸 / 频率 | 三路 848×480，30 Hz，仿真时间 stamp |
| frame | 三路均为 `camera_optical_frame`，X 右、Y 下、Z 前 |
| 深度范围 | 默认 0.07–0.50 m；可选理想仿真 profile 为 0.07–2.00 m；0、NaN、Inf 为无效 |
| 彩色裁剪 | 0.01–10 m；远处可以出现在彩色图中，但不会扩展深度工作范围 |
| 视场 | 水平 87°、垂直 58°，理想 pinhole 模型 |
| 桥接 / QoS | 三路单向 `GZ_TO_ROS`，SensorDataQoS：Best Effort、Volatile、Keep Last 5 |

`config/d405.yaml` 经 `camera_configuration.py` 校验后生成传感器和桥接配置。
原生 RGB-D 相机共用一次渲染、投影和 stamp，所以公开深度已经与彩色图对齐；
不额外创建独立深度相机或执行第二次图像重采样。

独立焦距同时满足两方向标称视场，不能简单令 `fx=fy`：

```text
fx = 848 / (2 tan(87° / 2)) = 446.802773119128
fy = 480 / (2 tan(58° / 2)) = 432.971461265142
Gazebo 渲染主点：cx_render = 424, cy_render = 240
ROS 整数像素中心主点：cx = 423.5, cy = 239.5
K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]
P = [fx, 0, cx, 0; 0, fy, cy, 0; 0, 0, 1, 0]
```

焦距与渲染主点进入 Gazebo 的实际投影；公开 CameraInfo 使用对应的 ROS 整数像素
中心约定。当前为理想无畸变、无噪声相机，不复刻立体匹配误差、
曝光、反射材料造成的空洞或厂商的软件后处理。

本机 gz-sensors 8.2.2 / Ogre2 的自动 P 矩阵会沿用单焦距，显式配置
`lens/intrinsics` 和 `lens/projection`，使 K/P 的 fx、fy 与渲染一致。
验收器会拒绝当前理想、无畸变相机的 K/P 不一致，场景反投影检验真实渲染结果。

本机原生 gz-sensors 8.2.2 直接将渲染主点复制到 CameraInfo，实际像素采样有半像素
偏移；2 m 靶标诊断暴露出该误差。仿真专用 `normalize_d405_camera_info` 将 K/P 的
cx、cy 各减 0.5，得到 `(423.5,239.5)`，保持 stamp、frame、fx/fy 和其他字段。
Gazebo 渲染主点仍为 `(424,240)`，图像不重采样。标准反投影仍使用整数 `(u,v)`
和公开 CameraInfo，无需在感知或检查器里再加半像素。
此校正只用于当前 Gazebo 仿真，不作用于 RealSense 实机数据或 ros2_ws 算法；
节点参数 `pixel_center_offset` 默认 0.5，渲染版本变化或上游已校正时可设为 0，避免叠加。

Gazebo 深度范围外可能产生 `−Inf` 或 `+Inf`，入口统一按无效值排除。验收器的
`depth_metres` helper 兼容 `32FC1` 米和 `16UC1` 毫米，向量化转换为 float32 米，
0/负值/NaN/Inf 归为 NaN；正式传感器当前输出 `32FC1`，不增加一层编码转换节点。
深度表示光学 Z，而不是从光心到表面的欧氏距离。

当前只桥接 `/d405/image`、`/d405/depth_image`、`/d405/camera_info`。
RGB/深度通过官方 `ros_gz_image/image_bridge` 发布，CameraInfo 通过
`ros_gz_bridge/parameter_bridge` 进入内部 raw info，再由
`normalize_d405_camera_info` 唯一发布公共 `/d405/color/camera_info`；使用 SensorDataQoS。
图像桥接支持后续 image_transport 订阅，验收使用原始图像流。
不直接桥接原生 `/d405/points`：本机原生点坐标沿 Gazebo X 前轴，而点云 header 使用
optical frame，会造成坐标语义冲突。后续需要相机点云时，从已对齐深度与 CameraInfo
反投影生成标准 optical XYZ，再经 TF 转换：

```text
X = (u - cx) Z / fx
Y = (v - cy) Z / fy
Z = depth[v, u]
```

## 大图像传输

RGB 和 float32 深度单帧分别约 1.22 MB 和 1.63 MB。本机 Fast DDS 2.14.6 默认
SHM 段为 512 KiB，首次测试 Gazebo 约 30 Hz，但 ROS RGB 仅约 15 Hz。
`config/d405_fastdds.xml` 为图像及 MID-360 点云桥接进程配置 64 MiB SHM 段、4 MiB 最大消息和
UDP socket buffer，保留 UDP 发现及 Best Effort QoS，不修改系统 sysctl 或其他进程。
Jazzy 2.x 使用 `FASTRTPS_DEFAULT_PROFILES_FILE` 加载配置。

launch 仅在 Fast DDS 且用户未指定 `FASTRTPS_DEFAULT_PROFILES_FILE` 或
`FASTDDS_DEFAULT_PROFILES_FILE` 时给这两个桥接进程设置该文件。用户的 DDS 配置优先，
其他 RMW 沿用原配置；自定义 DDS 配置需自行保证图像吞吐。普通 ROS 订阅端无需
额外设置，此次正式验收也使用默认订阅端配置。GPU 点云单帧同样超过默认 SHM 段，
相机共存时初测 ROS 雷达仅 8.82 Hz，因此也为点云桥接配置大缓冲。

仿真 robot_state_publisher 的 `publish_frequency` 设为 100 Hz，匹配现有 joint state
broadcaster 的 100 Hz，缩短 30 Hz 腕部相机查询图像时刻 TF 的等待，尤其在关节移动时。

缓冲容量选择依据 [Fast DDS 2.14 SHM 文档](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/transport/shared_memory/shared_memory.html)
与 [2.14 环境变量](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/env_vars/env_vars.html)。

## CAD 外观与光学坐标

canonical 模型现在为 26 个 link、25 个 joint；MID-360 阶段为 24/23，阶段一为 23/22。
已有支架保持 `camera_link`，原名为 `camera_optical_frame` 的有质量 CAD 壳体改用
`camera_lens_link`，继续引用原 STL，网格、惯量和安装位姿保持原装配位置。
新增两个无质量固定 frame：

```text
gripper_base_link
└── camera_link          腕部支架
    └── camera_lens_link CAD 壳体
        └── d405_sensor_frame     Gazebo X 前、Y 左、Z 上
            └── camera_optical_frame  ROS X 右、Y 下、Z 前
```

现有 CAD 壳体已经包含约 2.175° 倾斜，光学朝向按其前玻璃法线确定，避免把倾角
再次施加到网格上。虚拟 pinhole 放在前玻璃中心向内 3.7 mm，作为对齐 RGB-D 的
等效中央投影光心。相对 `camera_lens_link`：

```text
xyz = (0, -0.000143076909305455, 0.00376728403468913) m
RPY = (π, 1.53283577574196, 1.57079632679490) rad
```

`d405_sensor_frame -> camera_optical_frame` 固定旋转为
`RPY=(-π/2,0,-π/2)`，两者光心重合。仿真 sensor pose 为扫描 frame 的零位，
无质量 frame 正常参与 SDF 固定关节合并，转换后的相机位姿仍位于腕部。
相对底盘的 TF 随 CR5 关节变化，因此验收查询每组图像 stamp 的动态 TF。

2026-10-06 根据实际显示修正图像倒置：此前光轴和光心正确，但传感器绕光轴
的朝向相差 180°。机械臂零位时，旧 Gazebo +Z（图像上）几乎沿底盘 −Z，
导致彩色和深度都上下颠倒。将 `camera_lens_link -> d405_sensor_frame` 的
roll 从 0 改为 π，等价于绕 Gazebo 局部 X / optical 局部 Z 旋转 180°；
光心、光轴、CAD 装配与标准 optical 固定旋转保持原值，RGB、深度和 TF 同步修正。
当前零位姿的图像上方朝向底盘 +Z，图像右方朝向底盘 +X。

此前场景靶标由完整 optical TF 定向，错误的相机 roll 也会被用于摆放靶标，
因此反投影通过仍无法发现世界上下颠倒。现在几何检查额外锁定夹爪安装基准：
Gazebo +Z 接近 `gripper_base_link` +Y，optical +X 接近夹爪 −Z，
optical +Y 接近夹爪 −Y；零位姿再检查图像上方相对底盘 +Z 的方向。
靶标布置使用相机前向和世界 +Z 构造独立朝上基准，避免复用 optical 的横向轴。

STL 未给出左右 imager 镜头中心，中央 pinhole 不代表厂商标定的左眼光心。
厂家机械图给出左眼相对中线约 9 mm、双眼基线约 18 mm、后 M3 孔间距 20 mm；
后续可按真实设备替换左眼偏移并进行手眼标定。当前不据 CAD 几何宣称实机外参精度。

## 构建、运行与显示

使用项目已有 Jazzy/Harmonic、`ros_gz_bridge`、`ros_gz_image`、NumPy 和 TF2，不安装 RealSense
厂商插件或移植其双 RGB 软件功能；没有新增第三方底层依赖。构建顺序仍为
`/opt/ros/jazzy -> ros2_ws -> sim_ws`。

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ../sim_ws
colcon build --symlink-install
source install/setup.bash
cd ..
ros2 launch agri_sim_bringup simulation.launch.py rviz:=true
```

默认 `use_camera:=true`，`camera_config` 可指定 YAML 路径。
`use_camera:=false` 关闭 RGB-D sensor 与三路桥接。只加载模型使用：

```bash
ros2 launch agri_sim_bringup simulation.launch.py \
  use_control:=false use_lidar:=false use_camera:=false
```

可选 `d405_extended_sim.yaml` 将深度远裁剪扩展到 2 m，分辨率、投影和三路接口相同；
默认 `d405.yaml` 仍为 0.5 m。启动时 RViz 深度显示的 Min/Max 随所选 profile 同步。
这是供后续 YOLO 检测、远距离粗定位流程使用的理想仿真配置；YOLO 尚未实现。
后续感知节点的深度过滤上下限需参数化，扩展 profile 的粗定位允许使用至 2 m，
接近目标后在默认 0.10–0.50 m 工作区间重新估计抓取位姿。
官方标注 7–50 cm 为 [Ideal Range](https://www.realsenseai.com/products/stereo-depth-camera-d405/)，
该指标不表示超过 50 cm 完全没有深度；本仿真也不承诺实机 D405 在 2 m 的精度或有效率。

```bash
ros2 launch agri_sim_bringup simulation.launch.py rviz:=true \
  camera_config:="$(ros2 pkg prefix --share agri_sim_sensors)/config/d405_extended_sim.yaml"
```

在另一个已加载环境的终端验收扩展量程：

```bash
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120 --max-depth 2
```

报告 `nominal_d405_algorithm_working_range_m` 保留 0.10–0.50 m 的近距基线，
`extended_simulation_range` 标记扩展配置。2 m profile 与校正后的公共 CameraInfo
已通过实际运行验收，证据在 `artifacts/d405_extended_20261006/`：

| 本次检查 | 实际结果 |
| --- | --- |
| 连续接口 | RGB/深度各 1825 帧、60.192 仿真秒；内参 1820 帧、60.027 秒；均 30.303 Hz |
| 同 stamp / TF | 联合覆盖 99.671%，0 组 TF 逾时，最大等待 5.189 ms，深度范围违规 0 |
| 场景方向与配准 | 3 组同 stamp、世界上下/左右通过；红/绿球 P95 0.139/0.138 mm |
| 1.6 m 紫球 | 871 个内部像素全部有效，Z=1.536146–1.590560 m；球面 P95 0.354 mm，边界 1 px 匹配 100% |
| 裁剪 | 4 cm 近靶和 2.2 m 远靶内部深度均无效 100% |
| 构建 / 自动检查 | 4 包构建通过；传感器 109、验收器 118、bringup 1，共 228 项测试通过；相关 10 文件 Ruff 通过 |

原始通过报告为 [interface_60s.json](../artifacts/d405_extended_20261006/interface_60s.json)
和 [scene_report.json](../artifacts/d405_extended_20261006/scene_report.json)。初测场景的
远球边界匹配 76.22% 被判失败；采集中同时布置靶标及运行其他检查时，接口联合覆盖
98.35% 也判失败。`initial_scene_report.json`、`initial_interface_60s.json` 和
`sampling_probe.json` 均保留；校正后在稳定场景复测通过，未放宽判据。
本次以 `use_lidar:=false` 验证相机；后续 0.5 m profile 和 GPU/RGL 共存指标为历史证据，
没有将其表述为新版本共存复测。

`rviz:=true` 在相机启用时选择 `agri_sim_sensors/rviz/sensors.rviz`，显示雷达、RGB、
深度和 TF；相机关闭时沿用 MID-360 配置。server 使用 `gui:=false`，仍需要 Ogre2/EGL
渲染设备。暂停仿真会停止推进图像 stamp，验收需要先恢复仿真。

## 接口验收与场景验证

从另一个已 source 两个工作空间的终端执行：

```bash
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120
```

`duration` 为各流首尾 stamp 的仿真秒跨度，`timeout` 为墙钟上限；验收器已强制
`use_sim_time`，不需额外 ROS 参数。JSON 报告以退出码 0/1 表示通过/失败，并检查：

- 三路 frame、848×480、编码、非零严格递增 stamp，仿真频率 27–33 Hz；wall Hz 另作诊断。
- CameraInfo 的 K/P/R/D、正焦距、主点、矩阵和畸变模型一致性。
- 三路 stamp 缓存单次消费配组，整组跨度不超过 33 ms，报告丢弃和未配对记录。
- 按彩色图 stamp 查询 `base_footprint -> camera_optical_frame`，非阻塞重试，
  墙钟 100 ms 内可用；同步且及时 TF 的联合覆盖率至少 99%。
- 深度 NumPy 解码支持行 padding、大小端及两种编码，有限正值在配置的裁剪范围内
  （默认 0.07–0.50 m，2 m profile 验收使用 `--max-depth 2`），
  1 µm 容差容纳 float32 舍入；全无回波允许，负值或越界有限深度判失败。

有靶标的场景可加 `--require-valid-depth`，要求整个采集中至少一个有效深度像素；
没有近处物体时，全无效图像并不表示传感器故障。可用 `--color-topic`、`--depth-topic`、
`--info-topic`、`--frame`、`--width`、`--height` 等参数验收其他配置。
超时、采集不足或仍有 TF 等待项均不能返回通过。

独立测试世界为 `d405_validation.sdf`。`setup_d405_validation_scene.py` 只在验收
世界中读取 Gazebo 模型真值和当前腕部 TF，以相机前向和世界 +Z 定向，放置
visual-only 红/绿球及近黄/远蓝靶；
`check_d405_scene.py` 用彩色位置、深度和 CameraInfo 反投影检查几何、坐标轴、对齐
与近远裁剪。真值只用于测试，不向感知算法发布。从 `Agricultural_Bot` 根目录运行：

```bash
# 终端一：独立验收世界，默认 GPU LiDAR；RGL 可添加 lidar_mode:=rgl
ros2 launch agri_sim_bringup simulation.launch.py gui:=false world:=d405_validation.sdf
```

```bash
# 终端二：等待控制器启动后，在当前腕部光轴前放置靶标
python3 scripts/setup_d405_validation_scene.py
python3 scripts/check_d405_scene.py --require-upright
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120 \
  --require-valid-depth --sync-ms 0
```

两终端均按上方顺序 source 工作空间。`--sync-ms 0` 对原生三路实施同 stamp 的严格
检查；一般接口要求仍为 ≤33 ms。靶标脚本默认只接受 `d405_validation` 世界，替换
同名测试靶标，输出到 `artifacts/d405_scene_validation`；不会修改默认温室世界。
`--output-dir` 可指定准备脚本输出目录，场景检查器用 `--scene <目录>/scene.json`。
PNG 预览在已安装 matplotlib 时生成；纯 JSON 验收不依赖 matplotlib。

测试 2 m profile 时，启动独立世界也传入上述 `camera_config`，随后执行
`python3 scripts/setup_d405_validation_scene.py --max-depth 2` 与
`python3 scripts/check_d405_scene.py --require-upright`。准备脚本按量程设置有效目标
与越界靶标；检查器读取 `scene.json` 中的量程，不将默认 70 cm 越界靶用于 2 m 裁剪验收。

`--require-upright` 在机械臂零位姿下检验独立世界上下/左右基准：世界上方的红球
应出现在图像上方，世界下方的绿球应出现在图像下方，并检查左右顺序。
先保持机械臂零位完成此项，再执行其他腕部动作；腕部合法滚转后仍可不加该参数
运行反投影、对齐和裁剪验收，图像不要求始终相对世界重力朝上。

本次倒置修正已经运行验证，证据在 `artifacts/d405_upright_20261006/`；采用校正前
的公开主点 `(424,240)`，保留为历史证据：

| 倒置修正回归 | 实际结果 |
| --- | --- |
| 世界上下/左右 | 3 组同 stamp 帧通过；图像上方向与世界 +Z 点积 0.999820；红靶质心 (341.0,203.2)，绿靶 (502.0,283.0)，主点 (424,240) |
| 深度/配准 | 红绿球 P95 0.461/0.506 mm，边界 1 px 匹配 100%，近远靶内部深度无效 100% |
| 接口回归（RGL 共存） | 60 秒，RGB/深度/内参 1819/1819/1817 帧，约 30.253 Hz，同 stamp + 及时 TF 99.73%，1 组 TF 逾时计为失败样本 |
| 静态几何 | canonical、standalone、SDF 朝上及光轴检查通过；旧 roll=0 反例被拒绝，MID-360 几何回归通过 |
| 自动测试 | 新增 12 项方向回归通过；验收包共 83 项通过；相关脚本 Ruff 通过 |

`scene_report.json`、`color.png`、`depth.png`、`interface_60s.json`、`geometry.log`
包含本次实际结果。10 秒快速检查曾因 CameraInfo 发现比图像晚约 3 帧，联合覆盖
98.70% 判失败；该结果保留为 `interface_10s.json`，正式 60 秒窗口按原 ≥99%
阈值通过，未修改判据。修正后需重新启动 Gazebo/RViz，以重新加载 robot_description。

下表和后续历史指标记录首次接入验收，当时未包含独立世界朝上检查；本次方向回归
以以上新增结果为准。

| 检查 | 2026-10-06 状态 |
| --- | --- |
| 相机配置 / overlay | 新增 48 项测试通过；含投影、FOV、clip、桥接与 DDS 配置选择 |
| 相机验收器边界 | 新增 43 项通过；验收包含已有 MID-360 共 71 项通过 |
| 模型 / SDF | canonical 与 standalone 光心、前玻璃法线、光学轴、固定关节合并检查通过 |
| MID-360 回归 | 既有几何回归通过；此前 GPU/RGL 接口和场景证据保留 |
| RGB-D + RGL 连续运行 | 60 秒通过；RGB/深度/内参 1823/1822/1820 帧，各 30.303 Hz，同 stamp + 及时 TF 99.78%，最大等待 59.4 ms |
| RGB-D + GPU 连续运行 | 60 秒通过且包含腕部运动；三路各 1820 帧，约 30.303 Hz，同 stamp + 及时 TF 99.84%，2 组 TF 逾时，最大等待 101.2 ms |
| MID-360 共存回归 | GPU/RGL 均 601 帧 / 60 秒 / 10 Hz，量程与局部 FOV 违规 0 |
| RGB-D 场景几何 | 3 组同 stamp 图像通过；红/绿球 P95 0.462/0.508 mm，边界 1 px 内匹配 100% |
| 近远裁剪 | 4 cm 黄靶、70 cm 蓝靶均彩色可见、内部深度无效 100% |

本次 JSON 和图像证据保存在 `artifacts/d405_20261006/`（构建/验收产物不入 Git）：
`rgl_d405_60s.json`、`rgl_mid360_60s.json`、`gpu_d405_60s.json`、`gpu_mid360_60s.json`、
`scene_report.json`、`scene.json`、`color.png`、`depth.png`；腕部运动后的几何结果在
`gpu_scene/`。连续验收深度范围违规 0，所有配组 stamp 差为 0。
GPU 的 2 个 TF 逾时组计为失败样本，联合比例仍高于要求的 99%，没有放宽 100 ms
阈值。真实墙钟相机约 28.93 Hz（GPU）/29.97 Hz（RGL），仿真更新约 30.303 Hz；
相机按 1 ms 步长离散成约 33 ms 更新周期。

腕部动作通过隔离仿真的 FollowJointTrajectory action 执行：`cr5_joint6` 从 0 到
0.15 rad，2 秒，结果 SUCCEEDED。非零姿态三组反投影仍通过，光心和光轴随关节移动。
初测失败报告保留为 `gpu_initial_d405_60s.json`、`gpu_initial_mid360_60s.json`，用于
记录 DDS 与 TF 修复过程。
共用缓冲与 100 Hz TF 调整后，RGL 快速回归再次通过：相机 10 秒、305 组、联合
覆盖 100%、最大 TF 等待 39.4 ms；雷达 5 秒/51 帧/10 Hz。对应
`rgl_final_10s.json` 与 `rgl_final_mid360_5s.json`。

上述球面误差属于理想渲染和栅格离散误差，不代表 D405 实机精度。完整温室番茄资产、
SLAM/Nav2、MoveIt、YOLOv8 和采摘任务仍待开发，完整 M3 与采摘闭环尚未完成。

## 规格与源码依据

厂家 [D405 产品页](https://www.realsenseai.com/products/stereo-depth-camera-d405/) 提供
7–50 cm 理想工作范围、87°×58° 视场及彩色使用左深度 imager 的说明；
[D400 系列 October 2025 数据手册](https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf)
第 89–93 页用于机械安装和深度参考面说明，不作为现有 CAD 已完成实机标定的依据。

原生行为在本机 gz-sensors 8.2.2 源码核实：
[RGB-D 同次更新](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/RgbdCameraSensor.cc#L513)、
[独立深度裁剪](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/RgbdCameraSensor.cc#L531)、
[真实内参投影](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/CameraSensor.cc#L840)、
[CameraInfo optical frame](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/CameraSensor.cc#L773)。
光学轴和 Z 深度约定分别依据
[REP-103](https://www.ros.org/reps/rep-0103.html) 与
[REP-118](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst#L26)。
原生点云坐标语义参考
[Ogre2 点坐标生成](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8_8.2.3/ogre2/src/media/materials/programs/GLSL/depth_camera_fs.glsl#L109) 与
[PointCloudUtil 写入](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/PointCloudUtil.cc#L168)。
本项目未复制这些外部源码或插件。
