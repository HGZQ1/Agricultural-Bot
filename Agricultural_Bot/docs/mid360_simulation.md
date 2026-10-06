# MID-360 仿真、安装与验收

更新时间：2026-10-06。MID-360 已接入整机模型，默认采用 Gazebo GPU LiDAR；RGL
作为可选后端使用固定的官方 MID-360 扫描图样。公开接口已按需求文档 6.3 节约束。
GPU 后端的连续接口和场景检查已通过；RGL 本地编译、CUDA/OptiX 探针及 FOV
规范化后的公开点云连续 60 仿真秒接口、场景检查均已通过。
本次实测环境为 ROS 2 Jazzy、Gazebo Harmonic、NVIDIA RTX 4060 Laptop GPU，驱动
580.178.04；以下验收结果对应这台机器。

## 工作空间与接口

`ros2_ws/src/agri_robot_description` 保存机器人本体、`mid360_link` 和扫描光学 frame。
`sim_ws/src/agri_sim_sensors` 保存传感器 overlay、双后端参数、桥接和 RGL 规范化节点。
`agri_sim_description` 负责启动整机、世界和传感器，`agri_sim_tests` 负责只读接口验收。
算法以后只接收公开 `/mid360/points`，不依赖 Gazebo entity ID、真值或 RGL 内部 Topic。

| 契约 | 当前基线 |
| --- | --- |
| ROS Topic / 类型 | `/mid360/points` / `sensor_msgs/msg/PointCloud2` |
| frame ID | `mid360_sensor_frame` |
| 字段 | 至少 `x/y/z/intensity`；GPU 后端额外存在 `ring` |
| 更新时间 | 10 Hz，消息 stamp 使用仿真时间 |
| 视场 | 传感器局部坐标中水平 360°，垂直 −7° 至 +52° |
| 有效距离 | 传感器局部 XYZ 范数 0.1–40 m |
| 桥接 / QoS | 单向 `GZ_TO_ROS`，SensorDataQoS：Best Effort、Volatile、Keep Last 5 |

不承诺逐点时间、Livox 自定义消息、逐点运动畸变或物理厂商标定精度。
GPU 模式是规则栅格功能等效，不复刻 Livox 非重复扫描。

GPU 链路为 Gazebo `/mid360/points` → 桥接 → ROS `/mid360/points`。
RGL 链路为 Gazebo `/mid360/points` → 桥接到 ROS `/mid360/rgl_raw` → Python
`normalize_mid360_cloud` → ROS `/mid360/points`。规范化在传感器局部坐标中过滤
无效值、量程和垂直视场，保留 frame、stamp 和字段结构。

## 光心与安装基准

`mid360_link` 的原点是现有 CAD 质量属性参考点；visual/collision 网格通过反向偏移
保持原装配外观位置。独立 `mid360_sensor_frame` 放在现有 STL 光学穹顶拟合球心，
相对 `base_footprint`：

```text
xyz = (-0.401439121228, 0.003437758801, 0.459044570728) m
RPY = (0, -15°, 0)
quaternion xyzw ≈ (0, -0.130526192220, 0, 0.991444861374)
```

这是现有 CAD 的仿真几何基准，后续应以实测安装外参替换。传感器 sensor pose 为
该扫描 frame 的零位，两种后端一致；修改点云 header 名称不会转换点坐标。

仿真 overlay 保留有质量的 `mid360_link` 固定关节，质量为空的 optical frame 正常
参与固定关节合并，由 SDF 保留传感器位姿。RGL 的父 link 排除只忽略雷达外壳，
仍保留底盘、机械臂和篮筐遮挡；避免把整个机器人合并到被忽略的传感器父 link。

## 后端与固定版本

| 选项 | 行为 |
| --- | --- |
| `lidar_mode:=gpu_lidar` | 默认；Ogre2 GPU LiDAR，1,875×32 栅格、10 Hz、0.005 m 距离噪声标准差 |
| `lidar_mode:=rgl` | 固定 RGL 插件与官方命名图样，缺失依赖时明确报错 |
| `lidar_mode:=auto` | 预检 NVIDIA 设备、插件依赖和图样；不满足条件时回退 GPU LiDAR |

默认配置位于 `sim_ws/src/agri_sim_sensors/config/mid360.yaml`。启动前检查数值、量程、
角度顺序、样本数、canonical frame 和 Ogre2 引擎；`points_topic` 必须对应
`gz_topic + /points`。`use_lidar:=false` 关闭传感器、桥接及 RGL 规范化节点。

`horizontal_samples`、`vertical_samples`、`range_resolution`、`noise_stddev` 只作用
于 GPU 栅格后端。RGL 使用官方原版图样，目前未接入 RGL 距离噪声节点，场景回波为
理想几何结果；两种后端的噪声模型和原始点数不同。

双后端组织方式参考 [rm_sim_26](https://github.com/Neomelt/rm_sim_26)，参考提交
`fe5979a82684c261ee8a28785cbb410903d136b1`。未复制其机器人网格或源文件。
具体声明见 [agri_sim_sensors/NOTICE](../sim_ws/src/agri_sim_sensors/NOTICE)。

| 依赖 | 固定版本 / 作用 |
| --- | --- |
| [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) | `d4bf3cf36fe4a363a56df1bec2ce3809720db563`，包版本 0.2.0；Gazebo Harmonic 传感器插件 |
| [RobotecGPULidar](https://github.com/RobotecAI/RobotecGPULidar/releases/tag/v0.21.0) | 核心 v0.21.0；CUDA/OptiX GPU 射线追踪；匹配头文件提交 `a65f07f9565adbfe5cda33a58732638786cb34e8` |
| `LivoxMid360.mat3x4f` | 上游原版图样，Git blob `3d8faa6a44b8eaac5d56ffb320e3b10546705576` |

官方命名预设 `<pattern_preset>Livox Mid360</pattern_preset>` 把 800,000 条射线分成
40 段，每帧轮换使用 20,000 条。使用 `pattern_preset_path` 直接加载完整文件会造成
每帧重复全部射线，不采用该方式。

上游图样原始射线俯仰范围实测约 −8.22° 至 +54.36°，约 1.32% 射线超出本项目
标称 −7° 至 +52°。图样文件保持原版，公开点云由仿真侧规范化节点裁剪到项目契约；
原始图样的角度范围不作为公开接口的合规依据。

我方自研节点使用 Python 3.12、`rclpy`、`ament_python`。RGL 为外部 C++ 插件，
脚本编译它并链接固定版本的预编译核心库；我方未修改上游 C++。安装脚本保留插件和
核心库许可证，下载输入均校验固定 SHA-256。

## 安装与构建

在 `Agricultural_Bot` 根目录执行。ROS 2 Jazzy、Gazebo Harmonic 和 NVIDIA 驱动
沿用项目环境。若缺少构建/传感器 Python 依赖，可按包声明补齐：

```bash
sudo apt install build-essential cmake python3-numpy python3-yaml \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-sensor-msgs-py
```

安装 RGL 到项目内缓存：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/setup_mid360_rgl.py
python3 scripts/setup_mid360_rgl.py --verify-only
```

产物默认位于 `.cache/rgl/install`，图样位于
`.cache/rgl/source/RGLGazeboPlugin/lidar_patterns`。manifest 记录版本、下载/库校验值、
许可目录和探针结果；configure/build/install 日志位于 `.cache/rgl/logs`。
`--jobs 2` 可限制编译并行度。自定义 `--cache-dir` 必须在本项目内，运行时传入脚本
输出的 `rgl_install_prefix` 和 `rgl_patterns_dir`。`--skip-gpu-check` 只证明产物准备
完成，不能证明 CUDA/OptiX 可运行。

按依赖顺序构建两个工作空间：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ../sim_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

## 启动与查看

下列为不同启动方式，每次选择一条，在停止旧仿真后切换后端：

```bash
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=gpu_lidar rviz:=true
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=rgl rviz:=true
ros2 launch agri_sim_bringup simulation.launch.py lidar_mode:=auto
```

server 模式默认附加 `--headless-rendering`，仍使用 Ogre2/EGL：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false
```

AMD/NVIDIA 混合显卡机器可对单次启动选择 NVIDIA 渲染设备：

```bash
env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  ros2 launch agri_sim_bringup simulation.launch.py gui:=false
```

暂停状态下传感器的仿真时间不推进，需恢复仿真后才能完成频率验收。RViz 配置为
`sim_ws/src/agri_sim_sensors/rviz/mid360.rviz`，包含模型、TF 和公开点云显示。
RGL 点云使用 `rviz:=true` 查看；当前未装载 RGLVisualize GUI 插件，不提供 Gazebo
自定义传感器射线束显示。
launch 保留已有资源路径和原 world 目录，把临时世界与桥接配置写到独立临时目录，
退出时清理，避免修改用户世界或把资源解析基准误指向临时目录。

## 验收方法与结果

接口快速检查在另一个已加载两个工作空间的终端运行：

```bash
ros2 run agri_sim_tests check_mid360 --duration 5 --timeout 90
```

`duration` 是首尾 sensor stamp 的仿真时间跨度，`timeout` 是墙钟上限。检查 frame、
xyz/intensity 浮点字段、非零严格递增 stamp、9–11 Hz、每云有限回波、量程、局部垂直 FOV 和
`base_footprint -> mid360_sensor_frame` TF。NaN/Inf XYZ 作为无回波单独计数。
输出 JSON 并以退出码 0/1 表示通过/失败，记录 sim Hz、接收 wall Hz、点数、距离、FOV 和 TF。
有效递增 stamp 的消息数单独用于频率统计，异常 stamp 仍判失败。量程容差为 1 mm，
FOV 容差为 1e-6 rad，以容纳 float32 边界舍入；报告角度越界点数。
PointCloud2 行 padding、大小端、截断数据、无回波、FOV 和时间戳边界有独立测试。

60 仿真秒接口验收示例：

```bash
mkdir -p artifacts/mid360_20261006
ros2 run agri_sim_tests check_mid360 --duration 60 --timeout 90 \
  > artifacts/mid360_20261006/gpu_60s_final.json
```

已确认的最终验收结果：

| 项目 | 2026-10-06 结果 |
| --- | --- |
| GPU 最终接口 | 60.0 仿真秒，583 帧，9.7 Hz；26,142,303 个有效点，44,841 点/帧；frame、字段、stamp、有效距离、TF 通过，FOV 越界 0 |
| 安装位姿 | TF 与 CAD 拟合光心及 −15° 俯仰一致 |
| GPU 场景几何 | 112,479 个环境回波，全部距墙/地面小于 30 mm；中位误差 2.98 mm，P95 8.80 mm |
| RGL 安装 | 插件编译、动态库依赖及 CUDA/OptiX 单射线探针通过 |
| RGL 原始链路 | 6 仿真秒、61 帧、10 Hz；37,547 个环境点全部距墙/地面小于 30 mm，P95 表面误差 0.007285 mm |
| RGL 最终公开链路 | 60.0 仿真秒，601 帧，10 Hz；8,905,061 个有效点，14,711–14,955 点/帧；FOV 越界 0 |
| RGL 最终场景 | 36,770 个环境点全部距墙/地面小于 30 mm，P95 表面误差 0.007161 mm |
| 包测试 | agri_sim_sensors 25 项、agri_sim_tests 28 项、既有底盘运动学 7 项和 bringup 1 项通过 |
| 静态检查 | 本次 Python 代码和测试通过 Ruff 0.12.10 默认规则；SDF 场景几何检查通过 |

GPU JSON 分别位于
[gpu_60s_final.json](../artifacts/mid360_20261006/gpu_60s_final.json) 和
[gpu_geometry.json](../artifacts/mid360_20261006/gpu_geometry.json)。RGL 安装证据为本地
`.cache/rgl/manifest.json`，其中 `shared_library_load`、`gpu_raytrace` 均为 true。
GPU 最终接口范围为 0.142303–4.955094 m，局部垂直角实测 −7.0000016° 至 +51.9999971°，
在 1e-6 rad 容差内；早期 `gpu_60s.json` 保留作调试记录，最终结论使用增强验收器生成的
`gpu_60s_final.json`。
最终 RGL 证据为 [rgl_60s.json](../artifacts/mid360_20261006/rgl_60s.json) 和
[rgl_geometry.json](../artifacts/mid360_20261006/rgl_geometry.json)。公开点云范围
0.163194–4.953222 m，俯仰实测 −7.0000087° 至 +52.0000078°，在 1e-6 rad 数值容差
内，没有 FOV 违规点；TF 与 CAD 基准一致。
RGL 场景误差是在未加入距离噪声的理想几何仿真中测得，不能用于宣称实机测距精度。
原始 RGL 场景记录保留为历史调试证据：
[rgl_geometry_raw.json](../artifacts/mid360_20261006/rgl_geometry_raw.json)。原始链路通过
证明姿态、射线追踪和场景转换可用，不代表未过滤的图样符合项目 FOV。

场景检查使用静止机器人和 `mid360_validation.sdf`：四墙中心为 `x/y=±3 m`、厚
0.1 m，内表面为 ±2.95 m，地面 z=0。读取 Gazebo 模型位姿只用于评测，不传给算法。
采集最近三帧，排除不足 1.5 m 的近处机器人回波，至少 1,000 个环境点中 98% 距已知
表面小于 30 mm 才通过：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false world:=mid360_validation.sdf
```

```bash
python3 scripts/check_mid360_scene.py --backend gpu_lidar \
  --output artifacts/mid360_20261006/gpu_geometry.json
```

当前完成 MID-360 部分的 FR-05/TC-SENSOR 基线，不能等同于完整 M3 验收。
D405 图像/深度/CameraInfo 已接入，后续进度见 [D405 文档](d405_simulation.md)。
完整温室番茄资产、IMU、点云转 LaserScan、SLAM/Nav2、MoveIt、视觉和采摘任务
仍需后续实现；本页保留 MID-360 阶段独立验收证据。
