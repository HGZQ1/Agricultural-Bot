# agri_perception

这个包把训练好的 Ultralytics YOLO 番茄分割模型接入 ROS 2，并使用仿真的
Intel RealSense D405 RGB-D 相机进行检测。完整中文流程见
[`docs/yolo_tomato_integration.md`](../../../docs/yolo_tomato_integration.md)。

节点启动时只加载一次模型，读取 `/d405/color/image_raw`，并发布：

- `/agri_vision/annotated_image`：带检测框和分割掩膜的图像；
- `/agri_vision/detections`：`vision_msgs/Detection2DArray`，包含类别、置信度和像素框；
- `/agri_vision/tomato_positions_camera`：使用对齐深度和 CameraInfo 反投影得到的
  `camera_optical_frame` 三维位置；
- `/agri_vision/tomato_positions_base`：通过 TF 转换到 `base_link` 的三维位置。

新增机械臂接入前使用的结构化输出：

- `/agri_vision/tomato_targets_camera`：`agri_harvest_interfaces/TomatoTargetArray`；
- `/agri_vision/tomato_targets_base`：同类型，经采集时刻 TF 转到 `target_frame`。

每个三维目标同时携带会话内跟踪 `target_id`、类别、置信度、原始 `detection_index`、
位置、成熟标志、有效深度样本数/比例/MAD、是否使用分割、连续稳定帧数。
数组携带 RGB 时间戳、深度时间戳、坐标系、`observation_valid` 和 `rejection_reason`。
自动化只应使用有效且 `mature=true`、`temporally_stable=true` 的新鲜目标。
绿色果实仍在标注图和二维检测中显示；默认三维输出只保留 `tomato_R`。

## 模型文件

模型权重按项目结构放在 `data/weights/`，但不放进 Git 仓库。当前本机权重文件是：

```text
/home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/data/weights/best.pt
```

推荐用环境变量保存本机路径：

```bash
export AGRI_TOMATO_MODEL=/home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/data/weights/best.pt
```

也可以只在本次启动时传入 `model_path:=...`。如果两者都没有设置，节点会提示
模型路径为空并退出。

## 编译

从项目目录执行：

```bash
cd /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot
source /opt/ros/jazzy/setup.bash
colcon --log-base ros2_ws/log build --symlink-install --base-paths ros2_ws/src \
  --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-up-to agri_perception
source ros2_ws/install/setup.bash
```

## 启动

先启动番茄田 Gazebo 仿真，再在另一个终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/ros2_ws/install/setup.bash
source /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/sim_ws/install/setup.bash

ros2 launch agri_perception tomato_detector.launch.py \
  device:=0
```

未设置环境变量时，直接使用绝对路径：

```bash
ros2 launch agri_perception tomato_detector.launch.py \
  model_path:=/home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/data/weights/best.pt \
  device:=0
```

`device:=0` 使用 NVIDIA GPU；排查 GPU 问题时可改成 `device:=cpu`。

## 验证

```bash
ros2 topic hz /agri_vision/annotated_image
ros2 topic echo /agri_vision/detections --once
ros2 topic echo /agri_vision/tomato_positions_camera --once
ros2 topic echo /agri_vision/tomato_positions_base --once
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

```bash
ros2 topic echo /agri_vision/tomato_targets_camera --once
ros2 topic echo /agri_vision/tomato_targets_base --once
```

位置单位为米。旧 `PoseArray` 作为位置显示兼容输出保留，深度被拒绝的检测会被跳过，
不能用其下标与二维数组配对；其单位四元数只是占位值。结构化目标中的
`detection_index` 对应同一 RGB 帧二维检测下标，跟踪 ID 在检测顺序变化中匹配，丢失后重建，
不是跨行、跨停车位或重启后的植株/果实身份。

## 深度与时间门槛

| 参数 | 默认 | 作用 |
|---|---:|---|
| `mature_only` / `mature_class` | `true` / `tomato_R` | 三维成熟红果筛选 |
| `depth_sample_count` | 81 | 中心邻域无放回随机采样最大像素数 |
| `depth_sample_radius_px` / `depth_bbox_radius_fraction` | 8 / 0.25 | 邻域半径取像素半径与框宽高比例的较小值 |
| `depth_sampling_seed` | 42 | 可复现逐帧随机采样种子 |
| `depth_min_m` / `depth_max_m` | 0.07 / 2.0 | 有效深度范围，16UC1 毫米转米，32FC1 直接米 |
| `depth_min_valid_samples` / `depth_min_valid_fraction` | 15 / 0.6 | 最少有效样本数及有效比例 |
| `depth_max_mad_m` | 0.03 | 中值绝对偏差上限，过滤深度离散的邻域 |
| `require_segmentation` / `segmentation_erode_px` | false / 1 | 有掩膜时限制到内缩掩膜；无掩膜默认明确回退到框中心邻域 |
| `max_rgb_depth_skew_s` | 0.05 | RGB 与对齐深度时间差上限 |
| `max_observation_age_s` | 0.5 | 推理完成时 ROS 时间下的采集年龄上限 |
| `stream_timeout_wall_s` | 1.0 | 输入中断或仿真时钟暂停时清空旧目标 |
| `stability_min_frames` / `stability_window_s` | 3 / 0.5 | 稳定判断的最少帧数和历史窗口 |
| `stability_min_span_s` / `stability_max_spread_m` | 0.1 / 0.025 | 稳定帧时间跨度下限和位置波动上限 |
| `track_match_distance_m` | 0.08 | 同类、重叠检测框下的短期三维匹配距离 |

深度取随机有效样本的中值，然后使用检测框中心像素反投影。该位置是可见表面深度
估计，不能直接解释为果实几何中心、抓取姿态或机械臂可达证明。使用掩膜也不等于
完成果实分离、遮挡处理或表面法向估计；目前没有独立掩膜消息。

RGB/深度采用有界队列按时间戳配对，每帧只用一次；标定尺寸和 optical frame 必须匹配。
TF 使用 RGB 采集时刻及节点 ROS 时钟，禁止用最新变换替代历史变换。缺标定、无效深度、
过期、时间回跳和变换失败均清空受影响三维数组；有效但无红果的帧输出有效空数组。
观察失效会重置稳定判断。机械臂接入可配置 `target_frame:=base_footprint` 或经过标定的
机械臂规划坐标系，勿直接把 CAD 局部轴 `base_link` 的坐标当作标准车体轴。

推荐作业预检启动：

```bash
ros2 launch agri_perception tomato_detector.launch.py \
  model_path:="$PWD/data/weights/best.pt" device:=0 \
  target_frame:=base_footprint mature_only:=true require_segmentation:=true
```

CPU 推理若超过 0.5 s，会明确拒绝陈旧目标；仅调试观察时可调整最大年龄，作业门槛需要
同步配置，并验证实际延迟。节点不会驱动车辆或机械臂。
