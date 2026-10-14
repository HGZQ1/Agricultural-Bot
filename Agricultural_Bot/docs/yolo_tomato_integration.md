# YOLO 番茄检测接入说明

本文说明如何在 Agricultural_Bot 的 ROS 2 仿真中启动已经接入的 YOLO 番茄
分割节点。

可靠三维目标、成熟筛选、随机深度采样和停车作业握手已加入，完整构建与测试流程见
[机械臂接入前置检查](harvest_preflight.md)。本页的旧 `PoseArray` 输出仅供兼容；
采摘业务应使用携带类别、置信度、目标 ID 和深度质量的 `TomatoTargetArray`。

## 1. 模型文件

本项目使用 Ultralytics YOLO 分割模型。按照《需求文档.md》的目录约定，训练好的
权重放在 `data/weights/`：

```text
/home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/data/weights/best.pt
```

该模型包含两个类别：

```text
0: tomato_R
1: tomato_G
```

权重文件不提交到 Git 仓库。`.gitignore` 已排除 `*.pt`，因此仓库不会因为模型
文件过大而无法正常克隆。其他电脑应先取得权重并放到 `data/weights/`，或者在
启动时传入其他绝对路径。

推荐设置环境变量：

```bash
export AGRI_TOMATO_MODEL=/home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/data/weights/best.pt
```

每次重新打开终端后都需要重新执行，或者把这行加入个人 shell 配置文件。

## 2. 编译视觉包

在项目根目录执行：

```bash
cd /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot
source /opt/ros/jazzy/setup.bash

colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --packages-select agri_harvest_interfaces agri_perception

source ros2_ws/install/setup.bash
```

如果修改了 Python 节点或 launch 文件，重新编译后再启动检测节点。

## 3. 启动顺序

终端 1：先启动番茄田 Gazebo 仿真。保持该终端运行，不要在启动 YOLO 时关闭
Gazebo。项目当前的相机输入为：

```text
/d405/color/image_raw
/d405/aligned_depth_to_color/image_raw
/d405/color/camera_info
```

终端 2：启动 YOLO 节点：

```bash
cd /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash

ros2 launch agri_perception tomato_detector.launch.py \
  device:=0 mature_only:=true
```

如果没有设置环境变量，直接传入绝对路径：

```bash
ros2 launch agri_perception tomato_detector.launch.py \
  model_path:=/home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/data/weights/best.pt \
  device:=0
```

看到下面两类日志，说明模型已加载并开始处理图像：

```text
Loading YOLO model: .../best.pt
Processed frame 30: ... detections
```

`device:=0` 使用 NVIDIA GPU；如果只想测试 CPU，改成 `device:=cpu`。

## 4. 验证输出

终端 3 加载 ROS 环境后检查：

```bash
source /opt/ros/jazzy/setup.bash
source /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/ros2_ws/install/setup.bash

ros2 topic list | grep agri_vision
ros2 topic hz /agri_vision/annotated_image
ros2 topic echo /agri_vision/detections --once
ros2 topic echo /agri_vision/tomato_positions_camera --once
ros2 topic echo /agri_vision/tomato_positions_base --once
```

应当看到以下话题：

```text
/agri_vision/annotated_image
/agri_vision/detections
/agri_vision/tomato_positions_camera
/agri_vision/tomato_positions_base
```

其中：

- `annotated_image` 是带检测框和分割掩膜的图像；
- `detections` 包含类别、置信度和像素框；
- `tomato_positions_camera` 是相机坐标系下的三维位置，单位为米；
- `tomato_positions_base` 是通过 TF 转换到 `base_link` 后的位置，单位为米。

另有两个可靠三维观察话题：

```text
/agri_vision/tomato_targets_camera
/agri_vision/tomato_targets_base
```

类型为 `agri_harvest_interfaces/msg/TomatoTargetArray`。每个目标包含 `target_id`、
`class_name`、`confidence`、同帧二维 `detection_index`、三维 `position`、成熟标记、
有效深度样本数/比例/MAD、是否使用分割和多帧稳定标记。数组携带 RGB/深度时间戳、
点的坐标系及观察有效性。默认三维输出仅含 `tomato_R`，二维检测仍包含红绿两类。
若按前置检查命令设置 `target_frame:=base_footprint`，三维 base 话题的 header 即为
`base_footprint`，不是固定为 `base_link`。

```bash
ros2 topic echo /agri_vision/tomato_targets_base --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

如果模型有检测但 `tomato_positions_base` 没有消息，先等待几秒让 TF 缓冲区建立，
再确认上述 TF 命令能持续输出变换。

## 5. 当前边界和下一步

当前已经完成图像检测、分割、成熟三维目标筛选、随机深度采样中值估计、质量门、多帧
稳定标记和采集时刻 TF 转换。无效深度会跳过目标，旧 `PoseArray` 不能按下标与完整
二维检测数组配对；新消息已将类别和位置放在同一条目标记录中。

中心邻域最多随机采样 81 个像素，有掩膜时只采其内缩区域；有效比例至少 60%、
有效样本至少 15 个，取中值，MAD 大于 3 cm 时拒绝。RGB/深度最大时间差为 0.05 s，
观察最大年龄为 0.5 s；输入过时、TF 不可用和输入失联会清空可靠目标。
Point 是可见表面深度估计，尚未生成果实中心或抓取方向；旧四元数为兼容占位。
独立实例掩膜发布、TCP/抓取姿态与规划坐标系方案见前置检查文档第 5 节。

检测节点本身不发送底盘或机械臂命令。显式启动前置作业流程后，协调器可以把到站后
的新鲜成熟稳定目标交给模拟 worker；真实机械臂需要实现同一 Action 及新鲜收纳反馈。

## 当前演示果实与机械臂交接

当前默认田间世界保留 224 个自然原位果实，已撤销外伸果实及绿色支架方案。
[近侧果实演示分布](harvest_demo_fruit_profile.md) 保留为历史记录。本轮另用
50–60 mm 小果实候选进行实际复测，位置、编号和成熟度保持；场景、碰撞导航图及
20 点任务需要整套加载，见 [前置检查第 10.7 节](harvest_preflight.md#107-保留原位置的小果实复测)。
自然分布不保证每株都有红果或可达目标，尺寸缩小也不等于 IK、夹取或落篮通过。
仍须验证 TCP、夹爪开度和相机外参，并实测左右各一株的
检测 → 深度定位 → 预抓取 → 夹取 → 回撤 → 落篮。场景 JSON 的 world 标注只供离线
核对；在线控制仍使用视觉测量，不直接读取果实真值代替检测。
