# YOLO 番茄检测接入说明

本文说明如何在 Agricultural_Bot 的 ROS 2 仿真中启动已经接入的 YOLO 番茄
分割节点。

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
  --packages-select agri_perception

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
  device:=0
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

检查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

如果模型有检测但 `tomato_positions_base` 没有消息，先等待几秒让 TF 缓冲区建立，
再确认上述 TF 命令能持续输出变换。

## 5. 当前边界和下一步

当前已经完成图像检测、分割、深度反投影和 TF 坐标转换，但不会自动控制底盘或
机械臂。`PoseArray` 的位置顺序与同一帧 YOLO 检测框顺序一致，后续应将
`tomato_R`/`tomato_G` 类别和三维位置放入同一个目标消息，并增加时间滤波、目标
选择和安全距离检查，之后再接入采摘动作。
