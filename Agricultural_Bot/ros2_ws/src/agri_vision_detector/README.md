# agri_vision_detector

这个包把训练好的 Ultralytics YOLO 番茄分割模型接入 ROS 2，并使用仿真的
Intel RealSense D405 RGB-D 相机进行检测。完整中文流程见
[`docs/yolo_tomato_integration.md`](../../../docs/yolo_tomato_integration.md)。

节点启动时只加载一次模型，读取 `/d405/color/image_raw`，并发布：

- `/agri_vision/annotated_image`：带检测框和分割掩膜的图像；
- `/agri_vision/detections`：`vision_msgs/Detection2DArray`，包含类别、置信度和像素框；
- `/agri_vision/tomato_positions_camera`：使用对齐深度和 CameraInfo 反投影得到的
  `camera_optical_frame` 三维位置；
- `/agri_vision/tomato_positions_base`：通过 TF 转换到 `base_link` 的三维位置。

## 模型文件

模型权重不放进 Git 仓库。当前使用的权重文件是：

```text
/home/fsy/tomato_seg-6_package/weights/best.pt
```

推荐用环境变量保存本机路径：

```bash
export AGRI_TOMATO_MODEL=/home/fsy/tomato_seg-6_package/weights/best.pt
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
  --packages-select agri_vision_detector
source ros2_ws/install/setup.bash
```

## 启动

先启动番茄田 Gazebo 仿真，再在另一个终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/ros2_ws/install/setup.bash
source /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/sim_ws/install/setup.bash

ros2 launch agri_vision_detector tomato_detector.launch.py \
  device:=0
```

未设置环境变量时，直接使用绝对路径：

```bash
ros2 launch agri_vision_detector tomato_detector.launch.py \
  model_path:=/home/fsy/tomato_seg-6_package/weights/best.pt \
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

位置单位为米。当前 `PoseArray` 中的位置顺序与同一帧 YOLO 检测框顺序一致，
但位置消息本身还没有携带 `tomato_R`/`tomato_G` 类别。接入自动抓取前，还需要
增加类别与三维位置的一一对应以及时间滤波；本节点目前不会驱动车辆或机械臂。
