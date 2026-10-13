# 机械臂接入前：可靠目标输出与停车作业握手

本流程把成熟果实的视觉观察交给带身份的作业接口，并在底盘速度出口保持停车互锁。
当前作业节点为显式启动的模拟 worker，不控制关节，不证明 IK、实际夹取或果实脱离成功。
真实机械臂接入时可以沿用消息和作业 Action，替换模拟 worker。

## 1. 当前视觉输出

| 话题 | 消息类型 | 用途 |
| --- | --- | --- |
| `/agri_vision/annotated_image` | `sensor_msgs/Image` | 检测框、类别与分割掩膜叠加图 |
| `/agri_vision/detections` | `vision_msgs/Detection2DArray` | 同帧所有二维目标，含红果和绿果 |
| `/agri_vision/tomato_targets_camera` | `agri_harvest_interfaces/TomatoTargetArray` | 相机坐标下的带身份三维观察 |
| `/agri_vision/tomato_targets_base` | 同上 | 图像采集时刻 TF 转换后的三维观察 |
| `/agri_vision/tomato_positions_camera` | `geometry_msgs/PoseArray` | 兼容旧工具，仅含位置 |
| `/agri_vision/tomato_positions_base` | 同上 | 兼容旧工具，仅含位置 |

默认 `mature_only:=true`，三维输出自动筛选 `tomato_R`；二维检测和叠加图仍可显示绿果。
新业务应订阅 `tomato_targets_base`，不要用旧 PoseArray 下标拼接二维类别。
每个三维目标包含：

```text
target_id, class_name, confidence, detection_index, position(x,y,z), mature,
depth_sample_count, depth_valid_fraction, depth_mad_m, segmentation_used,
temporally_stable, stable_frame_count
```

数组包含 RGB 时间戳、点的坐标系、配对深度时间戳、`observation_valid` 和
`rejection_reason`。`detection_index` 指向同一 RGB 帧的完整二维检测数组，跳过无效深度
不会使类别与位置错配。ID 只在检测进程的短时跟踪窗口内有效，不是 Gazebo 果实 ID，
也不是已经确认的植株身份。有效帧中没有红果时发布有效空数组；观察无效或输入失联时
发布无效空数组，消费者不得继续使用旧目标。

## 2. 深度估计和有效性

在检测框中心附近的圆形区域内随机选择不同像素，采样半径取下面两个条件的较小值：
8 px，或检测框宽高的 25%。最多采样 81 个像素；有实例分割时先向内收缩掩膜 1 px，
只在掩膜内部采样。推荐作业模式使用 `require_segmentation:=true`，避免退回无掩膜采样。

剔除 NaN、无穷和量程外深度，要求至少 15 个有效样本、有效比例至少 60%，
再取中值作为轴向深度。中值绝对偏差 MAD 超过 0.03 m 时拒绝目标。
用检测框中心像素和相机内参反投影得到三维点。
多帧稳定条件为：0.5 s 窗口内至少 3 帧，覆盖至少 0.1 s，位置变化不超过 0.025 m。

RGB 与深度按采集时间配对，默认最大差 0.05 s、观察最大年龄 0.5 s。
TF 按 RGB 采集时间查询。时间、尺寸、坐标系、内参或 TF 不符合条件时拒绝观察。

随机采样和中值可以减弱孤立噪点，无法保证当多数样本属于背景时仍得到正确果实深度。
输出是**可见表面深度估计**，尚未求出球心、工具方向或抓取位姿。

检测节点的深度默认量程为 0.07–2.0 m。本流程使用仿真专用
`d405_extended_sim.yaml`，与该范围一致；它不表示实体 D405 已验证同等量程。
若使用其他相机配置，应同步 `depth_min_m / depth_max_m`。

## 3. 停车与作业接口

启用 `require_harvest_handshake:=true` 的任务执行器先完成 Nav2 到站、锁车和地图 TF
稳定检查，再要求新鲜里程计反馈连续 0.5 s 满足：

```text
线速度 < 0.01 m/s
角速度绝对值 < 0.02 rad/s
```

只有满足停车门限时，协调器才可选择本次到站之后的新鲜成熟稳定目标。
它通过采集时刻的 TF 转到 `base_footprint` 判断左/右作业侧，限制近侧选择窗口，并绑定
`run_id / waypoint_id / work_id / plant_id / work_side`。该绑定来自当前任务上下文，
不表示视觉已经识别出唯一植株；状态中的 `plant_identity_verified` 为 false。

作业选择门要求置信度至少 0.5、稳定至少 3 帧、15 个有效深度样本、有效比例至少 60%、
MAD 不超过 0.03 m，并独立检查 RGB/深度时间差不超过 0.05 s。
协调器默认 `require_segmentation:=true`，没有实际使用分割掩膜的目标不能开始作业。
沿车体 X 轴的选择窗为机械臂安装纵向位置附近 ±0.30 m，作业侧按 Y 正负判断；
这用于减少邻株混选，不是 IK 可达性证明。

`/harvest_work` 使用 `agri_harvest_interfaces/action/HarvestWork`：

- 请求携带本次作业身份、目标时间/坐标系及完整 `TomatoTarget`。
- worker 发布 `/harvest_worker/state`，持续反馈阶段和 `arm_stowed`。
- 作业结果成功且收到新鲜、同身份的收纳确认后，才允许结束本株并离站。
- 暂停、取消、失败、超时或 worker 失联时保持锁车；迟到的结果不能完成后续站点。

底盘速度门需同时启用 `require_harvest_interlock:=true`。它直接检查 worker 收纳租约，
未知、展开或失联会强制锁车，直接调用解锁服务或 Bool 命令也不能绕过。
新鲜收纳反馈恢复后仍需显式解锁，不自动恢复旧速度命令。看门狗使用单调墙钟，
仿真暂停不能使旧反馈无限有效。

这两个选项默认关闭，以保留原单纯导航流程。下面的前置作业测试必须显式开启两者。
模拟 worker 的收纳反馈是接口测试数据；真实接入必须由关节状态和控制器健康验证产生。

## 4. 构建和可视测试

从项目根目录构建接口与相关包：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-select agri_harvest_interfaces agri_perception agri_base_adapter agri_task_manager
source ros2_ws/install/setup.bash
colcon --log-base sim_ws/log build --symlink-install \
  --base-paths sim_ws/src --build-base sim_ws/build --install-base sim_ws/install \
  --packages-select agri_sim_description agri_sim_bringup
source sim_ws/install/setup.bash
```

所有终端先加载这两个 overlay，并使用同一 `ROS_DOMAIN_ID`。示例使用 domain 98；
不要与旧的同名仿真/导航进程同时启动。

终端 1：仿真、相机和带作业互锁的底盘。

```bash
export ROS_DOMAIN_ID=98
export GZ_PARTITION="agri_preflight_$(date +%s%N)"
CAMERA_SHARE="$(ros2 pkg prefix --share agri_sim_sensors)"
ros2 launch agri_sim_bringup tomato_field.launch.py \
  gz_partition:="$GZ_PARTITION" gui:=true paused:=false rviz:=false \
  use_control:=true use_kinematics:=true steering_effort_limit:=40.0 \
  require_harvest_interlock:=true \
  use_lidar:=true use_scan:=true use_camera:=true \
  camera_config:="$CAMERA_SHARE/config/d405_extended_sim.yaml" \
  spawn_x:=0.0 spawn_y:=-6.0 spawn_yaw:=1.5708
```

终端 2：保存地图导航，RViz 按 [阶段五](navigation_stage5.md) 加载地图和航点。

```bash
export ROS_DOMAIN_ID=98
ros2 launch agri_navigation navigation.launch.py use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

终端 3：可靠视觉输出。

```bash
export ROS_DOMAIN_ID=98
ros2 launch agri_perception tomato_detector.launch.py \
  model_path:="$PWD/data/weights/best.pt" device:=cpu \
  target_frame:=base_footprint mature_only:=true require_segmentation:=true \
  depth_min_m:=0.07 depth_max_m:=2.0
```

使用 GPU 时将 `device:=cpu` 改为 `device:=0`。推理耗时超过最大观察年龄时会拒绝观察，
先检查帧率、设备和时间同步，不应通过无限延长有效期隐藏延迟。

终端 4：显式模拟作业流程。启动时不自动出发。

```bash
export ROS_DOMAIN_ID=98
FIELD_SHARE="$(ros2 pkg prefix --share agri_greenhouse_worlds)"
ros2 launch agri_task_manager harvest_preflight.launch.py \
  autostart:=false enable_mock_worker:=true \
  field_metadata:="$FIELD_SHARE/worlds/tomato_field_default.json" \
  field_sdf:="$FIELD_SHARE/worlds/tomato_field_default.sdf"
```

第五个终端查看状态和目标，确认模型、worker 心跳及地图绑定后开始：

```bash
export ROS_DOMAIN_ID=98
ros2 topic echo /agri_vision/tomato_targets_base --once
ros2 topic echo /harvest_worker/state --once
ros2 topic echo /harvest_coordinator/status --once --qos-durability transient_local
ros2 topic echo /field_route_executor/status --once --qos-durability transient_local
ros2 service call /field_route_executor/start std_srvs/srv/Trigger '{}'
```

观察到站、停车门通过、选择目标、模拟作业、收纳确认和下一站的状态变化。
当前相机还没有通过左右扫描姿态验收，若到站后看不到稳定红果，协调器应等待并超时锁车；
不能用场景真值或上一株的旧目标代替当前视觉输入。手动 `complete_current` 不能绕过握手。

不启动 Gazebo 的接口回归检查：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/check_harvest_preflight.py --domain 198
```

脚本使用空闲的 DDS domain 198、197、196，启动实际执行器、协调器、速度门和模拟 worker，
通过 ROS 话题、服务和 Action 验证成功收纳、心跳失联、作业超时及绕过请求。
它保留地图文件绑定检查，Nav2、TF、里程计和视觉输入均为人工测试数据，
不能替代上面的仿真导航、检测准确率或真实机械臂验收。
报告写入 `artifacts/harvest_preflight_20261013/runtime_check.json`。
本轮三个 DDS 场景共 44 项检查通过：正常收纳完成 22 项，心跳失联 11 项，作业超时 11 项；
两个故障场景均进入 `FAILED`，保持末端锁车和零速。

## 5. 其余限制的修改方案

### 抓取姿态与分割结果

本轮的可靠消息使用 `Point`，避免把单位四元数误当成有效抓取方向。旧 PoseArray 的
四元数仍只是兼容占位值。建议机械臂侧增加目标到抓取位姿的适配器：标定 TCP 和夹爪
接近轴，按作业侧或局部表面估计接近方向，生成预抓取、夹取、回撤 `PoseStamped`，
经 IK 和轨迹碰撞检查后才交付执行。

若演示只使用固定方向，可先为左右作业侧分别配置接近方向和退距；复杂果实朝向再用
局部点云估计。把深度表面点直接当球心会引入果实半径量级的偏差，需通过标定偏移或
球面拟合处理，不能仅靠中值滤波解决。

实例分割目前已用于深度采样，但没有独立发布每个目标的掩膜。建议新增按 `target_id`
绑定的 ROI 掩膜数组，携带 header、ROI 原点和 `mono8` 掩膜；可视调试可另发布成熟果
联合掩膜，机械臂几何处理则使用单实例掩膜。若带宽受限，可发布轮廓多边形代替整幅图。

### CAD 与机械臂规划坐标系

使用 `target_frame:=base_footprint` 得到标准机器人轴方向下的观察；协调器已经通过 TF
在该坐标系判断作业侧。机械臂接入后，再按同一采集时间转到实际 MoveIt planning frame，
保留 header。当前 `base_link` 沿用 CAD 局部轴，不能只修改 frame_id 标签或手动交换 XYZ。
本轮不改 URDF 的局部轴和协作者的 IK/MoveIt 文件。

先用独立标记物核对相机 → 标准底盘 → 机械臂基座的变换，再在 TCP 标定后核对末端。
规划场景中的植物、车体和果实碰撞体也必须转换到同一 planning frame。

## 6. 验收边界

本轮相关包构建通过，视觉测试 21 项、底盘适配测试 33 项、任务包测试 121 项通过，
其中任务包包括 flake8/pep257 检查。训练权重已在 CPU 完成加载和空图推理检查，确认
类别为 `tomato_R / tomato_G`；空图检查不验证识别率。
ROS 握手检查的范围和结果见
`artifacts/harvest_preflight_20261013/runtime_check.json`，构建和任务包日志在同一目录。

前置检查覆盖成熟筛选、目标关联、采样质量、时间有效性、停稳与模拟作业互锁。
检测模型的实际田间识别率、左右相机视场、真实关节收纳、IK 和夹取仍需独立验收。
此前约 0.743 m 的 map/world 回程偏差尚未修复；本次消息与互锁升级不会自动消除该误差，
任务 `plant_id` 也不能替代实际作业植株身份核对。真值仅用于评测端。
