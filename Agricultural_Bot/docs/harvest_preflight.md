# 机械臂接入前：可靠目标输出与停车作业握手

本流程把成熟果实的视觉观察交给带身份的作业接口，并在底盘速度出口保持停车互锁。
第 4 节保留不控制关节的 mock worker 前置测试；第 9 节接入 Gazebo 关节、MoveIt、
腕部扫描和果实采摘物理。两种 worker 共用停车与作业接口，每次只启动一种。
Gazebo 的采摘结果不代表实体机械臂、相机或末端工具已经标定和验证。

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

## 7. 协作者机械臂备份的独立测试

本机安装 `pymoveit2 5.0.0`、MoveIt `2.12.4` 后，对
`agri_arm_backup_20261013_1555` 完成独立测试。该版本具有协作者使用的
`wait_for_joint_state(timeout_sec=...)`、`wait_until_executed(timeout_sec=...)`
和 `motion_succeeded`；此前针对旧版本的接口兼容性疑虑不适用于本机安装版本。

本次使用 ROS domain 194、`mock_components/GenericSystem` 和独立 `/tmp` 构建目录，
保留原配置参数，只省略 RViz。没有修改协作者备份、整机控制配置或开发日志日期。

| 检查 | 结果 |
| --- | --- |
| 虚拟硬件、关节状态广播器、六关节轨迹控制器 | 激活成功 |
| `(0.3, 0, 0.5)` 和 `(0.3, 0.3, 0.5)` 的预抓取、目标、撤离，以及篮点 `(0.2, 0.4, 0.2)` | 7 个开启碰撞检查的 IK 均成功 |
| 超出工作空间的 `(5, 5, 5)` | 拒绝，`error_code=-31` |
| 关节运动和两组完整动作链，包括测试回零 | 14 次虚拟轨迹执行成功，位姿 FK 最大位置误差约 0.988 mm |
| 原始 `grasp_node.py` 接收 Point 并发布 Float32 | 输入 `(0.3, 0.3, 0.5)`，约 26.5 s 返回 `1.0`，关节反馈确认运动和回零 |
| 夹爪动作 | 全程关节位置为 `0.0`，当前闭合/释放仅打印日志 |
| 原节点退出 | SIGINT 后重复 `rclpy.shutdown()` 报错，退出码为 1 |
| 测试后端停止 | `move_group` 在 SIGINT 后 5 s 内未退出，由 launch 发送 SIGTERM；所有测试后端进程已停止 |

协作者日志中的“入篮失败”本次没有复现，不能据此认定猜测篮点已经标定。
备份模型没有整机底盘、篮子等碰撞几何，安装位置也不同于 canonical URDF；
该点在独立模型下可达，不等于能在整机上完成入篮。
当前 `move_to_pose()` 未开启 `cartesian=True`，接近和撤离仍是到位姿的规划，
不保证末端沿直线运动。输出 `1.0` 不能作为果实已夹住、已释放或安全收纳的证明。

故障注入还确认：收臂失败仍返回成功；动作链中途失败不进入恢复流程；包装器忽略
传入的超时参数；库的等待超时默认不会取消尚在执行的动作。原始节点还没有接入
`HarvestWork`、目标身份/时间有效性和停车握手。这些问题需在整机接入前修正。

结果在 [summary.json](../artifacts/arm_moveit_validation_20261013/summary.json)，
详细服务/关节结果、离线故障注入和节点日志保存在同一目录。
首次测试夹具的清理方法调用有误，修正后完整复测正常退出；原节点的退出问题单独保留。

复现时，终端 1 从项目根目录构建并启动独立虚拟机械臂：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
ARM_TEST_WS="/tmp/agri_arm_validation_ws_20261013"
colcon --log-base "$ARM_TEST_WS/log" build \
  --base-paths "$PWD/../agri_arm_backup_20261013_1555/moveit_config" \
  --build-base "$ARM_TEST_WS/build" --install-base "$ARM_TEST_WS/install" \
  --packages-select agri_arm_moveit_config
source "$ARM_TEST_WS/install/setup.bash"
export ROS_DOMAIN_ID=194
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$PWD/artifacts/arm_moveit_validation_20261013/ros_logs"
ros2 launch "$PWD/artifacts/arm_moveit_validation_20261013/headless.launch.py"
```

终端 2 同样加载测试环境，按顺序运行。前两个脚本会发送独立虚拟机械臂动作，
最后一个仅做离线故障注入。复测会覆盖同名 JSON 报告，需要保留历史结果时先复制目录。

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source /tmp/agri_arm_validation_ws_20261013/install/setup.bash
export ROS_DOMAIN_ID=194
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$PWD/artifacts/arm_moveit_validation_20261013/ros_logs"
python3 artifacts/arm_moveit_validation_20261013/runtime_probe.py
python3 artifacts/arm_moveit_validation_20261013/node_probe.py
python3 artifacts/arm_moveit_validation_20261013/offline_probe.py
```

完成后在终端 1 用 `Ctrl+C` 停止测试后端。本次仍未验收 Gazebo 关节控制、完整车体
碰撞检查、TCP、夹爪开度、果实附着/释放和视觉到停车作业的闭环。

## 8. 更新版机械臂备份复核

复核对象为 `agri_arm_backup_20261013_20361/agri_arm_backup_20261013_2036`，
与上一节的 `1555` 版本及当前主项目进行比较。新版已形成整机模型下的抓取原型，
但仍不能直接作为当前停车采摘流程的执行端。没有修改两份协作者备份或开发日志日期。

### 已完成的更新

| 项目 | 新版状态 |
| --- | --- |
| 整机模型 | `arm_full.urdf` 含 27 个 link、26 个 joint，根为 `base_footprint`；底盘、轮子、篮子、雷达和相机恢复，机械臂安装位置与当前 canonical xacro 的静态展开一致 |
| 几何资产 | 随备份提供的 28 个 visual mesh 与主项目对应文件哈希一致；附带描述包仍是独立副本，不应覆盖主项目描述包 |
| MoveIt 碰撞 | 移除旧版机械臂对底盘的大范围豁免，恢复篮子、底盘等整机碰撞约束 |
| 独立控制后端 | `demo_full_robot.launch.py` 启动虚拟硬件、六关节机械臂轨迹控制器和单指夹爪轨迹控制器，合并轮子默认关节状态 |
| 夹爪与入篮 | 由只打印日志改为发送夹爪 Action；动作链新增篮子上方、篮口、释放及抬起步骤 |
| 包装器超时 | `move_to_pose()` / `move_to_joints()` 开始使用传入的等待超时；配置 YAML 的超时、重试参数仍未接入动作链 |

### 本机验证及边界

两份新 ROS 包在独立 `/tmp/arm_v2_review_*` 目录构建通过。本轮使用 domain 193、
`mock_components/GenericSystem`，只省略 RViz，保留备份模型、SRDF 和控制参数。
查询 `/compute_ik`、`/compute_fk`、`/check_state_validity`、控制器和 TF，
**没有发送机械臂轨迹、夹爪动作或视觉抓取任务**。

全部坐标均为新版 `base_link` 的 CAD 局部轴，IK 使用固定姿态
`[sqrt(0.5), 0, 0, sqrt(0.5)]` 和 1 s 求解时间。

| 查询 | 结果 |
| --- | --- |
| 独立后端与初始状态 | 三个控制器 active，完整关节状态可用；全零初始整机/机械臂状态通过当前碰撞检查 |
| 日志失败目标 `(0.3, 0.4, 0.3)` | 带碰撞查询返回 `-31`；忽略碰撞有解，但该返回解与底盘、篮子和末端等发生碰撞 |
| 上述目标的实际预抓取点 `(0.269130, 0.358840, 0.269130)` | 按原代码的 0.06 m 退距计算；带碰撞查询返回 `-31`，忽略碰撞的返回解与底盘/篮子发生碰撞 |
| 日志命令目标 `(0.3, 0.5, 0.5)` | 该点及原代码计算的预抓取、回撤点均找到通过碰撞检查的 IK 解 |
| 篮口 `(0, 0.5, 0.17)`、原代码“上方”点及修正竖直方向点 | 三个端点均找到通过碰撞检查的 IK 解；这不证明端点间轨迹、TCP 或装有果实时的净空合格 |
| 两个示例较高侧向点 | 端点 IK 成功；属于人工示例，未按当前航点/果实生成，不能作为默认场景可采摘率验收 |
| `world -> base_footprint` | 不存在；SRDF 的 virtual joint 不会自动发布 ROS TF，备份 RViz 却使用 `world` 作为 Fixed Frame |

`-31` 只表示本次限定时间内没有找到解，不能证明所有 IK 分支必然碰撞。
本次结果支持优先检查低位测试点的碰撞约束及固定抓取方向，不能替代原动作规划日志。
成功端点也不能替代完整轨迹验收，当前查询场景没有田间植株和果实碰撞体。

离线故障注入进一步确认：夹爪服务不可用时，动作链仍返回成功；夹爪 goal 被拒绝，
封装仍立即返回 `True`；回零失败仍返回成功；六个机械臂移动步骤任一步失败都直接退出，
没有执行收纳恢复。等待超时未取消动作，YAML 的独立超时和重试仍未使用。
`grasp_node.py` 与旧版相同，重复关闭 ROS context 的退出问题也仍在。

只读结果在 [model_query.json](../artifacts/arm_v2_review_20261013/model_query.json)、
[pregrasp_query.json](../artifacts/arm_v2_review_20261013/pregrasp_query.json)，
故障注入结果在 [offline_report.json](../artifacts/arm_v2_review_20261013/offline_report.json)。
测试后已停止独立后端；这些结果不等于新版完整抓取或 Gazebo 执行通过。
`move_group` 在 SIGINT 后 5 s 内未退出，由 launch 发送 SIGTERM 后停止，
退出行为与上一版独立测试一致；确认没有遗留本次测试进程。
复核汇总及源码指纹见 [summary.json](../artifacts/arm_v2_review_20261013/summary.json)。

### 接入前必须修正的问题

1. **入篮抬升方向。** `grasp_executor.py:55` 增加 `base_link` 的 Z 来表示上方，
   但当前 CAD 的 +Y 才是竖直向上，+Z 是标准底盘的向前方向。原篮口
   `[0, 0.50, 0.17]` 的“上方”被写成 `[0, 0.50, 0.32]`，实际向前平移 15 cm。
   同坐标系内升高 15 cm 的几何示例是 `[0, 0.65, 0.17]`；已查到端点 IK，
   尚未验收入篮轨迹。建议在标准底盘坐标系定义篮口、预抓取、回撤和抬升，
   按实际 TF 转到 MoveIt frame，避免手动交换 XYZ。
2. **夹爪与主项目控制器一致。** 备份客户端只发送左指的
   `FollowJointTrajectory`，主项目 Gazebo 的 `gripper_controller` 是
   `ForwardCommandController`，控制左右两个独立指关节，没有该 Action。
   可以采用双指轨迹控制器，或新增双指位置命令与反馈适配器；仅改话题名不能接通。
   夹爪必须等待 goal 接受、终态和关节到位。URDF 的两个指关节是 prismatic，
   `0.8 / 0.1` 的单位是米，不能当作开度百分比，需要标定实际行程和果实夹持宽度。
3. **TCP、方向和碰撞矩阵。** `tool0` 与夹爪基座重合，未标定两指间抓取 TCP；
   原篮口工具原点仅比篮沿高约 4.44 cm，不代表指尖或果实有足够净空。
   `try_grasp_orientations()` 仍只返回固定四元数，没有进行候选姿态 IK 选择；
   法向量是从目标指向 CAD 原点，不能代表果实局部表面。
   SRDF 将 `arm_link4/5` 与相机标作固定关系而全部豁免，但中间含可转动关节，
   应复核这些豁免。接近、回撤和入篮当前都是普通 pose 规划，不保证直线。
4. **故障必须保留停车互锁。** 检查夹爪结果，区分抓取成功与安全收纳，
   给规划/执行设置实际生效的期限；超时先取消并等到终态，确认关节停止，
   再评估恢复轨迹。异常不能继续入篮，收纳失败不能允许底盘前往下一站。
   收纳必须由指定关节的位置、速度反馈确认，不能只依赖函数返回或固定睡眠。
5. **复用当前仿真后端。** 整机接入时使用主项目 xacro 和 Gazebo 已有的
   controller manager、关节反馈、robot_state_publisher 及 `/clock`。
   不要与当前仿真同时启动这个 mock demo；它会新增同名控制器、TF/关节发布者，
   且没有统一设置 `use_sim_time`。独立演示可用 `base_footprint` 作为 RViz Fixed Frame，
   仿真接入则使用已有 `map/odom` TF 链，不固定世界坐标下的移动底盘。
6. **接入可靠作业接口。** 旧节点输入是裸 `Point`，输出 `Float32`，没有
   header、成熟类别、置信度、目标 ID、作业 ID 或停车状态。
   当前兼容视觉输出 `/agri_vision/tomato_positions_base` 是 `PoseArray`，
   可靠输出 `/agri_vision/tomato_targets_base` 是 `TomatoTargetArray`，都不能直接改名
   连接到裸 Point。正式执行端应实现现有 `HarvestWork` Action 和
   `HarvestWorkerState` 心跳/收纳反馈，由协调器传递当前站的成熟目标，保留
   run/work/waypoint/plant/side/target 身份以及采集时间。还需验证左右相机扫描姿态；
   若收纳视角看不到果实，需增加受停车互锁保护的扫描阶段，不能使用旧帧顶替。

### 与当前默认场景对齐及接入顺序

协作者日志使用的是 2 m 行距场景。当前默认为 3 m，六行 world X 为
`[-7.5, -4.5, -1.5, 1.5, 4.5, 7.5]`，单通道两侧作业行在 `X=±1.5`。
演示果实配置将作业侧约 85% 的果实放在高度 1.33–1.42 m、径向距离 0.68–0.74 m，
直径 0.05–0.065 m；其余果实仍保留普通随机分布，不能把所有目标都按日志的
普通分布处理。该配置仍属于距离候选，未完成真实 TCP/整段 IK 验收。
日志提到的 `tomato_bridge.py` 未包含在本次备份中。场景真值桥接只能用于验证与评测，
采摘输入应来自当前相机测量并通过采集时间对应的 TF 变换。

建议先修正 TCP/夹爪/坐标方向/失败处理，再将 MoveIt 配置包加入主项目，
新增复用 Gazebo 后端的 launch；不要复制附带的同名 `agri_robot_description` 覆盖当前包。
随后在停车锁定下验收左右扫描、真实视觉目标的预抓取/夹取/回撤/入篮/收纳轨迹，
加入车体、植物和果实规划碰撞体，并记录每个真实目标的 ID、时间和拒绝原因。
通过后接入 `HarvestWork`，验证成功、目标过期、取消、超时、夹爪失败及收纳失败分支。
最后实现 Gazebo 果实附着/脱离与入篮状态，才验收完整视觉采摘闭环；目前静态果实不会
仅因闭爪而自动被摘下。
## 9. Gazebo 机械臂接入与首站测试

协作者新版备份的配置和参考程序已复制到 `ros2_ws/src/agri_arm_moveit_config`、
`ros2_ws/src/agri_arm_control/reference/collaborator_2036`。运行入口使用主项目 Xacro，
不启动第二个机器人、关节状态发布器或 controller manager。

新作业链路为：Nav2 到站 → 锁车并实测停止 → 确认作业 nonce → 腕部相机转向作业侧
→ 筛选扫描姿态稳定后的新鲜成熟分割目标 → 碰撞检查候选 → 接近、闭爪、附着
→ 撤离、移到篮上方、下降、开爪 → 物理落篮确认 → 测量六关节收纳 → 结束本站。
`complete_current` 不能替代此结果。

### 9.1 构建、生成可采摘场景

本节保留早期联调流程。已有 `artifacts/harvest_demo/current` 是旧外伸果实/支架
资产，不能直接作为本轮自然原位小果实场景使用；本轮正确绑定入口见第 10.7 节。
不要只替换世界文件而继续加载旧任务或旧碰撞导航图。

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-up-to agri_arm_moveit_config agri_arm_control agri_perception agri_base_adapter
source ros2_ws/install/setup.bash
colcon --log-base sim_ws/log build --symlink-install \
  --base-paths sim_ws/src --build-base sim_ws/build --install-base sim_ws/install \
  --packages-up-to agri_sim_bringup
source sim_ws/install/setup.bash
python3 scripts/prepare_harvest_demo.py
gz sdf -k artifacts/harvest_demo/current/tomato_field_harvestable.sdf
```

脚本核对默认 SDF、metadata 与原任务的 SHA，生成独立副本：

- `tomato_field_harvestable.sdf`：果实初始视觉和碰撞位置保持一致，增加可脱离果实物理。
- `tomato_field.json`：原 metadata 副本。
- `harvest_mission_first_station.yaml`：只启用首株，不自动回原点，供首次联调。
- `harvest_mission.yaml`：全部 20 个作业点。
- `manifest.json`：源资产与新副本的 SHA 绑定。

当前转换不改变初始植物布局，沿用现有地图和 keepout；原默认场景及备份没有覆盖。
后续若修改行距、植物位置或外形，按原建图流程重建并重新绑定任务，再生成此副本。

果实派生网格还修正了三角面的绕序与法线方向。原网格朝内，RGB-D 在果实中心附近
读取的是远侧表面，实测比近侧表面深约 44 mm，导致球心代理偏移和掌部碰撞。
修正保留顶点、UV、尺寸及碰撞球；修改后需要重启 Gazebo，清除旧的渲染缓存。
提取脚本及网格测试会检查外向绕序，原始植物网格和协作者备份保持原样。

### 9.2 启动四个终端

每个终端都先执行下面的公共环境设置。使用同一个 DDS 域及 Gazebo 分区；先关闭
该域/分区旧的仿真、导航、mock worker 和任务执行器。

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=98
export GZ_PARTITION=agri_harvest_demo_98
export HARVEST_DIR="$PWD/artifacts/harvest_demo/current"
```

终端 1：从正常出生点启动可采摘场景、关节控制、相机、雷达和底盘作业互锁。

```bash
ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$HARVEST_DIR/tomato_field_harvestable.sdf" \
  gz_partition:="$GZ_PARTITION" \
  gui:=true paused:=false rviz:=false \
  use_control:=true use_kinematics:=true steering_effort_limit:=40.0 \
  require_harvest_interlock:=true \
  use_lidar:=true use_scan:=true use_camera:=true \
  camera_config:="$PWD/sim_ws/src/agri_sim_sensors/config/d405_extended_sim.yaml"
```

终端 2：保存地图定位及导航，不启动 SLAM。

```bash
ros2 launch agri_navigation navigation.launch.py use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

终端 3：可靠成熟目标输出。CPU 测试可限制线程数；有可用 CUDA 时可将 `device` 改为 `0`。

```bash
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2
ros2 launch agri_perception tomato_detector.launch.py \
  model_path:="$PWD/data/weights/best.pt" device:=cpu \
  target_frame:=base_footprint mature_only:=true require_segmentation:=true
```

终端 4：连接现有 Gazebo 的 MoveIt、场景服务、机械臂 worker、协调器和任务执行器。
先只测首站；该入口默认不自动开始作业。

```bash
ros2 launch agri_sim_bringup harvest_arm.launch.py \
  mission_file:="$HARVEST_DIR/harvest_mission_first_station.yaml" \
  field_metadata:="$HARVEST_DIR/tomato_field.json" \
  field_sdf:="$HARVEST_DIR/tomato_field_harvestable.sdf" \
  gz_partition:="$GZ_PARTITION" \
  use_sim_time:=true use_rviz:=true autostart:=false
```

MoveIt RViz 显示机器人及场景障碍；地图/导航显示可另外启动：

```bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
```

### 9.3 检查并开始

另开终端，加载公共环境后检查：

```bash
ros2 control list_controllers
ros2 action info /harvest_work
ros2 action info /arm_controller/follow_joint_trajectory
ros2 action info /gripper_controller/follow_joint_trajectory
ros2 topic echo /harvest_arm_worker/status --once
ros2 topic echo /harvest_worker/state --once
ros2 topic echo /field_route_executor/status --once --qos-durability transient_local
ros2 lifecycle get /bt_navigator
```

`arm_controller`、`gripper_controller`、`joint_state_broadcaster` 均须 active。
首次启动时，JTC 可能尚未发布任何 Action 状态。先保持底盘锁定并初始化真实控制器状态：

```bash
ros2 service call /harvest_arm_worker/initialize_sim_hold std_srvs/srv/Trigger '{}'
```

该服务只允许已核验的 Gazebo 控制器，在实测停车且关节静止后取消旧目标、确认终态，
给两路控制器发送当前实测位置的持位目标；不会将手臂直接移到收纳位。若重启时
手臂停在非收纳位置，初始化成功后仍保持 FAULT，再按 9.4 的受控收臂步骤恢复。
机械臂状态须 `backend_ready=true`、`measured_stowed=true`、`phase=IDLE/STOWED`；
任务状态须 live map/keepout 有效且 IDLE，导航 active。夹爪两个手指由同一个轨迹 Action 控制。
底盘在初始化阶段保持锁定，直到测得收纳状态和任务启动检查通过。

```bash
ros2 service call /field_route_executor/start std_srvs/srv/Trigger '{}'
ros2 topic echo /harvest_arm_worker/status
# 在另外的终端观察：
ros2 topic echo /agri_vision/tomato_targets_base
```

收纳时相机可能看不到果实，这是扫描模式允许空目标启动的原因。扫描后仍须取得
新鲜、稳定、成熟且实际使用分割的目标；不接受扫描前缓存。三维点仍是表面观察，
当前沿接近方向加 `0.025 m` 名义半径作为球心代理，每个候选仍需碰撞 IK 与规划通过。

首站成功须同时满足 Action 成功、`sim_fruit_verified=true`、果实 `IN_BASKET`、
新鲜同身份的实测 STOWED，随后任务进入 COMPLETED。仿真中可看到果实脱离植株并落篮。
只看到手臂运动、Action accepted 或开爪，不算采摘成功。

闭合阶段采用双指独立的 1 mm 小步轨迹：每步必须得到真实控制器成功终态及停稳反馈。
已接触的手指保持实测位置，另一指继续闭合。Gazebo 原生碰撞数据须证明两指在
选中果实的相对两侧接触，穿透量不超过 2 mm、没有掌部或其他物体误接触，且双指
稳定至少 0.1 秒仿真时间；实测持位后再次复核，通过才附果。果心偏离夹爪中线时
两指位移可以不同，不能强行压到同一闭合值。任意控制器失败仍终止作业。

首站验证通过后，关闭这四个终端，再依次重新启动以恢复果实和时钟状态；终端 4 改用
`mission_file:="$HARVEST_DIR/harvest_mission.yaml"` 可执行全部作业点。
这不是 20 点采摘成功率已经验收的声明，需逐站检查视野、深度、IK 和物理结果。

### 9.4 失败恢复和参数位置

失败、超时、取消或反馈失联会保持 FAULT 与锁车。查看 worker 日志中的具体拒绝原因，
不要发送底盘速度或手动伪造收纳反馈。底层轨迹终态已确认、底盘仍实测停止时，允许：

```bash
ros2 service call /harvest_arm_worker/recover_stow std_srvs/srv/Trigger '{}'
```

只有反馈确认回收才返回 success。恢复不自动完成失败站点，也不自动继续导航；确认
`measured_stowed=true` 且上一作业终态已确认后，再调用 `/field_route_executor/start`
重新开始。若持有果实或存在碰撞，
回收规划也会拒绝，此时先停止测试并重启完整仿真，恢复初始场景。

主要配置为 `ros2_ws/src/agri_arm_control/config/harvest_arm_worker.yaml`：
扫描关节、预接近/撤离距离、双指开合、篮中释放位置、候选数和执行超时均在其中。
扫描姿态的相机光学位置为 `base_footprint` 下 `[-0.18, ±0.30, 1.00] m`，
光轴水平朝向作业侧；这里的 1.00 m 是车体坐标高度，须逐站复核视野。
篮中释放 TCP 为 `[0.17, 0, 0.60] m`，上方点为 `[0.17, 0, 0.75] m`，
同样采用 `base_footprint`；空篮自由落球测试已确认内腔，采摘仍须单独取得果实入篮回执。
轨迹期限按实际轨迹长度和仿真时钟计算，同时保留单条机械臂轨迹 120 秒、夹爪
20 秒的墙钟硬上限；时钟连续 5 秒不前进或发生倒跳会取消动作。整次作业最多
400 秒，协调器和任务执行器分别留出 420、440 秒；这些期限不会放宽停车与反馈检查。
`status.motion` 提供轨迹时长、仿真/墙钟进度、实时速率和控制器终态，便于区分
仿真运行较慢与关节受阻。
`status.stow_evidence` 提供六关节实测位置/速度、反馈年龄及收纳拒绝原因；
位置已符合要求但稳定计时尚未完成时等待，位置确实超差时才按非收纳处理。
接触闭合参数 `gripper_contact_step_m`、`gripper_contact_step_s`、
`maximum_gripper_contact_steps`、`gripper_contact_timeout_s` 同样在 worker YAML 中，
默认 1 mm、0.25 秒仿真时间、最多 50 步及 120 秒墙钟总期限。
`grasp_tcp` 在主机器人 `gripper.xacro` 定义，为几何候选，尚非实体工具标定。
作业坐标采用 `base_footprint` 的 X 前、Y 左、Z 上；MoveIt 的 CAD `base_link` 经 TF 转换。
篮上方沿标准 Z 增高，不按 CAD +Z 增高。程序只允许仿真/显式 mock 控制后端。

规划场景包含全部植株茎、果实、地面及机器人模型。仅允许当前选中果实与两手指的
预期接触、四轮与地面的接触；退出或失联会恢复果实接触豁免，机械臂与其他障碍仍检测。

## 10. 保留自然果实、靠近植株的仿真候选

默认世界、metadata 和匹配的默认任务已恢复到演示果实配置之前的版本：60 株、224 个
自然果实。没有长绿色支梗，不调整果实位置、半径或成熟度。旧的外伸果实配置备份保存在
`artifacts/close_harvest_20261014/previous_outstretched_default`，不会用来运行此测试。

叶片没有物理碰撞体，但普通 GPU 雷达会看到叶片，所以仍能在 SLAM 地图和实时障碍层
留下占据。直接扩大叶片会进一步遮挡果实、增大普通扫描的障碍面积；仅减小 inflation
也不能清除静态地图中的已有占据。本候选让两种输入分别服务定位和导航：

- `/map` 和 `/scan` 保留原来的植物外观，供 AMCL 定位。
- `/navigation_map` 使用已知 Gazebo 实际碰撞生成的导航蓝图，保留原栅格、原点、
  未知区域及非植株障碍；茎杆和果实实际碰撞继续占据。
- 可选仿真雷达 `/hard_obstacles/points` 排除没有碰撞的枝叶，仍看到茎杆代理、果实和
  其他可视物体，投影到 `/scan_obstacles` 后供两代价地图及 Collision Monitor 使用。

这使用仿真几何和渲染掩码，不是对真实 MID-360 的叶片识别能力声明。原定位地图不改，
完整 footprint、padding、keepout、停车检测多边形和 0.40 m 膨胀半径不缩小。
现有 stem 碰撞盒对应的代理只供导航雷达看见，在普通相机和默认 GUI 中隐藏；它没有
新增碰撞，不是托住外伸果实的支杆。

### 10.1 生成与启动

使用第 9 节公共环境，并在关闭该域旧测试后使用新的 Gazebo 分区。构建新入口：

```bash
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-up-to agri_navigation agri_lidar_adapter agri_task_manager agri_base_kinematics
source ros2_ws/install/setup.bash
colcon --log-base sim_ws/log build --symlink-install \
  --base-paths sim_ws/src --build-base sim_ws/build --install-base sim_ws/install \
  --packages-up-to agri_sim_bringup
source sim_ws/install/setup.bash
python3 scripts/prepare_collision_navigation.py
python3 scripts/prepare_close_harvest.py
export CLOSE_DIR="$PWD/artifacts/close_harvest_20261014"
export HARVEST_DIR="$CLOSE_DIR/harvestable"
export GZ_PARTITION=agri_natural_close_harvest_98
```

每个终端都设置相同环境变量。终端 1 使用第 9.2 节仿真指令，额外加入：

```bash
# 加到 tomato_field.launch.py 的参数中：
use_collision_scan:=true lidar_mode:=gpu_lidar
```

另开投影终端，同时保留仿真入口已经启动的原 `/scan` 投影：

```bash
ros2 launch agri_lidar_adapter collision_scan.launch.py use_sim_time:=true
```

导航终端改用：

```bash
ros2 launch agri_navigation navigation.launch.py use_sim_time:=true \
  navigation_map:="$CLOSE_DIR/navigation_collision.yaml" \
  obstacle_scan:=/scan_obstacles \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

检测终端仍使用第 9.2 节的可靠成熟分割输出。机械臂终端改用：

```bash
ros2 launch agri_sim_bringup harvest_arm.launch.py \
  mission_file:="$HARVEST_DIR/harvest_mission_first_station.yaml" \
  field_metadata:="$HARVEST_DIR/tomato_field.json" \
  field_sdf:="$HARVEST_DIR/tomato_field_harvestable.sdf" \
  map_yaml:="$CLOSE_DIR/navigation_collision.yaml" map_topic:=/navigation_map \
  gz_partition:="$GZ_PARTITION" use_sim_time:=true use_rviz:=true autostart:=false
```

继续执行第 9.3 节持位初始化和真实状态检查，再开始任务。首站目标为
`map: x=1.18, y=0.75, yaw=0`，茎杆位于 `y=1.50`；停车偏向左作业侧。
本候选默认仅首站 enabled。完整 20 点的连接线在保留的障碍区域仍未通过离线验证，
不能直接将其他站点启用；disabled 点也尚未验证近距离停车或机械臂可达。

### 10.2 当前验收边界

实际仿真已验证两路扫描同一水平帧、10 Hz 仿真频率、递增时间戳，双地图格网一致且
实际 occupancy SHA 匹配保存文件；Nav2 参数确实切换到新输入，AMCL 仍使用 `/scan`。
原占据格 1880，新占据格 365，2537 个未知格逐字保留。首站 padded footprint 在原图
接触 23 个占据格，在新图为 0；其微调范围及 home `(0.5, 0, 0)` 到首站的连接线通过离线检查。
该连接线检查不包含出生点 `(0, 0, 0)`：其后部 footprint 超出了保存地图左边界，
不能把这个结果称为出生点到首站的完整栅格验证。
输入证据为 `artifacts/close_harvest_20261014/navigation_inputs_runtime.json`，地图对比图为
同目录 `localization_vs_collision_navigation.png`。

首轮实际到站测试被收纳互锁取消：第二关节位置约 `0.000018 rad`，瞬时实测速度
约 `0.0408 rad/s` 超过 `0.03 rad/s` 门槛。其记录已单独保存在
`first_station_navigation_velocity_fault_v1.json`。

随后增加 SIM 时间下的转向参考限速 `0.5 rad/s`、限加速度 `1.0 rad/s²`，保留原始
目标对实际转向反馈的驱动许可与紧急停车规则。79 项相关离线测试通过，真实参数已
核对生效。重新冷启动整套 Gazebo，从正常出生点复测，约 35.9 秒墙钟后仍因同一互锁
取消导航：第二关节最大观测速度约 `-0.0528 rad/s`；实际触发取消的反馈在
`29.450 s` 仿真时间，速度约 `-0.0397 rad/s`、位置约 `-0.0000568 rad`。
同步的 JTC 持位参考仍是零位，速度尖峰在锁车零指令之前，不能归因于取消后的制动。
根因仍需确认，转向平滑不能宣称已解决。没有放宽收纳阈值或改写实际反馈。

第二轮记录为 `first_station_navigation_velocity_fault_v2.json`，同步数据为
`arm_turn_trace_v2.jsonl`，失败后实测锁车、收纳恢复及生效参数见 `v2_final_state.json`。
`first_station_runtime.json` 保留第二轮记录，后续独立对照使用带版本的文件。不能据输入检查通过声明导航到站、夹取或
落篮成功；当前仅确认双地图/扫描输入和离线首站几何检查。失败保持锁车。

自然果实也有尺寸和视野限制：首株的一颗成熟果实直径约 68 mm，另一颗约 78 mm，
而几何夹爪最大开口约 74 mm。较小果实位置偏低，现有扫描姿态可能无法完整观察。
当前三维输出仍为表面点加名义 25 mm 的果心代理；不能用简单检测框宽度乘表面深度
直接放行尺寸，也不能根据场景真值挑选视觉目标来伪造成功。
必须在碰撞检查后的扫描姿态下，验证新鲜图像、尺寸不确定度、IK 与双指实际接触。
完整自然果实采摘闭环及 20 点成功率尚未验收。

低位观察候选也已完成实际只读查询：光学帧在 `base_footprint` 中的位置为
`[-0.18, ±0.30, 0.90] m` 或 `[-0.18, ±0.30, 0.80] m`，均找到通过 IK、FK、
状态碰撞检查及 MoveIt 规划的解，优先测试 0.90 m。证据为
`lower_scan_pose_readonly.json`。查询发生在第二轮失败后的实际停车位置，
不是首站的可达性、执行或夹取证明；作业开始时必须按实际到站位置重新规划。
仅供本次测试的参数文件为 `harvest_arm_worker_lower_scan_090.yaml`，未替换默认参数。
作业节点在启动时读取扫描参数，修改 ROS 参数不能直接改变已缓存的观察关节目标；
需要在实测锁车、收纳及控制动作终止后重新启动该节点，再执行持位初始化。

### 10.3 轮胎接触与靠侧导航对照

新增只读 Gazebo 原生物理诊断后，在继续导航时记录到了轮胎与地面的接触力尖峰，
同时发生车体垂向/角加速度和第二关节速度瞬态；该窗口没有植株接触。
CAD 轮胎表面是离散三角网格，外轮廓存在约 0.215 mm 的滚动支承高度变化。
这支持轮胎地面接触影响机械臂持位的判断，不能仅凭共时记录排除所有控制因素。
原始证据为 `native_motion_trace_v3.jsonl`，图为 `native_motion_v3_contact_impulse.png`。

可选参数 `wheel_collision_geometry:=cylinder` 将四个轮胎的仿真碰撞体设为半径
0.127 m、宽度 0.10 m 的圆柱。保留 CAD 外观、质量惯量、关节和车体 footprint；
圆柱包络原轮胎倒角，所以并非原网格接触面的完全等价替换。默认仍为 `mesh`。

第四轮从正常出生点冷启动，显式使用圆柱碰撞体，其他导航参数不变。完整原生观察
覆盖 40,659 个 1 ms 物理步，第二关节最大速度约 `1.48e-10 rad/s`，没有收纳速度
尖峰，也没有触发收纳互锁。但是首站导航仍未通过：RPP 多次报告前方碰撞并进入旋转
恢复，最终在 120 秒墙钟期限内未到站。记录为 `first_station_cylinder_v4.json`。
该结果证明本轮持位稳定性改善，不能当作导航、扫描或夹取成功。

四轮运动学和里程计支持横向速度，但默认速度门控的 `max_linear_y: 0.0` 禁止横移。
下一轮使用独立低速全向控制候选，尝试保持目标朝向靠近作业侧；通过新参数
`base_adapter_config` 显式加载候选门控文件，不改变默认门控、收纳要求、停车锁、
碰撞检测多边形或未知区/keepout 边界。低实时速率测试的首站导航期限单独设为
360 秒墙钟，原任务及其他禁用站点不变。仍须实测到站、视野、IK 与双指接触。

### 10.4 全向候选与出生点入口检查

第五轮显式加载 `nav2_close_harvest_omni.yaml` 和
`base_adapter_close_harvest_omni.yaml`。实际轮速、转向、停车和收纳限制继续生效，
候选门控仅允许最高 `0.08 m/s` 的横向速度。但是本轮没有起步，已取消并实测锁车。
本机安装的 DWB C++ 插件检查确认：LimitedAccelGenerator 的初始速度窗口只有
`0.0025 m/s`，1.5 秒预测位移最多 `3.75 mm`，小于 `5 cm` 栅格，导致网格评分相同。
记录为 `first_station_omni_v5.json`，取消说明为 `v5_cancellation_note.json`。
`v5_nav_feedback_evidence.json` 只覆盖取消后的停车，不证明导航阶段通过。

第六轮使用独立 `nav2_close_harvest_omni_v6.yaml`：改为带加速度积分的
StandardTrajectoryGenerator，前后最高 `0.12 m/s`、横向最高 `0.08 m/s`，
保留完整 footprint、碰撞扫描和 keepout。ObstacleFootprint 的合法软代价权重降为
`0.01`；lethal、unknown 和越界仍由实际插件拒绝。该 critic 检查多边形边界，
不等于填充内部的离线 footprint 验证。34 项导航测试及本机实际 C++ 插件检查通过，
数值证据为 `v6_controller_offline_evidence.json`。默认导航仍使用原配置。

入口采用独立 `harvest_mission_omni_entry_v6.yaml`，增加到 home 的必经中转后再靠侧。
`spawn_entry_geometry_certificate_v6entry.json` 校验实际初始 SDF 的 60 个茎杆、
224 个自然果碰撞体和地板边缘。在中心 `x=[-0.10, 0.60] m`、`y=±0.10 m`、
`yaw=±0.1123 rad` 范围内，额外加入 35 mm 几何储备后仍通过物理净空检查。
这属于已知 Gazebo 初始几何证明，保存地图的出生点覆盖仍明确未通过，没有把未知格
改成自由格。运行采证器先等待完整新鲜 TF，再在 entry_home 导航期间检查上述边界；
超界或 TF 失效会请求取消并锁车。它不证明其他入口或跨行运动安全。

首轮第六轮采证器因启动时 TF 尚未连接立即取消，没有移动，记录保留在
`first_station_omni_entry_v6.json`。修正为启动路线前等待 TF 就绪后，另开
`first_station_omni_entry_v6retry.json`，不覆盖前一结果；最终验收以该轮真实终态为准。

该重测已结束：entry_home 在约 34.5 秒墙钟完成导航，随后停车检查通过。
入口守护累计 337 次有效检查，无边界或 TF 新鲜度违规。首株靠侧导航仍未完成：
右后轮实际转向角约 `-0.8566 rad`，参考约 `-0.4988 rad`，误差约 `0.3578 rad`
超过 `steering_stop_error=0.35`，四轮驱动指令因此归零。机械臂收纳稳定，尚未开始扫描。
最终在本轮约 288.3 秒墙钟出现 worker 心跳时间戳不递增，速度门控锁车，路线报告
`FAILED: motion gate changed state during navigation`。首站采摘、夹取和落篮均未验收。

只读 Gazebo 位置与定位的对照也发现，停滞时实际位置映射到 map 约
`(0.6049, 0.1602)`，轮里程计约 `(0.6204, 0.1720)`，AMCL 约
`(0.9612, 0.1810)`；AMCL 沿行偏差约 0.356 m，不能将它的到站反馈当作物理到位。
`v6_stall_world_readonly_snapshot.json` 仅用于独立评测，没有按真值重写定位或挑选果实。
AMCL 实际仍加载 DifferentialMotionModel；横移控制下需要验证 OmniMotionModel 候选，
但这不能单独证明已消除重复植株导致的定位歧义。

首站视觉采证的两个等待窗口均未进入 SCANNING、没有图像/目标/IK证据。
增强原生转向观察器在任务失败后才开始采样，记录为 `v6_r2_stall_native.jsonl`，
只证明锁后静止，不证明活跃导航时转向力矩不足。没有取消转向对齐、碰撞、
命令有效期、关节限位或停车采摘互锁。

### 10.5 心跳误锁车修复与受控转向诊断

作业节点的墙钟定时器和工作终态回调可能在同一次 `/clock` 更新间隔内发布消息。
原速度门控把完全相同的时间戳也当作异常，因此正常收纳状态会误撤销通行许可。
修复后，发布端只去重真实时间与完整 `HarvestWorkerState` 均相同的消息；
相同时间发生状态或工作身份变化仍立即发布。接收端忽略完全重复的消息，但不会
延长原墙钟租约；旧消息、同时间状态变化、时钟回退与反馈超时仍撤销许可。
危险状态立即输出零速度，恢复后仍要求显式解锁，没有制造递增时间戳。

两个软件包与真实 worker→gate 方法联动回归共 140 项通过，构建完成。
隔离测试只重启所属 worker 和速度门控，保留 Gazebo、控制器及轮里程计连续性；
重新执行两个当前关节位置保持目标，终态、实际收纳和锁车停稳均确认。
记录为 `artifacts/close_harvest_20261014/heartbeat_v8_restart_retry1/after.json`。

手动转向诊断必须先确认任务无活动目标且处于待机。任务执行器处于 FAILED 时会
对外部解锁正确发起重锁；这不是心跳修复失败。新诊断采证器还要求解锁服务回复
之后收到新的 `locked=false`，避免把 DDS 缓存的旧 `true` 当作运行中故障。

`v8_r2_repaired_manual_b.json` 记录了从实际停车位开始、保留所有保护的低速复测：
`vx=0.06 m/s`、`vy=0.03225 m/s`、`wz=0`，最多 3 秒仿真时间、30 秒墙钟或
3 cm 轮里程计位移，结束后清零并锁车。该轮有 367 个采样，3 秒仿真期间没有误锁车，
实际收纳始终有效；最终锁车、零速度确认。右后轮角度仅从约 `-0.8561` 变为
`-0.8242 rad`，末段与目标 `-0.4988 rad` 仍差约 `0.3254 rad`，因此轮速继续
按原转向对齐阈值衰减。该结果只证明心跳修复在本次运行有效，尚不证明转向或采摘通过。

原生观察器 `v8_r2_repaired_native_b.jsonl` 必须具有版本
`steering_v8_axis_limits_contact_stamps`；旧同名插件可能被运行中的加载器缓存。
另需注意，Gazebo Physics 在 Update 阶段消费并清零 `JointVelocityCmd`、
`JointForceCmd`，因此 PostUpdate 的零值不能证明执行端没有收到命令。
转向限幅与受力判断须结合物理步进前的命令、步进后的实际关节反馈及反作用力矩。
没有关闭停止保护、提高转向误差阈值或把本轮位移记为首站导航成功。

### 10.6 独立右后轮理想转向补偿（演示可选）

`agri_harvest_sim/ideal_r2_steering.py` 是单独运行的仿真补偿程序，默认不启动。
它监听原有 `/steering_controller/commands`，按原控制器的关节顺序取
`r2_steer_joint` 的目标角度，单位为 rad。Gazebo 插件在物理步进前直接设置
该关节的位置，并清零该关节当步原生速度指令，避免原 SERVO 把角度拉回。
轮胎滚动、其他三个转向关节、机械臂和车体位姿继续由原仿真驱动；
`/joint_states`、TF、轮里程计仍读取 Gazebo 实际状态。

该模式绕过右后转向执行器的力矩与轮胎转向摩擦限制。原上游转向速度、
加速度和关节限位保留，目标角不会越限；转向对齐、碰撞检查、命令超时、
停车作业互锁也保留。直接设置角度产生的轮胎接触扰动仍可能影响底盘，
不能据此验证真实电机的转向能力。该关节的物理速度读数接近零，也不表示
理想位置设置期间角度没有变化；应对照位置随时间的变化。

构建（终端一）：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cd sim_ws
colcon build --packages-select agri_harvest_sim --symlink-install --parallel-workers 2
```

先启动仿真及原有控制节点，并确保底盘锁车停稳、机械臂实测收纳。
补偿程序会核对实际控制器为 `GazeboSimSystem`、原转向发布者唯一、
关节顺序正确、当前目标与右后轮实际角度差不超过 `0.03 rad`。
启动不匹配时应先保持停车，排查原控制器状态，不能靠覆盖反馈通过检查。

在新的终端启用（以下域与分区对应本次仍运行的隔离测试仿真；新仿真时
必须与启动仿真的终端使用相同值，不能只在补偿终端更改分区）：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=185
export GZ_PARTITION=agri_close_harvest_root_20261014_v5

ros2 run agri_harvest_sim ideal_r2_steering.py \
  --gz-partition "$GZ_PARTITION" \
  --world-name field --robot-name agri_robot --load-system
```

`--load-system` 显式允许向该 Gazebo world 加入理想转向插件。插件已经存在
且身份正确时会复用；去掉该选项则只允许连接已有插件。每次 Gazebo 重启后
插件也随 world 退出，下次需要重新显式加载。脚本不发送导航目标，不自动解锁。

在相同 ROS 域的终端检查状态：

```bash
ros2 topic echo /ideal_r2_steering_compensator/status --once
```

可运行条件为 `armed=true`、`fault=false`，且 `native.active=true`、
`native.joint_name=r2_steer_joint`。它只对收到的新指令续约：源数组没有消息头，
因此补偿时间戳为真实 ROS 接收时间，不能冒称原始控制器采样时间。
200 ms 仿真时间或 500 ms 墙钟租约过期后，原生插件停止理想设置，
由原物理执行器继续接管。无效、越限、断流、时间回退或发布者变化会停发补偿并请求锁车。

机械臂作业期间，如果底盘已锁车停稳，脚本挂起补偿，状态显示
`suspended_for_arm_work=true`；实测收纳恢复后，仍需保持停车并收到新的
连续目标，才重新启用。不能在 `awaiting_continuous_reclaim=true` 时解锁。
异常已进入 `fault=true` 时需要排查并重新运行脚本，不会自行撤销故障。

按 `Ctrl+C` 正常退出会发出锁车请求、停止续约；原生补偿租约失效后停止设置角度。
强制杀死进程不能保证发出锁车请求，但原生租约仍会失效，原控制与保护继续工作。
完整停用后的状态可在同一 Gazebo 分区中读取：

```bash
gz service -s /world/field/ideal_steering/status \
  --reqtype gz.msgs.StringMsg --reptype gz.msgs.StringMsg \
  --timeout 1000 --req 'data: "{\"op\":\"status\"}"'
```

本次原生边界测试和 76 项 Python 校验通过。真实复测保留全部保护，输入
`vx=0.06 m/s`、`vy=0.03225 m/s`、`wz=0`，在轮里程计超过 3 cm 时开始停车，
另有 3 秒仿真时间、30 秒墙钟终止条件。首次采证未收齐新鲜里程计，未移动且锁停；
重试先等待 DDS 发现完整反馈，记录于 `v10_ideal_r2_manual_retry1.json`，不覆盖首轮。

重试有 156 个 ROS 采样，约 `0.969 s` 仿真时间触发停车。右后轮位置由
`-0.82428` 跟随到约 `-0.51699 rad`；异步 ROS 参考与反馈最大差约 `0.010 rad`，
同一原生状态中的目标与实际位置最大差约 `1.61e-13 rad`。
原生真实车体位姿净移约 `3.11 cm` 后触发停车，最终净移约 `3.35 cm`；
3 cm 是停车触发阈值，原减速流程存在约 3.46 mm 的停止余量，并非绝对位移上限。
5 ms 原生采样间最大位移约 `0.34 mm`，机械臂速度始终远低于收纳阈值。

正常退出验证 `v10_ideal_r2_shutdown.json` 已确认底盘锁车、轮速归零、机械臂收纳，
插件 `active=false` 且设置计数不再增长，原因 `WALL_LEASE_EXPIRED`。
随后不带 `--load-system` 重新启用，成功复用已有插件；
`v10_ideal_r2_reuse_shutdown.json` 再次确认正常退出和锁停，没有重复加载插件。
当前补偿已停用；需要测试时按上面的命令重新启用。
该结果验证了短距离理想转向与退出恢复，尚不代表完整航点、自然果实夹取和落篮通过。

### 10.7 保留原位置的小果实复测

用户选择将全部果实缩小至 50–60 mm，保留自然分布。对应候选位于
`artifacts/close_harvest_20261014/small_fruit_50_60_candidate/`：运行世界为
`tomato_field_harvestable.sdf`，标注为 `tomato_field.json`，完整作业任务为
`harvest_mission_full20.yaml`。不要启动该目录内仅用于离线投影的
`projection_source_static.sdf` 或 `navigation/tomato_field_collision_navigation.sdf`。

转换脚本 `scripts/prepare_small_fruit_candidate.py` 从保存的旧场景逐果转换，
没有重新随机生成。224 个果实的中心、编号、所属植株、成熟度及原挂接位置保持；
直径从原 60–90 mm 区间线性映射到 50–60 mm 区间，实际为 50.026–59.961 mm。
视觉缩放、球碰撞半径和原生采摘插件的半径同步，质量和惯量按原密度缩放。
没有增大夹爪，也没有改变叶片、枝茎或底盘保护。

v16 实例已实际加载该候选：原生场景中 224 个果实的半径与标注逐一匹配，
原位中心误差小于 `1.3e-15 m`，165 个成熟果实均保持原状态；MoveIt 实际规划场景
中的 224 个果实碰撞球也使用相同小半径，没有旧尺寸代理或已挂接物体。
分别记录于 `v16_small_fruit_native_geometry_verification.json` 和
`v16_smallfruit_moveit_scene_readonly.json`。这些是加载一致性证据，不是夹取成功证据。

候选生成方式如下，输出到新的目录，避免覆盖已绑定的复测资产：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
python3 scripts/prepare_small_fruit_candidate.py --help
python3 scripts/prepare_small_fruit_candidate.py \
  --output-dir artifacts/close_harvest_20261014/small_fruit_reproduction
gz sdf -k artifacts/close_harvest_20261014/small_fruit_reproduction/tomato_field_harvestable.sdf
```

脚本输出的 manifest 记录全部旧、新直径及源哈希。仅执行上述转换不会自动生成
导航地图和航点绑定。现有完整候选的绑定与全车体几何检查见
`small_fruit_50_60_candidate/navigation_route_binding_report.json`：20 个作业位姿、
4 个过渡点、返航点和停车微调范围保持，碰撞导航地图从 365 个障碍格减至 302 个，
2,537 个未知格及 keepout 保持。定位扫描切片顶部为 0.859 m，新果实最低下沿
为 0.973 m，叶片和枝茎未变，因此本候选保留原 AMCL 定位地图。

旧大果场景首站已实际到位并完成视觉选果、IK、预抓取和接近轨迹；但选中果实
直径 78.186 mm，大于夹爪约 74.082 mm 的实际张开空间，接近后的实际关节未达到
原 0.03 rad 到位容差，未进入夹紧、挂接和落篮。控制器返回成功不能替代关节到位。
缩小果实只改善几何余量，仍须用新场景的真实相机、接触及落篮反馈验收。
原自然分布中 tomato_35 没有成熟红果，尺寸转换没有更改这一事实。

完整导航复测另发现关节反馈撤证问题：同一时间戳的反馈原先会清空收拢证据。
修复后仅忽略名称、位置、速度完全相同的重复采样，原 SIM/WALL 时效不续约；
同时间戳但数值不同、旧时间戳、异常或过期采样仍立即撤证。JointState 回调改为
独立互斥组，避免多线程本地处理倒序。183 项机械臂、理想补偿与心跳联动回归通过。
旧失败运行未完整保存重复采样的原始数组，不能据此证明该修复覆盖所有发布端问题。

小果实导航测试预定使用实际 Nav2 action 逐点执行 20 个停车点、4 个过渡点及返航。
测试期间生产任务保持待机，导航测试不会发送采摘命令或调用 `complete_current`。
实际机械臂收拢、TF 时效、完整车体、未知格和碰撞保护均保留；原生车体位姿只用于
独立评测到位误差，不用于修正定位或导航目标。导航测试和完整采摘测试的结果分别记录。

第一轮小果实导航复测 `v15_navigation_only_full20/report.json` 未通过完整路线：
前四株 tomato_20–23 和两个过渡点实际到位，第五株 tomato_24 在墙钟期限内未到位。
独立原生位姿对照中，前四株停车位置误差为 7.21、9.98、4.64、4.81 cm；
第二株接近 10 cm 验收边界，不应解读为毫米级停车精度。该轮未发送采摘动作。

第五株停滞的实测记录见 `v15_plant24_steering_nav_readonly_report.json`：
Nav2 每约 0.46 秒仿真时间在横移和斜移之间切换，要求转向角变化约 1.03 rad。
四个转向关节正常跟随限速参考，但通常无法对齐不断变化的最终方向；
234 次轮速输出中 231 次被原 0.35 rad 对齐保护置零。此处没有取消停止保护。
该轮结束后已核实底盘锁停、机械臂实测收纳、所有动作终止，并正常关闭本轮仿真。

下一轮 v16 使用单独的 v8 速度平滑候选，先检查正常指令时间戳、断流、时钟暂停、
碰撞及锁车的实际停止链，再开始全路线测试。该候选仍保留原对齐阈值、
关节速度/加速度限值、未知格、碰撞与停车互锁；验证完成前不设为默认配置。

本轮首站实测停车误差约 7.7 cm，随后扫描部署规划因
`fruit_tomato_20_fruit_3` 与 `arm_link2` 碰撞被拒绝，未完成夹取或落篮。
用户允许果实与机械臂穿模后，正在使用显式仿真演示模式复测，尚未确认采摘成功。

在现有 `ros2 launch agri_arm_control harvest_arm.launch.py` 启动命令中加入
`fruit_arm_pass_through:=true`，保持本轮元数据、ROS 域和 Gazebo 分区参数一致。

机械臂仿真启动中的 `parking_timeout_s` 默认改为 `15.0` 秒墙钟时间。
本轮实测 5 秒内仅推进 0.914 秒仿真时间，停车检查先验证 0.5 秒停止，
再验证 0.5 秒 TF 稳定，原等待预算不足。停车精度仍为 0.10 m / 5°，
两段稳定时间、里程计新鲜度和底盘锁定条件保持原值。
该选项同时启用 MoveIt ACM 许可和原生物理两类碰撞 mask 过滤，范围仅为
224 个果实与 `arm_link1`–`arm_link5`，包含未熟果实；夹爪本体、双指、底盘、
植株主干、篮筐及机器人自身碰撞保持原规则。选中果实的双指接触、挂接及落篮
仍需真实反馈验收，穿透模式不能替代采摘证明。

该参数默认 `false`，仅用于仿真演示。以 `fruit_arm_pass_through:=false` 重新启动
机械臂启动流程可恢复规划中的果实/机械臂碰撞检查；已经应用到现有 world 的
物理 mask 过滤还需正常关闭并重新启动仿真，不能只关闭参数便宣称物理碰撞已恢复。

### 10.8 实际落篮结果与人工可视复核

v16 已从真实视觉目标 `6b9dc1fcf9ae-000133` 夹住 `tomato_20_fruit_2`，完成退臂、
搬运、下降、开爪、释放及原生 `IN_BASKET` 确认，随后六关节实测收纳。
结果为 `artifacts/close_harvest_20261014/v16_held_fruit_recovery_native_bounds/report.json`，
`passed=true`；最大收纳误差 0.00518 rad，原生位置、反馈及果实身份没有重写。
这是首颗果实的实际恢复采摘结果，原生产任务仍保留此前失败记录，不能当作完整
20 航点已验收。该轮测试实例已正常关闭，关闭记录为 `v16_manual_takeover_shutdown.json`。

已修复 MoveIt 附着后的重复世界物体删除，并增加实际场景读回验证。
规划请求仅对运行 URDF 边界 1e-6 rad 内的数值误差修正副本，真实关节反馈不改。
本机 DART 的微小越下限状态曾使腕部无法返回；本轮临时增加原生下限 1e-9 rad
余量，让原驱动退出边界后恢复限位，没有使用机械臂位姿复位或理想驱动。
后续正常规划改为优先选择离限位至少 0.02 rad 的 IK 解，轨迹末点保留 0.01 rad
余量。新姿态选择已通过 25 项边界测试，仍需人工复核新启动的实际采摘。

下面使用同一小果候选，全部果实 50–60 mm、位置不变、没有额外支梗。
首株任务保留完整场景和 20 个停车位定义，只启用 `tomato_20`，完成后回到入口。
完整任务则启用全部 20 站；自然分布中 `tomato_35` 没有成熟红果，当前流程可能在
该站等待后失败，不能把没有成熟目标当作一次采摘成功。

**每个新终端先执行：**

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source scripts/harvest_demo_env.bash
```

此文件统一 source 两个工作空间，设置 `ROS_DOMAIN_ID=185` 和
`GZ_PARTITION=agri_close_harvest_user_20261014`，并定义下列资产变量。
最新 ROS 与仿真包已构建；这轮人工复核无需重新生成场景或地图。

**终端 1：Gazebo、机器人、底盘控制、相机与雷达。**

```bash
ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$AGRI_HARVEST_ASSETS/tomato_field_harvestable.sdf" \
  gz_partition:="$GZ_PARTITION" \
  gui:=true paused:=false rviz:=false headless_rendering:=false \
  physics_step_size:="$AGRI_SIM_STEP_SIZE" real_time_factor:="$AGRI_SIM_TIME_FACTOR" \
  use_control:=true use_kinematics:=true steering_effort_limit:=40.0 \
  wheel_collision_geometry:=cylinder \
  base_adapter_config:="$AGRI_PROJECT_ROOT/ros2_ws/src/agri_base_adapter/config/base_adapter_close_harvest_omni.yaml" \
  require_harvest_interlock:=true \
  use_lidar:=true lidar_mode:=gpu_lidar use_scan:=true \
  use_collision_scan:=true use_camera:=true \
  camera_config:="$AGRI_PROJECT_ROOT/sim_ws/src/agri_sim_sensors/config/d405_extended_sim.yaml"
```

等机器人生成、控制器加载完成，再启动以下模块。默认出生 world 位姿为
`(0,-6,0.40)`、yaw `1.5708`；保存地图中的初始平面位姿为 `(0,0,0)`。

**终端 2：底盘高度的障碍投影。**

```bash
ros2 launch agri_lidar_adapter collision_scan.launch.py use_sim_time:=true
```

**终端 3：AMCL 和 Nav2。**

```bash
ros2 launch agri_navigation navigation.launch.py \
  use_sim_time:=true \
  map:="$AGRI_LOCALIZATION_MAP" keepout_mask:="$AGRI_KEEPOUT_MAP" \
  navigation_map:="$AGRI_HARVEST_ASSETS/navigation/navigation_collision.yaml" \
  obstacle_scan:=/scan_obstacles \
  params_file:="$AGRI_NAV_PARAMS" amcl_params_file:="$AGRI_NAV_PARAMS" \
  velocity_smoothing:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

AMCL 使用原定位地图 `/map`；底盘代价地图使用物理障碍投影 `/navigation_map`。

**终端 4：成熟红果检测和可靠三维目标。**

```bash
ros2 launch agri_perception tomato_detector.launch.py \
  model_path:="$AGRI_PROJECT_ROOT/data/weights/best.pt" device:="$AGRI_DETECTOR_DEVICE" \
  target_frame:=base_footprint mature_only:=true require_segmentation:=true
```

**终端 5：MoveIt、实际机械臂 worker、采摘场景桥和任务。**

```bash
ros2 launch agri_sim_bringup harvest_arm.launch.py \
  mission_file:="$AGRI_HARVEST_ASSETS/harvest_mission_first_station_manual.yaml" \
  field_metadata:="$AGRI_HARVEST_ASSETS/tomato_field.json" \
  field_sdf:="$AGRI_HARVEST_ASSETS/tomato_field_harvestable.sdf" \
  map_yaml:="$AGRI_HARVEST_ASSETS/navigation/navigation_collision.yaml" \
  map_topic:=/navigation_map keepout_yaml:="$AGRI_KEEPOUT_MAP" \
  gz_partition:="$GZ_PARTITION" use_sim_time:=true use_rviz:=true \
  worker_params_file:="$AGRI_ARM_PARAMS" \
  fruit_arm_pass_through:=true parking_timeout_s:=15.0 autostart:=false
```

要跑完整路线，在启动该终端前，将 `mission_file` 的文件名改为
`harvest_mission_full20.yaml`。不要另启 mock worker、临时 worker 或第二套机械臂控制器。

**终端 6：初始化真实控制器持位，然后常驻右后轮补偿。**

```bash
ros2 service call /harvest_arm_worker/initialize_sim_hold std_srvs/srv/Trigger '{}'
ros2 topic echo /harvest_arm_worker/status --once
```

初始化必须返回 `success=True`，worker 应处于 `IDLE` 或 `STOWED`、
`backend_ready=true`、`fault=false`。随后运行并保持该终端：

```bash
ros2 run agri_harvest_sim ideal_r2_steering.py \
  --gz-partition "$GZ_PARTITION" --load-system
```

**终端 7：到入口，然后开始自动采摘任务。**

```bash
ros2 lifecycle get /bt_navigator
ros2 topic echo /ideal_r2_steering_compensator/status --once
```

导航应为 `active [3]`；补偿应为 `armed=true`、`fault=false`、
`awaiting_continuous_reclaim=false`。先从出生点前进到入口：

```bash
ros2 service call /base_motion/set_lock std_srvs/srv/SetBool '{data: false}'
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
```

等 action 返回 `SUCCEEDED`，锁停并启动任务：

```bash
ros2 service call /base_motion/set_lock std_srvs/srv/SetBool '{data: true}'
ros2 service call /field_route_executor/start std_srvs/srv/Trigger '{}'
```

机器人会导航到首株、停车、部署扫描姿态、选择当前成熟红果、夹取、搬运落篮，
实际收纳后返航。完整任务则会继续下一站。无需手动调用 `complete_current`。

**观察采摘：**

```bash
ros2 topic echo /harvest_worker/state
```

另一个同环境终端可监听实际作业结果：

```bash
ros2 topic echo /field_route_executor/work_report
```

真正完成应有 `phase=SUCCESS`、`sim_fruit_verified=true`、`arm_stowed=true`。
Gazebo 中应看到果实离开植株、随夹爪移动、落入篮筐；MoveIt RViz 中观察机械臂运动。
要看相机检测画面，运行 `ros2 run rqt_image_view rqt_image_view`，选择
`/agri_vision/annotated_image`。导航地图可另开：

```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

Map topic 使用 `/map`，Durability 使用 `Transient Local`。

### 10.9 右后轮补偿报里程计反馈超时

`fresh odom feedback missing` 表示补偿节点超过 0.5 秒墙钟时间没有处理到
新的 `/odom`。本轮人工可视测试的只读复查中，32 秒内收到 256 条 `/odom`，
时间戳持续递增、最大接收间隔约 0.432 秒；仿真速度约为实时的 0.16 倍。
这证明复查期间里程计仍在发布，不能据此排除报错瞬间的短暂停顿。

旧补偿脚本在 ROS 回调中同步查询 Gazebo 状态；本机 `gz.transport13` 查询
会持有 Python GIL，250 ms 查询超时也会阻塞其他 Python 线程。慢仿真下
这可能使里程计消息排队并触发反馈超时。周期状态查询改为独立子进程，
ROS 回调只消费查询结果。原来的里程计、关节反馈及指令新鲜度保护保留，
查询失败或超时仍然锁停，不会使用缓存指令续租。

修复已构建安装，90 项目标测试通过。真实只读查询复测在 14 秒内收到
27 次原生状态回复和 120 条里程计，最慢查询约 0.205 秒，主循环最大间隔
约 0.012 秒。记录为 `manual_r2_async_status_readonly.json`，安装核对为
`manual_r2_fix_installation.json`，均位于 `artifacts/close_harvest_20261014/`。
本轮没有重新发送导航目标，完整行驶及采摘继续按人工复核流程执行。

已经进入故障的旧进程不会自动恢复。先在补偿终端按 `Ctrl+C` 退出该节点，
然后在同环境终端取消未完成的导航、锁停底盘并重新启动补偿：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source scripts/harvest_demo_env.bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
ros2 service call /base_motion/set_lock std_srvs/srv/SetBool '{data: true}'
ros2 run agri_harvest_sim ideal_r2_steering.py \
  --gz-partition "$GZ_PARTITION" --load-system
```

另开同环境终端检查 `/ideal_r2_steering_compensator/status`，必须确认
`armed=true`、`fault=false`，再继续第 10.8 节的入口导航流程。
上述操作保留当前 Gazebo 场景、地图、视觉检测及机械臂模块。
