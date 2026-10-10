# 导航系统开发流程

更新时间：2026-10-09。用户已明确取消 FAST-LIO，采用
[ysftzc/robot_workspaces](https://github.com/ysftzc/robot_workspaces) 的导航与任务组织作为模板。
本文件同时记录已交付的阶段和后续计划；截至 2026-10-09，阶段二的 MID-360
水平扫描 `/scan`、阶段三 SLAM Toolbox 建图基线和阶段四保存地图 AMCL＋Nav2
单目标导航基线均已接入；参数化巡查路线和导航任务闭环留给后续阶段。
阶段一的底盘、轮式里程计、安全速度出口和标准 `/odom` 适配已实现，并已在番茄田
仿真中完成低速动态基线；弧线的旧固定时长判据仍需按换舵延迟重构。
参考源码评估见 [robot_workspaces_review.md](robot_workspaces_review.md)。

## 1. 确定技术路线

| 层次 | 本项目方案 |
| --- | --- |
| 传感器 | 保留现有 MID-360 PointCloud2，通过过滤与 pointcloud_to_laserscan 生成二维 `/scan` |
| 连续里程计 | `/wheel/odom` 为四舵轮原始输出；阶段一由 `agri_base_adapter` 校验并发布 `/odom`，按测试结果再接入 IMU/robot_localization EKF |
| 建图 | SLAM Toolbox，在线二维占据栅格与回环；保存地图和可继续建图的 pose graph |
| 保存地图定位 | map_server＋AMCL，使用 `/scan` 与连续里程计 |
| 全局规划 | Nav2 NavFn，首版使用 Dijkstra 模式 |
| 局部控制 | Nav2 Regulated Pure Pursuit（RPP） |
| 任务 | Python/rclpy 状态机，通过 NavigateToPose 执行行间路线、扫描站点及返航 |
| 速度出口 | `agri_base_adapter` 对 `/cmd_vel`（遥控）和 `/cmd_vel_nav`（导航）仲裁、限速、超时归零和停车锁，向四舵轮唯一输出 `/cmd_vel_safe` |

只借鉴上游的模块职责和配置组织，使用本项目四舵轮、CR5、MID-360、D405 与参数化田。
不迁入 Panther 差速控制器、FR3 关节姿态、旧地图坐标和 Gazebo 真值定位路径。
SLAM Toolbox 不要求 FAST-LIO 或逐点扫描时间；IMU 也不是其建图硬前提。

## 2. 包和接口边界

| 包 | 计划新增/完善的内容 |
| --- | --- |
| `agri_lidar_adapter` | 点云 TF 变换、自过滤/高度裁剪、可选体素降采样，以及由 Jazzy `pointcloud_to_laserscan` 生成 `/scan` |
| `agri_base_adapter`、`agri_base_kinematics` | 轮式里程计质量、标准 `/odom`、速度仲裁/停车门控、舵轮换向协调 |
| `agri_navigation` | 已交付 SLAM Toolbox 建图、保存地图 AMCL、NavFn＋RPP、footprint、静态/实时障碍代价地图、碰撞监控和互斥模式入口；后续补充路线、启用禁行过滤器及地图管理 |
| `agri_task_manager` | 覆盖/巡查路线、扫描站点、导航 Action 调度、暂停/恢复/跳过/返航 |
| `agri_interfaces` | 后续扫描作业请求、底盘互锁状态、任务结果等业务接口；复用标准 Nav2 Action |
| `agri_sim_sensors` | 若接入 IMU，新增仿真传感器、桥接、安装 frame 与配置 |
| `agri_sim_bringup` | 将导航与任务入口加入现有番茄田启动，隔离 mapping/navigation 两种模式 |
| `agri_sim_tests`、`agri_tests` | 扫描、TF、建图、导航、互锁和故障恢复验收；Gazebo 真值仅供独立评测 |

自研应用节点继续使用 Python 3.12/rclpy。SLAM Toolbox、Nav2、robot_localization 与
pointcloud_to_laserscan 使用 Jazzy 发行组件。`ros2_ws` 不依赖 `sim_ws`。
阶段四的配置与入口已经可以执行；阶段五以后列出的业务节点名称仍是拟定交付。

拟定数据链：

```text
/mid360/points → 自过滤/高度裁剪 → /scan → SLAM Toolbox 或 AMCL
/wheel/odom → agri_base_adapter `/odom`（后续可替换为 EKF）→ Nav2
/map + /scan + TF + /odom → NavFn + RPP → `/cmd_vel_nav` → 速度门控 `/cmd_vel_safe` → 四舵轮
站点路线 → NavigateToPose → 到站复核 → 锁车且停稳 → 扫描作业接口
```

TF 所有权必须随模式明确配置：

| 变换 | 唯一发布者 |
| --- | --- |
| `map → odom`，mapping 模式 | SLAM Toolbox |
| `map → odom`，navigation 模式 | AMCL |
| `odom → base_footprint` | 基线由轮式里程计；启用 EKF 后关闭轮式 TF，由 EKF 发布 |
| 机器人固定与关节 TF | robot_state_publisher |

出生点用于初始化配置。轮式 odom 从零开始，地图坐标由建图建立；固定出生点不意味着
world、map、odom 坐标自动相同。站点在 map 中定义，导入田布局时保存明确对齐配置。

## 3. 按阶段交付

### 阶段 1：底盘与导航输入基线

- 从 2 行少量植株开始，机械臂保持收纳；验证直行、限位内转弯、停车、换舵延迟。
- 核对轮速/舵角反馈、odom 速度和航向、TF 时间与唯一发布者。明确协方差及 `/odom`。
- 统一仿真时钟和 TwistStamped；建立导航/遥控速度仲裁与最终出口，测试超时归零。
- 初始调试建议线速 0.1 m/s、角速 0.2 rad/s，随后根据制动、换舵和行间测试调限。

**验收：** 指令、反馈及运动方向一致，零速后持续停稳，TF 无重复发布或时间跳变。
原地转向/横移只有实测通过后才加入导航能力，不能直接复制上游的 spin 恢复。

**当前实现状态（2026-10-08）：** `agri_base_adapter` 已加入 ROS 2 工作空间并接入
`simulation.launch.py`。门控默认限制为线速度 0.30 m/s、横向速度 0、角速度 0.50 rad/s；
输入断流 0.35 s 后归零，四舵轮节点保留 0.5 s 低层看门狗。节点拒绝过期/未来/错误
frame/非有限 `TwistStamped`，锁车服务为 `/base_motion/set_lock`，锁定和解锁都会清除
缓存。四舵轮节点要求同一 `JointState` 同时包含四个舵角和四个轮速，使用实测 CAD 零偏，
发布带协方差的 `/wheel/odom`；适配器将其校验后发布 `/odom`。当前 TF 唯一所有者仍是
四舵轮节点，启用 EKF 前不能再启动第二个 `odom → base_footprint` 发布者。

独立域 196 的回归中，直行/倒车、稳态左右弧、里程计、锁车接口和断流停车通过；换舵
释放延迟约 0.359–0.532 s。旧测试将换舵等待包含在固定 3 s 弧线目标内，右弧端到端
误差仍超 3°，因此阶段一后续应把导航验收改为“先确认舵轮到位，再计稳态窗口”，并
补充节点退出/控制器低层 watchdog 验证。报告中的 `transition_aware_passed=true` 表示
稳态基线通过，不代表完整导航已经验收：
`artifacts/chassis_stage1_20261008/report_transition_aware.json`。

### 阶段 2：MID-360 转二维扫描

- 使用现有 `/mid360/points`；增加车体自过滤、量程与导航高度参数，接入
  [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan)。
- 新增水平虚拟扫描 frame，原点保持在 MID-360 扫描中心，XY 平面与底盘导航平面平行。
  通过 TF 处理雷达的安装俯仰；不直接重命名倾斜点云，也不把扫描原点移到 base_link。
- 高度过滤以明确的底盘/扫描坐标定义，实际观察地面、茎杆、叶片、机器人和行末结构。
- 匹配 QoS、时间戳、角分辨率、scan_time、range 与空角度处理。缺失观测不能直接
  当作整段空闲空间；核对无返回/被遮挡角度的处理和代价地图射线清除。

**当前实现（2026-10-08）：** `ros2_ws/src/agri_lidar_adapter` 已发布
`/mid360/navigation_points`，并启动 Jazzy 官方 `pointcloud_to_laserscan` 发布 `/scan`。
新增 `mid360_scan_frame` 固定 frame：原点与 `mid360_sensor_frame` 相同，+15° 固定俯仰
抵消雷达 −15° 安装俯仰。默认高度切片为扫描 frame 中 `-0.40..0.40 m`，量程
`0.10..40 m`，角分辨率 1°；`self_filter_enabled` 和 `voxel_size` 默认关闭，便于先
审计原始障碍。过滤节点保留点云 stamp、字段和大小端，缺失 TF 的帧丢弃并限频告警。
标准转换器采用 SensorDataQoS，并按 `/scan` 订阅者懒启动点云订阅。

番茄田实测命令（无 GUI）为：

```bash
ros2 launch agri_sim_bringup tomato_field.launch.py \
  gui:=false paused:=false rviz:=false \
  use_control:=false use_kinematics:=false use_lidar:=true use_scan:=true use_camera:=false
ros2 run agri_sim_tests check_scan --duration 8 --timeout 60
```

本次报告：81 帧、10 Hz、`mid360_scan_frame`，扫描原点与雷达原点误差小于 1 cm，
水平姿态误差小于 1°，有限回波 12,879 个，退出码 0。`pointcloud_to_laserscan`
在 360°/1° 配置下按上边界排除输出 360 个 bin，验收器兼容该标准行为。

**验收：** RViz 中扫描与障碍物对应，扫描原点正确；机器人转向时扫描稳定，无明显
地面/车体伪障碍。先用箱体和单行测试，再用完整植株高度/冠幅场景测试。

### 阶段 3：SLAM Toolbox 建图基线

- `ros2_ws/src/agri_navigation` 已新增 `slam_toolbox.yaml` 和
  `mapping.launch.py`。入口只启动 SLAM Toolbox synchronous mapping lifecycle node，
  订阅 `/scan`，由 SLAM 发布 `map → odom`；不会同时启动 AMCL、`map_server`、Nav2 或
  Gazebo，避免同一 TF 边重复发布。
- 默认契约为 `map → odom → base_footprint`，地图分辨率 0.05 m，扫描量程
  0.10–40 m，最小运动阈值 0.15 m/0.15 rad，并启用回环检测。轮式节点仍是
  `odom → base_footprint` 的唯一发布者。
- 先低速手动采集一条行间及行末闭合路径，检查地图拖影、重复行、错误回环和返回误差；
  通过 `agri_sim_tests check_mapping` 同时检查 `/map`、`/scan`、`/odom`、TF 和时间戳。
- 用 `nav2_map_server map_saver_cli` 保存 YAML/PGM，用
  `/slam_toolbox/serialize_map` 保存可继续建图的 pose graph；将生成的田间 SDF、
  随机种子、扫描配置、地图分辨率/原点和验收报告绑定为同一地图版本。

当前可复现入口（先在另一终端启动番茄田和 `/scan`）：

```bash
ros2 launch agri_navigation mapping.launch.py
ros2 run agri_sim_tests check_mapping --duration 5 --timeout 90
```

2026-10-09 的无界面闭环验证中，扫描约 10 Hz、里程计约 50 Hz，`/map` 使用 0.05 m
分辨率；机器人沿行间低速移动后，已知栅格从初始稀疏观测增加到 2,305 个，地图、TF
和时间戳检查通过。保存地图与 pose graph 的命令也已实测成功。

**验收：** 当前完成的是传感器、TF、地图发布和保存接口基线；保存并重载地图的完整
AMCL/Nav2 验收、通道连续性和回到入口误差属于阶段四。手动建图只完成接口基线；
自主建图还需要阶段六的覆盖执行器。

### 阶段 4：AMCL 与单目标 Nav2

- 新建 `amcl.yaml`、`nav2.yaml` 和保存地图导航入口；关闭 SLAM，加载保存地图。
- 固定出生点映射为 map 中的初始位姿；用受控 initialpose 初始化，验证定位与协方差。
- 使用 NavFn＋RPP，按四舵轮实测能力配置转弯/旋转、速度/加速度、lookahead 和恢复。
- 测量机械臂收纳状态整机多边形 footprint，配置膨胀、障碍/voxel 层、有效高度和范围。
- Nav2 所有速度相关节点启用 stamped 消息；实际启用并测试碰撞监控，而不是照搬
  对方默认关闭的监控输入。初始恢复限制为已验收的动作。

**验收：** 先 RViz 单点，再直线、转弯、行末掉头和临时障碍测试；目标取消、输入超时
都能停车。定位未收敛或跳变时停止任务，不能不断重发导航目标掩盖错误。

**当前实现状态（2026-10-09）：** `localization.launch.py` 提供 map server＋AMCL，
`navigation.launch.py` 用同一 lifecycle manager 先激活定位，再依次激活 RPP、NavFn、
behaviors、BT Navigator 与 collision monitor。全局代价地图启用静态、LaserScan 障碍和
膨胀层，局部代价地图启用实时障碍和膨胀层；速度链接入现有底盘门控。启动、发目标及
布局变化后的地图重建命令见 [阶段四导航说明](navigation_stage4.md)。参数化多站点、
KeepoutFilter 服务器与停车作业状态机尚未纳入此入口。

### 阶段 5：参数化田的巡查与扫描路线

- 新建 `waypoints.yaml` 与 `field_route_executor.py`：定义地图版本、地图/布局校验值、
  home、行入口/出口、站点 `[x,y,yaw]`、`row_id`、`side`、作业侧、扫描姿态标识和
  允许微调区域。站点的坐标是 **map 坐标**，不能直接复制 Gazebo world 坐标。
- 可利用生成器的行数、株距和行距作为布局先验，经明确 world→map 对齐后生成候选站点；
  例如当前生成器默认各行沿世界 Y 方向延伸、行中心沿 X 方向相隔 2.0 m，候选底盘
  停车点应位于行中心外侧并扣除底盘半宽与安全裕量，再经过 footprint、代价地图和
  CR5 IK/FOV 核验。果实真值不作为站点或在线采摘目标。
- 站点生成器必须记录输入布局（行距、株距、冠幅、底盘 footprint、对齐变换）和输出
  seed；只改变果实数量/高度不必移动底盘站点，改变行距、冠幅或株高导致通道变化时
  必须重新生成并重新验证站点。
- 参考上游预定义路线，先实现单行往返，再相邻两行、完整蛇形巡查和返航。
- 增加暂停、继续、跳过、取消、超时和任务日志。生成与地图同尺寸/分辨率/原点的植株行
  禁行遮罩，避免规划器穿行植株间缝隙；遮罩版本与地图绑定。

**验收：** 自动完成多站点巡查并返航，路径保持在通道内；行数/行距/冠幅等变化使旧
地图和旧路线失效，重新建图/验证。单独改变果实数量或高度时也检查导航层是否受影响。

### 阶段 6：自主覆盖建图

- 在 mapping 模式接入阶段 5 的覆盖调度结构，使用在线 `/map` 的 Nav2 栈，走行入口、
  行间和行末；不启动 AMCL，不读取 Gazebo 位姿持续修正 SLAM。
- 首版按已知温室行列拓扑自动覆盖，不扩展为完全未知环境的 frontier 探索。
- 初始地图不完整时分段推进，在可观测通道内更新目标；明确未知区域、地图边界、
  阻塞等待/跳过和覆盖完成判据，不能把未知格一概当作可行驶空间。
- 路线结束后自动保存地图/pose graph/路线对齐与覆盖报告，测试中断恢复。

**验收：** 无人工遥控完成约定行间覆盖并保存地图；重启切换 navigation 模式后仍可
执行巡查。手动采包或仅启动 SLAM 节点不算完成自主建图。

### 阶段 7：采摘停车接口

- 任务状态定义为 `NAVIGATING → STOPPING → BASE_LOCKED → SCANNING → RESUMING`，
  同时定义暂停和错误恢复。扫描/采摘阶段通过正式接口接入，当前先验证模拟作业请求。
- 到站后取消/结束活动导航目标，屏蔽导航和遥控速度源，在最终出口持续保持零速。
- 确认反馈 `|v| < 0.01 m/s`、`|w| < 0.02 rad/s` 连续 0.5 s，且定位、TF、控制器健康，
  才允许机械臂离开收纳位。展开期间持续监测速度，超限则停止机械臂并保持底盘锁。
- 作业结束须收到机械臂已收纳的反馈后解锁；先完成停车握手，不在本轮导航里实现
  YOLO、IK 和真实采摘。站点偏置与间距在后续视觉/机械臂可达性验收中确定。

**验收：** Nav2 success 后实际停稳才触发作业；暂停、任务异常、超时、遥控输入和
作业进程失联不会使已展开机械臂期间的底盘移动。

### 阶段 8：联合验收与运行入口

- 交付 `mapping`、`navigation`、`field_mission` 三种清晰入口与独立环境配置。
- 在当前默认 6 行×10 株、行距 2.0 m、株高 2.0 m、冠幅 1.0 m 的场景完成巡查；联合保留 MID-360
  和 D405，记录实时率、导航延迟及资源使用，而不是只验收空场景。
- 按 TC-NAV 验证 10 个固定目标，每目标超时 120 s，位置误差 ≤0.10 m、航向误差
  ≤5°，至少 9/10 成功，禁止碰撞为 0；另验收自主覆盖建图、返航与停车互锁。
- 保存 rosbag、地图/布局版本、seed、目标结果、轨迹、真值误差和恢复次数。换布局、
  随机种子或障碍条件后重复必要测试；真值只进入评测端。

## 4. 现在首先做什么

阶段一底盘/里程计、阶段二 MID-360 二维 `/scan`、阶段三 SLAM Toolbox 建图接口和
阶段四 AMCL＋Nav2 单目标基线已经完成。下一里程碑是阶段五：依据参数化田布局和
world→map 对齐生成巡查路线、扫描站点与版本绑定的禁行遮罩。
IMU/EKF 可并行准备，启用时切换 odom TF 发布者；不能让两套节点同时发布同一变换。

## 5. 官方参考

- [Nav2 Jazzy 建图与定位](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)
- [Nav2 Jazzy 里程计融合](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/)
- [Nav2 Jazzy RPP 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [Nav2 Jazzy AMCL 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)
- [Nav2 Jazzy Controller Server 与 stamped 速度](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/)
