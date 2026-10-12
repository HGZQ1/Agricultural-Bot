# agri_task_manager

`agri_task_manager` 实现默认中央通道的阶段五预设采摘路线。当前任务为通道两侧
`tomato_20..39` 共 20 个植株任务，每株都有独立的底盘停车位。执行器通过 Nav2
`NavigateToPose` 沿左侧 row 2 的列 0→9 前进，在行端经过 3 个纯导航中转点换到右侧，
再沿右侧 row 3 的列 9→0 倒车返回，最后回到 home。到站后锁定底盘，发布带植株
身份、预设位姿、作业侧、扫描/机械臂姿态标识及微调上限的 JSON 事件。

默认路线布局为 `route.layout: independent_side_stops`。停车纵向坐标为
`base_x=plant_map_x+0.17998`，用于补偿机械臂底座相对车体中心的后移；侧向期望为
左侧 `y=+0.40 m`、右侧 `y=-0.40 m`，在期望值附近搜索 ±0.25 m。保存地图与微调
包络校验后，左侧停车 Y 为 0.16–0.26 m，右侧为 -0.38–-0.23 m，底盘原点到
最近占据栅格的距离均至少 0.80 m。此距离不是车身外缘的净距。

上述参数已在 domain 197 完成 20 个采摘站、3 个中转点和 home 的整线及锁后停位
验收。3 次纠偏均重发原目标，27 个 Nav2 成功结果对应 24 个已验证的逻辑到站。
默认场景、地图和 keepout 为 3 m 版本，上述任务已设为默认并重新安装。控制服务、
三个故障探针与无任务路径覆盖的默认入口检查均通过，源码、归档及安装任务哈希
一致。原期望侧偏 0.50 m、最小原点距离 0.60 m 的方案，在舵轮修复后的完整
回归中于第 7 站出现碰撞预测/进度失败并超时。静态 `yaw=0` footprint 可通行不能
保证终点航向修正时矩形车角的扫掠也可通行，因此当前方案收回侧偏并增大转向余量。
机械臂靠近程度与车辆可通行性仍需联合验收。
当前矩形 footprint 最远车角距原点约 0.665 m，0.80 m 的栅格中心距离约束为转向
半径留下约 0.135 m 余量；栅格尺寸和实际定位误差还需在运行中核验。

20 个采摘站、中转点和 home 的预设 `yaw=0`，转弯行驶期间允许改变航向。
中转顺序为 `(8.50,+0.40,0)→(9.00,0,0)→(8.50,-0.40,0)`，插在
`harvest_tomato_39` 之前；home 为 `(0.50,0,0)`。当前侧偏地图为 `206×76`、分辨率
`0.05 m`、原点 `[-0.401,-2.058,0]`，资产保存在
`artifacts/maps/$FIELD_ID/side_stops/`。

Nav2 内部目标判据为平面误差 `0.07 m`、航向误差 `0.05236 rad`（约 3°），Shim 的
退出阈值同为 `0.05236 rad`，用于给减速与锁车留出余量。对外阶段五验收检查
`AT_STATION` 且底盘已锁定后的 map 位姿：相对预设停车点的平面距离误差 ≤0.10 m、
航向误差 ≤0.0873 rad（约 5°）；home 完成并锁定后采用同一标准。Nav2 成功结果与
锁车后的最终停位验收应分别记录。独立真值、IK/FOV 和实际采摘精度由阶段七/八另验。

当前同时关闭 `general_goal_checker.stateful` 和 Shim 内部
`.position_checker.stateful`。前者在首次命中 XY 后不再检查 XY、后者记住位置已到达，
都可能在终点转向或 AMCL 修正后掩盖位置超差。关闭后持续重查终点位置。
仅收紧公差但保留状态记忆时曾出现 0.126 m 超差，因此最终关闭两处状态记忆后重新
验收整条路线的锁车后停位误差。
该点前缀来自当前 Jazzy Shim 在设置插件名之前初始化内部位置检查器的顺序，实际运行
应通过 `ros2 param get /controller_server .position_checker.stateful` 核对，不要假定
其他 Nav2 版本沿用同一名称。机制参考
[SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/simple_goal_checker.cpp)、
[PositionGoalChecker](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/position_goal_checker.cpp)和
[Rotation Shim](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)源码。

底盘舵轮使用有硬限位的 revolute 关节，跨 ±π 时必须按限位内的直接角差选择可行
轮向，不能当作连续关节取短弧。当前 `agri_base_kinematics` 联合 URDF 硬限位、模块
符号/零位和运动学限位选角；轮向相差 π 时反转对应轮速，任一轮无可行方向时全车
驱动轮速为零。该修复解决左侧第 2 站附近提前返航时的换舵受阻，Nav2 参数保持不变；
停车位另按转向扫掠余量调整为上述位姿。更新后需构建该底盘包并重启仿真底盘节点。

完整生成、启动和验收步骤见
[阶段五导航说明](../../../docs/navigation_stage5.md)。

## 绑定和拒绝条件

任务 YAML 不只是坐标列表，它绑定以下资产和几何：

- 田间 `FIELD_ID`、生成 seed、SDF SHA-256 和元数据 JSON SHA-256；
- 保存地图的原始 PGM SHA、ROS OccupancyGrid SHA、尺寸、分辨率、原点和阈值语义；
- keepout PGM SHA、keepout OccupancyGrid SHA 及其地图几何；
- world→map 对齐、整车矩形 footprint、home、通道范围和允许微调包络。

离线验证会检查 home、所有唯一停车位、中转点、每个任务的组合微调包络，以及包含
中转的路线段和各锚点到 home 的直线连接，按半栅格间距采样。占据图、地图外、超量
未知格或 keepout 有任一相交都会失败。实际控制器转弯扫掠仍需要 Gazebo 路线验收。

执行器默认还订阅 transient-local `/map` 和 `/keepout_filter_mask`，逐项核对 frame、
尺寸、分辨率、原点和完整 OccupancyGrid 内容哈希。未收到匹配的两张实时栅格前不会发送
导航目标；任务运行中发生变化会取消活动目标、请求底盘锁并进入失败状态。正式运行保持
`require_live_map:=true` 和 `require_live_keepout:=true`。

底盘安全状态以 transient-local `/base_motion/locked` 为准，而不是以执行器本地缓存为准。
每次锁定或解锁都必须同时收到 `SetBool` 成功响应和匹配的状态反馈；解锁超时、反馈相反
或服务异常时，执行器会取消/使当前目标失效，反复请求补偿锁定，并且只有观察到
`locked=true` 后才进入失败终态。若已经收到
`/navigate_to_pose/_action/status` 样本，`navigator_active_goals` 必须为 0，避免只重启任务
节点后旧目标与新任务并发。全新 Nav2 可能在首个目标前暂时没有状态样本，此时不能仅凭
`navigator_status_seen=false` 证明没有旧目标；无法确认旧目标已结束时，应重启 Nav2 后再
运行任务。

## 构建和正式离线验证

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  agri_base_kinematics agri_navigation agri_task_manager
source install/setup.bash
cd ..

FIELD_ID="rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"
MISSION="$(ros2 pkg prefix --share agri_task_manager)/config/missions/tomato_field_default_single_corridor.yaml"
MAP_DIR="$(ros2 pkg prefix --share agri_navigation)/maps"

ros2 run agri_task_manager validate_field_route "$MISSION" \
  --map-yaml "$MAP_DIR/tomato_field_default.yaml" \
  --keepout-yaml "$MAP_DIR/tomato_field_default_keepout.yaml" \
  --field-metadata "$FIELD_DIR/tomato_field.json" \
  --field-sdf "$FIELD_DIR/tomato_field.sdf"
```

正式验证必须同时传入 `--field-metadata` 和 `--field-sdf`。只省略这两个参数时仍可做
地图几何开发检查，但报告会显示 `field_assets_verified: false`，不能据此宣称当前运行
场景与任务属于同一资产版本。成功的默认报告应包含 20 个 waypoint、20 个 unique stop、
3 个 transit、`selected_rows: [2,3]` 和 `field_assets_verified: true`。

从同版本场地、保存地图与 keepout 重新生成任务时，以下命令给出最终默认任务的
源码输出路径。本文记录的整线、控制与故障验收已通过；自行改变资产后应先输出
独立候选，重新验收后再覆盖默认任务：

```bash
NAV_MAP_DIR="$PWD/ros2_ws/src/agri_navigation/maps"

ros2 run agri_task_manager generate_single_corridor_mission \
  --field-id "$FIELD_ID" \
  --metadata "$FIELD_DIR/tomato_field.json" \
  --sdf "$FIELD_DIR/tomato_field.sdf" \
  --map-yaml "$NAV_MAP_DIR/tomato_field_default.yaml" \
  --keepout-yaml "$NAV_MAP_DIR/tomato_field_default_keepout.yaml" \
  --map-package agri_navigation \
  --map-relative-yaml maps/tomato_field_default.yaml \
  --keepout-relative-yaml maps/tomato_field_default_keepout.yaml \
  --layout independent_side_stops \
  --side-offset-m 0.40 --side-search-m 0.25 \
  --min-obstacle-clearance-m 0.80 \
  --arm-longitudinal-offset-m -0.17998 \
  --headland-forward-m 1.02 --headland-cross-forward-m 0.50 \
  --output "$PWD/ros2_ws/src/agri_task_manager/config/missions/tomato_field_default_single_corridor.yaml"
```

生成后重新构建 `agri_task_manager`，再执行上述正式离线验证。
旧文件未写 `route.layout` 时仍按 `shared_pairs` 处理，保留左右同列共用停车位的严格校验。

## 运行

先启动匹配仿真和 `agri_navigation navigation.launch.py`；该导航入口会发布绑定的
`/map`、`/keepout_filter_mask` 和 `/costmap_filter_info`。然后在同一
`ROS_DOMAIN_ID` 中运行：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash

FIELD_ID="rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"

ros2 launch agri_task_manager field_route.launch.py \
  use_sim_time:=true autostart:=false \
  field_metadata:="$FIELD_DIR/tomato_field.json" \
  field_sdf:="$FIELD_DIR/tomato_field.sdf" \
  require_live_map:=true require_live_keepout:=true \
  require_motion_lock:=true require_station_ack:=true
```

如使用命名空间或非默认接口，可覆盖 `map_topic`、`keepout_topic`、`navigate_action`、
`navigate_status_topic` 和 `motion_lock_service`；默认值分别为 `/map`、
`/keepout_filter_mask`、`/navigate_to_pose`、`/navigate_to_pose/_action/status` 和
`/base_motion/set_lock`。

```bash
ros2 topic echo /base_motion/locked --once \
  --qos-reliability reliable --qos-durability transient_local
timeout 3s ros2 topic echo /navigate_to_pose/_action/status --once \
  --qos-reliability reliable || true

ros2 service call /field_route_executor/start std_srvs/srv/Trigger '{}'
```

执行器发布 transient-local JSON：

- `/field_route_executor/status`
- `/field_route_executor/current_harvest_point`

`status` 中的 `live_map_valid`、`live_keepout_valid`、`base_lock_observed`、
`base_lock_recovery`、`navigator_status_seen` 和 `navigator_active_goals` 是启动和故障排查
的必要字段。正式任务开始前应看到两张栅格为 `true`、`base_lock_recovery=false`，并且
`navigator_active_goals=0`（若已收到 Nav2 状态消息）。不要用 `require_motion_lock:=false` 绕过正式安全链路；该参数
只用于无底盘的离线执行器冒烟测试。

控制服务位于 `/field_route_executor/` 下：`start`、`pause`、`resume`、`skip`、
`cancel`、`return_home` 和 `complete_current`。使用 `require_station_ack:=true` 时，外部作业
协调器必须在每株任务完成后调用 `complete_current`。本路线 20 个 `stop_id` 都不同，
每个到站事件的 `reused_base_pose=false`。旧共享布局仍支持第二株复用同一停车位。

新增 `PARKING_CHECK` 在每个采摘站、中转和 home 的 Nav2 成功且锁车反馈确认后执行。
要求本次锁定后更新的 `map→base_footprint` TF 新鲜、连续稳定至少 0.5 s，位置误差
≤0.10 m、航向误差 ≤5°，才记录 `parked_pose_verified` 并进入 `AT_STATION`、完成
中转或进入 home `COMPLETED`。检查期间作业点为 `active=false`。

稳定后仍超差时记录 `parked_pose_retry` 并解锁重发原目标，最多 3 次；重复纠偏不消耗
采摘序号，不发布重复作业事件，也不提前标记中转完成。纠偏耗尽，或 5 s 内未取得
新鲜稳定的 TF，记录 `parked_pose_failed`，确认底盘锁定后进入 `FAILED`。
默认参数为 `parking_settle_s=0.5`（ROS 时间）、`parking_timeout_s=5.0`（墙钟）、
`parking_tf_max_age_s=0.35`、`parking_position_tolerance_m=0.10`、
`parking_yaw_tolerance_rad=0.0872665`、`parking_max_retries=3`；稳定窗口内允许变化为
`parking_stability_position_m=0.01`、`parking_stability_yaw_rad=0.0174533`。TF 时间戳须
不早于本次检查开始，未来时间最多允许 0.05 s。该检查验证地图位姿稳定；机械臂作业前
实际底盘持续停稳、收纳反馈仍须阶段七验收。新机制已通过 domain 197 整线、控制
服务与故障复测。

`route.transits` 每项包含 `id`、`before_waypoint_id`、`pose` 和可选
`goal_timeout_s`（默认 120 s）。中转只调用导航，不发布采摘事件，也不等待
`complete_current`；`status.current_transit` 给出当前中转，完成后写入
`transit_completed` 事件。中转期间 `skip` 被拒绝，`pause/resume` 会继续尚未完成的
中转；`cancel`、`return_home` 沿用取消活动目标后锁车的流程。

任务日志为 JSON Lines，默认位于 `~/.ros/log/agri_task_manager/`；设置 `ROS_LOG_DIR`
后位于 `$ROS_LOG_DIR/agri_task_manager/`。

## 测试

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select agri_base_kinematics agri_task_manager \
  --python-testing pytest \
  --event-handlers console_direct+
colcon test-result --verbose
```

测试覆盖两种任务布局、独立站位的左右行序、中转引用和取消恢复、路线生成、精确
footprint/栅格相交、地图与 mask 绑定、错误哈希、非法作业侧和不安全路线等契约。
domain 197 的整线及锁后停位验收已通过，`run_id=291028000000`。本轮任务包含
20 株独立停车点、3 个行端中转及 home，共 24 个唯一 `parked_pose_verified`；
3 次同目标纠偏使 Nav2 成功结果总数为 27，没有增加采摘序号或重复作业事件。
采摘站 map 最大位置/航向误差为 0.091774 m / 3.79979°，home 为
0.057565 m / 1.29146°；运行 502.701 s，记录 7392 个倒车指令样本。
所有 `AT_STATION` 均反馈已锁定，终态 `COMPLETED`、底盘锁定、活动目标数 0，
监测未报告不安全锁车状态。原始本轮结果见
[整线运行结果](../../../artifacts/navigation_stage5_20261012/run_side_3m/final_result.json)。
已验收默认任务 SHA-256 为
`443aec43490081c7e1c697a98ffe8b45deb96141e454b2723b229da51ed11a27`。新版控制和
故障复测已通过，默认任务已推广、重新安装并通过重现检查，结果和资产/源码哈希见
[绑定汇总](../../../artifacts/navigation_stage5_20261012/run_side_3m/summary.json)。

旧侧偏方案的舵轮修复后第 7 站碰撞预测/超时记录见
[bounded_side_collision/result.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/bounded_side_collision/result.json)。
当前几何在加入停位检查前，曾出现 home Nav2 成功却稳定后为 0.147 m / 10.13°
的超差，见
[settled_home_pose.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/clearance_first_complete/settled_home_pose.json)。
车角转向余量和锁后停位检查均须保留。

domain 197 的 6 项控制服务检查已通过：暂停时取消目标并锁车、恢复至首株并人工
确认、下一株独立停车及确认、跳过一株、导航中提前返航、导航中取消。本次显式
传入已验收任务的 `mission_file`；推广后另外检查未传该覆盖参数的默认安装入口，
确认同一任务哈希、20 个唯一停车位和 3 个中转，启动后取消可锁车且活动目标数为 0。
结果见
[default_entry_result.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/default_entry_result.json)。
第 2 个左侧站 `(1.88,0.26,0)` 附近返航，Nav2 成功耗时 25.702 s，
锁后停位验证完成耗时 26.798 s，终态锁车且活动目标数为 0。报告见
[default_controls_result.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/default_controls_result.json)，
同目录保留 `default_controls_trace.jsonl`、`default_controls_events.jsonl` 和
`default_controls_timings.json`。

同版本的锁车反馈异常、运行中 keepout 内容变化、导航超时三个故障探针全部通过，
均取消目标并确认锁车后进入 `FAILED`，活动目标数为 0。报告见
[lock_feedback.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/lock_feedback.json)、
[keepout_change.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/keepout_change.json)和
[timeout.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/timeout.json)。
[validation_binding.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/validation_binding.json)
绑定本轮 domain、原任务及超时专用任务哈希、执行源码哈希和报告摘要。旧控制记录
位于 `pre_clearance_controls/`，不用于本轮验收。

构建成功，12 个源码/安装资产的哈希一致。任务管理包通过 91 项检查（89 项功能，
含默认任务重现；2 项包目录 lint），导航包 3 项和底盘运动学包 34 项测试通过，
底盘 flake8 通过，见
[package_validation.json](../../../artifacts/navigation_stage5_20261012/run_side_3m/package_validation.json)。

生成任务的静态 `navigation_status` 仍初始化为 `pending_runtime`，运行验收以匹配
资产哈希的报告为准。重新生成任务后，应重新测试该版本；执行器不会自动改写 YAML。

## 当前边界

`scan_pose_id`、`arm_pose_id` 和 `micro_adjust` 已作为阶段七接口契约发布，但当前包不做
D405 视场验证、CR5 IK、果实识别或真实采摘，也不执行视觉微调。实际底盘持续停稳判定与机械臂
收纳握手属于阶段七；在线 SLAM 自主覆盖建图属于阶段六。当前 profile 只覆盖固定出生点
前方的中央单通道，不代表已经解决完整多行蛇形巡查或重复植株结构下的跨行重定位。
侧偏停车减少了机械臂的伸距需求，果实高度、近侧/远侧位置及工具碰撞仍需实际 IK/FOV
验证，不能仅凭 20 个停车位判定所有果实均可采摘。
本轮 map 与独立 Gazebo 真值的站点最大差异为 0.195671 m / 6.86288°，阶段八绝对
定位精度尚未验收；稀疏真值样本与 Collision Monitor 不能替代接触式碰撞审计。

只读 [可达性粗界审计](../../../artifacts/navigation_stage5_20261012/run_side_3m/arm_reach_bound_audit.json)
按 URDF/STL 给出 `tool0` 极宽松半径上界约 1.05665 m，名义夹爪 `q=0` 的任意几何点
粗界约 1.27957 m。当前 56 个成熟果仅 20 个表面满足后者的必要条件、36 个不满足；
6 株没有满足条件的成熟果，另 1 株没有成熟果。满足粗界不等于 IK、视场或抓取通过。
该审计使用实际车体 world 高度约 0.36 m、机械臂安装高度约 0.815 m；导航 TF 的
`base_footprint z=0` 不能替代实际离地高度。采摘前需校准基座高度、TCP 和夹爪开度，
再挑选通过 IK/FOV 的近侧较低果实。进一步侧移还受通道及转向余量限制，本轮未改
场景或机器人尺寸。
