# agri_task_manager

`agri_task_manager` 实现默认中央通道的阶段五预设采摘路线。当前任务为通道两侧
`tomato_20..39` 共 20 个逻辑植株任务；左右相对植株共享底盘位姿，因此实际有 10 个
停车点。执行器通过 Nav2 `NavigateToPose` 依次到站，锁定底盘，发布带植株身份、预设
位姿、作业侧、扫描/机械臂姿态标识及微调上限的 JSON 事件，最后同朝向倒车返航。

完整生成、启动和验收步骤见
[阶段五导航说明](../../../docs/navigation_stage5.md)。

## 绑定和拒绝条件

任务 YAML 不只是坐标列表，它绑定以下资产和几何：

- 田间 `FIELD_ID`、生成 seed、SDF SHA-256 和元数据 JSON SHA-256；
- 保存地图的原始 PGM SHA、ROS OccupancyGrid SHA、尺寸、分辨率、原点和阈值语义；
- keepout PGM SHA、keepout OccupancyGrid SHA 及其地图几何；
- world→map 对齐、整车矩形 footprint、home、通道范围和允许微调包络。

离线验证会检查 home、所有唯一停车位、每个任务的组合微调包络，以及相邻停车位之间
按半栅格间距采样的路线段。占据图、地图外、超量未知格或 keepout 有任一相交都会失败。

执行器默认还订阅 transient-local `/map` 和 `/keepout_filter_mask`，逐项核对 frame、
尺寸、分辨率、原点和完整 OccupancyGrid 内容哈希。未收到匹配的两张实时栅格前不会发送
导航目标；任务运行中发生变化会取消活动目标、请求底盘锁并进入失败状态。正式运行保持
`require_live_map:=true` 和 `require_live_keepout:=true`。

## 构建和正式离线验证

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  agri_navigation agri_task_manager
source install/setup.bash
cd ..

FIELD_ID="rows6_n10_row2p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42"
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
场景与任务属于同一资产版本。成功的默认报告应包含 20 个 waypoint、10 个 unique stop、
`selected_rows: [2,3]` 和 `field_assets_verified: true`。

## 运行

先启动匹配仿真和 `agri_navigation navigation.launch.py`；该导航入口会发布绑定的
`/map`、`/keepout_filter_mask` 和 `/costmap_filter_info`。然后在同一
`ROS_DOMAIN_ID` 中运行：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash

FIELD_ID="rows6_n10_row2p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"

ros2 launch agri_task_manager field_route.launch.py \
  use_sim_time:=true autostart:=false \
  field_metadata:="$FIELD_DIR/tomato_field.json" \
  field_sdf:="$FIELD_DIR/tomato_field.sdf" \
  require_live_map:=true require_live_keepout:=true \
  require_motion_lock:=true require_station_ack:=true

ros2 service call /field_route_executor/start std_srvs/srv/Trigger '{}'
```

执行器发布 transient-local JSON：

- `/field_route_executor/status`
- `/field_route_executor/current_harvest_point`

控制服务位于 `/field_route_executor/` 下：`start`、`pause`、`resume`、`skip`、
`cancel`、`return_home` 和 `complete_current`。使用 `require_station_ack:=true` 时，外部作业
协调器必须在每株任务完成后调用 `complete_current`。同一 `stop_id` 的第二株会携带
`reused_base_pose=true`，底盘保持锁定且不会重复导航。

任务日志为 JSON Lines，默认位于 `~/.ros/log/agri_task_manager/`；设置 `ROS_LOG_DIR`
后位于 `$ROS_LOG_DIR/agri_task_manager/`。

## 测试

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select agri_task_manager \
  --event-handlers console_direct+
colcon test-result --verbose
```

测试覆盖任务 schema、路线生成、精确 footprint/栅格相交、地图与 mask 绑定、错误哈希、
非法作业侧和不安全路线等离线契约。完整 Gazebo 路线、Nav2 取消时序和底盘锁仍需按阶段
五文档做端到端运行验收。

## 当前边界

`scan_pose_id`、`arm_pose_id` 和 `micro_adjust` 已作为阶段七接口契约发布，但当前包不做
D405 视场验证、CR5 IK、果实识别或真实采摘，也不执行视觉微调。持续停稳判定与机械臂
收纳握手属于阶段七；在线 SLAM 自主覆盖建图属于阶段六。当前 profile 只覆盖固定出生点
前方的中央单通道，不代表已经解决完整多行蛇形巡查或重复植株结构下的跨行重定位。
