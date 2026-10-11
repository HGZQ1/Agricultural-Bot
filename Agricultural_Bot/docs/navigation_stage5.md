# 阶段五：中央单通道预设采摘路线

本文给出当前默认番茄田的阶段五操作流程。任务范围是固定出生点前方的中央通道：
机器人依次到达 10 个底盘停车位，为通道两侧 20 株植株各发布一次逻辑采摘任务，最后
保持 `yaw=0` 沿原通道倒车回到 home。本文中的 `left` 和 `right` 都以机器人到站后的
`base_footprint` 为参考，不是地图坐标轴方向的别名。

阶段五使用保存地图导航，不启动 SLAM Toolbox。任务文件同时绑定田间 SDF/元数据、
占据地图、keepout mask、world→map 对齐、底盘 footprint 和停车点。执行器启动前做
离线绑定校验，运行前还要求实际发布的 `/map` 与 `/keepout_filter_mask` 内容完全匹配；
运行中任一内容改变会取消活动目标并进入锁车失败状态。

## 1. 默认任务范围

当前默认场景为：

```text
FIELD_ID=rows6_n10_row2p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42
6 行 × 10 株，行距 2.0 m，株距 0.7 m
中央通道两侧：row_id=2 和 row_id=3（从 0 开始编号）
home=(0.50, 0.00, 0.00)，坐标系为 map
```

建图从 Gazebo world 位姿 `(0,-6,π/2)` 开始，因此本地图采用：

```text
map = [6, 0] + R(-π/2) × world
```

每株植株都有独立 `waypoint_id`、植株身份、作业侧、扫描姿态标识和机械臂姿态标识。
同一纵向位置的左右植株共享 `stop_id` 和 `base_pose`：底盘只导航一次，在保持锁车的
情况下依次发布两次逻辑作业事件。

| 停车位 | 底盘预设位姿 `[x,y,yaw]` / m,rad | 左侧任务 | 右侧任务 |
| --- | --- | --- | --- |
| `stop_00` | `[1.00, 0.07, 0]` | `tomato_20` | `tomato_30` |
| `stop_01` | `[1.70, 0.05, 0]` | `tomato_21` | `tomato_31` |
| `stop_02` | `[2.40, 0.02, 0]` | `tomato_22` | `tomato_32` |
| `stop_03` | `[3.10, -0.01, 0]` | `tomato_23` | `tomato_33` |
| `stop_04` | `[3.80, -0.04, 0]` | `tomato_24` | `tomato_34` |
| `stop_05` | `[4.50, -0.08, 0]` | `tomato_25` | `tomato_35` |
| `stop_06` | `[5.20, -0.08, 0]` | `tomato_26` | `tomato_36` |
| `stop_07` | `[5.90, -0.16, 0]` | `tomato_27` | `tomato_37` |
| `stop_08` | `[6.60, -0.23, 0]` | `tomato_28` | `tomato_38` |
| `stop_09` | `[7.30, -0.30, 0]` | `tomato_29` | `tomato_39` |

这些 Y 坐标来自最终 PGM 中实际可通行区域。保存地图后半段存在轻微弯曲，不能把所有
停车点简单改成 `y=0`。

默认任务文件位于：

```text
ros2_ws/src/agri_task_manager/config/missions/
  tomato_field_default_single_corridor.yaml
```

其中每个任务包含：

- `base_pose`：底盘在 `map` 中的预设位姿；
- `plant`：植株 ID、行号、行内索引及 world/map 固定位姿；
- `work_side`：到站车体坐标系中的 `left` 或 `right`；
- `scan_pose_id`、`arm_pose_id`：阶段七使用的作业接口标识；
- `micro_adjust`：允许后续视觉作业请求的纵向、横向和航向微调上限；
- `goal_timeout_s`、`station_dwell_s`、`enabled` 与离线验证状态。

默认允许微调范围为纵向 ±0.05 m、横向 ±0.025 m、航向 ±0.025 rad。生成器会对整个
组合微调包络做 footprint、未知栅格和 keepout 校验。执行器目前只发布这些允许量，不会
自行根据相机结果移动底盘。

## 2. 构建

修改任务、地图、keepout 或启动文件后重新构建：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash

cd ros2_ws
colcon build --symlink-install --packages-select \
  agri_navigation agri_task_manager
source install/setup.bash
cd ..

cd sim_ws
colcon build --symlink-install --packages-select \
  agri_greenhouse_worlds agri_sim_description agri_sim_bringup
source install/setup.bash
cd ..
```

## 3. 从场地与地图重新生成 keepout 和任务

如果当前默认资产没有改变，可以直接跳到第 4 节。以下流程用于重新建图、改变田间参数
或更换地图后更新默认任务。命令会覆盖源码树内的默认 keepout 和 mission，应先确认
`FIELD_ID`、SDF、JSON 和保存地图确实属于同一版本。

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash

FIELD_ID="rows6_n10_row2p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"
NAV_MAP_DIR="$PWD/ros2_ws/src/agri_navigation/maps"
MISSION="$PWD/ros2_ws/src/agri_task_manager/config/missions/tomato_field_default_single_corridor.yaml"

test -s "$FIELD_DIR/tomato_field.sdf"
test -s "$FIELD_DIR/tomato_field.json"
test -s "$NAV_MAP_DIR/tomato_field_default.yaml"
test -s "$NAV_MAP_DIR/tomato_field_default.pgm"
```

先生成与保存地图尺寸、分辨率和原点完全一致的 keepout mask：

```bash
python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_keepout_mask.py \
  --metadata "$FIELD_DIR/tomato_field.json" \
  --map-yaml "$NAV_MAP_DIR/tomato_field_default.yaml" \
  --output "$NAV_MAP_DIR/tomato_field_default_keepout" \
  --robot-start-world 0.0 -6.0 1.570796326795 \
  --rows 6 --plants-per-row 10 \
  --row-spacing 2.0 --plant-spacing 0.7 \
  --plant-width 0.06 --keepout-margin 0.15
```

`--plant-width 0.06` 对应当前简化物理碰撞宽度，默认 row-band 会把每条植株行连接成
连续禁行区域，防止全局规划穿过株间缝隙。它不是 1.0 m 的可视冠幅；若改用冠幅生成
mask，必须先重新核算中央通道是否仍容纳整车 footprint。

再根据同一地图和 keepout 生成 20 个逻辑任务：

```bash
ros2 run agri_task_manager generate_single_corridor_mission \
  --field-id "$FIELD_ID" \
  --metadata "$FIELD_DIR/tomato_field.json" \
  --sdf "$FIELD_DIR/tomato_field.sdf" \
  --map-yaml "$NAV_MAP_DIR/tomato_field_default.yaml" \
  --keepout-yaml "$NAV_MAP_DIR/tomato_field_default_keepout.yaml" \
  --map-package agri_navigation \
  --map-relative-yaml maps/tomato_field_default.yaml \
  --keepout-relative-yaml maps/tomato_field_default_keepout.yaml \
  --robot-start 0.0 -6.0 1.570796326795 \
  --corridor-world-x 0.0 \
  --home-x 0.50 --home-y 0.00 \
  --front-m 0.495 --rear-m 0.535 --half-width-m 0.395 \
  --lateral-search-m 0.40 --search-step-m 0.01 \
  --max-unknown-cells 8 \
  --goal-timeout-s 120 --station-dwell-s 1.0 \
  --micro-longitudinal-m 0.05 \
  --micro-lateral-m 0.025 --micro-yaw-rad 0.025 \
  --output "$MISSION"
```

生成器会选择 `world x=0` 两侧最近的两行，按株索引形成 10 对任务，并从最终占据图的
自由区域搜索停车点。它同时检查 home、全部停车位、微调包络及相邻停车位间的密集直线
采样；任一点触碰占据栅格、地图外、超量未知栅格或 keepout 都会拒绝生成。

生成完成后重新安装被更新的资产：

```bash
cd ros2_ws
colcon build --symlink-install --packages-select \
  agri_navigation agri_task_manager
source install/setup.bash
cd ..
```

## 4. 离线正式验证

验证时显式传入场地资产，才能把任务文件中记录的 SDF/JSON SHA-256 与实际文件一起
核对：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

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

成功报告应至少包含：

```text
passed: true
field_assets_verified: true
waypoint_count: 20
unique_stop_count: 10
selected_rows: [2, 3]
excluded_plant_count: 40
```

每个 `stops` 项的 `occupied_cells` 和 `outside_cells` 应为 0，未知格数不得超过 8。
校验还覆盖 PGM 原始 SHA、转换后的 OccupancyGrid SHA、地图语义、尺寸/分辨率/原点、
keepout SHA 和地图几何。不要通过手改任务中的哈希来绕过失配。

运行包测试：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select agri_task_manager \
  --event-handlers console_direct+
colcon test-result --verbose
```

## 5. 三终端启动

所有终端必须使用相同 `ROS_DOMAIN_ID`。启动前关闭旧 Gazebo、SLAM、AMCL、Nav2、任务
执行器和遥控节点；每次仿真使用新的 `GZ_PARTITION`，避免旧 `/clock` 混入。

### 终端一：启动匹配场景

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96
export GZ_PARTITION="agri_stage5_${ROS_DOMAIN_ID}_$(date +%s%N)"

FIELD_ID="rows6_n10_row2p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"

ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$FIELD_DIR/tomato_field.sdf" \
  gz_partition:="$GZ_PARTITION" \
  spawn_x:=0.0 spawn_y:=-6.0 spawn_yaw:=1.5708 \
  gui:=true paused:=false rviz:=true \
  use_control:=true use_kinematics:=true \
  use_lidar:=true use_scan:=true use_camera:=false
```

### 终端二：启动保存地图导航和 keepout

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

ros2 launch agri_navigation navigation.launch.py \
  use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

这个入口同时启动 `map_server`、AMCL、Nav2、`filter_mask_server` 和
`costmap_filter_info_server`。全局和局部代价地图都启用 KeepoutFilter。不要同时启动
`mapping.launch.py`。

检查导航与 mask：

```bash
for node in map_server amcl filter_mask_server costmap_filter_info_server \
  controller_server planner_server behavior_server bt_navigator collision_monitor; do
  ros2 lifecycle get "/$node"
done

ros2 action info /navigate_to_pose
ros2 topic echo /map --once --qos-durability transient_local
ros2 topic echo /keepout_filter_mask --once --qos-durability transient_local
```

九个生命周期节点都应为 `active [3]`，Action 应显示一个 server。

### 终端三：启动任务执行器

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

FIELD_ID="rows6_n10_row2p0_plant0p7_h2p4_w1p0_c0p06_fh1p0p1p7_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"

ros2 launch agri_task_manager field_route.launch.py \
  use_sim_time:=true autostart:=false \
  field_metadata:="$FIELD_DIR/tomato_field.json" \
  field_sdf:="$FIELD_DIR/tomato_field.sdf" \
  require_live_map:=true require_live_keepout:=true \
  require_motion_lock:=true require_station_ack:=true
```

`require_station_ack:=true` 用于逐株检查，每个作业点都需要显式完成。只验证自动路线时
可以改为 `false`，执行器会在 `station_dwell_s` 后自动完成当前逻辑任务。

另开一个已加载相同环境的终端查看状态：

```bash
ros2 topic echo /field_route_executor/status \
  --qos-durability transient_local

ros2 topic echo /field_route_executor/current_harvest_point \
  --qos-durability transient_local
```

`status` 中开始任务前必须出现：

```text
live_map_valid: true
live_keepout_valid: true
```

两者之一为 false 时不要绕过检查启动。检查 Nav2、map server、mask server、任务 YAML
和当前 ROS 域是否属于同一次构建和同一地图版本。

## 6. 执行和控制服务

启动任务：

```bash
ros2 service call /field_route_executor/start std_srvs/srv/Trigger '{}'
```

当状态进入 `AT_STATION` 时，`current_harvest_point` 会发布 JSON，其中包含：

```text
active=true, run_id, waypoint_id, stop_id, plant_id, row_id,
index_in_row, base_pose, plant_pose_map, work_side,
scan_pose_id, arm_pose_id, micro_adjust, reused_base_pose
```

人工确认该植株任务完成：

```bash
ros2 service call /field_route_executor/complete_current \
  std_srvs/srv/Trigger '{}'
```

同一 `stop_id` 的第二株会立即发布，`reused_base_pose=true`，底盘保持锁定且不会再次发送
导航目标。第二株完成后才解锁前往下一停车位。完成 20 个逻辑任务后，执行器发送 home
目标；到达 `(0.50,0.00,0)` 后重新锁车并进入 `COMPLETED`。

其他服务：

```bash
# 暂停；活动导航会先取消目标并锁车
ros2 service call /field_route_executor/pause std_srvs/srv/Trigger '{}'

# 从当前逻辑任务继续
ros2 service call /field_route_executor/resume std_srvs/srv/Trigger '{}'

# 跳过当前植株任务
ros2 service call /field_route_executor/skip std_srvs/srv/Trigger '{}'

# 取消整项任务并保持锁车
ros2 service call /field_route_executor/cancel std_srvs/srv/Trigger '{}'

# 取消当前导航/作业并沿同一通道返航
ros2 service call /field_route_executor/return_home std_srvs/srv/Trigger '{}'
```

任务事件写入 `$ROS_LOG_DIR/agri_task_manager/`；未设置该变量时写入
`~/.ros/log/agri_task_manager/`。日志为 JSON Lines，`status` 中的 `log_path` 是本次文件。
任务开始时会生成新的 `run_id`。作业完成、跳过、返航、终态或新任务开始时，
`current_harvest_point` 会发布 `active=false` 的 tombstone，晚加入的订阅者不能把旧作业点
误认为当前活动任务。

## 7. 完整验收过程

先做一次无人工作业的全路线测试：

1. 使用 `require_station_ack:=false` 启动执行器；
2. 确认 `/map` 和 `/keepout_filter_mask` 实时绑定均为 true；
3. 调用 `start`，在 RViz 观察规划路径始终沿中央通道；
4. 检查 20 个 `at_harvest_point` 事件只对应 10 次物理停车；
5. 确认每对事件依次为左、右，第二个事件 `reused_base_pose=true`；
6. 确认行末不原地掉头，返程线速度为负、车头保持 `yaw≈0`；
7. 最终到达 home、底盘锁定、状态为 `COMPLETED`，全程无碰撞、定位跳变或 TF 时间异常。

再使用 `require_station_ack:=true` 检查逐株握手，并分别执行故障分支：

- 导航中 `pause`，确认目标取消且 `/base_motion/locked=true`，再 `resume`；
- 到站后 `skip`，确认只跳过当前植株且事件被清除；
- 导航中 `return_home`，确认先取消当前目标、锁车，再解锁发送返航目标；
- 导航中 `cancel`，确认终态为 `CANCELED` 且保持锁车；
- 使用不匹配地图或 mask 启动，确认执行器拒绝开始；
- 运行中替换 `/map` 或 `/keepout_filter_mask` 内容，确认活动目标被取消并锁车失败关闭；
- 让目标超过 `goal_timeout_s`，确认状态为 `FAILED` 且保持锁车。

阶段五中央单通道 profile 的验收条件为：

- 20 株 ID 唯一、序号连续、作业侧与植株 map 位姿一致；
- 10 个停车位、home、微调包络及相邻路线段通过占据图和 keepout 校验；
- 保存资产、安装资产及实时 `/map`/mask 为同一绑定版本；
- 自动完成全部停车点并返航，路径没有离开中央通道；
- 暂停、继续、跳过、取消、返航、超时和绑定失配产生正确状态、锁车结果和日志。

任务 YAML 中的 `navigation_status: pending_runtime` 只有在完成上述整条路线实测并保存报告
后才能更新。离线校验成功只证明预设几何与保存栅格一致。

## 8. 参数变化与重新生成规则

以下内容变化后，旧地图、keepout 和任务必须一起作废并重新建图/生成/验证：

- 行数、每行株数、行距、株距、场地原点或机器人建图出生位姿；
- 植株冠幅、物理碰撞宽度或会改变雷达可见结构的株高；
- 保存地图 PGM、分辨率、原点、阈值语义或 world→map 对齐；
- 底盘 footprint、keepout margin、home、通道范围或允许微调范围。

单独改变果实数量、成熟比例或果实高度理论上不改变底盘停车几何，但仍应确认修改后的
视觉模型没有改变激光观测或物理碰撞，再复用导航资产。果实 world 真值不用于在线目标
选择，也不用于修正定位。

## 9. 当前边界

当前交付完成了中央单通道阶段五所需的任务格式、20 株预设点、10 个停车位、keepout、
离线/实时版本绑定、Nav2 Action 调度、暂停/继续/跳过/取消/返航/超时、底盘锁和日志。
下列能力仍属于后续阶段：

- 多通道、行末换行及完整蛇形巡查；
- 阶段六的在线 SLAM 自主覆盖建图和自动保存；
- 阶段七的持续 0.5 s 停稳确认、机械臂收纳反馈和正式作业握手；
- D405 视场验证、CR5 IK、视觉识别与真实采摘；
- 对动态障碍和 AMCL 定位误差的联合性能指标验收。

因此本阶段的“每株一个采摘点”是逻辑作业点：中央通道两侧共 20 个任务，左右成对任务
共享一个安全底盘位姿。它不表示机械臂已经能够到达每个果实，也不表示重复结构下的
全田跨行定位问题已经解决。
