# 阶段五：中央单通道预设采摘路线

本文给出番茄田的阶段五操作流程。任务范围是固定出生点前方的中央通道：
机器人沿左侧前进到达 10 个停车位，在行端经过 3 个导航中转点换到右侧，再倒车到达
右侧 10 个停车位，为通道两侧 20 株植株各发布一次逻辑采摘任务，最后回到 home。
20 个采摘停车位、中转点和 home 都预设 `yaw=0`；行驶转弯期间允许调整航向。
本文中的 `left` 和 `right` 都以机器人到站后的
`base_footprint` 为参考，不是地图坐标轴方向的别名。

阶段五使用保存地图导航，不启动 SLAM Toolbox。任务文件同时绑定田间 SDF/元数据、
占据地图、keepout mask、world→map 对齐、底盘 footprint 和停车点。执行器启动前做
离线绑定校验，运行前还要求实际发布的 `/map` 与 `/keepout_filter_mask` 内容完全匹配；
运行中任一内容改变会取消活动目标并进入锁车失败状态。

## 1. 任务范围与侧偏停车点

当前默认场景为近侧果实 `harvest_demo_v1`，生成参数及待验证项见
[近侧果实演示场景](harvest_demo_fruit_profile.md)。本轮只改变果实分布并更新任务资产哈希，
下面的底盘路线和地图沿用原资产。新场景短程往返的 map 到站检查已通过，但独立真值显示
回程定位偏差最大 0.743 m，实际停车精度尚未达标；详见上述文档的验证记录。
实际夹取仍待定位精度、IK/MoveIt 和相机验证。
机械臂接入前的速度反馈停稳、模拟作业 Action 和收纳互锁见
[前置检查流程](harvest_preflight.md)；该模式显式开启，不改变下文原导航人工确认流程。

当前场景：

```text
FIELD_ID=rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_harvest_demo_v1_seed42
6 行 × 10 株，行距 3.0 m，株距 0.7 m
中央通道两侧：row_id=2 和 row_id=3（从 0 开始编号）
home=(0.50, 0.00, 0.00)，坐标系为 map
```

建图从 Gazebo world 位姿 `(0,-6,π/2)` 开始，因此本地图采用：

```text
map = [6, 0] + R(-π/2) × world
```

默认路线的[航点预览图](../artifacts/navigation_stage5_20261012/run_side_3m/waypoint_preview.png)
叠加了保存地图、禁行遮罩和 20 个采摘停车位。蓝点为左侧、橙点为右侧，紫色菱形为
行尾中转；箭头表示停车时 `yaw=0` 的车头方向，右侧路线采用倒车返回。

每株植株都有独立 `waypoint_id`、植株身份、作业侧、扫描姿态标识和机械臂姿态标识。
默认路线采用 `route.layout: independent_side_stops`，每株都有唯一 `stop_id` 和独立
`base_pose`，共 20 次采摘停车。左侧 row 2 按列 0→9 前进，右侧 row 3 按列 9→0
倒车返回；底盘只在行端换侧，避免逐株横跨通道。

下面的位姿和生成参数此前已在 domain 197 完成整线及锁车后停位验收，包含 20 个采摘站、
3 个中转点和 home。加入停位检查后，3 次同目标纠偏使 27 个 Nav2 成功结果对应
24 个已验证的逻辑到站；每个到站最终均满足 0.10 m / 5°。该历史版本的控制服务、三个
故障探针与无任务路径覆盖的默认入口检查均通过。当前近侧果实场景沿用上述地图、
keepout 和停车几何，任务场景绑定已更新并重新安装，源码、归档和安装任务哈希一致；
本轮未重跑完整 20 站巡航。

| 顺序 | 停车位 | 底盘预设位姿 `[x,y,yaw]` / m,rad | 植株 | 作业侧 |
| --- | --- | --- | --- | --- |
| 1 | `stop_00_left` | `[1.18, 0.26, 0]` | `tomato_20` | left |
| 2 | `stop_01_left` | `[1.88, 0.26, 0]` | `tomato_21` | left |
| 3 | `stop_02_left` | `[2.58, 0.26, 0]` | `tomato_22` | left |
| 4 | `stop_03_left` | `[3.28, 0.26, 0]` | `tomato_23` | left |
| 5 | `stop_04_left` | `[3.98, 0.26, 0]` | `tomato_24` | left |
| 6 | `stop_05_left` | `[4.68, 0.21, 0]` | `tomato_25` | left |
| 7 | `stop_06_left` | `[5.38, 0.21, 0]` | `tomato_26` | left |
| 8 | `stop_07_left` | `[6.08, 0.21, 0]` | `tomato_27` | left |
| 9 | `stop_08_left` | `[6.78, 0.16, 0]` | `tomato_28` | left |
| 10 | `stop_09_left` | `[7.48, 0.16, 0]` | `tomato_29` | left |
| 11 | `stop_09_right` | `[7.48, -0.38, 0]` | `tomato_39` | right |
| 12 | `stop_08_right` | `[6.78, -0.33, 0]` | `tomato_38` | right |
| 13 | `stop_07_right` | `[6.08, -0.33, 0]` | `tomato_37` | right |
| 14 | `stop_06_right` | `[5.38, -0.33, 0]` | `tomato_36` | right |
| 15 | `stop_05_right` | `[4.68, -0.33, 0]` | `tomato_35` | right |
| 16 | `stop_04_right` | `[3.98, -0.33, 0]` | `tomato_34` | right |
| 17 | `stop_03_right` | `[3.28, -0.28, 0]` | `tomato_33` | right |
| 18 | `stop_02_right` | `[2.58, -0.28, 0]` | `tomato_32` | right |
| 19 | `stop_01_right` | `[1.88, -0.28, 0]` | `tomato_31` | right |
| 20 | `stop_00_right` | `[1.18, -0.23, 0]` | `tomato_30` | right |

机械臂底座相对车体中心的纵向偏移约为 `-0.17998 m`，因此停车位采用
`base_x=plant_map_x+0.17998`，再按毫米舍入。侧向期望为左侧 `y=+0.40 m`、
右侧 `y=-0.40 m`，生成器在期望值附近搜索 ±0.25 m，并要求底盘原点到最近占据
栅格的距离至少 0.80 m。保存地图、整车 footprint 和组合微调包络约束后，实际左侧
Y 为 0.16–0.26 m，右侧为 -0.38–-0.23 m。0.80 m 是原点到障碍的距离，不是
车身外缘的净距。

停车点偏向作业侧仍需保留转向余量。车体修正终点航向时，矩形前后角会比 `yaw=0`
停车状态更接近植株；原期望侧偏 0.50 m、最小原点距离 0.60 m 的方案虽通过静态
footprint 校验，舵轮修复后仍在第 7 站出现碰撞预测/进度失败并超时。当前方案收回侧偏、
增大原点距离来给转向扫掠留余量；该几何约束不能替代动态路线和接触碰撞验收。
按当前 footprint 后伸 0.535 m、半宽 0.395 m，最远车角距原点约 0.665 m；
0.80 m 约束给该转向半径留下约 0.135 m 的栅格中心距离余量，仍需考虑栅格尺寸、
定位误差和实际车辆扫掠。

第 10 和第 11 个采摘站之间插入以下纯导航中转，均位于当前通道的行端：

| 中转点 | 位姿 `[x,y,yaw]` | 插入位置 |
| --- | --- | --- |
| `headland_out` | `[8.50, 0.40, 0]` | `harvest_tomato_39` 之前 |
| `headland_cross` | `[9.00, 0.00, 0]` | `headland_out` 之后 |
| `headland_in` | `[8.50, -0.40, 0]` | `headland_cross` 之后 |

三个中转点属于 `route.transits`，只调用导航，不发布采摘任务、不等待采摘完成确认，
也不占用 20 株的序号。侧偏地图、posegraph、keepout 和任务保存于
`artifacts/maps/$FIELD_ID/side_stops/`。当前占据图为 `206×76`、分辨率 `0.05 m`，
原点 `[-0.401,-2.058,0]`，包含行端换侧所需的已建图区域。

该路线依赖三项配套的 Nav2 配置：NavFn 使用
`use_final_approach_orientation: false` 保留任务要求的目标 `yaw=0`；Rotation Shim 使用
`use_path_orientations: true` 并包装 RPP；RPP 使用 `allow_reversing: true` 和
`use_rotate_to_heading: false` 跟踪去程及返程倒车路径。RPP 到达停车位置后，Shim 只负责
收敛较小的终点航向误差。不要单独修改其中一项；更换规划器、控制器或相关参数后，必须
重新执行短距离往返和完整路线验收。

Shim 同时设置 `rotate_to_heading_once: true`：同一个目标的周期性重规划不能反复抢占
RPP 的曲线倒车指令。否则机器人为靠近入口而调整航向时，Shim 又把车头转回路径中的
`yaw=0`，形成反复换舵、停车和碰撞预测失败。该设置仍保留到达目标位置后的航向修正。

Rotation Shim 采用 `closed_loop: false`。四轮转向舵角建立需要时间，闭环 Shim 在这段
时间内读到接近零的实测角速度，会反复把指令限制在初始 0.025 rad/s；开环加速度则根据
上一条指令逐步升到有效的 0.20 rad/s，同时 Collision Monitor 仍用实时 footprint 执行
安全停车。该参数只改变 Shim 的角加速度反馈方式，不会关闭里程计、TF、AMCL 或碰撞反馈。

内部 Nav2 Goal Checker 已收紧为 `xy_goal_tolerance: 0.07 m`、
`yaw_goal_tolerance: 0.05236 rad`（约 3°）；Shim 的
`angular_disengage_threshold` 同为 `0.05236 rad`。这些内部判定给减速和锁车后的微小
位姿变化留出余量。阶段五的对外验收仍要求：在 `AT_STATION` 且底盘反馈已锁定后，
`map→base_footprint` 相对预设位姿的平面距离误差 ≤0.10 m、绝对航向误差 ≤0.0873 rad
（约 5°）；home 在 `COMPLETED` 且锁定后按同一标准检查。Nav2 返回成功不能替代这项
最终停位测量。

这些指标属于地图坐标中的导航停位验收。Gazebo 独立真值误差、机械臂 IK、相机视场和
真实采摘精度由阶段七/八另行验收，不能仅凭地图坐标误差达标就判定全部果实可达。

还需关闭位置命中的状态记忆。Jazzy 的 SimpleGoalChecker 在 `stateful: true` 时，
首次进入 XY 公差后会停止检查 XY，只继续检查航向；终点转向或 AMCL 位姿修正使车体
移出 XY 公差后，仍可能返回成功。Shim 内部的 PositionGoalChecker 也能记住首次
位置命中，从而继续只做航向修正。当前配置将两处都设为 false，使终点航向修正时
持续重查位置，超出位置公差时可以重新由 RPP 修正。
仅收紧公差而保留状态记忆的复测曾出现 0.126 m 的停车位置误差；最终同时关闭两处
状态记忆后，再对完整路线验证锁车后的停位误差。
参见 [SimpleGoalChecker 源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/simple_goal_checker.cpp)、
[PositionGoalChecker 源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/position_goal_checker.cpp)和
[Rotation Shim 源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)。

当前 Jazzy Shim 源码在写入控制器插件名之前初始化内部位置检查器，由空插件名生成
`.position_checker` 参数前缀，因此本版本 YAML 使用：

```yaml
controller_server:
  ros__parameters:
    general_goal_checker:
      stateful: false
    .position_checker:
      stateful: false
```

这个内部前缀依赖实际 Nav2 版本，不能直接假定为 `FollowPath.position_checker`。
本次实节点已核对两处参数均为 false，并完成整线复测。后续升级或重新启动仍应使用
第 5 节命令核对实际节点参数。

提前返航还依赖底盘正确处理舵轮的有界关节角度。四个舵轮使用有硬限位的 revolute
关节，不能把跨越 ±π 当作连续关节的短距离转动。底盘运动学现按限位内的直接角差
选择可行轮向，把 `steering_joint_limits` 与 URDF 限位、模块符号/零位及
`steering_limits` 联合校验；选择相差 π 的等效轮向时，同步反转对应轮速。校准和
对齐判定也保留该有界坐标。任一轮无可行方向时，全车驱动轮速为零，不能截断舵角
后继续行驶。该修复针对左侧第 2 站附近提前返航时的转向受阻；Nav2 参数保持不变，
停车位另按转向扫掠余量调整为上述位姿。更新工程后必须构建 `agri_base_kinematics`，并重启仿真
底盘节点；只构建导航和任务包不会加载这项修复。

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
  agri_robot_description agri_base_kinematics agri_lidar_adapter \
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

FIELD_ID="rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_harvest_demo_v1_seed42"
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
  --row-spacing 3.0 --plant-spacing 0.7 \
  --plant-width 0.06 --keepout-margin 0.15
```

`--plant-width 0.06` 对应当前简化物理碰撞宽度，默认 row-band 会把每条植株行连接成
连续禁行区域，防止全局规划穿过株间缝隙。它不是 1.0 m 的可视冠幅；若改用冠幅生成
mask，必须先重新核算中央通道是否仍容纳整车 footprint。

再根据同一地图和 keepout 生成 20 个独立采摘停车位及 3 个行端中转点。下列命令给出
当前默认任务的源码输出路径；本文记录的整线、控制及故障验收已通过。自行改变
资产后，应先输出独立候选，重新验收后再覆盖默认任务：

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
  --layout independent_side_stops \
  --side-offset-m 0.40 --side-search-m 0.25 \
  --min-obstacle-clearance-m 0.80 \
  --arm-longitudinal-offset-m -0.17998 \
  --headland-forward-m 1.02 --headland-cross-forward-m 0.50 \
  --search-step-m 0.01 \
  --max-unknown-cells 8 \
  --goal-timeout-s 120 --station-dwell-s 1.0 \
  --micro-longitudinal-m 0.05 \
  --micro-lateral-m 0.025 --micro-yaw-rad 0.025 \
  --output "$MISSION"
```

生成器会选择 `world x=0` 两侧最近的两行，为每株搜索靠近其作业侧的独立停车点，并
生成先左侧去程、后右侧返程的序列。它同时检查 home、全部停车位、中转点、微调包络、
包含中转的路线段及各导航锚点到 home 的直线连接；任一点触碰占据栅格、地图外、超量
未知栅格或 keepout 都会拒绝生成。直线采样只证明预设几何可通行，实际控制器的转弯
扫掠仍要通过仿真验收。

旧 `route.layout: shared_pairs` 文件仍可加载；未写 `layout` 的旧文件按共享停车位校验。
新默认命令显式使用 `independent_side_stops`，不要用旧共享文件代替侧偏任务。

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

FIELD_ID="rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_harvest_demo_v1_seed42"
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
unique_stop_count: 20
transit_count: 3
selected_rows: [2, 3]
excluded_plant_count: 40
```

每个停车位和中转点报告的 `occupied_cells`、`outside_cells` 应为 0，未知格数不得超过 8。
校验还覆盖 PGM 原始 SHA、转换后的 OccupancyGrid SHA、地图语义、尺寸/分辨率/原点、
keepout SHA 和地图几何。不要通过手改任务中的哈希来绕过失配。

运行包测试：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select agri_base_kinematics agri_task_manager \
  --python-testing pytest \
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

FIELD_ID="rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_harvest_demo_v1_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"

ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$FIELD_DIR/tomato_field.sdf" \
  gz_partition:="$GZ_PARTITION" \
  spawn_x:=0.0 spawn_y:=-6.0 spawn_yaw:=1.5708 \
  steering_effort_limit:=40.0 \
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
ros2 topic echo /costmap_filter_info --once \
  --qos-reliability reliable --qos-durability transient_local
```

九个生命周期节点都应为 `active [3]`，Action 应显示一个 server。
`/map` 和 `/keepout_filter_mask` 都应为 `map` frame、`0.05 m` 分辨率、`206×76`，
原点 `[-0.401,-2.058,0]`，
`/costmap_filter_info` 应为 `type=0`、`base=0.0`、`multiplier=1.0`。

核对目标判据和当前 Jazzy 内部位置检查器参数：

```bash
ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance
ros2 param get /controller_server general_goal_checker.stateful
ros2 param get /controller_server .position_checker.stateful
```

应分别为 `0.07`、`0.05236`、`false`、`false`。内部参数不存在或名称不一致时，先根据
当前 Nav2/插件版本核对实际参数名，再启动正式路线；源码 YAML 值不等于运行节点已生效。

### 终端三：启动任务执行器

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

FIELD_ID="rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_harvest_demo_v1_seed42"
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

正式启动还要检查底盘门控和 Nav2 目标状态：

```bash
ros2 topic echo /base_motion/locked --once \
  --qos-reliability reliable --qos-durability transient_local
timeout 3s ros2 topic echo /navigate_to_pose/_action/status --once \
  --qos-reliability reliable || true
```

任务执行器以 `/base_motion/locked` 的实际反馈为准；锁/解锁只有在服务响应和反馈都匹配
时才继续。`status.base_lock_recovery=true` 时，执行器正在反复补偿锁定，不能调用
`resume` 或重新发目标。如果已收到 Nav2 状态消息，启动前 `navigator_active_goals` 必须为 0；如果任务节点重启后
仍检测到旧 Nav2 目标，先重启导航栈，不能只重启任务节点。

## 6. 执行和控制服务

启动任务：

```bash
ros2 service call /field_route_executor/start std_srvs/srv/Trigger '{}'
```

新增执行器在 Nav2 返回成功并确认底盘锁定后进入 `PARKING_CHECK`，采摘站、中转点和
home 使用同一停位检查。它要求 `map→base_footprint` TF 在本次锁定后更新、时间戳
新鲜且连续稳定至少 0.5 s，位置误差 ≤0.10 m、航向误差 ≤5°，然后才发布
`parked_pose_verified`。采摘站随后进入 `AT_STATION` 并发布作业事件；中转随后才写
`transit_completed` 并记为完成；home 随后才进入 `COMPLETED`。

稳定后的位姿超差会产生 `parked_pose_retry`，解锁并重新发送原位姿，最多纠偏 3 次。
重发不增加或消耗植株序号，不发布重复采摘任务，也不提前标记中转完成。纠偏耗尽，
或 5 s 内无法获得新鲜、稳定的 TF，会记录 `parked_pose_failed` 并在确认锁车后
进入 `FAILED`。`PARKING_CHECK` 期间的作业点保持 `active=false`，作业协调器必须
等待正式 `AT_STATION`。这项机制已通过 domain 197 整线、控制及故障复测，单次
Nav2 成功不能代替锁后停位验收。

| 执行器参数 | 默认值 | 含义 |
| --- | --- | --- |
| `parking_settle_s` | 0.5 s | 连续稳定窗口，使用 ROS 时间 |
| `parking_timeout_s` | 5.0 s | 单次锁后检查超时，使用墙钟时间 |
| `parking_tf_max_age_s` | 0.35 s | TF 最大年龄；还须不早于本次锁定检查开始时刻 |
| `parking_position_tolerance_m` | 0.10 m | 锁后平面距离公差 |
| `parking_yaw_tolerance_rad` | 0.0872665 rad | 锁后航向公差，约 5° |
| `parking_stability_position_m` | 0.01 m | 稳定窗口允许的位置变化 |
| `parking_stability_yaw_rad` | 0.0174533 rad | 稳定窗口允许的航向变化，约 1° |
| `parking_max_retries` | 3 | 同一目标最多重复纠偏次数 |

TF 未来时间最多允许 0.05 s。启动文件提供上述参数中的稳定位置/航向变化量以外
各项覆盖入口；稳定变化量可通过执行器 ROS 参数设置。此处的“稳定”是地图 TF
位姿稳定，机械臂作业前的实际底盘停稳和收纳反馈仍须阶段七确认。

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

本路线每株 `stop_id` 都不同，20 个到站事件的 `reused_base_pose` 均为 false。
每株完成后，底盘解锁并导航到下一停车位。左侧最后一株完成后，执行器依次导航到三个
行端中转点，再到右侧最后一株；中转期间 `status.current_transit` 给出中转 ID 和位姿，
`current_harvest_point` 为 `active=false`。完成 20 个逻辑任务后，执行器发送 home
目标；到达 `(0.50,0.00,0)` 后重新锁车，完成停位检查才进入 `COMPLETED`。

旧共享布局仍支持 `reused_base_pose=true` 的第二株任务。中转期间 `skip` 会被拒绝，
可使用 `pause`、`cancel` 或 `return_home`；恢复会继续尚未完成的中转，不跳过换侧路径。

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
3. 确认 `navigator_active_goals=0`、`base_lock_recovery=false`，再调用 `start`，在 RViz
   观察规划路径始终沿中央通道；
4. 检查 20 个 `at_harvest_point` 事件对应 20 个唯一停车位，全部 `reused_base_pose=false`；
5. 前 10 个事件对应 `tomato_20→29`、`work_side=left`；后 10 个对应
   `tomato_39→30`、`work_side=right`。两段之间恰好完成 3 个 `transit_completed`
   事件，中转期间没有采摘事件或人工采摘确认；
6. 确认每个采摘站、中转点和 home 都有 `parked_pose_verified`，位姿在锁车后保持
   新鲜且稳定至少 0.5 s；如有 `parked_pose_retry`，应重复原目标且不增加采摘序号。
   确认在行端已建图自由区换侧；右侧返程段线速度为负。在底盘锁定后测量各停车位、
   中转点和 home，平面距离误差均 ≤0.10 m、绝对航向误差均 ≤0.0873 rad。
   转弯期间允许航向变化，检查车身完整扫掠没有触碰植株或 keepout；
7. 最终到达 home、底盘锁定、状态为 `COMPLETED`，检查 Collision Monitor、定位和
   TF 时间反馈。接触传感器的碰撞审计及连续真值精度评测另行记录。

再使用 `require_station_ack:=true` 检查逐株握手，并分别执行故障分支：

- 导航中 `pause`，确认目标取消且 `/base_motion/locked=true`，再 `resume`；
- 到站后 `skip`，确认只跳过当前植株且事件被清除；
- 中转中 `pause/resume`，确认恢复尚未完成的中转；调用 `skip` 应被拒绝；
- 导航中 `return_home`，确认先取消当前目标、锁车，再解锁发送返航目标；
- 导航中 `cancel`，确认终态为 `CANCELED` 且保持锁车；
- 让 `/base_motion/locked` 短时变为 false，确认活动目标被停止、执行器进入锁恢复，
  只有观察到 `locked=true` 后才报告 `FAILED`；
- 使用不匹配地图或 mask 启动，确认执行器拒绝开始；
- 运行中替换 `/map` 或 `/keepout_filter_mask` 内容，确认活动目标被取消、执行器恢复锁车
  并进入失败终态；
- 让目标超过 `goal_timeout_s`，确认状态为 `FAILED` 且保持锁车。

阶段五中央单通道 profile 的验收条件为：

- 20 株 ID 唯一、序号连续、作业侧与植株 map 位姿一致；
- 20 个独立停车位、3 个中转点、home、微调包络及实际预设路线段通过占据图和 keepout 校验；
- 保存资产、安装资产及实时 `/map`/mask 为同一绑定版本；
- 自动完成全部停车点并返航，路径没有离开中央通道；
- 各停车位、中转点和 home 锁定后的 map 平面距离误差 ≤0.10 m、绝对航向误差 ≤0.0873 rad；
- 暂停、继续、跳过、取消、返航、超时和绑定失配产生正确状态、锁车结果和日志。

domain 197 的完整 Gazebo 路线已通过，运行 `run_id=291028000000`。本轮使用有界舵轮
修复、侧偏期望 0.40 m / 最小原点距离 0.80 m 的任务，以及 `PARKING_CHECK`。
全部采摘站、中转点和 home 都在锁车反馈确认后通过新鲜 TF 和连续稳定窗口检查。
近侧果实方案实施前，已验收侧偏路线任务的历史 SHA-256 为
`443aec43490081c7e1c697a98ffe8b45deb96141e454b2723b229da51ed11a27`。
原始本轮结果见
[整线运行结果](../artifacts/navigation_stage5_20261012/run_side_3m/final_result.json)，运行与
事件日志保存在同目录 `final_monitor.jsonl`、`final_events.jsonl`。新版控制服务和三个
故障探针已通过，默认任务已推广、重新安装并通过重现检查，结果见
[绑定汇总报告](../artifacts/navigation_stage5_20261012/run_side_3m/summary.json)
及对应资产、源码和测试报告哈希。

| 核验项 | 实测结果 |
| --- | --- |
| 采摘站点 | 20 个独立停车位，20 株各一次 |
| 行端中转 | 3 个，均完成 |
| 停位检查 | 24 个唯一 `parked_pose_verified`：20 站 + 3 中转 + home |
| Nav2 目标 | 27 个成功，包含 3 次同目标纠偏 |
| 倒车指令 | 7392 个负线速度指令样本 |
| 采摘站锁定后 map 最大位置误差 | 0.091774 m |
| 采摘站锁定后 map 最大航向误差 | 3.79979° |
| home 锁定后 map 位置/航向误差 | 0.057565 m / 1.29146° |
| 路线运行时间 | 502.701 s，约 8 分 23 秒 |
| 终态 | `COMPLETED`、底盘锁定、活动目标数 0 |

本轮 20 个 `AT_STATION` 均确认 `base_lock_observed=true`，24 个逻辑到站均有唯一的
`parked_pose_verified`。事件顺序为真实锁车反馈、`motion_locked`、停位验证，随后才
发布采摘站、中转完成或 home 完成。3 次纠偏分别发生在 `headland_in`、
`harvest_tomato_39` 和 `harvest_tomato_34`，最终均通过；采摘序号没有增加，作业事件
没有重复。终态没有活动目标，监测未报告不安全锁车状态。

旧侧偏方案的第 7 站碰撞预测/超时记录保留于
[bounded_side_collision/result.json](../artifacts/navigation_stage5_20261012/run_side_3m/bounded_side_collision/result.json)。
同一 0.40 m / 0.80 m 几何在加入停位检查前，曾出现 home Nav2 成功却稳定后为
0.147 m / 10.13° 的超差，见
[settled_home_pose.json](../artifacts/navigation_stage5_20261012/run_side_3m/clearance_first_complete/settled_home_pose.json)。
这说明既要给车角转向留余量，也要在锁定后重新检查位姿，不能以 Nav2 成功代替
最终停位验收。

domain 197 的 6 项控制服务检查已通过：暂停时取消目标并锁车、恢复至首株并人工
确认、下一株独立停车及确认、跳过一株、导航中提前返航、导航中取消。本轮控制测试
显式传入已验收任务的 `mission_file`；推广后又独立检查未传该覆盖参数的默认安装
入口，确认加载同一任务哈希、20 个唯一停车位和 3 个中转，启动后取消可锁车且
活动目标数为 0，见
[default_entry_result.json](../artifacts/navigation_stage5_20261012/run_side_3m/default_entry_result.json)。
第 2 个左侧停车点 `(1.88,0.26,0)` 附近提前返航，Nav2 成功
耗时 25.702 s，锁后停位验证完成耗时 26.798 s，终态锁车且活动目标数为 0。
结果见
[default_controls_result.json](../artifacts/navigation_stage5_20261012/run_side_3m/default_controls_result.json)，
响应、事件和计时分别保存在同目录 `default_controls_trace.jsonl`、
`default_controls_events.jsonl`、`default_controls_timings.json`。

同版本的锁车反馈异常、运行中 keepout 内容变化、导航超时三个故障探针全部通过：
取消目标、确认锁车后进入 `FAILED`，活动目标数为 0。结果见
[lock_feedback.json](../artifacts/navigation_stage5_20261012/run_side_3m/lock_feedback.json)、
[keepout_change.json](../artifacts/navigation_stage5_20261012/run_side_3m/keepout_change.json)和
[timeout.json](../artifacts/navigation_stage5_20261012/run_side_3m/timeout.json)。
[validation_binding.json](../artifacts/navigation_stage5_20261012/run_side_3m/validation_binding.json)
绑定 domain 197、原任务及超时专用任务哈希、执行源码哈希和各报告摘要。旧配置的
控制记录保存在 `pre_clearance_controls/`，不用于本轮验收。

相关包已构建成功，12 个源码/安装资产的哈希一致。任务管理包通过 91 项检查
（89 项功能测试，含默认任务重现检查；2 项在包目录执行的 lint），导航包 3 项和
底盘运动学包 34 项测试通过；底盘 flake8 通过。结果见
[package_validation.json](../artifacts/navigation_stage5_20261012/run_side_3m/package_validation.json)。

生成器仍为新任务写入 `validation.navigation_status: pending_runtime`，执行器不会自动
改写静态任务 YAML。各版本的运行结果由匹配的报告及资产哈希证明；重新生成任务
或修改地图、参数后，要对新版本重新执行整线验收。

地图停位达标与绝对定位精度分开记录。上述历史复测的采摘站 map 位姿与独立 Gazebo 真值的最大
差异为 0.195671 m / 6.86288°，尚未通过阶段八绝对精度验收。稀疏真值样本和导航碰撞
监测也不能替代接触式碰撞审计；机械臂 IK、D405 视场和实际采摘仍属于后续验收。

## 8. 参数变化与重新生成规则

行距从 2.0 m 增大到 3.0 m 可增加约 1.0 m 的横向间距，适合需要更大转向空间的演示。
但扩大场地不会修复车体伪回波、换舵受阻或错误回环。先检查实时障碍、转向反馈和定位，
再根据真实通行余量决定是否修改。默认场景已为 3.0 m，地图与 keepout 已包含行端
换侧区域；新的 20 站侧偏任务已通过整线、锁后停位、控制服务及故障复测，已设为
默认并重新安装。
原 2.0 m 默认资产备份于 `artifacts/navigation_stage5_20261012/legacy_default_2m/`。
原 3.0 m 中央共享停车位资产保留于
`artifacts/navigation_stage5_20261012/legacy_center_3m/`。

当前默认为 6 行、每行 10 株、3.0 m 行距，生成场地时采用 `--origin-x -7.5`，使行中心
为 `[-7.5,-4.5,-1.5,1.5,4.5,7.5]`，中央通道仍以 `world x=0` 为中心。使用新的
`FIELD_ID` 保存 SDF/JSON，重新建图、生成 keepout 与 mission，再重复离线检查及完整
往返。不要仅修改启动中的行距并继续使用旧地图和旧采摘点。

以下内容变化后，旧地图、keepout 和任务必须一起作废并重新建图/生成/验证：

- 行数、每行株数、行距、株距、场地原点或机器人建图出生位姿；
- 植株冠幅、物理碰撞宽度或会改变雷达可见结构的株高；
- 保存地图 PGM、分辨率、原点、阈值语义或 world→map 对齐；
- 底盘 footprint、keepout margin、home、通道范围或允许微调范围。

单独改变果实数量、成熟比例或果实高度理论上不改变底盘停车几何，但仍应确认修改后的
视觉模型没有改变激光观测或物理碰撞，再复用导航资产。果实 world 真值不用于在线目标
选择，也不用于修正定位。

## 9. 当前边界

当前已提供中央单通道阶段五所需的任务格式、20 个侧偏采摘停车位、3 个行端中转点、
keepout、离线/实时版本绑定、Nav2 Action 调度、暂停/继续/跳过/取消/返航/超时、底盘锁
和日志。新转向余量路线与停位检查机制已通过整线、控制与故障复测，已设为默认并
重新安装，实际结果见第 7 节。
下列能力仍属于后续阶段：

- 多通道、行末换行及完整蛇形巡查；
- 阶段六的在线 SLAM 自主覆盖建图和自动保存；
- 阶段七的实际底盘持续停稳确认、机械臂收纳反馈和正式作业握手；
- D405 视场验证、CR5 IK、视觉识别与真实采摘；
- 对动态障碍和 AMCL 定位误差的联合性能指标验收。

本阶段的“每株一个采摘点”对应独立的底盘预设位姿和逻辑作业任务。侧向靠近与纵向
对齐减少了机械臂需要跨越的距离；果实高度、果实位于植株的近侧或远侧、工具姿态及
碰撞仍影响可达性。20 个停车位不表示所有果实已通过 IK/FOV 验证，也不表示重复结构下
的全田跨行定位问题已经解决。

只读 [机械臂可达性粗界报告](../artifacts/navigation_stage5_20261012/run_side_3m/arm_reach_bound_audit.json)
记录的是近侧果实方案实施前的原随机果实，确认当时侧偏停车没有解决全部伸距问题。按当前 URDF/STL，`tool0` 的非常
宽松半径上界约 1.05665 m；名义夹爪 `q=0` 时，任意夹爪几何点的粗界约 1.27957 m。
该历史场景所选 20 株共有 56 个成熟果，其中仅 20 个果实表面满足后一个粗界的必要条件，36 个
连必要条件都不满足；6 株没有满足该条件的成熟果，另 1 株没有成熟果。通过粗界的
果实仍可能因关节限位、接近姿态、视场或碰撞而无法采摘，报告没有执行 IK/FOV。

审计采用 Gazebo 实际车体 world 高度约 0.36 m，机械臂安装点 world 高度约 0.815 m；
导航 `map/odom→base_footprint` 的 `z=0` 是平面导航约定，不能替代物理离地高度。
进入采摘阶段前应先校准基座高度、抓取 TCP 和真实夹爪开度，再选择近侧、较低且通过
IK/FOV 的果实。个别远侧果实所需停车侧偏会越过通道或压缩转向余量，不能仅靠继续
把全部停车点移近植株来解决。当前已改用[近侧果实演示方案](harvest_demo_fruit_profile.md)，
20 株均有成熟距离候选；该结果仍没有替代 IK/FOV/夹取验证，URDF 尺寸未改。

## 10. RViz 航点可视测试

从全新仿真开始，按第 5 节启动场景、导航和执行器；可视测试时把场景启动参数改为
`rviz:=false`，执行器使用 `require_station_ack:=true`。所有终端加载同一套工作空间，
并设置相同的 `ROS_DOMAIN_ID=96`。不要同时启动建图或遥控，也不要在任务执行期间
用 RViz 的 Nav2 Goal 发送另一个目标。

第四个终端启动导航 RViz：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
```

第五个终端加载同样环境后运行只读显示脚本，无需重新构建：

```bash
python3 scripts/view_harvest_waypoints.py
```

脚本读取已安装的默认任务文件，在 `/harvest_waypoint_markers` 发布 20 个采摘站、
3 个中转点和 home 的箭头及文字，不发送导航目标或控制命令。`01:L`–`10:L` 是蓝色
左侧停车点，`11:R`–`20:R` 是橙色右侧停车点，`H1`–`H3` 为紫色中转，`HOME` 为绿色。
箭头表示停车车头方向；右侧返程虽然倒车，预设车头仍朝 `map +X`。

RViz 设置：

- Fixed Frame 为 `map`，Views Type 为 `TopDownOrtho`；
- Map 使用 `/map`，Durability 为 `Transient Local`，Reliability 为 `Reliable`；
- 勾选 RobotModel，Description Topic 为 `/robot_description`，同样使用
  `Transient Local` 和 `Reliable`，避免晚启动时收不到描述；
- Add → By display type → MarkerArray，Topic 为 `/harvest_waypoint_markers`，同样使用
  `Transient Local` 和 `Reliable`；
- 保留 `/scan`、`/plan`、`/global_costmap/costmap`、`/local_costmap/costmap` 和
  `/local_costmap/published_footprint`，便于观察激光、实际规划路径、膨胀区和车体轮廓。
  默认配置中的 `/local_plan` 不一定由本项目 RPP 发布，不作为必检项。

第六个终端先完成第 5 节状态检查，再调用第 6 节的 `start`。每站进入 `AT_STATION`
后查看当前植株身份、作业侧和锁后停位误差，再调用一次 `complete_current`。
读取 JSON 字符串时加 `--full-length`，避免长字符串被截断：

```bash
ros2 topic echo /field_route_executor/status --once --full-length \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /field_route_executor/current_harvest_point --once --full-length \
  --qos-reliability reliable --qos-durability transient_local
```

顺序为左侧 10 站 → 自动通过 3 个中转 → 右侧倒车 10 站 → 自动返航。
采摘站需满足 `AT_STATION`、`active=true`、`base_lock_observed=true`，锁后位置误差
≤0.10 m、航向误差 ≤5°。中转不需要人工确认。第 20 站确认后，到 home 完成停位检查，
终态应为 `COMPLETED`、`waypoint_index=20`、锁车、活动导航目标数为 0。
测试内容是底盘导航与逻辑采摘握手，机械臂 IK/FOV 和真实采摘仍按第 9 节另行验收。

首次完成逐点可视检查后，可重启执行器并改为 `require_station_ack:=false` 做自动巡航。
已有定位正确且停在 home 的同一仿真可继续使用，不要重设 AMCL 到原点。
关闭正在执行的测试前，先调用 `cancel` 并确认锁车和无活动目标，再依次停止执行器、
导航、显示脚本/RViz 和 Gazebo；已经 `COMPLETED` 的任务可直接按该顺序关闭。

### 请求确认超时与 FAILED 状态

`complete_current` 只接受 `AT_STATION`。返回 `no current work while FAILED` 时，先查看
`status.detail` 和本次 Nav2 日志，不能靠重复完成确认恢复导航。
`waypoint_index` 从 0 开始；`waypoint_index=9` 对应第 10 个采摘站。

本次人工可视测试 `run_id=221870000000` 已完成前 9 站。第 9 站 `tomato_28` 停位误差
为 0.063486 m / 2.44104°，已接受完成确认；切换到第 10 站 `tomato_29` 时，
BT Navigator 报 `Timed out while waiting for action server to acknowledge goal request
for compute_path_to_pose`，然后导航中止并锁车。该次失败发生在规划 Action 的目标确认
阶段。日志不能进一步证明具体的调度延迟来源。

`nav2_navigation.yaml` 中的 `bt_navigator.default_server_timeout` 已由 20 调为 1000。
单位是毫秒，控制 BT 对规划/控制 Action 目标确认的等待预算，不是每站的 120 秒导航
超时，也不是服务器发现参数 `wait_for_service_timeout`。Jazzy 在配置 BT 时读取该值，
因此只调用 `ros2 param set` 不会更新现有行为树，需重启导航并重新测试。

构建并重新启动后检查：

```bash
ros2 param get /bt_navigator default_server_timeout
```

应输出 `Integer value is: 1000`。从第 5 节的出生点初始位姿重新开始测试时，要一起
重启仿真，避免把仍停在第 9 站的机器人定位到原点。当前 `start` 会从第 1 站重建任务，
`resume` 只接受 `PAUSED`，不支持从 `FAILED` 直接恢复第 10 站。
本项参数更新不改变场景、地图、航点和停位公差；更新后的完整可视巡航仍需复测。
