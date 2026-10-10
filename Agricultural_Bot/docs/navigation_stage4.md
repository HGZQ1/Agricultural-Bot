# 阶段四：保存地图定位与 Nav2 单目标导航

本文说明阶段四的使用边界和场景变更流程。阶段四使用阶段三保存的二维地图，
由 `map_server` 发布地图、AMCL 发布 `map → odom`，Nav2 通过 `/cmd_vel_nav`
发送导航速度；建图模式的 SLAM Toolbox 不应同时运行。

## 当前默认 2 m 番茄田：从参数修改到导航测试的完整流程

本节以当前默认的 `6 行 × 10 株、2 m 行距、0.7 m 株距` 场景为准。命令均从
`Agricultural_Bot` 仓库根目录执行。所有 ROS 终端必须使用相同的 `ROS_DOMAIN_ID`；
`FIELD_ID` 是 shell 变量，每打开一个新终端都要重新定义。建图模式只运行
SLAM Toolbox，导航模式只运行 map server、AMCL 和 Nav2，两种模式不能同时运行。

### 1. 修改场地参数

场地布局通过生成命令的参数修改，不需要编辑 Python 源码。当前推荐值如下：

| 内容 | 参数 | 当前值 |
| --- | --- | --- |
| 行数 | `--rows` | 6 |
| 每行植株数 | `--plants-per-row` | 10 |
| 行距 | `--row-spacing` | 2.0 m |
| 株距 | `--plant-spacing` | 0.7 m |
| 第一行/第一株坐标 | `--origin-x/--origin-y` | -5.0 / -5.0 m |
| 地面尺寸 | `--ground-x/--ground-y` | 22 / 14 m |
| 冠幅/株高 | `--plant-width/--plant-height` | 1.0 / 2.0 m |
| 简化茎杆碰撞宽度 | `--plant-collision-width` | 0.06 m |

偶数行以 `x=0` 对称排列时，可用下面的公式计算第一行坐标：

```text
origin-x = -((rows - 1) × row-spacing) / 2
```

若要把每行植株也沿 Y 方向严格居中，可使用：

```text
origin-y = -((plants-per-row - 1) × plant-spacing) / 2
```

当前 `origin-y=-5.0` 在第一株前保留 1 m 出生区；6 行的 X 坐标为
`-5、-3、-1、1、3、5`，机器人从 `world=(0,-6)` 沿世界 `+Y` 进入位于
`x=-1` 与 `x=1` 两行之间的中央通道。1 m 视觉冠幅下通道净宽约 1 m，0.06 m
简化茎杆碰撞盒下物理净宽约 1.94 m。碰撞宽度只影响 Gazebo 物理碰撞，GPU LiDAR
仍会看到由 `--plant-width` 决定的枝叶视觉网格。2 m profile 应沿行直行并在行末
宽阔区域转弯；若需要在通道内完成更大的转向扫掠，使用后文的 3 m 宽通道 profile。

### 2. 构建工作空间并生成场景

首次使用或修改过 launch、配置和生成器代码后执行一次：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash

cd ros2_ws
colcon build --symlink-install --packages-select agri_navigation
source install/setup.bash

cd ../sim_ws
colcon build --symlink-install --packages-select \
  agri_greenhouse_worlds agri_sim_description agri_sim_bringup agri_sim_tests
source install/setup.bash
cd ..
```

只改变行数、株数和间距等运行参数时，可以跳过重新构建，直接重新生成 SDF。生成当前
默认 2 m 场景：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
FIELD_ID="rows6_n10_row2p0_plant0p7_h2p0_w1p0_c0p06_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"
MAP_DIR="$PWD/artifacts/maps/$FIELD_ID"
mkdir -p "$FIELD_DIR" "$MAP_DIR"

python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --rows 6 --plants-per-row 10 \
  --row-spacing 2.0 --plant-spacing 0.7 \
  --origin-x -5.0 --origin-y -5.0 \
  --ground-x 22.0 --ground-y 14.0 \
  --plant-height 2.0 --plant-width 1.0 \
  --plant-collision-width 0.06 \
  --randomize-fruits --fruit-visual mesh \
  --fruit-count-min 2 --fruit-count-max 6 \
  --fruit-height-min 0.80 --fruit-height-max 1.60 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --ripe-ratio 0.7 --seed 42 \
  --output "$FIELD_DIR/tomato_field.sdf" \
  --metadata "$FIELD_DIR/tomato_field.json"

test -s "$FIELD_DIR/tomato_field.sdf"
test -s "$FIELD_DIR/tomato_field.json"
gz sdf -k "$FIELD_DIR/tomato_field.sdf"
```

最后一条命令应输出 `Valid.`。这个 profile 会生成 60 株；若改变任一布局参数，应使用
新的 `FIELD_ID`，以免地图覆盖或错配。

### 3. 终端一：启动待建图仿真

先用 `Ctrl+C` 关闭旧仿真、SLAM、AMCL、Nav2 和 RViz。每次重新启动 Gazebo 都创建新
分区，避免旧 Gazebo 时钟混入当前运行：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash

export ROS_DOMAIN_ID=96
export GZ_PARTITION="agri_map_${ROS_DOMAIN_ID}_$(date +%s%N)"
FIELD_ID="rows6_n10_row2p0_plant0p7_h2p0_w1p0_c0p06_seed42"

ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$PWD/artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  gz_partition:="$GZ_PARTITION" \
  spawn_x:=0.0 spawn_y:=-6.0 spawn_yaw:=1.5708 \
  gui:=true paused:=false rviz:=false \
  use_control:=true use_kinematics:=true \
  use_lidar:=true lidar_mode:=gpu_lidar use_scan:=true \
  use_camera:=false
```

机器人从世界坐标 `(0,-6)` 出生，朝向世界 `+Y`，位于两条中央植株行之间。保持机器人
静止，等待控制器、`/scan` 和 `/odom` 开始发布后再启动 SLAM。

### 4. 终端二：启动 SLAM Toolbox

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

ros2 launch agri_navigation mapping.launch.py \
  use_sim_time:=true scan_topic:=/scan \
  map_frame:=map odom_frame:=odom base_frame:=base_footprint
```

启动后这个终端应一直运行。SLAM Toolbox 负责 `map → odom`；此时不要启动
`localization.launch.py`、`navigation.launch.py`、AMCL 或另一个 map server。

### 5. 终端三：执行建图前检查

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

ros2 lifecycle get /slam_toolbox --no-daemon --spin-time 10
ros2 node list --no-daemon --spin-time 10 | \
  grep -E '^/(slam_toolbox|amcl|map_server)$'
ros2 topic info /clock --verbose --no-daemon --spin-time 8
ros2 control list_controllers
ros2 run agri_sim_tests check_mapping --duration 5 --timeout 90
```

`check_mapping` 会在超时时间内静默等待 `/map`、`/scan`、`/odom` 和 TF 数据；运行期间
暂时没有终端输出属于正常现象。等待它返回 JSON 结果后再判断是否通过。

如果第一条命令显示 `/slam_toolbox` 为 `inactive [2]`，先在 SLAM 终端确认没有启动报错，
再执行下面的恢复命令：

```bash
ros2 lifecycle set /slam_toolbox activate --no-daemon --spin-time 10
ros2 lifecycle get /slam_toolbox --no-daemon --spin-time 10
ros2 run agri_sim_tests check_mapping --duration 5 --timeout 90
```

第二条命令应显示 `active [3]`。如果激活失败，不要开始遥控建图；回到第 4 步检查
`mapping.launch.py` 的日志以及 `/scan`、`/odom` 是否存在。

继续建图前应同时满足：

- `/slam_toolbox` 为 `active [3]`，节点筛选结果中没有 `/amcl` 或 `/map_server`；
- `/clock` 的 ROS publisher 只有一个，发布节点是 `/clock_bridge`；
- `joint_state_broadcaster`、`steering_controller` 和 `wheel_controller` 为 `active`；
- `check_mapping` 退出码为 0，并能收到 `/map`、`/scan`、`/odom` 以及完整 TF 链；
- 终端中不再连续出现 `Detected jump back in time` 或 `TF_OLD_DATA`。

需要查看地图时，可在另一个终端启动 Nav2 的 RViz 配置：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

rviz2 -d "$(ros2 pkg prefix --share nav2_bringup)/rviz/nav2_default_view.rviz" \
  --ros-args -p use_sim_time:=true
```

RViz 的 Fixed Frame 应为 `map`，并确认 `/map`、`/scan` 与机器人 TF 能正确叠加。

### 6. 终端四：低速遥控并完成闭环

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p use_sim_time:=true \
  -p speed:=0.10 -p turn:=0.20
```

先直行通过中央通道，再依次覆盖需要导航的行间通道、两端回转区和外围边界。转弯时降低
速度，多次经过已有区域，并最终回到出生点附近和初始朝向，使 SLAM 获得回环约束。
建图过程中在 RViz 检查植株行没有明显重影、断裂或整体错层；出现错层时应在保存前重新
经过同一区域，无法恢复时重新开始本轮建图。

### 7. 保存占据地图和 pose graph

按 `k` 停车，保持仿真和 `mapping.launch.py` 继续运行。在另一个已加载相同 ROS 域的
终端执行：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

FIELD_ID="rows6_n10_row2p0_plant0p7_h2p0_w1p0_c0p06_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"
MAP_DIR="$PWD/artifacts/maps/$FIELD_ID"
mkdir -p "$MAP_DIR"

ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/tomato_field" \
  --ros-args -p use_sim_time:=true \
    -p map_subscribe_transient_local:=true \
    -p save_map_timeout:=10.0

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_DIR/tomato_field'}"

sha256sum "$FIELD_DIR/tomato_field.sdf" "$FIELD_DIR/tomato_field.json" \
  > "$MAP_DIR/field.sha256"
ls -lh "$MAP_DIR"
sed -n '1,20p' "$MAP_DIR/tomato_field.yaml"
```

至少应生成非空的 `tomato_field.yaml` 和 `tomato_field.pgm`；序列化成功时还会生成
`tomato_field.posegraph` 和 `tomato_field.data`。YAML 中分辨率应为 `0.05`，`image`
应指向同目录的 PGM。保存前还应在 RViz 确认所有目标通道均为已知自由空间、行末有足够
转弯区、闭环后没有双墙或双排植株。

### 8. 停止建图并重启同一场景

保存完成后依次停止遥控、SLAM、RViz 和仿真。为了让导航初始位姿可重复，推荐重新以
同一 SDF 和同一出生点启动仿真，而不是让机器人停留在巡图终点。所有旧仿真相关节点
都退出后，再次执行第 3 步的仿真命令；只把 `GZ_PARTITION` 换成新的唯一值，例如：

```bash
export GZ_PARTITION="agri_nav_${ROS_DOMAIN_ID}_$(date +%s%N)"
```

如果不重启仿真，就必须记录建图终点的 `map → base_footprint` 位姿，并把该 `x/y/yaw`
传给 AMCL，或者在 RViz 用 `2D Pose Estimate` 设置当前位置；巡图终点不能直接填
`0,0,0`。

### 9. 启动保存地图定位与 Nav2

仿真已经从相同出生点重新启动后，在第二个终端执行：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

FIELD_ID="rows6_n10_row2p0_plant0p7_h2p0_w1p0_c0p06_seed42"
MAP_DIR="$PWD/artifacts/maps/$FIELD_ID"
test -s "$MAP_DIR/tomato_field.yaml"
test -s "$MAP_DIR/tomato_field.pgm"

ros2 launch agri_navigation navigation.launch.py \
  map:="$MAP_DIR/tomato_field.yaml" \
  use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

这里的 `0,0,0` 是本流程中 SLAM 开始时的机器人 `map` 位姿，不是 Gazebo world 坐标。
若激光与地图没有重合，应先在 RViz 用 `2D Pose Estimate` 校正，不能靠反复发送导航目标
修正定位。确认没有 `/slam_toolbox` 后，再检查七个生命周期节点和 Action server：

```bash
for node in map_server amcl controller_server planner_server \
  behavior_server bt_navigator collision_monitor; do
  ros2 lifecycle get "/$node"
done

ros2 action info /navigate_to_pose
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

七个节点都应为 `active [3]`，`/navigate_to_pose` 应显示 `Server count: 1`。最后一个
TF 命令会持续输出，确认坐标稳定后按 `Ctrl+C` 结束。导航测试前要关闭
`teleop_twist_keyboard`，否则高优先级手动速度源可能暂时压住 Nav2。

### 10. 发送 0.5 m 短距离目标

先在 RViz 中确认 `map` 坐标 `(0.50, 0.00)` 位于中央通道自由栅格，再执行：

```bash
ros2 action send_goal -f -t 120 \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {
    position: {x: 0.50, y: 0.00, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }}}"
```

成功时 Action 结果应为 `SUCCEEDED`，机器人最终停止，定位无跳变。配置中的目标容差为
位置 0.10 m、航向约 0.0873 rad。确认原点在 RViz 中是自由空间后，可把目标位置改成
`x: 0.00, y: 0.00` 测试返回起点。

### 11. 常见故障定位

- 一直显示 `Waiting for an action server`：检查命令是否误写成 `os2`，确认所有终端的
  `ROS_DOMAIN_ID` 相同，并运行 `ros2 lifecycle get /bt_navigator`、
  `ros2 action list -t | rg navigate`；这类等待与目标坐标是否可达无关。
- 出现 `Detected jump back in time` 或大量 `TF_OLD_DATA`：停止本轮操作，关闭旧仿真和
  所有使用旧仿真时间的 ROS 节点，使用新 `GZ_PARTITION` 完整重启。不要在异常时保存图。
- AMCL 有输出但机器人位置偏移：检查是否加载了旧 `stage3_baseline.yaml`、SDF 与地图的
  `FIELD_ID` 是否一致，以及 `/scan` 在 RViz 中是否与静态地图重合。
- 有路径但机器人不动：按顺序检查速度链：

  ```bash
  ros2 topic echo /cmd_vel_nav_raw --once
  ros2 topic echo /cmd_vel_nav --once
  ros2 topic echo /cmd_vel_safe --once
  ros2 topic echo /collision_monitor_state --once
  ros2 topic echo /base_motion/locked --once
  ros2 topic echo /base_motion/active_source --once
  ros2 control list_controllers
  ```

  `/cmd_vel_nav_raw` 非零而 `/cmd_vel_nav` 为零时检查 collision monitor；
  `/cmd_vel_nav` 非零而 `/cmd_vel_safe` 为零时检查运动锁、速度门控和旧遥控进程；
  `/cmd_vel_safe` 非零但车辆不动时检查四舵轮节点与三个控制器。

当前阶段的 `NavigateToPose` 属于仿真人工验收，仓库还没有完整的端到端导航自动测试。
地图、SDF、元数据、pose graph、后续 keepout mask 和停车点必须使用同一个
`FIELD_ID` 管理。

如果已经在本轮更新前构建过工作空间，先重新安装仿真启动文件：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot/sim_ws"
source /opt/ros/jazzy/setup.bash
source ../ros2_ws/install/setup.bash
colcon build --symlink-install --packages-select agri_sim_description agri_sim_bringup
cd ..
```

## 启动基线

先构建并加载两个工作空间：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96
# 每次仿真使用新分区，避免上一次未完全退出的 Gazebo 时钟串入本次运行。
export GZ_PARTITION="agri_stage4_${ROS_DOMAIN_ID}_$(date +%s%N)"
```

先启动仓库默认的 6 行×10 株番茄田、底盘、里程计、MID-360 和 `/scan`。
省略 `world` 参数时会使用 `tomato_field_default.sdf`。仓库内的
`stage3_baseline.yaml` 是旧阶段测试夹具，不与该默认场景匹配；完成本节前半部分建图并
显式传入同一 `FIELD_ID` 的地图后，才能启动导航：

```bash
ros2 launch agri_sim_bringup tomato_field.launch.py \
  gz_partition:="$GZ_PARTITION" \
  gui:=true paused:=false rviz:=true \
  use_control:=true use_kinematics:=true \
  use_lidar:=true use_scan:=true use_camera:=false
```

仿真入口将当前 world 的专属 Gazebo 时钟以 `CLOCK` QoS 发布为 ROS `/clock`。
若 `rviz2`、TF、里程计和四舵轮节点连续报告时间倒退或旧数据，在仿真终端执行：

```bash
ros2 topic info /clock --verbose --no-daemon --spin-time 8
gz topic -l | rg '^/clock$|^/world/.*/clock$'
gz topic -i -t /world/field/clock
```

ROS `Publisher count` 应为 `1`，发布节点应为 `/clock_bridge`，QoS 应为 best effort、
volatile、depth 1；`/world/field/clock` 应只有一个 Gazebo publisher 地址。只检查 ROS
发布者数量不足以判断 bridge 上游是否混入多个 Gazebo 时钟。自定义 SDF 使用其他 world
名称时，把命令中的 `field` 换成 `gz topic -l` 列出的名称。若 ROS 侧仍有旧 bridge，
关闭旧 launch，或在仿真与导航终端统一换一个未使用的 `ROS_DOMAIN_ID`。

如果只检查保存地图定位，在另一终端加载环境后启动：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

ros2 launch agri_navigation localization.launch.py \
  params_file:="$(ros2 pkg prefix --share agri_navigation)/config/amcl.yaml" \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0 \
  use_sim_time:=true
```

需要运行 Nav2 单目标导航时，改为在第二终端启动下面的总入口；它已经包含定位节点，
不要同时运行上面的 `localization.launch.py`：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96

ros2 launch agri_navigation navigation.launch.py \
  params_file:="$(ros2 pkg prefix --share agri_navigation)/config/nav2_navigation.yaml" \
  amcl_params_file:="$(ros2 pkg prefix --share agri_navigation)/config/amcl.yaml" \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0 \
  use_sim_time:=true
```

省略 `map` 参数时会使用 `agri_navigation/maps/stage3_baseline.yaml`，它只是旧阶段测试
夹具，不适用于当前默认番茄田。完成建图后必须显式传入同一 `FIELD_ID` 的 YAML。`params_file`
覆盖 Nav2 planner/controller/costmap 参数，`amcl_params_file` 覆盖 AMCL
参数；不要在同一 ROS 域中再启动 `mapping.launch.py`。导航节点先发布
`/cmd_vel_nav_raw`，collision monitor 输出 `/cmd_vel_nav`，再由
`agri_base_adapter` 门控到 `/cmd_vel_safe`。确认生命周期节点 active 后，在 RViz 的
2D Pose Estimate 设置初始位姿，或发布 `geometry_msgs/msg/PoseWithCovarianceStamped`
到 `/initialpose`。

如果已经按后文生成并建图了自定义田场景，先定义真实的场景版本，再显式传入 SDF 和
地图。`FIELD_ID` 只是文档中的版本变量，不能把字符串 `FIELD_ID` 原样写进路径：

```bash
FIELD_ID="rows6_n10_row2p0_plant0p7_h2p0_w1p0_c0p06_seed42"

ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$PWD/artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  gz_partition:="$GZ_PARTITION" \
  gui:=true paused:=false rviz:=true \
  use_control:=true use_kinematics:=true \
  use_lidar:=true use_scan:=true use_camera:=false
```

另一终端加载相同环境、设置相同 `ROS_DOMAIN_ID` 和 `FIELD_ID` 后启动导航：

```bash
FIELD_ID="rows6_n10_row2p0_plant0p7_h2p0_w1p0_c0p06_seed42"
ros2 launch agri_navigation navigation.launch.py \
  map:="$PWD/artifacts/maps/$FIELD_ID/tomato_field.yaml" \
  use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

单目标调试可以使用 Nav2 标准 Action：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {
    position: {x: 1.20, y: -0.80, z: 0.0},
    orientation: {z: 0.7071, w: 0.7071}}}}"
```

坐标必须是当前地图的 `map` 坐标。示例位置和航向只用于说明消息格式，不能直接当作
番茄田采摘站点。

## 阶段四验收清单

建议把下列项目写入 `agri_sim_tests` 的导航检查器或人工验收记录：

1. 启动检查：`map_server`、AMCL、Planner、Controller、BT Navigator 等生命周期节点
   active；SLAM Toolbox 未运行；`map → odom → base_footprint` 只有一个发布者。
2. 定位检查：发布 `/initialpose` 后观察 `/amcl_pose` 和 TF，确认粒子收敛、协方差在项目
   阈值内且短时间内没有跳变。
3. 单目标检查：在行间入口、行中和行末各发送一个目标，记录 Action 结果、最终位置/航向
   误差、路径是否进入植株禁行区域，以及最终 `/cmd_vel_safe` 是否归零。
4. 障碍检查：在局部代价地图范围内加入临时障碍，确认障碍层、膨胀层、重新规划和速度
   门控生效；目标取消、传感器超时和控制器失活时应停车。
5. 数据记录：至少记录 `/map`、`/tf`、`/tf_static`、`/scan`、`/odom`、`/amcl_pose`、
   `/plan`、`/local_plan`、`/cmd_vel_nav` 和 `/cmd_vel_safe`，并保存地图与场景清单。

常用检查命令：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic hz /scan
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action info /navigate_to_pose
```

## 番茄田布局变更与代价地图更新

阶段四使用的是保存的静态 `OccupancyGrid`。改变植株间距、行数、每行株数或行间距后，
原地图中的墙、土槽和植株位置已经失效，不能只修改 Nav2 参数中的分辨率或膨胀半径。
按以下顺序生成新版本：

1. 用场地生成器重新生成 SDF 和元数据，至少同步修改 `--rows`、
   `--plants-per-row`、`--plant-spacing`、`--row-spacing`；若边界变化，同时修改
   `--origin-x/--origin-y` 和 `--ground-x/--ground-y`。行距从 2 m 改到 3 m 时，
   可以减少行数以适配原 22×14 m 地面；例如 6 行、`origin-x=-7.5` 可在 `x=0`
   留出中心间距为 3 m 的中央通道（1 m 视觉冠幅后净宽约 2 m）。每行株数控制 Y 向长度，
   可按需要减少，不必为了行距改变而盲目保留
   10 行。
2. 用新 SDF 重启 Gazebo，重新启动 `/scan` 和 `mapping.launch.py`，沿每条需要通行的
   行间与行末采集闭合轨迹。
3. 保存新的 YAML/PGM 和 pose graph；为地图记录 SDF、生成参数、随机种子、分辨率、原点
   与时间戳。导航启动时把 `map` 指向这一版本的 YAML。
4. 如果已经生成植株行 keepout mask、禁行层或 `waypoints.yaml`，必须使用新地图和新布局
   重新生成，并更新地图/布局校验值。旧航点不能因为坐标数值“看起来接近”而继续使用。
5. Nav2 的局部障碍层会根据实时 `/scan` 自动更新，通常不需要手工编辑局部代价栅格；
   但静态层、keepout 层、footprint、膨胀半径和行末转弯空间仍要按新通道复核。

参数与场景几何的对应关系如下：

| 场景变化 | 生成器参数 | 需要重新生成的内容 |
| --- | --- | --- |
| 行数 | `--rows` | SDF、SLAM 地图、植株 keepout、路线/站点 |
| 每行植株数量 | `--plants-per-row` | SDF、元数据、keepout、路线/站点 |
| 行间距 | `--row-spacing` | SDF、SLAM 地图、通道 footprint/膨胀复核、keepout、路线 |
| 株距 | `--plant-spacing` | SDF、元数据、keepout、扫描/停车站点 |
| 植株简化碰撞盒宽度 | `--plant-collision-width` | SDF、果实净空；若用于规划还要重新建图并复核 `/scan` |
| 植株简化碰撞盒高度 | `--plant-collision-height` | SDF、果实净空；机械臂/果实高度检查 |
| 场地边界或首株位置 | `--ground-x/--ground-y`、`--origin-x/--origin-y` | SDF、SLAM 地图边界、静态层和路线 |

例如将回归场景改为 8 行、每行 12 株、行距 1.8 m、株距 0.6 m 时，只需先替换生成器
参数并使用新的 `FIELD_ID`；不要在旧地图上直接改四个数字：

```bash
FIELD_ID="rows8_n12_row1p8_plant0p6_seed42"
mkdir -p "artifacts/fields/$FIELD_ID"
python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --rows 8 --plants-per-row 12 --row-spacing 1.8 --plant-spacing 0.6 \
  --plant-height 2.6 --plant-width 1.0 \
  --randomize-fruits --fruit-visual mesh \
  --seed 42 \
  --output "artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  --metadata "artifacts/fields/$FIELD_ID/tomato_field.json"
```

随后按上面的建图、保存地图和 keepout mask 命令完成新版本；如果仅修改果实数量、结果
高度或成熟比例而不改变植株碰撞边界，则不必重建导航静态地图。

当前默认导航建图使用 2 m 行距的 6 行×10 株 profile。它保持原地面尺寸，
将 6 行布置在 `x=-5…5`，并把简化茎杆碰撞盒 X/Y 缩为 0.06 m；枝叶视觉冠幅仍为
1 m。完整生成说明见 [植株尺寸与番茄果实参数化](tomato_fruit_randomization.md)
的“导航建图默认场景（2 m 行距）”一节。需要更宽的转弯通道时，可以改用该文档中的
“3 m 行距的宽通道备选场景”，但必须使用独立 `FIELD_ID` 重新建图。碰撞盒缩小只改变
Gazebo 物理碰撞，GPU LiDAR 仍可能看到枝叶视觉网格。

建图时使用同一场景和出生点：

```bash
ros2 launch agri_navigation mapping.launch.py use_sim_time:=true
# 沿中央通道、各行入口/行末行驶并闭环后保存：
MAP_DIR="artifacts/maps/$FIELD_ID"
mkdir -p "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/tomato_field" \
  --ros-args -p use_sim_time:=true \
    -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_DIR/tomato_field'}"
```

停止 mapping 后再加载这张同版本地图：

```bash
ros2 launch agri_navigation navigation.launch.py \
  map:="$PWD/$MAP_DIR/tomato_field.yaml" \
  use_sim_time:=true \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

若机器人仍在原点附近被判定为障碍，先在 RViz 重新发送 `2D Pose Estimate`，再检查
`/scan`、`/amcl_pose` 和 `ros2 run tf2_ros tf2_echo map base_footprint`。旧的
`stage3_baseline.yaml` 与当前参数化场景坐标不匹配，不能继续用于定位；静态地图、
keepout mask、路线和停车点必须使用同一 `FIELD_ID`。

仓库中的 `generate_keepout_mask.py` 可以把植株元数据投影到保存地图的同尺寸栅格。它
要求显式提供 world→map 平面变换，避免把 Gazebo world 坐标误当成导航 map 坐标：

```bash
python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_keepout_mask.py \
  --metadata "artifacts/fields/$FIELD_ID/tomato_field.json" \
  --map-yaml "artifacts/maps/$FIELD_ID/tomato_field.yaml" \
  --output "artifacts/maps/$FIELD_ID/keepout_mask" \
  --robot-start-world 0.0 -6.0 1.5708 \
  --rows 6 --plants-per-row 10 \
  --row-spacing 2.0 --plant-spacing 0.7 --plant-width 1.0 \
  --keepout-margin 0.15
```

`--robot-start-world` 填开始建图时机器人在 Gazebo world 中的 `x y yaw`，脚本会推导
world→map 平面变换。当前默认出生点对应 `0 -6 1.5708`；若使用外部配准结果，可改用
`--world-to-map TX TY YAW`。只有确认 map 与 world 原点、朝向一致时才可以填
`--world-to-map 0 0 0`。输出的 `keepout_mask.yaml/.pgm` 必须与同一地图版本绑定，再由
Nav2 KeepoutFilter 配置加载；阶段四当前只启用静态/障碍/膨胀层，所以生成遮罩不会自动
改变代价地图，KeepoutFilter 在阶段五路线开发时接入。

推荐为每次场景建立独立目录：

```bash
FIELD_ID="rows6_n10_row2p0_plant0p7_h2p0_w1p0_c0p06_seed42"
mkdir -p "artifacts/fields/$FIELD_ID" "artifacts/maps/$FIELD_ID"

python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --rows 6 --plants-per-row 10 \
  --row-spacing 2.0 --plant-spacing 0.7 \
  --origin-x -5.0 --origin-y -5.0 \
  --ground-x 22.0 --ground-y 14.0 \
  --plant-height 2.0 --plant-width 1.0 \
  --plant-collision-width 0.06 \
  --randomize-fruits --fruit-visual mesh \
  --fruit-count-min 2 --fruit-count-max 6 \
  --fruit-height-min 0.80 --fruit-height-max 1.60 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --seed 42 \
  --output "artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  --metadata "artifacts/fields/$FIELD_ID/tomato_field.json"
```

阶段四的默认回归场景采用 6 行×10 株、行距 2.0 m、株距 0.7 m；这是当前场地生成器
按开源番茄温室行列组织方式整理的可复现 profile。它不是对方 `map` 坐标或温室尺寸的
直接复制；更换机器人 footprint、温室边界或行向后，必须重新建图和标定站点。

只改变果实数量、果实高度或成熟比例时，植株碰撞边界和通道通常不变，可以复用地图；
改变植株冠幅、株高、株距或行距导致通道边界变化时，应按上述流程重新生成地图并复核
footprint、keepout mask 和停车点。
