# 阶段四：保存地图定位与 Nav2 单目标导航

本文说明阶段四的使用边界和场景变更流程。阶段四使用阶段三保存的二维地图，
由 `map_server` 发布地图、AMCL 发布 `map → odom`，Nav2 通过 `/cmd_vel_nav`
发送导航速度；建图模式的 SLAM Toolbox 不应同时运行。

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

先启动仓库自带的阶段四基线番茄田、底盘、里程计、MID-360 和 `/scan`。
省略 `world` 参数时会使用与默认导航地图配套的 `tomato_field_22x14.sdf`：

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

省略 `map` 参数时会使用 `agri_navigation/maps/stage3_baseline.yaml`。`params_file`
覆盖 Nav2 planner/controller/costmap 参数，`amcl_params_file` 覆盖 AMCL
参数；不要在同一 ROS 域中再启动 `mapping.launch.py`。导航节点先发布
`/cmd_vel_nav_raw`，collision monitor 输出 `/cmd_vel_nav`，再由
`agri_base_adapter` 门控到 `/cmd_vel_safe`。确认生命周期节点 active 后，在 RViz 的
2D Pose Estimate 设置初始位姿，或发布 `geometry_msgs/msg/PoseWithCovarianceStamped`
到 `/initialpose`。

如果已经按后文生成并建图了自定义田场景，先定义真实的场景版本，再显式传入 SDF 和
地图。`FIELD_ID` 只是文档中的版本变量，不能把字符串 `FIELD_ID` 原样写进路径：

```bash
FIELD_ID="rows10_n15_row2p0_plant0p7_h2p6_w1p0_seed42"

ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$PWD/artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  gz_partition:="$GZ_PARTITION" \
  gui:=true paused:=false rviz:=true \
  use_control:=true use_kinematics:=true \
  use_lidar:=true use_scan:=true use_camera:=false
```

另一终端加载相同环境、设置相同 `ROS_DOMAIN_ID` 和 `FIELD_ID` 后启动导航：

```bash
FIELD_ID="rows10_n15_row2p0_plant0p7_h2p6_w1p0_seed42"
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

针对当前底盘转向测试，可直接使用 3 m 行距的 6 行 profile。它保持原地面尺寸，
把行列居中到 `x=-7.5…7.5`，并把简化茎杆碰撞盒 X/Y 缩为 0.06 m；枝叶视觉冠幅仍为
1 m。完整生成和启动命令见 [植株尺寸与番茄果实参数化](tomato_fruit_randomization.md)
的“3 m 行距的兼容测试场景”一节。这里的碰撞盒缩小只改变 Gazebo 物理碰撞，GPU
LiDAR 仍可能看到枝叶视觉网格，所以必须用该新 SDF 重新跑 SLAM 并保存新地图。

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
`stage3_baseline.yaml` 与新的 3 m 场景坐标不匹配，不能继续用于定位；静态地图、
keepout mask、路线和停车点必须使用同一 `FIELD_ID`。

仓库中的 `generate_keepout_mask.py` 可以把植株元数据投影到保存地图的同尺寸栅格。它
要求显式提供 world→map 平面变换，避免把 Gazebo world 坐标误当成导航 map 坐标：

```bash
python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_keepout_mask.py \
  --metadata "artifacts/fields/$FIELD_ID/tomato_field.json" \
  --map-yaml "artifacts/maps/$FIELD_ID/tomato_field.yaml" \
  --output "artifacts/maps/$FIELD_ID/keepout_mask" \
  --robot-start-world 0.0 -6.0 1.5708 \
  --rows 8 --plants-per-row 12 \
  --row-spacing 1.8 --plant-spacing 0.6 --plant-width 1.0 \
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
FIELD_ID="rows10_n15_row2p0_plant0p7_h2p6_w1p0_seed42"
mkdir -p "artifacts/fields/$FIELD_ID" "artifacts/maps/$FIELD_ID"

python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --rows 10 --plants-per-row 15 \
  --row-spacing 2.0 --plant-spacing 0.7 \
  --plant-height 2.6 --plant-width 1.0 \
  --randomize-fruits --fruit-visual mesh \
  --fruit-count-min 2 --fruit-count-max 6 \
  --fruit-height-min 0.80 --fruit-height-max 1.60 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --seed 42 \
  --output "artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  --metadata "artifacts/fields/$FIELD_ID/tomato_field.json"
```

阶段四的默认回归场景采用 10 行×15 株、行距 2.0 m、株距 0.7 m；这是当前场地生成器
按开源番茄温室行列组织方式整理的可复现 profile。它不是对方 `map` 坐标或温室尺寸的
直接复制；更换机器人 footprint、温室边界或行向后，必须重新建图和标定站点。

只改变果实数量、果实高度或成熟比例时，植株碰撞边界和通道通常不变，可以复用地图；
改变植株冠幅、株高、株距或行距导致通道边界变化时，应按上述流程重新生成地图并复核
footprint、keepout mask 和停车点。
