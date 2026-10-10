# 植株尺寸与番茄果实参数化

实际温室尺寸尚未确定，本功能先提供可复现的参数生成流程。以下数值为测试示例，
不是实测株高、冠幅或结果高度。可以分别调整枝叶株高、冠幅、简化茎杆碰撞盒，
以及独立果实的数量、高度和直径。不传植株尺寸且不启用 `--randomize-fruits` 时，旧的
150 株兼容基线保持逐字节复现；项目启动默认场景的 6 行×10 株 profile 见下文。

## 生成并查看

更新功能后先构建 `agri_greenhouse_worlds` 和支持自定义世界路径的启动入口：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cd sim_ws
colcon build --symlink-install --packages-select agri_greenhouse_worlds agri_sim_bringup
source install/setup.bash
cd ..

python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --randomize-fruits --fruit-visual mesh \
  --plant-height 2.0 --plant-width 1.0 \
  --rows 2 --plants-per-row 3 --origin-x -1 --origin-y -5 \
  --fruit-count-min 2 --fruit-count-max 5 \
  --fruit-height-min 0.80 --fruit-height-max 1.60 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --seed 42 \
  --output artifacts/tomato_random_demo.sdf \
  --metadata artifacts/tomato_random_demo.json

export ROS_DOMAIN_ID=94
export GZ_PARTITION=agricultural_bot_random_fruits
ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$PWD/artifacts/tomato_random_demo.sdf" \
  gz_partition:="$GZ_PARTITION" gui:=true paused:=true
```

此示例使用 2 m 株高、1 m 冠幅和 0.80–1.60 m 果实中心高度，均为测试值，
实际温室尺寸仍待实测。示例生成 6 株，便于先检查画面。删除 `--rows`、`--plants-per-row`、
`--origin-x`、`--origin-y` 四个覆盖项，会恢复生成器原有的 10 行 × 15 株基线布局；
当前导航建图默认场景使用后文单独定义的 6 行 × 10 株 profile。
修改参数后需重新生成文件，并退出对应 Gazebo 后重启；已加载的世界不会自动刷新。
始终显式填写 `--output`，避免覆盖正式基线或向安装目录写入场景。

随机场景默认使用原番茄果实网格和贴图。已经生成的旧球体场景需要重新执行生成命令，
然后重启 Gazebo，才能看到新果形。临时比较简化球体时可改用 `--fruit-visual sphere`；
两种视觉模式在同参数、同种子下的果实数量、中心位置和成熟状态相同。

需要底盘控制时，将启动参数改为 `paused:=false use_control:=true use_kinematics:=true`。
控制终端也必须设置同一个 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`，具体运动测试见
[底盘运动指南](chassis_field_motion.md)。传感器仍可用 `use_camera:=true`、
`use_lidar:=true` 启用；本轮随机化验证不替代传感器联合验收。

## 同时启动底盘、参数化番茄田和传感器（150 株示例）

先在旧仿真的启动终端按 Ctrl+C 退出，再在终端一执行。此例为 150 株，
株高 2 m、冠幅 1 m、每株 2–6 个果实、中心高度 0.80–1.60 m（测试值），启用底盘控制、MID-360、
D405 的 2 m 扩展仿真量程和 RViz：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=94
export GZ_PARTITION=agricultural_bot_random_fruits

python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --randomize-fruits --fruit-visual mesh \
  --plant-height 2.6 --plant-width 1.0 \
  --rows 10 --plants-per-row 15 \
  --fruit-count-min 2 --fruit-count-max 6 \
  --fruit-height-min 0.80 --fruit-height-max 1.60 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --ripe-ratio 0.7 --seed 42 \
  --output artifacts/tomato_random_field.sdf \
  --metadata artifacts/tomato_random_field.json

ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$PWD/artifacts/tomato_random_field.sdf" \
  gz_partition:="$GZ_PARTITION" \
  gui:=true paused:=false rviz:=true \
  use_control:=true use_kinematics:=true \
  use_lidar:=true lidar_mode:=gpu_lidar \
  use_camera:=true \
  camera_config:="$(ros2 pkg prefix --share agri_sim_sensors)/config/d405_extended_sim.yaml"
```

终端二加载同样的环境和隔离参数后遥控：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=94
export GZ_PARTITION=agricultural_bot_random_fruits

ros2 control list_controllers
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint -p use_sim_time:=true \
  -p speed:=0.1 -p turn:=0.2
```

确认 `steering_controller` 和 `wheel_controller` 为 `active` 后操作，英文输入法下
`i` 前进、`,` 后退、`k` 停车。已安装 RGL 后端时可将 `lidar_mode:=gpu_lidar`
改为 `lidar_mode:=rgl`。这些组合参数已核对启动入口，尚未对完整 150 株新网格场景
重新执行底盘与两种传感器联合验收。

## 导航建图默认场景（2 m 行距）

当前导航建图默认使用 22×14 m 地面、6 行×10 株、行距 2 m、株距 0.7 m。6 行的
世界 X 坐标为 `-5、-3、-1、1、3、5`，因此机器人从世界坐标
`(0,-6,1.5708)` 出生时位于 `x=-1` 与 `x=1` 两行之间，并沿世界 `+Y` 进入中央通道。
株高为 2 m、视觉冠幅为 1 m，简化茎杆碰撞盒 X/Y 为 0.06 m。该 profile 共生成
60 株；它与地图、pose graph、keepout mask 和停车点共同使用同一个 `FIELD_ID`：

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

最后一条命令应输出 `Valid.`。完整的仿真启动、SLAM、地图保存和导航测试流程见
[阶段四导航文档](navigation_stage4.md#当前默认-2-m-番茄田从参数修改到导航测试的完整流程)。

## 3 m 行距的宽通道备选场景

默认地面为 22×14 m。若仍保留 10 行并把行距改成 3 m，原来的 `origin-x=-9` 会让
植株位置扩展到 `x=18`，超出地面边界；同时可能把一行放在机器人出生通道上。测试阶段
可把行数改为偶数 6，使行列以 `x=0` 对称并留下中心间距为 3 m 的中央通道（1 m
视觉冠幅扣除后净宽约 2 m）。每行植株数量只影响
沿 Y 方向的长度，因此先保留 15 株；需要更短的测试路线时可改成 8 或 10 株。
生成器现在会在写文件前拒绝越出地面边界的行列布局，报错时按提示同步调整行数、
每行株数、首株坐标或地面尺寸。

下面的备选 profile 保持 22×14 m 地面、6 行×15 株、行距 3 m、株距 0.7 m，视觉冠幅为
1 m，简化茎杆碰撞盒 X/Y 为 0.06 m。`--plant-collision-height` 未指定时按株高比例
保留完整的茎杆高度；若只关心底盘通行，优先缩小 X/Y，不要把视觉冠层一起缩小：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
FIELD_ID="rows6_n15_row3p0_plant0p7_h2p6_w1p0_c0p06_seed42"
mkdir -p "artifacts/fields/$FIELD_ID"

python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --rows 6 --plants-per-row 15 \
  --row-spacing 3.0 --plant-spacing 0.7 \
  --origin-x -7.5 --origin-y -5.0 \
  --ground-x 22.0 --ground-y 14.0 \
  --plant-height 2.6 --plant-width 1.0 \
  --plant-collision-width 0.06 \
  --randomize-fruits --fruit-visual mesh \
  --fruit-count-min 2 --fruit-count-max 6 \
  --fruit-height-min 0.80 --fruit-height-max 1.60 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --ripe-ratio 0.7 --seed 42 \
  --output "artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  --metadata "artifacts/fields/$FIELD_ID/tomato_field.json"

source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=96
export GZ_PARTITION="agri_row3_${ROS_DOMAIN_ID}_$(date +%s%N)"

ros2 launch agri_sim_bringup tomato_field.launch.py \
  world:="$PWD/artifacts/fields/$FIELD_ID/tomato_field.sdf" \
  gz_partition:="$GZ_PARTITION" \
  spawn_x:=0.0 spawn_y:=-6.0 spawn_yaw:=1.5708 \
  gui:=true paused:=false rviz:=true \
  use_control:=true use_kinematics:=true \
  use_lidar:=true use_scan:=true use_camera:=false
```

6 行的横向坐标为 `-7.5、-4.5、-1.5、1.5、4.5、7.5`，所以默认出生点
`(0,-6,1.5708)` 位于中央通道。若改成 4 行或 8 行，仍应使用偶数行并重新计算
`origin-x = -(rows - 1) * row-spacing / 2`；若每行株数改变，按
`origin-y`、`plant-spacing` 重新检查 Y 边界。每次布局变化都必须重新建图，旧地图和
旧停车点不能继续配套使用。

## 参数含义

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--plant-height` | 不覆盖：1.298104 | 枝叶冠顶离地高度，单位 m；只调整枝叶 Z 缩放 |
| `--plant-width` | 不覆盖：0.866695 | 偏航前枝叶局部 X/Y 包围盒跨度的较大值，单位 m；X/Y 同倍缩放，与株高独立 |
| `--plant-collision-width` | 不覆盖：按冠幅缩放 | 简化茎杆碰撞盒的 X/Y 边长，单位 m；正方形，独立于枝叶冠幅 |
| `--plant-collision-height` | 不覆盖：按株高缩放 | 简化茎杆碰撞盒的 Z 高度，单位 m；独立于枝叶株高，底面保持贴地 |
| `--randomize-fruits` | 关闭 | 移除旧模型中的固定果实，生成独立随机果实 |
| `--fruit-visual` | `mesh` | 原单果网格与贴图；`sphere` 可回退纯色球体 |
| `--fruit-count-min / --fruit-count-max` | 2 / 6 | 每株整数数量范围，包含两个端点；允许 0 |
| `--fruit-height-min / --fruit-height-max` | 0.60 / 1.10 | 果实中心的世界 Z 坐标，单位 m，地面 Z=0 |
| `--fruit-diameter-min / --fruit-diameter-max` | 0.06 / 0.09 | 果实包围球直径范围，单位 m；网格等比例缩放 |
| `--fruit-radius-min / --fruit-radius-max` | 0.12 / 0.25 | 茎杆中心到果实中心的水平距离，单位 m |
| `--ripe-ratio` | 0.7 | 每个果实取红色的概率；其余为绿色，不保证总数恰好 70% |
| `--fruit-min-clearance` | 0.01 | 果实与其他果实、地面和简化茎杆碰撞体的最小净间距，单位 m |
| `--seed` | 42 | 随机种子；同参数、同种子生成相同世界与标注 |
| `--metadata` | 不输出 | 可选 JSON，记录参数、植株几何、每株位置及逐果 ID、世界坐标、包围球半径和成熟状态 |

上下限相同可固定该维度。水平距离 `fruit-radius` 不是果实球体半径。
网格保持原果形，三轴尺寸约为包围球直径的 0.932、0.904、0.941 倍；
`diameter` 不是每一轴的真实长度，JSON 中的 `radius` 是碰撞/包围球半径。
高度以整个世界地面为基准：例如 0.35 m 高种植床上方 0.80 m 的果实中心应填 1.15 m。
果实数量均匀抽样；位置在水平半径、高度及方位范围内抽样，并拒绝发生上述碰撞的候选。
因此，在拥挤参数下，最终位置分布会受到间距约束。

生成器会拒绝非有限数值、颠倒范围及会侵入地面/茎杆的参数。若布局过密，
单果尝试 1000 次仍无法放置则报错，保留已有输出文件；可减小数量/直径，
或扩大高度范围、水平半径范围和株距。
JSON 用于离线核对和评测，不自动发布给 YOLO 或机器人算法作为定位结果。

## 独立调整株高和冠幅

`--plant-height` 和 `--plant-width` 调整枝叶视觉网格，并在未指定独立碰撞参数时
按比例推导简化茎杆碰撞体；不会自动改变独立果实的直径、世界 Z 高度或水平分布半径。
只增高植株时，
可以仅传 `--plant-height`，冠幅保持原值；只增大冠幅时仅传 `--plant-width`，
株高保持原值。`--plant-collision-width` 和 `--plant-collision-height` 可覆盖碰撞盒
的对应尺寸，两者均须为有限正数。

缩放以地面 Z=0 和原模型坐标为基准：

```text
枝叶 X/Y 缩放倍数 = plant-width / 0.866695
枝叶 Z 缩放倍数   = plant-height / 1.298104
茎杆碰撞体 X/Y    = plant-collision-width（未指定时为 0.1 × X/Y 倍数）
茎杆碰撞体 Z      = plant-collision-height（未指定时为 1.2416 × Z 倍数）
茎杆中心 Z        = 茎杆碰撞体高度 / 2
```

上述茎杆公式适用于随机果实模式或显式设置植株尺寸。未启用随机化且未传植株尺寸时，
保留基线旧碰撞体，尺寸仍为 `[0.1, 0.1, 1.0]`，中心 Z 为 `0`，不改变原场景。

这里的株高是枝叶冠顶到地面的高度；原叶片最低点略低于地面，不以整个网格
Z 包围盒跨度定义株高。冠幅是偏航前局部 X/Y 跨度的最大值，并非要求两个方向
都恰好等于该数值；设置 1 m 冠幅时，仍保留原 X/Y 长宽比例。植株随机偏航后，
世界轴方向上的包围盒宽度会随角度变化。

果实独立缩放，所以增高枝叶不会拉长番茄。使用默认推导碰撞盒时，增大冠幅会同时
加宽茎杆碰撞体；若显式指定 `--plant-collision-width`，果实净空检查使用该实际宽度。
`--fruit-radius-min` 至少须满足：

```text
fruit-radius-min >= collision-width / 2 + fruit-diameter-max / 2 + fruit-min-clearance

其中 collision-width 为 --plant-collision-width；未指定时为 0.1 × X/Y 缩放倍数。
```

生成器还会检查旋转茎杆和跨株果实的实际净空。拥挤布局中的候选拒绝可能改变
随机位置；相同尺寸、其他参数和种子仍可复现。扩大冠幅不会自动扩大行距、株距或
果实水平分布范围，应同时检查行间通道，必要时调整 `--row-spacing`、
`--plant-spacing` 和果实半径范围。叶片仍为视觉网格，检查不包含冠层互相遮挡。

未启用 `--randomize-fruits` 时也可调整枝叶尺寸，但原模型的固定果实和花朵保持
原位置及原大小，果实参数不生效。需要改变果实数量、高度和直径时，应启用
`--randomize-fruits`，由独立果实模型替代原固定果实。

显式指定任意植株尺寸或碰撞盒尺寸时，生成器把调整后的植物模型写入输出 SDF；
它继续引用原网格和贴图，不修改或覆盖共享资产。不指定这两个参数时仍使用原
模型引用方式。JSON 的 `plant_geometry` 记录实际生效的 `height`、`width`、
`mesh_scale: [X, Y, Z]`、`stem_size: [X, Y, Z]` 和 `stem_center_z`，
后两项是实际茎杆碰撞体的尺寸与中心 Z，便于核对配置和生成结果。

## 模型结构与限制

原模型的番茄嵌入 DAE，不能仅靠修改世界中一个数字独立调整每个果实。
新增 `tomato_plant` 仅保留原资产的茎杆与叶片，继续引用已有网格和纹理；
随机果实为独立静态模型，默认使用从原 `Fruit1` 提取的单个果体网格，
保留原法线和 UV；红果使用原 `AG15frt1.png`，青果使用原 `AG15frt4.png`。
每个 ID 只对应一个果实，避免复制整串果实或与旧网格番茄重复。
网格中心归零并等比例归一化，世界模型的中心位置和碰撞球中心相同。
视觉网格完整包在简化球形碰撞体内，原间距与地面检查继续有效。
新枝叶资产的简化茎杆碰撞体默认从地面延伸至 1.2416 m，指定植株尺寸时按上述
倍数调整并保持底面贴地；显式碰撞参数可以缩小其 X/Y 或 Z 尺寸；叶片仍只有视觉网格。
碰撞盒缩小不会缩小枝叶视觉网格，也不会直接消除 GPU LiDAR 对叶片的回波。

本阶段适合验证检测、深度反投影及目标选择。果实位置不保证落在真实果柄上，
果柄连接、夹持后脱落、落入筐或计数状态更新尚未实现；不能据此宣称采摘物理完成。
原果体顶部有一个小开口，近景可能看到背景；本次保留原几何，未额外补面。
碰撞球封闭且包住视觉网格，后续需要更真实的果形或物理接触时可再修整视觉网格和碰撞体。

派生资产位于 `sim_ws/src/agri_greenhouse_worlds/models/tomato_fruit/`，
`SOURCE.json` 记录来源及归一化参数。通常直接构建即可；需要从原资产重建单果时，
从工程目录执行 `python3 sim_ws/src/agri_greenhouse_worlds/scripts/extract_tomato_fruit.py`。

原植株最高约 1.298 m，冠幅约 0.815 × 0.867 m，原果实中心约在 0.761–1.020 m。
**果实高度参数不会同步长高枝叶**；高位果实需要配合 `--plant-height` 调整株高。
这些参数只缩放原枝叶结构，不会生成更真实的枝条、果柄或连接关系；果实仍可能
位于没有枝条的位置。待实测株高、冠幅和结果区间确定后，可直接替换测试参数，
需要更真实的植株拓扑时再重建模型。
机械臂零位相机高度约 1.878 m，低位目标还需要通过机械臂俯视或侧向观测及可达性检查，
不能仅靠抬高果实替代机械臂规划。

## 验证

黑盒测试覆盖基线逐字节复现、数量/尺寸/高度范围、种子复现、世界与标注一致、
跨株果实间距、旋转茎杆净空、非法参数和拥挤失败时不覆盖输出，以及派生资产引用。
运行命令：

```bash
python3 -m pytest sim_ws/src/agri_sim_bringup/test/test_random_fruits.py \
  sim_ws/src/agri_sim_bringup/test/test_fruit_mesh.py \
  sim_ws/src/agri_sim_bringup/test/test_plant_dimensions.py \
  sim_ws/src/agri_sim_bringup/test/test_tomato_field.py -q
```

植株尺寸测试 `test_plant_dimensions.py` 用于检查高度/冠幅独立缩放、茎杆碰撞体、
果实参数独立性、JSON 几何标注、非法尺寸及原资产保留。

株高与冠幅参数验证（2026-10-08）：场地包构建通过，bringup 的 24 项相关测试通过，
包括新增 8 项独立尺寸测试。指定 2 m 株高、1 m 冠幅时，枝叶缩放为
`1.1538084332 1.1538084332 1.5407086027`，茎杆碰撞体为
`0.1153808433 × 0.1153808433 × 1.9129438011 m`，底面贴地。
独立 Gazebo 分区中加载 6 株、25 果，核对实际加载的 18 个枝叶视觉缩放值和
运行世界导出的 6 个茎杆碰撞体；1024×768 RGB 渲染正常。
完整 150 株离线生成 636 果（460 个红果），果径与高度均在配置范围内；
未对该完整新尺寸场景重跑底盘、D405 和 MID-360 联合验收。
原模型、网格、纹理和基线世界共 24 个文件的 SHA256 保持一致。
证据位于 `artifacts/plant_dimensions_20261008/`，包括场景、标注、
`generation_report.json`、`runtime_report.json`、运行世界导出、截图和 JUnit。
验证实例已停止。[查看新尺寸植株截图](../artifacts/plant_dimensions_20261008/field.png)。

以下为此前验证记录。

初版球体验证（2026-10-07）：两个相关包构建成功，随机化、旧场景与启动入口共 11 项测试通过。
示例 seed=42 的 6 株生成并在独立 Gazebo 实例查询到 21 个果实；960×640 RGB
验证画面中红绿果实、枝叶和阴影正常。完整 150 株离线生成 625 果（459 个红果），
果实表面最小净间距 0.010641 m，高度与数量均在配置范围内。
原场景及原模型 SHA256 保持一致。
证据位于 `artifacts/random_fruits_20261007/`，包括生成参数、逐果标注、
`generation_report.json`、`runtime_report.json`、`render.png` 和测试报告。
本次仅小场景运行渲染；完整 150 株随机场景的运行性能、D405/MID-360 联合运行与采摘物理待测。

同日原果实网格替换：两个包重新构建成功，网格提取、随机化与旧场景/启动入口
共 16 项测试通过。独立检查原形、三角面方向、法线、UV、单果连通性、原点归零、
视觉包围和重复提取，确认切换 mesh/sphere 不改变随机目标位置及数量。
独立 Gazebo 小场景仍查到 6 株、21 果；场景与原红/青贴图近景渲染正常，
保留原顶部开口。原资产所有文件及基线世界 SHA256 均未改变。
新证据位于 `artifacts/fruit_mesh_20261007/`：`field.png`、`closeup.png`、
`runtime_report.json`、逐果标注及 JUnit。临时近景样果与验证相机不属于生成器输出。
