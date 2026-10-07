# 番茄数量与结果高度随机化

实际温室尺寸尚未确定，本功能先提供可复现的参数生成流程。以下数值为测试示例，
不是实测株高、冠幅或结果高度。未启用 `--randomize-fruits` 时，原有 150 株场景
及 `tomato_0` 模型保持原样。

## 生成并查看

先构建新增的枝叶资产和支持自定义世界路径的启动入口：

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
  --rows 2 --plants-per-row 3 --origin-x -1 --origin-y -5 \
  --fruit-count-min 2 --fruit-count-max 5 \
  --fruit-height-min 0.60 --fruit-height-max 1.10 \
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

此示例生成 6 株，便于先检查画面。删除 `--rows`、`--plants-per-row`、
`--origin-x`、`--origin-y` 四个覆盖项，即使用原有 10 行 × 15 株布局。
修改参数后需重新生成文件，并退出对应 Gazebo 后重启；已加载的世界不会自动刷新。
始终显式填写 `--output`，避免覆盖正式基线或向安装目录写入场景。

随机场景默认使用原番茄果实网格和贴图。已经生成的旧球体场景需要重新执行生成命令，
然后重启 Gazebo，才能看到新果形。临时比较简化球体时可改用 `--fruit-visual sphere`；
两种视觉模式在同参数、同种子下的果实数量、中心位置和成熟状态相同。

需要底盘控制时，将启动参数改为 `paused:=false use_control:=true use_kinematics:=true`。
控制终端也必须设置同一个 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`，具体运动测试见
[底盘运动指南](chassis_field_motion.md)。传感器仍可用 `use_camera:=true`、
`use_lidar:=true` 启用；本轮随机化验证不替代传感器联合验收。

## 同时启动底盘、参数化番茄田和传感器

先在旧仿真的启动终端按 Ctrl+C 退出，再在终端一执行。此例为 150 株、每株
2–6 个果实、中心高度 0.60–1.10 m（测试值），启用底盘控制、MID-360、
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
  --rows 10 --plants-per-row 15 \
  --fruit-count-min 2 --fruit-count-max 6 \
  --fruit-height-min 0.60 --fruit-height-max 1.10 \
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

## 参数含义

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--randomize-fruits` | 关闭 | 移除旧模型中的固定果实，生成独立随机果实 |
| `--fruit-visual` | `mesh` | 原单果网格与贴图；`sphere` 可回退纯色球体 |
| `--fruit-count-min / --fruit-count-max` | 2 / 6 | 每株整数数量范围，包含两个端点；允许 0 |
| `--fruit-height-min / --fruit-height-max` | 0.60 / 1.10 | 果实中心的世界 Z 坐标，单位 m，地面 Z=0 |
| `--fruit-diameter-min / --fruit-diameter-max` | 0.06 / 0.09 | 果实包围球直径范围，单位 m；网格等比例缩放 |
| `--fruit-radius-min / --fruit-radius-max` | 0.12 / 0.25 | 茎杆中心到果实中心的水平距离，单位 m |
| `--ripe-ratio` | 0.7 | 每个果实取红色的概率；其余为绿色，不保证总数恰好 70% |
| `--fruit-min-clearance` | 0.01 | 果实与其他果实、地面和简化茎杆碰撞体的最小净间距，单位 m |
| `--seed` | 42 | 随机种子；同参数、同种子生成相同世界与标注 |
| `--metadata` | 不输出 | 可选 JSON，记录参数、每株位置及逐果 ID、世界坐标、包围球半径和成熟状态 |

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

## 模型结构与限制

原模型的番茄嵌入 DAE，不能仅靠修改世界中一个数字独立调整每个果实。
新增 `tomato_plant` 仅保留原资产的茎杆与叶片，继续引用已有网格和纹理；
随机果实为独立静态模型，默认使用从原 `Fruit1` 提取的单个果体网格，
保留原法线和 UV；红果使用原 `AG15frt1.png`，青果使用原 `AG15frt4.png`。
每个 ID 只对应一个果实，避免复制整串果实或与旧网格番茄重复。
网格中心归零并等比例归一化，世界模型的中心位置和碰撞球中心相同。
视觉网格完整包在简化球形碰撞体内，原间距与地面检查继续有效。
新枝叶资产的简化茎杆碰撞体从地面延伸至 1.2416 m，叶片仍只有视觉网格。

本阶段适合验证检测、深度反投影及目标选择。果实位置不保证落在真实果柄上，
果柄连接、夹持后脱落、落入筐或计数状态更新尚未实现；不能据此宣称采摘物理完成。
原果体顶部有一个小开口，近景可能看到背景；本次保留原几何，未额外补面。
碰撞球封闭且包住视觉网格，后续需要更真实的果形或物理接触时可再修整视觉网格和碰撞体。

派生资产位于 `sim_ws/src/agri_greenhouse_worlds/models/tomato_fruit/`，
`SOURCE.json` 记录来源及归一化参数。通常直接构建即可；需要从原资产重建单果时，
从工程目录执行 `python3 sim_ws/src/agri_greenhouse_worlds/scripts/extract_tomato_fruit.py`。

原植株最高约 1.298 m，冠幅约 0.815 × 0.867 m，原果实中心约在 0.761–1.020 m。
**果实高度参数不会同步长高枝叶**；超出当前植株的范围会产生悬空目标，需要另改植株结构。
当前保持枝叶尺寸，等待实测株高、冠幅和结果区间后再建模。整体等比例放大会同时改变
果实尺寸和行间净空；只拉伸 Z 会使原网格果实变形，因此应继续分开调整枝叶与独立果实。
机械臂零位相机高度约 1.878 m，低位目标还需要通过机械臂俯视或侧向观测及可达性检查，
不能仅靠抬高果实替代机械臂规划。

## 验证

黑盒测试覆盖基线逐字节复现、数量/尺寸/高度范围、种子复现、世界与标注一致、
跨株果实间距、旋转茎杆净空、非法参数和拥挤失败时不覆盖输出，以及派生资产引用。
运行命令：

```bash
python3 -m pytest sim_ws/src/agri_sim_bringup/test/test_random_fruits.py \
  sim_ws/src/agri_sim_bringup/test/test_fruit_mesh.py \
  sim_ws/src/agri_sim_bringup/test/test_tomato_field.py -q
```

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
