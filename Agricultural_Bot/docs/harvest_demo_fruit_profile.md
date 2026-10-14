# 近侧果实演示场景：生成、检查与后续夹取验证

## 当前使用说明

用户已选择保留原始自然果实分布。默认场景及匹配任务已恢复到施加下述演示果实配置前的
版本：60 株、224 个果实，保留原位置、半径和成熟度，不再使用长绿色支梗。
本文下述外伸果实配置保留为历史方案，不再是默认场景。

靠近植株的替代方案与验证入口见 [harvest_preflight.md](harvest_preflight.md) 第 10 节。
该方案使用独立的仿真物理碰撞导航图及扫描，保留原定位地图。最新小果实候选保留
自然果位，将全部直径缩至 50–60 mm，并已准备完整 20 点复测任务，见该文档第 10.7 节。
实际导航及采摘验收分别记录，不能据此保证每株都有可见、可达的红果。

历史演示场景采用 `harvest_demo_v1`：保留 6 行 × 10 株、3 m 行距、2.4 m 株高、
1 m 冠幅和 0.06 m 茎杆碰撞宽度，在中央通道两侧的 20 株上生成近侧候选果实。
20 个底盘停车位、3 个中转点、home、占据地图和 keepout 沿用已经验收的侧偏路线。
这次完成的是场景准备和离线距离筛选；IK、完整机械臂碰撞、相机视场和实际夹取仍待协作者验证。

## 1. 历史方案参数与结果

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| 作业植株 | `tomato_20`…`tomato_39` | 航点文件中的中央两行 |
| `--harvest-profile-ratio` | `0.85` | 按每株原果数计算近侧配额，四舍五入，至少 1 个 |
| `--harvest-height-min/max` | `1.33 / 1.42` m | 果实中心的 Gazebo world 高度 |
| `--harvest-radius-min/max` | `0.68 / 0.74` m | 茎杆至近侧果实中心的水平距离 |
| `--harvest-sector-half-angle` | `0.25` rad | 朝该株停车位的扇区半角 |
| `--harvest-diameter-min/max` | `0.050 / 0.065` m | 近侧果实直径 |
| `--harvest-tcp-offset` | `0.16` m | 暂定法兰至夹取中心距离，尚未标定 |
| `--harvest-pregrasp-distance` | `0.08` m | 水平接近方向上的预抓取退距 |
| `--harvest-flange-radius` | `0.86` m | 肩部至法兰的距离筛选预算，不是 IK 工作空间 |
| `--harvest-arm-reference-height` | `0.891556378` m | 当前落地车体下的 J2 肩部 world 高度假设 |
| `--harvest-shoulder-lateral-offset` | `0.08` m | J1 轴至肩部的水平最坏偏置 |
| `--harvest-parking-xy/yaw-allowance` | `0.10` m / `0.08726646` rad | 距离筛选假设，取自 map 坐标到站容差；world 实际停车精度尚未达标 |
| `--harvest-preserve-scan-max-z` | `1.22` m | 保护原扫描高度内果实球几何的阈值 |

seed=42 的全场共有 **240 个果实**；作业 20 株共有 94 个，其中 **66 个近侧候选，
56 个成熟候选**，每株至少 1 个成熟候选。其余作业株背景果为绿色。原 78 个作业株果实中
28 个处于保护高度，保留其 ID、坐标和半径；必要时追加近侧果实，所以配额 `0.85`
不等于最后果数的 85%，本例有效近侧比例为 `66/94≈70.2%`。非作业 40 株沿用原种子结果。

这些果实伸出原冠幅，使用半径 4 mm 的绿色可视支梗连接到茎杆；支梗没有物理碰撞体，
果实仍有球形碰撞体。该布局服务于演示，不能作为实际植株形态或枝条接触模型。

布局和高度见[候选果实预览图](../artifacts/harvest_demo_profile_20261013/profile_preview.png)，
独立审计结果见[profile_check.json](../artifacts/harvest_demo_profile_20261013/profile_check.json)。

## 2. 距离筛选的假设

以航点变换得到的名义 world 底盘位姿和纵向机械臂偏置 `-0.17998 m` 求 J1 轴的 XY，
Z 取肩部高度 `0.891556378 m`。不能直接使用导航 TF 中的 `base_footprint.z=0`；
落地后的物理底盘约高 0.36 m。

令 `u` 为 J1 水平指向果实的单位向量，果实中心为 `F`：

```text
夹取法兰 P = F - 0.16 u
预抓取法兰 Ppre = F - (0.16 + 0.08) u
停车水平余量 e = 0.10 + 2 × abs(-0.17998) × sin(5°/2) ≈ 0.115701 m
Dmax(P) = hypot(norm(Pxy - J1xy) + 0.08 + e, Pz - 0.891556378)
```

两个法兰位置都必须满足 `Dmax <= 0.86 m`。0.86 m 相对 CR5 手册 0.9 m 名义伸距
留出 0.04 m；停车余量已另外计入。本例最坏夹取距离为约 **0.85124 m**。
生成器拒绝不满足条件或与其他果实、茎杆、地面侵入的样本；放不下整株配额时失败退出，
不会自动放宽预算。

距离条件只是候选筛选。它没有求解关节角，也没有证明指定工具姿态、完整接近路径或
机械臂与植物之间的碰撞净距。元数据统一标为 `distance_candidate_pending_ik`，
`ik_status/full_arm_collision_status/fov_status` 均为 `pending`。这些 world 标注只用于
离线场景生成和核对，不应替代在线 YOLO + 深度测量的目标坐标。

以当前碰撞 STL 和零位 FK 检查，20 个名义停车位上的新果实没有与收纳零位机械臂
包围盒相交；计入 ±0.10 m 平移和 ±5° yaw 的停车误差后，最紧下界约 18.6 mm。
该余量较小，只覆盖已知底盘高度下的零位停车几何，不能推及伸臂、回撤或行驶转弯。

## 3. 文件与可复现生成

- 历史方案使用的默认世界和标注：`sim_ws/src/agri_greenhouse_worlds/worlds/tomato_field_default.sdf/.json`。
- 果实生成用停车几何快照：同目录 `tomato_field_harvest_stations.yaml`。
- 历史方案使用的默认任务：`ros2_ws/src/agri_task_manager/config/missions/tomato_field_default_single_corridor.yaml`。
- 历史场景版本：`rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_harvest_demo_v1_seed42`。
- 原默认场景备份：`artifacts/harvest_demo_profile_20261013/previous_default/`。

停车几何快照不含场景哈希，避免“生成元数据记录 mission 哈希，mission 又绑定元数据哈希”的
循环。修改停车点后必须同步快照，并重新生成场景、任务和验证结果。

从项目根目录生成独立候选，避免意外覆盖默认世界：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
FIELD_ID="rows6_n10_row3p0_plant0p7_h2p4_w1p0_c0p06_harvest_demo_v1_seed42"
FIELD_DIR="$PWD/artifacts/fields/$FIELD_ID"
mkdir -p "$FIELD_DIR"

python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --rows 6 --plants-per-row 10 \
  --row-spacing 3.0 --plant-spacing 0.7 \
  --origin-x -7.5 --origin-y -5.0 --ground-x 22 --ground-y 14 \
  --plant-height 2.4 --plant-width 1.0 --plant-collision-width 0.06 \
  --randomize-fruits --fruit-visual mesh \
  --fruit-count-min 2 --fruit-count-max 6 \
  --fruit-height-min 1.0 --fruit-height-max 1.7 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --ripe-ratio 0.7 --seed 42 \
  --harvest-mission sim_ws/src/agri_greenhouse_worlds/worlds/tomato_field_harvest_stations.yaml \
  --harvest-profile-ratio 0.85 --harvest-sector-half-angle 0.25 \
  --harvest-height-min 1.33 --harvest-height-max 1.42 \
  --harvest-radius-min 0.68 --harvest-radius-max 0.74 \
  --harvest-diameter-min 0.05 --harvest-diameter-max 0.065 \
  --harvest-flange-radius 0.86 --harvest-tcp-offset 0.16 \
  --harvest-pregrasp-distance 0.08 \
  --harvest-parking-xy-allowance 0.10 --harvest-parking-yaw-allowance 0.08726646 \
  --harvest-preserve-scan-max-z 1.22 \
  --output "$FIELD_DIR/tomato_field.sdf" --metadata "$FIELD_DIR/tomato_field.json"

gz sdf -k "$FIELD_DIR/tomato_field.sdf"
python3 scripts/check_harvest_fruit_profile.py \
  --field-metadata "$FIELD_DIR/tomato_field.json" \
  --field-sdf "$FIELD_DIR/tomato_field.sdf" \
  --mission ros2_ws/src/agri_task_manager/config/missions/tomato_field_default_single_corridor.yaml \
  --baseline-metadata artifacts/harvest_demo_profile_20261013/previous_default/tomato_field_default.json \
  --output "$FIELD_DIR/profile_check.json"
```

不传 `--harvest-mission` 时，旧生成器随机布局和兼容基线保持原行为。
原种子分布的高度/直径参数仍控制背景植株；`harvest-*` 参数只控制选定株的近侧果实。

## 4. 本机查看

默认资产已切换，构建后重新启动 Gazebo 才会生效。不要向运行中的旧世界直接覆盖 SDF。

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
colcon --log-base sim_ws/log build --symlink-install \
  --base-paths sim_ws/src --build-base sim_ws/build --install-base sim_ws/install \
  --packages-select agri_greenhouse_worlds agri_sim_bringup
colcon --log-base ros2_ws/log build --symlink-install \
  --base-paths ros2_ws/src --build-base ros2_ws/build --install-base ros2_ws/install \
  --packages-select agri_task_manager
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=98
export GZ_PARTITION="agri_harvest_demo_$(date +%s%N)"

ros2 launch agri_sim_bringup tomato_field.launch.py \
  gz_partition:="$GZ_PARTITION" \
  spawn_x:=0.0 spawn_y:=-6.0 spawn_yaw:=1.5708 \
  gui:=true paused:=false rviz:=false \
  use_control:=true use_kinematics:=true steering_effort_limit:=40.0 \
  use_lidar:=true use_scan:=true use_camera:=true
```

保存地图导航和 20 航点可视检查沿用 [阶段五流程](navigation_stage5.md#5-三终端启动)，
所有终端改用相同 domain。YOLO 启动方式见 [相机接入文档](yolo_tomato_integration.md)。
默认 D405 深度量程及其姿态仍需按相机文档设置；有候选果不等于相机当前就能测到它。
可靠视觉输出与模拟停车作业握手的独立测试流程见
[机械臂接入前置检查](harvest_preflight.md)。使用该流程的扩展仿真相机配置时，深度量程为
0.07–2.0 m；真实扫描姿态和实体量程仍须单独验证。

## 5. 地图兼容与交接

当前雷达 world 高度约 0.819 m，二维扫描高度带约 `[0.419, 1.219] m`。
原低果球几何保留，全场 73 个低球、其中作业株 28 个；新近果最低球面高于 1.22 m，
支梗也高于该阈值。叶片、茎杆、其他 40 株和停车几何保持原样，地图图像及 keepout
哈希未改变。3D 雷达采样与遮挡可能仍受高处模型影响，应结合实际 `/scan`、定位和
短距离导航结果判断地图是否继续适用；若修改扫描高度范围，重新做该检查。

2026-10-13 在独立 domain 199 中完成本轮检查：26 项相关测试、SDF 校验、20 站任务绑定、
`/scan`、里程计和 `map→odom` 均通过。左右首站、左侧短程前进和倒车返航共 6 个逻辑目标、
7 次 Nav2 请求均成功；最终 map 到站误差符合 0.10 m / 5°。但独立 Gazebo 真值检查发现，
回程时 map 位姿与实际位姿最大相差 **0.743 m**，实际目标位置误差最大 **0.669 m**。
因此本轮只通过地图坐标中的短程导航检查，**没有通过实际停车精度检查**，不能把上面的
0.10 m 距离筛选假设当作已验证能力，也不能据此认定实际到站后一定能夹取。
0.669 m 接近 0.7 m 株距，也可能影响实际作业植株身份。每株成熟候选的保证按名义停车
几何成立。原因尚未确定，当前检查不能将偏差归因于新增果实。
默认任务执行器也已加载新场景，20 航点、实时地图和 keepout 绑定检查通过；本次未发送
采摘命令。完整准备结果见 [validation_summary.json](../artifacts/harvest_demo_profile_20261013/validation_summary.json)，
导航和真值记录见 [navigation_smoke.json](../artifacts/harvest_demo_profile_20261013/navigation_smoke.json)。

后续应先核对实际到站位姿和重复植株环境下的定位偏差，再使用相机与深度实测重算果实
相对机械臂的坐标。若实际停车误差超出筛选余量，需调整停车或果实布局并重新筛选，
不能直接使用预设 world 果实坐标驱动机械臂。

协作者完成 IK/MoveIt 后，先标定 CR5 安装高度、抓取 TCP、夹爪开闭行程和 D405 外参，
再按每株至少一个成熟候选依次验证：当前到站位姿 → 预抓取 → 夹取 → 回撤 → 收纳。
每一步检查关节限位、自碰撞、车体/植株碰撞、相机可见性和定位误差。先通过一个左侧站和
一个右侧站，再扩大到 20 站。验证失败时调整候选果实参数或工具姿态，保留原失败结果；
当前 `0.16 m` TCP 代理和 `0.891556378 m` 肩高假设均可通过参数替换。
