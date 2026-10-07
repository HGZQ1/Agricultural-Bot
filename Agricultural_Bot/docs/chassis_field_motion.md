# 番茄田底盘运动测试

本测试使用 `tomato_field.launch.py` 的中央通道出生位置，验证底盘控制、
Gazebo 实际运动、轮式里程计及停车。先关闭传感器完成底盘基线，后续再单独验证
MID-360、D405 与运动同时运行的效果。

## 1. 启动可运动的番茄田

在原仿真所属终端按 Ctrl+C 结束旧启动；不同世界也不要共享同一组 ROS 控制话题。
以下采用独立 ROS 域，两个终端均需设置相同的 `ROS_DOMAIN_ID`。

终端 A：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=92
export GZ_PARTITION=agricultural_bot_tomato_field

ros2 launch agri_sim_bringup tomato_field.launch.py \
  gui:=true paused:=false use_control:=true use_kinematics:=true \
  use_lidar:=false use_camera:=false \
  gz_partition:="$GZ_PARTITION"
```

默认入口为 `paused:=true use_control:=false`，仅打开模型不能验证运动。
本入口的世界名为 `field`；中央通道出生 `(0,-6,0.40)`、yaw `1.5708`，
车头朝世界 +Y。机器人落地后模型原点高度约 0.36 m，与 CAD 根坐标有关。

终端 B：

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
export ROS_DOMAIN_ID=92
export GZ_PARTITION=agricultural_bot_tomato_field

ros2 control list_controllers
gz model -m agri_robot -p
```

确认 `joint_state_broadcaster`、`steering_controller`、`wheel_controller` 为 `active`；
`arm_controller`、`gripper_controller` 也应为 `active`，以保持机械臂初始姿态。
Gazebo 必须正在播放。`GZ_PARTITION` 只隔离 Gazebo，不能隔离 `/cmd_vel` 等 ROS 话题。

## 2. 手动观察

在终端 B 执行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_footprint \
  -p use_sim_time:=true \
  -p speed:=0.1 \
  -p turn:=0.2
```

| 按键 | 动作 |
| --- | --- |
| `i` | 前进 |
| `,` | 后退 |
| `u` / `o` | 前进同时左转 / 右转 |
| `j` / `l` | 原地左转 / 右转，属于当前仿真的条件能力 |
| `k` / 空格 | 停车 |

保持英文输入法、焦点在键盘控制终端。当前安装的 Jazzy 键盘节点按按键事件发布；
持续运动需长按或连续按键，控制器在 0.5 仿真秒未收到新指令后停车。
先在中央行间前进约 1 m，停车，再倒车回入口；不要从初始位置先倒车 1 m，
南侧地面边界为 Y=-7。小角度转弯在入口空地进行，避免把车头转向植株。

当前轮半径 0.127 m、轮速上限 2 rad/s，直行指令上限对应约 0.254 m/s。
`speed:=1.0` 会触发轮速限幅，不能用于证明底盘实际达到 1 m/s。

另一个同域终端可观察反馈：

```bash
ros2 topic echo /wheel/odom
ros2 topic echo /joint_states
gz model -m agri_robot -p
```

轮式里程计初始原点为 `(0,0,0)`，与 Gazebo 出生 `(0,-6,π/2)` 不同，
不能直接比较两路绝对 X/Y 数值。

## 3. 自动量化测试

先在键盘终端 Ctrl+C 退出遥控，再从默认出生位置重新启动世界。
自动测试会发布速度，不能同时运行键盘、导航或其他 `/cmd_vel` 发布者。
在终端 B 执行：

```bash
python3 scripts/check_chassis_motion.py --execute --repeats 5 \
  --output artifacts/chassis_field_manual/report.json
```

测试按 **仿真时间** 控制每次前进/后退 1 m；五轮分别记录命令结束与停稳后
的目标终点误差、航向漂移、整轮回起点误差、里程计相对 Gazebo 真值的误差。
启用 `--full-turns` 时，在五轮往返后单独测试五次整圈；随后执行左右小角度转弯，
以及停发指令后的超时停车检查。
Gazebo 真值仅用于此仿真评测，不供机器人算法使用。
JSON 报告旁生成同名 `.truth.csv`，记录带仿真时间的世界位姿与连续展开 yaw。
退出码 0 为通过、1 为指标失败、2 为前提不满足/超时/中断；没有 `--execute` 时只检查就绪。
默认总墙钟上限为 600 s，较低实时因子可用 `--timeout 1200` 延长采集。

可选的整圈转向测试在独立的新场景中启用：

```bash
python3 scripts/check_chassis_motion.py --execute --repeats 5 --full-turns \
  --output artifacts/chassis_field_manual/full_turns.json
```

整圈检查使用连续展开的实际 yaw，避免“车没转、最终朝向相同”被误判通过。
实机转向限位尚未确认，此项验证当前仿真配置，不代表硬件已支持原地旋转。

## 4. 判据与范围

底盘目标采用需求文档 TC-MOTION 的首版阈值：每段 1 m 直行终点误差 ≤0.05 m；
若启用整圈，实际累计转角与 360° 的误差 ≤3°，平移偏离 ≤0.05 m。
停车采用真值差分检查速度，要求线速度 <0.01 m/s、角速度 <0.02 rad/s
持续至少 0.5 仿真秒。断流停车要求停发前实际速度 >0.05 m/s，在末次指令后的
0.6 s 内开始持续静止；其中 0.5 s 为控制器看门狗，0.1 s 为动力响应和采样容差。

补充回归阈值为：小弧线位置误差 ≤0.08 m，动作航向误差 ≤3°，里程计相对
真值位置/航向误差 ≤0.15 m/5°；它们不替代轮径或滑移标定。
全程真值采样间隙 ≤0.1 s、模型高度相对初始变化 ≤0.03 m、倾斜 ≤10°，
六个机械臂关节相对初始保持位置的变化 ≤0.02 rad。检查机器人异常时提前停车。

植株碰撞目前是 `0.1×0.1×1 m` 简化盒，叶片和单个番茄只有视觉网格。
当前世界没有 Contact 数据输出，因此本轮无法判定需求中的全部禁止接触；
机械臂十条安全轨迹也未包含在此底盘测试内。该测试不能代替完整 TC-MOTION、
导航避障、叶片碰撞或采摘验收。

## 5. 2026-10-07 实测记录

独立 ROS 域 92、Gazebo 分区 `agri_chassis_field_20261007`，150 株基线世界，
开启物理与全部五个控制器；关闭 GUI、LiDAR、相机。前进/倒车各五次，随后整圈
转向五次、左右小弧、显式零速及断流停车，共运行 305.398 仿真秒。
本轮量化测试已完成，整体结果为 **未通过**。

| 检查 | 实测 | 结果 |
| --- | --- | --- |
| 五轮 1 m 前进/倒车 | 10 段全部通过；停稳后最大位置误差 3.585 mm，航向误差 0.293° | 通过 |
| 五轮回起点 | 最大偏差 3.094 mm | 通过 |
| 显式零速停车 | 持续静止检查通过，最大制动位移 4.502 mm | 通过 |
| 整圈转向 | 停稳实际转角 350.814°、356.379°、357.675°、356.937°、357.163° | 未全部达到 360°±3° |
| 左/右小弧 | 停稳航向误差 4.449° / 3.262° | 超过 3° |
| 轮式里程计 | 全程最大位置/航向偏差 0.12082 m / 11.180° | 航向超过补充 5° 回归阈值 |
| 断流停车 | 停发前实际速度 0.10001 m/s；末次指令后 0.641 s 开始持续静止 | 超过本测试自定 0.6 s 截止时间 |
| 接地、倾斜、臂保持 | 高度变化最大 0.166 mm，roll/pitch 最大 0.0365°/0.0252°，臂关节最大变化 0.000259 rad | 通过 |

第三、第五次整圈的停稳航向误差在 3° 内，完整动作仍受累计里程计误差等判据影响。
报告同时保留命令结束帧与停稳帧；测试整体未通过，未修改运动学参数或放宽阈值。

断流段约在末次指令后 0.536 s 开始减速，0.573 s 首次低于静止速度阈值，
随后 0.222 mm 的小幅回动使速度短暂达到 0.01308 m/s，持续静止从 0.641 s 重新计时。
这项失败反映停车动态与本测试的停稳截止要求；不能据此说看门狗到 0.641 s 才触发。
本轮未记录轮速控制指令，无法精确确定零轮速命令的发布/接收时刻。

预检 rosbag 诊断发现换舵达稳约需 0.5–0.8 s，当前同时下发舵角和轮速，
换舵期间实际轨迹偏离目标。预检左弧前 1 s 的指令与轮式里程计转角差约 3.91°，
后 2 s 仅约 0.10°；稳态轮式里程计角速度接近指令，轮速未饱和。
配置轮心与 CAD 轮心一致，该证据不支持直接改轮径补偿。
下一步优先处理换舵与轮速协调、基于正常里程计的到角控制，以及断流停车响应；
Gazebo 真值继续只作为评测依据。

本地证据位于 `artifacts/chassis_field_20261007/`：`report.json`、`summary.json`、
`report.truth.csv`（18,299 条位姿）、`motion_tests.xml`、`unit_tests.xml`、
`formal_rosbag/`、`formal_launch.log`、`test.log`、`environment.json`、
`turn_diagnosis.json`、`watchdog_diagnosis.json` 与 `motion_evidence.png`。
`initial_report.json` 和初轮 rosbag
保留预检结果；正式报告使用固定 1 m / 360° 目标和停稳终点。
场景 SHA、源文件 SHA、软件版本、随机种子及启动命令记录在环境快照中。
新增数学边界测试 7 项通过，Python 语法和 `git diff --check` 通过。
仅关闭本次独立实例；用户域 0 的 ROS 会话保留。GUI 和传感器共存尚未在本轮验收。
