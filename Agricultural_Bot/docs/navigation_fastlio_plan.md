# FAST-LIO 建图、Nav2 导航与采摘停车方案

**状态更新（2026-10-08）：用户已取消 FAST-LIO，本文件保留为历史选型记录，以下内容不再作为实施路线。当前方案见 [导航开发流程](navigation_development_plan.md)，采用 SLAM Toolbox、AMCL 和 Nav2。**

记录日期：2026-10-08。**本文是源码选型和接口实施计划，IMU、FAST-LIO、Nav2
尚未在本项目接入或验收。** 文中节点、话题和配置是拟定接口，不是现有启动入口。
本轮选择 FAST-LIO 作为三维建图前端；需求文档中原有 SLAM Toolbox 建图路线仍待
随后统一调整，不能将旧目标入口视为已实现，也不同时运行两套建图定位 TF 发布者。

## 1. 源码选型

主候选是 [JossueE/autonomous_robot_simulation 的 fast_lio_ros2](https://github.com/JossueE/autonomous_robot_simulation/tree/a6a24c2d3894b52b228326e08bdeae20e34216c2/fast_lio_ros2)，
本轮审计固定提交 `a6a24c2d3894b52b228326e08bdeae20e34216c2`。
其[根 README](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/README.md)
明确记录 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 环境验证；这是上游验证声明，
并不代表我方 MID-360、四舵轮和番茄田已经通过验证。仅选择 LIO 相关目录及必要依赖，
不直接迁入上游整车、导航参数和仿真场景。

该候选的[包清单](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/fast_lio_ros2/package.xml)
及[构建配置](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/fast_lio_ros2/CMakeLists.txt)
使用仿真 `PointCloud2` 和 IMU 输入，不强制依赖 `livox_ros_driver2`。
其[simulated.yaml](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/fast_lio_ros2/config/simulated.yaml)
设置 `lidar_type: 5`；[preprocess.cpp](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/fast_lio_ros2/src/preprocess.cpp)
的 `default_handler` 接收 XYZI，并将所有点的相对时间置零。它适合先接当前瞬时云，
不能据此宣称已经验证真实扫描时序及运动去畸变。

依赖导入必须保留根 [.gitmodules](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/.gitmodules)
中 `fast_lio_ros2/include/ikd-Tree` 的子模块信息，固定子模块提交
`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`，并保留许可证和来源记录。
不能仅复制 `fast_lio_ros2` 目录而遗漏子模块源码。

备用为 [Ericsii/FAST_LIO_ROS2 的 ros2 分支](https://github.com/Ericsii/FAST_LIO_ROS2/tree/ros2)。
其 README 推荐 Humble，未声明 Jazzy CI；外部 Jazzy 使用报告只能辅助评估。
备用涉及 Livox 驱动接口，不能直接替代当前仿真输入配置。后续若切换源码，先重新
冻结提交、依赖和接口，再在独立环境编译与回放验收。

## 2. 当前条件与接入接口

| 项目 | 当前状态／下一步 |
| --- | --- |
| MID-360 点云 | 已有 `/mid360/points`，`mid360_sensor_frame`，10 Hz；GPU 为 XYZ、intensity、ring，RGL 为 XYZI，均无逐点时间 |
| IMU | 当前没有 IMU 仿真和 ROS 输出；新增固定安装 frame、Gazebo IMU、桥接和外参，拟用 `/mid360/imu` |
| 底盘 | 已接 `/cmd_vel` 的 `TwistStamped`；输出 `/wheel/odom`，默认发布 `odom → base_footprint` |
| 已安装导航依赖 | `/opt/ros/jazzy` 中已有 `nav2_bringup`、`slam_toolbox`、`robot_localization`；安装不等于配置或验收完成 |
| 待选／待安装依赖 | 当前没有 `pointcloud_to_laserscan`、`octomap_server`、Livox 驱动；按实际点云投影与定位方案选择，不必全部安装 |
| 业务包 | `agri_navigation`、感知、MoveIt 配置、操作规划、任务管理仍是目录骨架 |

先确认点云单位、有效点、frame 和时间戳，再将候选话题配置改成我方接口。
点云与 IMU 必须使用同一 `/clock` 和有效采样时间，IMU 验证包括静止重力、角速度、
频率、时间连续性及雷达外参，不能仅检查话题存在。
上游 [mapping.launch.py](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/fast_lio_ros2/launch/mapping.launch.py)
的 `use_sim_time` 默认关闭，集成时必须显式设为 `true`。
其 [laserMapping.cpp](https://github.com/JossueE/autonomous_robot_simulation/blob/a6a24c2d3894b52b228326e08bdeae20e34216c2/fast_lio_ros2/src/laserMapping.cpp)
使用 Reliable IMU 订阅，需匹配桥接发布端 QoS 或调整订阅；Reliable 订阅不能接收
仅提供 Best Effort 的发布端数据。规则见 [ROS 2 QoS 文档](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)。

Nav2 Jazzy 默认速度消息仍为 `Twist`。本项目拟保持 `TwistStamped`，为所有涉及速度
发布／订阅的 Nav2 节点设置 `enable_stamped_cmd_vel: true`，并核对仲裁、限速、遥控
和底盘出口类型；也可在唯一出口转换，不能同名混用两种消息。
依据：[Nav2 Jazzy 速度平滑器配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/)。

## 3. 出生点、地图对齐与 TF 所有权

当前 [田间入口](../sim_ws/src/agri_sim_bringup/launch/tomato_field.launch.py)
出生点为世界 `(0,-6,0.40)`、yaw `1.5708`，允许作为可复现实验的固定初始配置。
但[底盘节点](../ros2_ws/src/agri_base_kinematics/agri_base_kinematics/four_wheel_steering_node.py)
启动时 odom 的 x/y/yaw 都为零；FAST-LIO 还有自身初始化坐标和 IMU/body 外参。
不能将出生坐标、轮式里程计或 LIO 输出坐标直接当成同一个 `map`。

建议把 TF 职责固定为：

| 变换 | 拟定唯一发布者 |
| --- | --- |
| `map → odom` | 当前模式的地图定位／对齐后端；建图与保存地图定位模式互斥 |
| `odom → base_footprint` | LIO 位姿适配器或融合节点，二选一；接入时关闭底盘 `publish_tf` |
| `base_footprint → base_link → sensors/arm` | `robot_state_publisher`，使用 URDF 和关节反馈 |

轮式节点继续输出 `/wheel/odom`，最终连续底盘位姿和速度拟统一到 `/odom`。
适配器必须把 LIO 的 IMU/body 位姿通过外参转换为底盘位姿，补充合理协方差，
再提供平面导航接口；不能仅修改 `frame_id`。上游自身 TF 也须纳入同一所有权检查。
TF 和融合配置见 [Nav2 Jazzy robot_localization 指南](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/)。

固定初始位置可以初始化地图对齐，但不替代持续定位。保存地图后需单独选定点云地图
定位／重定位后端；回环修正也单独设计，不能把保存 PCD 当成这三项已经具备。
若需使用世界中的行布局坐标，先保存明确的 world/map 对齐；首版更适合在已保存地图
上直接标注行和站点。持续运行不得用 Gazebo 真值修正机器人定位。

## 4. 从点云地图到 Nav2

FAST-LIO 的三维地图或 PCD 需要转换为 Nav2 使用的二维 `OccupancyGrid`，同时保存
分辨率、原点和地图版本。仅把点落入栅格可以标出障碍，不能说明其他格子已经观测为空闲。
应利用传感器位姿和射线更新保留 occupied/free/unknown 的区别，过滤地面、车体自身和
不应阻断底盘的高度层，再核对行间宽度。保存后的 PCD 不包含完整的自由空间观测历史，
投影时应结合建图过程中的射线／占据记录。

Nav2 的标准 AMCL 消费二维占据地图和激光扫描，不能直接读取 PCD；如选择 AMCL 作为
保存地图后的定位后端，还需二维地图和正确的 LaserScan 投影及相应验证。
另一条路线是点云地图定位后端提供 `map → odom`，二维地图仅供 Nav2 规划。
这两条后端路线应单独选型，不把旧 SLAM Toolbox 配置直接套到 FAST-LIO。
接口依据：[Nav2 建图和定位指南](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)；
Jazzy 仿真接入参见 [Gazebo 指南](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/)。

Nav2 最小运行条件：统一仿真时间、完整 TF、连续里程计、全局地图、实时点云或扫描
障碍输入、真实底盘 footprint、膨胀和高度过滤、规划／控制／目标判定配置、lifecycle
管理及唯一速度出口。扫描站点由 `NavigateToPose` 调度；地图只描述通行空间，
不会自动说明哪株有成熟番茄或哪里适合机械臂采摘。

## 5. 机器人如何决定在哪里停车

首版在 `map` 中保存行入口、出口和固定扫描工作位。站点配置至少包含地图版本、
`station_id`、`row_id`、底盘 `[x,y,yaw]`、作业侧、扫描姿态和允许微调区域。
它是预先定义的观察站位，不是果实的精确位置。

1. 机械臂收纳，任务管理器向 Nav2 发送下一站点，采用低速行间通行。
2. 导航结束后锁住导航速度通道，保持零速；确认反馈线速 `<0.01 m/s`、角速
   `<0.02 rad/s` 持续至少 `0.5 s`，且定位、TF 和控制器正常后允许展开机械臂。
3. CR5 执行侧向和不同高度的观察姿态。YOLO、同步 RGB-D、CameraInfo 及图像时刻的
   TF 生成成熟果三维候选，完成稳定跟踪和去重，再做 IK 与碰撞检查。
4. 优先采当前站位可达果实。视野差或不可达时，先收臂，再在当前行走廊内规划有限次
   底盘微移；重新停稳和扫描。当前站点无可采目标时进入下一站点。
5. 抓取、入篮后更新目标状态和数量；机械臂收纳后才能解锁底盘。

`NavigateToPose` 成功只表示满足导航目标判据，不等于物理停稳。底盘互锁使用实际
反馈和速度仲裁，不通过把车固定到世界掩盖漂移。历史底盘测评显示直行、显式零速
通过，但换舵、小弧和累计里程计航向仍需改进，见[底盘运动评测](chassis_field_motion.md)。
站距、离株偏置、可采高度必须通过 CR5 的 IK、TCP、碰撞和 D405 FOV 验证。
株高改成 2.6 m 不会扩大机械臂工作空间；900 mm 名义工作半径也不保证所有果实可达。
果实目标必须来自视觉定位，随机化 JSON、模型 ID 或 Gazebo 果心真值仅用于离线
标注与评测，不得直接作为导航微移目标或抓取输入。

## 6. 分阶段实施和验收

| 阶段 | 交付与出口条件 |
| --- | --- |
| 输入与源码验证 | 固定依赖并编译；新增 IMU、验证字段/时间/QoS/外参；低速采包回放确认 LIO 初始化、输出与地图可保存 |
| `mapping` 自动建图 | FAST-LIO＋在线占据地图＋行间蛇形覆盖调度；自主走入口、行间和行末，避障并保存 PCD、二维地图及对齐信息 |
| `navigation` 保存地图导航 | 启动选定定位后端和 Nav2，不启动在线建图 TF 发布者；完成保存地图中的 10 个目标及返航 |
| 停车与扫描 | 固定站点导航、反馈停稳、速度互锁、扫描与有限微移；验证机械臂展开期间底盘持续静止 |
| 视觉与采摘 | 补齐 YOLO 三维定位、MoveIt/IK、夹爪接口和果实附着/释放，最终完成多站点闭环 |

手动采包仅是前置接口验证，不算自主建图完成。蛇形覆盖需确认行末掉头空间、地图
未知区域策略及中断恢复；第一次从单行低速闭环开始，再扩大到完整番茄田。
保存地图导航采用需求 TC-NAV 的首版标准：每目标超时 120 s，位置误差 ≤0.10 m、
航向误差 ≤5°，10 点至少成功 9 次且禁止碰撞为零；同时记录定位漂移、TF/QoS
异常、路线覆盖、停稳时间、返回误差和恢复次数。Gazebo 真值仅用于独立评测。
