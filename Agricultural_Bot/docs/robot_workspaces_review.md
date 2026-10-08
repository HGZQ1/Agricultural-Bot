# robot_workspaces 导航与采摘源码评估

审计日期：2026-10-08。仓库：[ysftzc/robot_workspaces](https://github.com/ysftzc/robot_workspaces)。
固定 `main` 提交：`bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f`。
本轮是公开源码审计，未下载运行权重、编译插件或启动对方仿真；下述功能属于对方代码，
不表示已经接入 Agricultural_Bot。

## 1. 项目定位和实际导航技术

作者的 [README](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/README.md)
记录 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic，机器人由 Husarion Panther 底盘与
Franka FR3 机械臂组成。环境与本项目相同，机器人本体和控制接口不同。

下面以 `sera_spawn_harvest_demo.launch.py → sera_mission.launch.py` 的温室主入口为准，
不能只看仓库根目录中保留的另一份 Nav2 参数。

| 层次 | 当前主入口使用的技术 | 对应代码 |
| --- | --- | --- |
| 建图 | SLAM Toolbox，二维 `/lidar/scan`，5 cm 栅格；独立建图入口 | [SLAM 配置](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/config/sera_slam_params.yaml)、[建图入口](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/launch/sera_mapping.launch.py) |
| 保存地图定位 | AMCL 与已保存的二维地图；启动发布预设 initialpose，导航入口 `slam=False` | [导航入口](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/launch/sera_nav2.launch.py) |
| 局部里程计 | robot_localization EKF，融合轮式里程计与 IMU；底盘控制器关闭自身 odom TF | [EKF 配置](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/config/ekf_sim_config.yaml)、[控制器](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/config/combined_controllers.yaml) |
| 全局规划 | Nav2 NavfnPlanner，`use_astar: false`，即 Dijkstra 模式 | [Nav2 参数](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/config/sera_nav2_params.yaml#L237) |
| 局部控制 | Regulated Pure Pursuit；主配置不是 DWB；速度链使用 TwistStamped | [控制器配置](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/config/sera_nav2_params.yaml#L105) |
| 通行约束 | 全局/局部代价地图、膨胀、植株行禁行遮罩；碰撞监控节点的区域/scan 输入关闭，不能计为额外保护 | [Nav2 参数](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/config/sera_nav2_params.yaml)、[禁行遮罩入口](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/launch/sera_nav2.launch.py) |
| 路线与停车作业 | YAML 预定义航点和观察姿态，任务节点逐点发送 NavigateToPose，再扫描或采摘 | [路线](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/config/sera_waypoints.yaml)、[任务节点](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/combined_robot/mission_manager.py) |

仓库没有 FAST-LIO 接入。主导航使用二维激光扫描；临时三维雷达在默认演示中关闭。
路线是已知温室布局中的巡查/采摘站点，不是未知场景的自主探索建图。
用户随后已取消 FAST-LIO。本项目采用 SLAM Toolbox＋AMCL＋Nav2 的同类路线，
需要完成 MID-360 二维扫描投影和四舵轮适配，见[导航开发流程](navigation_development_plan.md)。

## 2. 已有功能与采摘停车方式

源码串起了温室巡查、机械臂观察、YOLO 成熟度检测、RGB-D 三维记录、MoveIt 障碍物、
候选目标选择、接近/抓取/退离、成熟果与坏果分篮，以及失败目标跳过、超时处理。
YOLO 的[训练配置](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/yolo_models/tomato/args.yaml)
记录 YOLO11s；[类别](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/yolo_models/tomato/data.yaml)
是 `fully_ripened`、`green`、`rotten`。Git 树包含 `best.pt`，本轮没有推理验证，
不能据此保证它对本项目 D405 图像和番茄网格的识别效果。

停车位置主要来自固定航点，配合行/植株编号、左右作业侧和 FR3 观察姿态。
任务管理器到站后等待视觉记录更新，再在邻近候选中选择目标，不会自动根据 IK
生成新的底盘最佳停车位。可以借鉴“巡查建立果实记录，再按站点采摘”的阶段组织。

到站逻辑有位置/角度判断和视觉等待，但未实现轮式速度反馈持续静止判定，也没有
独立速度仲裁互锁。我们需要按自身停车方案补齐停稳、锁车、收臂后解锁的条件。
任务层默认到站复核为 0.30 m/0.35 rad，候选筛选主要基于类别、距离和高度范围，
这些条件不能替代 CR5 的 IK、碰撞和最终采摘停车精度验证。
本项目重新生成行数、行距、株距后，应重新生成或标注 map 中的站点与禁行区域；
对方世界/map 的固定偏置和旋转不能用于我们的番茄田。

固定站点采摘触发链存在，但通用 `_plan_harvest()` 分支仍有未实现代码；参考时应
沿默认演示实际调用链阅读，不能将所有状态名都解释为完成的功能。

## 3. 感知中值得借鉴与需要修正的部分

[tomato_depth_mapper.py](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/combined_robot/tomato_depth_mapper.py)
提供 bbox 内有效深度筛选和中位数、CameraInfo 反投影、TF 变换、邻近记录合并，
以及目标状态通知。先取框内较小区域，再使用中心/整框回退的思路可用于减少背景混入，
但深度中位数通常是表面点，不能直接当作精确果心或完整抓取姿态。

该节点使用最近缓存的 RGB、depth、检测和 CameraInfo，没有严格匹配四路时间；检测
JSON 的图像时间在解析时未用于同步，默认 TF 也取最新。腕部相机移动时需要改成
带时间戳的同步与 TF 查询，设置记录时效，并避免同一帧重复增加稳定观测次数。
传感器订阅默认 Reliable，迁入时需匹配本项目 Best Effort 数据 QoS。

本项目需改为 `/d405/color/image_raw`、`/d405/aligned_depth_to_color/image_raw`、
`/d405/color/camera_info`，使用 `camera_optical_frame` 和当前 0.07–2.0 m 深度范围。
感知输出应按我方接口定义目标 ID、成熟度、位置、半径、观测时间、不确定性与状态。
对方模型匹配还依赖 `tomato_*` 和 ripe/unripe/rotten 名称。本项目植株与果实命名不同，
不能直接沿用模型名筛选，以免把植株当作果实或用模型名代替成熟度识别。

## 4. 真值辅助与演示结果的范围

主入口的默认配置使用模型辅助：

- [sera_mission.launch.py](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/launch/sera_mission.launch.py#L933)
  默认优先模型果心，并读取 Gazebo 机器人位姿；mapper 默认启用模型匹配与果心吸附。
- [完整演示入口](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/launch/sera_spawn_harvest_demo.launch.py#L58)
  开启 YOLO、关闭独立 Gazebo 检测器，但仍启用 `snap_to_model_center`；关闭 mapper 的
  live pose 参数也没有关闭任务层的模型和机器人真值辅助。
- [mapper](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/combined_robot/tomato_depth_mapper.py#L458)
  可以用模型投影补缺失深度，匹配后将输出位置替换为已知模型中心。
- [任务节点](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/combined_robot/mission_manager.py#L1582)
  在没有合适视觉候选时，还存在按当前植株模型构造候选的回退路径。

因此，它是带模型辅助的仿真闭环。作者 README 报告 24 次尝试、19 次成功，79.2%；
本轮没有复现这些数据，不能将其解释为纯视觉定位或真实番茄采摘成功率。
我方应将模型真值与果实 JSON 用于离线误差评测，在线目标使用传感器估计。

## 5. 夹取流程与物理简化

[greenhouse_nearest_pick_place.py](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/combined_robot/greenhouse_nearest_pick_place.py#L1290)
确实实现 OMPL 接近规划、Pilz LIN 直线进给、闭合夹爪、附着、植株释放、直线退离和
投篮。候选接近姿态、目标通知、尝试次数及超时管理值得参考。

实际附着是夹爪轨迹完成后发送服务请求。
[contact_attach_system.cpp](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot_gz_plugins/src/contact_attach_system.cpp#L338)
按指定父子 link 创建 `DetachableJoint(fixed)`，没有接触传感器、双指接触或力阈值
判定，也不模拟果梗断裂。其 gz-sim8 等依赖与 Harmonic 匹配，可评估移植为仿真果实
附着/释放组件，但抓取成功条件要另外实现。

对方果实是带质量惯量的独立动态模型，初始通过可分离关节固定到植株。
完整任务还启用持果位姿稳定化：附着失败时存在静态代理及 set_pose 跟随 TCP 的回退。
投篮包含固定 FR3 关节轨迹和果实位置设置/重建操作，属于演示稳定化。

碰撞场景会[删除被选中的目标球](https://github.com/ysftzc/robot_workspaces/blob/bf3e2c15817a0b17d6b6990ae57e185e7cfcc98f/combined_ws/src/combined_robot/combined_robot/tomato_collision_scene_manager.py#L185)，
持果后没有同步添加 MoveIt AttachedCollisionObject。移植应限定允许接触的夹爪 link，
持果后更新附着碰撞体，并对失败、超时、持果中断建立一致的恢复状态。

## 6. 对 Agricultural_Bot 的建议

| 参考内容 | 我方承接位置 | 必要适配 |
| --- | --- | --- |
| Nav2 规划、代价地图、禁行遮罩 | `agri_navigation` | 四舵轮 footprint、速度/加速度、MID-360 扫描输入与 SLAM Toolbox/AMCL 接口 |
| 航点、观察姿态与巡查/采摘任务状态 | `agri_task_manager` | 参数化田布局、map 对齐、实际停稳互锁、失败恢复；拆分导航与采摘动作接口 |
| YOLO、RGB-D 记录与目标去重 | `agri_perception` | D405 话题/QoS/时间同步、果心与抓取方向估计、取消模型真值吸附 |
| OMPL 接近＋Pilz LIN 进给/退离 | `agri_robot_moveit_config`、`agri_manipulation` | CR5 六关节、TCP、IK、关节限位、真实场景和持果碰撞、篮位规划 |
| 果实附着与释放 | `agri_greenhouse_worlds`、仿真插件层 | 当前 static `fruit_link` 改成动态果实＋初始植株约束；质量惯量、触发条件、释放与重置 |
| 夹爪动作与反馈 | `agri_gripper_adapter` | 对方两指 FollowJointTrajectory；我方当前 ForwardCommandController，需新增动作适配和成功反馈 |

本项目当前 `arm_controller` 已有 JointTrajectoryController，但 CR5 的关节数、名字、
安装与 TCP 都不同于 FR3。观察姿态、候选姿态和投篮关节值不能直接复制。
当前随机果实是 `<static>true</static>` 的独立模型，尚无质量惯量和可释放植株约束；
先完善果实生命周期，才能验证“夹住、离株、搬运、松开入篮”。

建议按“导航到站并停稳 → D405/YOLO 定位与 IK 可达检查 → 单果采摘物理 → 入篮 →
多站点任务”逐步接入，分别记录定位误差、计划成功率、实际抓持和入篮结果。
仅借鉴各模块流程，保留我方机器人本体与运行场景；当前导航路线为 SLAM Toolbox、
AMCL 和 Nav2，FAST-LIO 不再接入。
