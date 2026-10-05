# 现有资产审计

> 审计日期：2026-10-03  
> 本文件只记录输入资产状态，不表示当前导出模型已经可用于 ROS 2 或 Gazebo。

## 1. 已阅读资料

| 来源 | 内容 | 当前用途 |
| --- | --- | --- |
| `../../农机/urdf/番茄自主采摘机器人.urdf` | SolidWorks URDF Exporter 生成的整机 URDF | 只作几何、质量属性和安装关系初稿 |
| `../../农机/urdf/番茄自主采摘机器人.csv` | link、joint、惯量和 SolidWorks 组件映射 | 辅助追溯 CAD 组件 |
| `../../农机/meshes/` | 20 个 STL 网格 | 视觉模型候选；碰撞模型必须另行简化 |
| `../../农机/launch/` | ROS 1 RViz/Gazebo launch | 不迁移，只参考旧包入口 |
| `../../番茄自主采摘机器人/` | 约 237 MB SolidWorks/STEP 资产 | 原始 CAD 来源，暂不复制 |
| `../../共享资源/机器人及场地尺寸要求.md` | 四舵轮参数、运动学与 TF 说明 | 底盘运动学基线 |
| `../../共享资源/Weixin Image_20260930171843_30_43.png` | 整机工程图 | 外形、安装关系和关键尺寸校核 |
| `../../共享资源/Dobot CR系列使用手册V1.8_20231226_cn.pdf` | 2023-12-26 版 CR 系列手册 | CR5 尺寸、坐标系、法兰、负载和安全校核 |

## 2. 整机与底盘已确认参数

机器人图纸和尺寸文档确认底盘为四舵轮，而不是普通差速四轮。由于当前 SolidWorks/URDF 已更新，本项目本阶段以 `robot_pick_robot11` 当前 URDF 的尺寸、原点和安装位姿为准；下表的旧图纸数值仅保留作历史追溯，不能覆盖当前模型。

| 参数 | 旧版图纸数值（仅历史参考） |
| --- | --- |
| 轴距 | 449.99 mm / 0.44999 m |
| 轮距 | 364.05 mm / 0.36405 m |
| 轮胎直径 | 254 mm / 0.254 m |
| 轮胎半径 | 127 mm / 0.127 m |
| 轮胎宽度 | 100 mm / 0.100 m |
| 轮心坐标 | FL/FR/RL/RR = (±0.224995, ±0.182025) m |
| 图纸整车长度 | 约 759.92 mm |
| 图纸整车宽度 | 约 662.05 mm |
| CR5 展开总高 | 约 1903.96 mm |
| MID-360 支架倾角 | 图纸标注约 15°；方向仍需按 TF 轴校核 |

图纸还显示 CR5、MID-360、D405、非标夹爪和番茄存放篮已经进入总装。

### 底盘控制结论

- 必须建立四舵轮运动学节点，输入底盘 `(vx, vy, wz)`，输出四个转向角和
  四个驱动轮角速度。
- 当前机械转向限位尚未确认。在限位确认前，横移和原地旋转不能作为验收能力。
- `ros2_ws/src/agri_base_kinematics` 负责 Python 逆运动学、转向角优化和限位。
- `sim_ws/src/agri_sim_control` 负责仿真关节控制器参数，不实现任务算法。

## 3. CR5 手册校核结果

本地手册为 Dobot CR 系列使用手册 V1.8（2023-12-26）。

| 项目 | CR5 数据/要求 |
| --- | --- |
| 机械臂重量 | 25 kg |
| 最大负载 | 5 kg |
| 工作半径 | 900 mm |
| 重复定位精度 | ±0.02 mm |
| 竖直零位总高 | 1096 mm |
| 底座安装孔 | 4 x φ9 均布，PCD φ132 |
| 底座定位/安装尺寸 | 60 mm 基准尺寸；图纸另标 φ8 定位孔 |
| 末端法兰 | GB/T 14468.1-50-4-M6 |
| 法兰螺孔 | 4-M6，PCD φ50 ±0.1 |
| 法兰定位尺寸 | φ31.5 H7、φ63 h7、φ6 H7 |

手册还明确：

- 六个机械臂关节均为旋转关节。
- 机械臂竖直且零点贴纸对齐时，各关节角度为 0°。
- 默认用户坐标系以机械臂底座中心为基准。
- 默认工具坐标系以末端法兰中心为基准。
- 末端工具、D405、夹爪、支架和番茄的总质量与重心必须满足 CR5 负载曲线。

最终 CR5 URDF、SRDF、零位、关节轴、限位和 TCP 必须以手册与官方资料复核，不能只依赖
当前 SolidWorks 导出值。

## 4. SolidWorks 资产概况

- 共识别约 235 个 SolidWorks/STEP 相关文件，其中约 40 个是 `~$` 开头的
  SolidWorks 锁/临时文件，迁移时必须排除。
- 主装配体包括：
  - `番茄自主采摘机器人.SLDASM`，约 12.8 MB。
  - `农业采摘机器人场景校核.SLDASM`，约 20.0 MB。
  - `篮子 (1)/装配体51.SLDASM`，约 27.2 MB。
- 可交换几何包括：
  - MID-360：`mid-360-asm.stp`，约 8.0 MB。
  - 篮筐：`Canastos.STEP`，约 8.4 MB。
- CR5 子装配和零件位于舵轮资源的深层目录中，迁移前需从总装中重新整理来源关系。
- D405 和非标夹爪已包含在总装/导出 CSV 的组件树中，但没有发现独立、命名清晰的
  D405 或夹爪 STEP 包，后续应从总装导出独立基准模型。

## 5. 当前 URDF 检查结果

`xmllint` 能解析 XML，但 `check_urdf` 失败：

```text
joint ' arm_base_link' is not unique
```

已确认的阻塞项：

1. `arm_base_link` 固定关节被重复定义。
2. ` arm_base_link`、` f2_wheel_link`、
   ` r2_wheel_link` 及对应 STL 文件名含前导空格。
3. 机器人和 ROS 包使用中文名称，不适合作为 ROS 2 package/resource 标识。
4. 现有 `package.xml`、CMake 和 launch 属于 ROS 1 catkin/Gazebo Classic。
5. 没有 `ros2_control`、transmission、Gazebo Harmonic 传感器或 bridge 配置。
6. `gripper_link` 被定义为 prismatic，但轴为 `0 0 0`，上下限、速度和
   effort 全为 0，当前不可运动。
7. `camera_optical_link` 与 `camera_link` 使用零旋转，尚未按 ROS optical
   frame 规范校核。
8. visual 与 collision 共用完整 STL；MID-360 网格约 4.15 MB，部分碰撞网格过重。
9. 三个 steer STL 只有 80 bytes，另一个 steer STL 约 3.39 MB，导出结果明显不对称。
10. 底盘 CAD 原点没有位于几何中心：当前轮关节坐标与已确认轴距/轮距不一致。
11. 当前 CR5 link 质量求和明显高于手册给出的 25 kg，需要重新核验组件归属和惯量。
12. link 与 joint 普遍同名，虽不必然非法，但应迁移为明确的 `*_link` /
    `*_joint` 命名。

因此当前 URDF 不复制到 canonical 描述包。迁移时从 CAD、尺寸图和官方 CR5 数据重新建立
Xacro，并将现有网格作为对照输入。

## 6. 资产进入工作空间的规则

| 原始资源 | 目标位置 | 进入条件 |
| --- | --- | --- |
| 清理后的整机 visual mesh | `ros2_ws/.../agri_robot_description/meshes/visual` | 单位、原点、法向和许可通过 |
| 简化后的 collision mesh | `ros2_ws/.../agri_robot_description/meshes/collision` | 凸包/基础几何和性能测试通过 |
| canonical Xacro | `ros2_ws/.../agri_robot_description/urdf` | `check_urdf` 和 TF 测试通过 |
| Gazebo 专用 Xacro/SDF | `sim_ws/.../agri_sim_description` | 不改变 canonical link/joint 名 |
| 温室和番茄资产 | `sim_ws/.../agri_greenhouse_worlds` | 尺寸、碰撞、许可和性能通过 |
| 原始 CAD/图纸副本 | `models_source` | 确需版本化时再复制；当前保留原位 |

## 7. 尚待确认

- 四个舵轮的实际机械转向限位、零位、驱动/转向减速比和最大速度。
- 图纸外廓尺寸与 SolidWorks 配置版本是否完全一致。
- 整车总质量、底盘质心和机械臂最大伸展时的抗倾覆裕量。
- CR5 的准确代际和控制器协议版本。
- 非标夹爪的两个手指结构、行程、最大开口、质量、TCP 和驱动方式。
- D405 相对 `gripper_tcp` 的实际安装外参和视线遮挡。
- MID-360 的 15° 倾角方向、传感器原点和遮挡范围。
- 篮筐内部有效判定体积和 `basket_drop_pose`。

## 8. robot_pick_robot11 最新 URDF 审计

输入文件：`../../robot_pick_robot11/urdf/robot_pick_robot11.urdf`。

### 8.1 已通过的检查

- XML 语法和 `check_urdf` 解析通过。
- 机器人树为 21 个 link、20 个 joint，根 link 为 `base_link`。
- mesh URI 在 `robot_pick_robot11/meshes` 中有对应文件。
- 相比旧导出物，中文包名和多数前导空格已清理，四个转向关节与四个驱动关节已拆出。

### 8.2 P0 阻塞问题

1. `gz sdf -p` 转换失败（退出码 255）。Gazebo 报告 `f2_steer_joint` 和
   `gripper_right_finger_Link` link/joint 同名冲突、frame graph cycle 和零轴错误。
2. `f2_steer_joint` 同时作为 link 和 joint 名（第 170、210 行）；右指 link/joint 也同名
   （第 1175、1215 行）。必须分别改为 `fr_steer_link`/`fr_steer_joint` 和
   `right_finger_link`/`right_finger_joint` 一类的唯一命名。
3. 右指 prismatic joint 的 axis 为 `0 0 0`（第 1224-1225 行），Gazebo 无法生成关节。
4. 包仍是 ROS 1：`package.xml` 使用 catkin（第 12-17 行），CMake 和 launch 使用 ROS 1
   `$(find)`/`gazebo_ros`；它不能被 Jazzy 的 `ros2 pkg` 直接发现。

### 8.3 P1 物理和运动学问题

1. URDF 总质量约 520.728 kg；四个转向 link 各为 104.717 kg（第 53、175、297、419 行），
   与实际舵轮不符。其质心在对应 STL 局部包围盒外，说明 CAD 全局坐标被写进了局部惯量。
2. CR5 链只有 `arm_joint1` 至 `arm_joint5`，没有清晰的 J6、`tool0` 和 TCP；CR5 手册明确
   为六个旋转关节。`gripper_joint` 不能替代第六轴。
3. 四个舵轮 origin 与旧图纸尺寸不一致。按用户决定，本阶段不根据旧图纸重定位，保留当前
   `robot_pick_robot11` URDF 的 origin、rpy 和网格变换；待更新后的 SolidWorks 模型确认后，
   再单独冻结轮心平面和运动学参数。
4. 零位 FK 下转向轴并非竖直 Z、轮轴并非统一车体 Y；必须按 REP-103 重新定义
   `base_link`、steer axis 和 wheel axis。`r2_wheel_joint` velocity 为 0（第 526-530 行），
   会锁死右后轮。
5. 右指行程 upper=`0.8 m`（第 1165-1169、1226-1230 行）不符合网格尺寸；需要实际行程、
   mimic 关系和单一 actuator 方案。
6. `mid360_joint` 原点为零（第 575-587 行），篮筐 joint 也为零；D405/MID-360 没有标准
   optical frame、CameraInfo 或 Gazebo sensor 定义。

### 8.4 P2 工程问题

- visual 与 collision 共用完整 STL；舵轮约 66k 三角面、MID-360 约 83k 三角面，需简化
  collision mesh。
- 没有 `ros2_control`、transmission、Gazebo plugin、`base_footprint`、odom 或 map 定义。
- `joint_names_robot_pick_robot11.yaml` 第 1 行含空字符串，并把右指 link 名当作 joint 名。
- ROS 1 `display.launch` 还引用包内不存在的 RViz 配置路径。

### 8.5 结论和下一步

`robot_pick_robot11.urdf` 的正面结论是：XML 能解析、link/joint 树可读取、mesh 文件齐全；
但它不能直接运行在 ROS 2 Jazzy/Gazebo Harmonic。不要继续在导出 URDF 上局部打补丁，应在
`ros2_ws/src/agri_robot_description` 重新建立 canonical Xacro，并在
`sim_ws/src/agri_sim_description` 增加仿真控制/传感器 overlay。

重建顺序：唯一命名与坐标系 -> 四舵轮局部 origin/惯量 -> CR5 六轴与 `tool0` -> 夹爪
左右指/mimic -> D405/MID-360 optical frame -> collision mesh -> ros2_control -> Gazebo SDF。

## 9. 当前 ROS 2/Gazebo 转换状态

已创建 canonical 副本：

`../ros2_ws/src/agri_robot_description/urdf/robot_pick_robot11.urdf`

本副本已做的最小修正：

- 四个转向 link 的质量改为 15 kg；错误的装配全局质心已替换为局部质心，当前惯量按居中后的
  mesh 包围盒近似计算，待新版 CAD 局部质量属性替换；
- 右后轮 velocity 上限改为 2 rad/s，与其他驱动轮一致；
- 右指轴改为 `0 0 -1`，joint 改名为 `gripper_right_finger_joint`，加入 mimic 关系；
- 在 `base_footprint -> base_link` 固定关节加入 CAD 到 ROS 的整体轴变换
  `rpy="1.5707963267949 0 1.5707963267949"`，将
  `(x_cad, y_cad, z_cad)` 映射为 `(x_ros, y_ros, z_ros)=(z_cad, x_cad, y_cad)`；
- Gazebo 生成高度参数化为 `spawn_z`，默认 `0.40 m`，使轮底与地面接触；
- `cr5_joint6` 的父坐标 Z 从 `-0.083 m` 修正为 `-0.0615 m`；当前 STL 计算得到的
  `arm_link5`/夹爪安装面间隙由约 `21.5 mm` 降至小于 `0.001 mm`；
- 为底盘、轮胎、CR5、夹爪、传感器和篮筐设置非空 material 名称及可区分的 RGBA 颜色；
- 将 `r2_steer_joint` 重定基到右后轮轮心、把 `r2_wheel_joint` 平移归零，并改用已居中的
  同侧转向网格；四个轮组的 steer/wheel frame 均同心，转向轴与轮轴点积为零；
- 将四个 steer frame 统一移动到轮胎几何中心，并以 visual/collision 的反向 50 mm 偏移保持
  零位外观不变；转向 90° 测试中轮胎中心位移为零；
- 四个 steer inertial origin 已从错误的装配全局坐标改为各自局部网格内部；在新版 CAD 局部
  质量属性可用前，使用 15 kg 和 mesh 包围盒计算的临时对角惯量；
- 修复 f2 转向 link/joint 重名；
- 增加 `base_footprint`、`tool0` 和 `cr5_joint6` 语义 frame；
- 移除 fixed joint 的非法零轴；
- 规范 package URI 和 mesh 文件名。

验证结果：

- `ros2_ws` 中 `agri_robot_description` 构建成功；
- `sim_ws` 中 `agri_sim_description` 与 `agri_sim_bringup` 构建成功；
- canonical URDF 通过 `check_urdf`；
- `gz sdf -p` 转换成功；
- Gazebo Harmonic 启动成功，`ros_gz_sim create` 返回 `Entity creation successful`。
- 轴变换修正后，四个轮心在 SDF 中均位于同一 `z≈-0.233 m` 平面；干净启动并连续观察
  约 11 s，模型整体姿态保持接近零滚转/俯仰/偏航，没有初始侧躺。
- canonical 模型包含 23 个 link、22 个 joint，link/joint 名称无重复；质量总和约
  `161.8588 kg`（原始导出值约 `520.7283 kg`，差异主要来自四个转向 link 的质量修正）。
- `config/joint_names.yaml` 和 RViz 配置已随描述包安装，可作为后续控制器/MoveIt 的命名入口。

仍未完成、不能宣称为最终可采摘模型的部分：

- 四舵轮安装位置仍由 robot11 当前几何反算，尚未用新版 CAD 基准或实机尺寸标定；
- CR5 关节轴、零位、J6 是否对应原 `gripper_joint`、TCP 和限位尚未按实机校准；
- MID-360 和篮筐等 link 的惯性 origin 仍疑似是 SolidWorks 全局坐标；四个 steer link 当前使用
  包围盒近似惯量。正式动力学控制前必须用 CAD 局部质量属性替换近似值；
- collision mesh 仍暂时复用 visual STL；
- `ros2_control`、四舵轮控制器、CR5 控制器和夹爪控制器已加入并完成 Gazebo 动态加载验证；
  当前仍需做低速实车符号/限位标定；
- D405/MID-360 仍只有几何 frame，尚未加入 Gazebo 传感器、CameraInfo 和真实 optical TF；
- mimic 约束在当前 DART 物理引擎中会发出“不支持 mimic constraint”的警告，仿真控制阶段应改用
  单 actuator/parallel gripper controller 或显式同步控制。
