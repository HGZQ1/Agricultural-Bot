# URDF 模型外观与装配位姿修改

当前运行模型位于：

`ros2_ws/src/agri_robot_description/urdf/robot_pick_robot11.urdf`

修改后需要重新构建 `ros2_ws`，关闭旧 Gazebo 进程并重新启动，安装空间中的模型才会更新。

## 1. 修改颜色

颜色只写在 link 的 `visual` 中，RGBA 每项范围为 `0.0-1.0`：

```xml
<visual>
  <geometry>
    <mesh filename="package://agri_robot_description/meshes/visual/arm_link5.STL"/>
  </geometry>
  <material name="arm_blue">
    <color rgba="0.22 0.48 0.78 1"/>
  </material>
</visual>
```

- `R/G/B` 控制颜色，`A` 控制透明度；实体模型通常使用 `A=1`。
- 颜色不应写进 `collision`，碰撞几何不会被渲染。
- 使用非空 material 名称，便于 RViz/Gazebo 识别和后续 Xacro 统一管理。
- Gazebo 必须重启；已经生成的 entity 不会自动重新读取 URDF。

当前配色：

| 部件 | material | RGBA |
| --- | --- | --- |
| 底盘 | `chassis` | `0.16 0.24 0.32 1` |
| 舵轮机构 | `steer_blue` | `0.28 0.52 0.78 1` |
| 轮胎 | `rubber_black` | `0.06 0.07 0.08 1` |
| CR5 | `arm_blue` | `0.22 0.48 0.78 1` |
| 夹爪底座 | `gripper_orange` | `0.95 0.48 0.10 1` |
| 夹指 | `finger_metal` | `0.78 0.80 0.84 1` |
| D405 | `camera_black` / `camera_lens` | 深灰 |
| MID-360 | `sensor_gray` | `0.35 0.38 0.42 1` |
| 篮筐 | `basket_green` | `0.20 0.55 0.25 1` |

## 2. 修改组件相对父节点的位姿

父 link 到子 link 的装配关系由 joint 的 `origin` 决定：

```xml
<joint name="cr5_joint6" type="revolute">
  <origin xyz="-0.048 0 -0.0615"
          rpy="1.5708 -1.5708 0"/>
  <parent link="arm_link5"/>
  <child link="gripper_base_link"/>
  <axis xyz="1 0 0"/>
</joint>
```

- `xyz` 单位为米，表达在父 link 坐标系中。
- `rpy` 单位为弧度，依次为 roll、pitch、yaw。
- 调整整个子组件及其后代时，修改 joint origin。
- 只校正 mesh 相对本 link 坐标系的导出偏移时，必须同时修改该 link 的
  `visual/origin` 和 `collision/origin`。
- `inertial/origin` 只表示质心，不能用于移动外观或装配位置。

不要只改 `visual/origin` 来修复装配关系，否则画面虽然贴合，碰撞体、关节轴和质心仍会留在旧位置。

## 3. 当前夹爪修正

原 `cr5_joint6` 的父坐标 Z 为 `-0.083 m`。STL 包围盒计算显示，`arm_link5` 与
`gripper_base_link` 最近安装面相距约 `0.0215 m`。当前值已修正为 `-0.0615 m`，其余轴和
旋转保持不变；修正后两安装面的计算间隙小于 `0.000001 m`。

该值是根据当前 STL 几何得到的仿真基线。获得新版 CAD 法兰基准后，应再以法兰坐标和实际转接板
厚度复核，不能把视觉贴合作为最终机械尺寸依据。

## 4. 验证

```bash
cd "/home/hgzq/Agricultural Bot/Agricultural_Bot"
source /opt/ros/jazzy/setup.bash

cd ros2_ws
colcon build --symlink-install
source install/setup.bash
check_urdf src/agri_robot_description/urdf/robot_pick_robot11.urdf

cd ../sim_ws
source ../ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agri_sim_bringup simulation.launch.py paused:=true
```

在 RViz 中可显示 TF 坐标轴，或运行：

```bash
ros2 run tf2_ros tf2_echo arm_link5 gripper_base_link
```

当前零位期望平移为约 `[-0.048, 0, -0.0615]`，旋转应与 `cr5_joint6` 定义一致。

## 5. 修正关节轴偏移或不垂直

### 5.1 先分清三个对象

以轮组为例：

```text
base_link
  └── steer_joint       # 决定转向轴位置、方向
      └── steer_link
          └── wheel_joint  # 决定车轮轴位置、方向
              └── wheel_link
```

- 轴心位置由对应 joint 的 `origin xyz` 决定。
- 轴向由 `origin rpy` 和 `axis xyz` 共同决定。
- mesh 是否围绕轴心正确放置，由 link 的 mesh 局部坐标以及 `visual/collision origin` 决定。

不能只看 `<axis xyz>` 是否互相垂直，因为 axis 需要经过 joint 的 `rpy` 旋转后才能在同一个
父坐标系中比较。

### 5.2 右后轮本次修正

原模型同时存在三段配套补偿：

```text
r2_steer_joint xyz = -0.24413146 -0.19489488 -0.31673129
r2_wheel_joint xyz = -0.088 -0.12414 -0.038105
r2_steer_link.STL = 居中网格 + 同一段平移
```

因此轮子看起来处在大致正确的位置，但 `r2_steer_link` 的转向轴心与车轮轴心分离。

修正后的 joint：

```xml
<joint name="r2_steer_joint" type="revolute">
  <origin xyz="-0.170488836152483 -0.232999882221513 -0.228314645098511"
          rpy="-1.57079632679473 1.56515770851357 0"/>
  <axis xyz="0 0 -1"/>
</joint>

<joint name="r2_wheel_joint" type="continuous">
  <origin xyz="0 0 0" rpy="0 0.8213 0"/>
  <axis xyz="0 -1 0"/>
</joint>
```

`r2_steer_link` 使用与同侧前轮相同的 `f2_steer_link.STL`。随后四个轮组统一把 steer frame
移动到轮胎几何中心，并给 steer/wheel 的 visual 和 collision 增加 `xyz="0 -0.05 0"` 反向
补偿。零位外观位置不变，但 steer/wheel frame 现在同心，转向轴与轮轴在 ROS 坐标中严格垂直，
轮胎几何中心在转向过程中不再公转。

### 5.3 通用重定基公式

若子 joint 使用非零平移 `t_child` 把轴心补回正确位置，并且父 joint 的旋转矩阵为
`R_parent`，可将父关节轴心移动到子关节轴心：

```text
p_parent_new = p_parent_old + R_parent * t_child
p_child_new  = [0, 0, 0]
```

若只是移动 link 坐标系、希望网格在世界中的位置不变，则必须同步处理网格：

```text
visual_origin_new    = translation(-t_child) * visual_origin_old
collision_origin_new = translation(-t_child) * collision_origin_old
inertial_origin_new  = inertial_origin_old - t_child
```

本次 `r2_steer_link.STL` 已确认等于 `f2_steer_link.STL + t_child`，所以直接改用居中的
`f2_steer_link.STL`，不再额外增加 visual/collision 平移。

如果坐标系还发生旋转，不能只减 xyz；需要使用完整的齐次变换，并将惯性矩阵按
`I_new = R * I_old * R^T` 转换。此类情况优先从 CAD 以正确局部原点重新导出。

### 5.4 每个节点的修改规则

| 异常 | 应修改的位置 |
| --- | --- |
| 子组件整体相对父组件偏移 | 父子 joint 的 `origin xyz/rpy` |
| 关节轴心没有穿过组件 | joint origin；必要时重新居中 mesh |
| 两个轴应垂直但实际不垂直 | 将两个 axis 转到同一坐标系后，修正 joint `rpy` 或 `axis` |
| TF 正确但视觉网格偏移 | 同时修改 `visual/origin` 和 `collision/origin` |
| 物理重心偏移但外观正确 | 修改 `inertial/origin` 和惯性矩阵 |
| 同类对称部件只有一个异常 | 对比同侧/对角部件的 joint、mesh 包围盒及局部原点，不要只复制 xyz 符号 |

修改完成后至少检查：

```bash
check_urdf robot_pick_robot11.urdf
gz sdf -p robot_pick_robot11.urdf >/tmp/agri_robot.sdf
ros2 launch agri_robot_description display.launch.py
```

在 RViz 中开启 `TF -> Show Axes` 和 `TF -> Show Names`，确认轴心重合。对于应垂直的两个
单位轴向量 `a`、`b`，应满足 `abs(dot(a, b)) < 1e-6`。

## 6. 舵轮偏心旋转与质心修正

当前轮胎 STL 的局部包围盒在 Y 方向为 `0-0.10 m`，因此原 wheel frame 位于轮胎内侧面，
轮胎几何中心相对转向轴偏移 `0.05 m`。当 steer joint 转动时，轮胎中心会绕转向轴产生
50 mm 半径的公转。

四个轮组统一采用以下修正：

1. steer joint origin 沿 steer frame 的 `+Y` 移动 `0.05 m`，将转向轴移到轮胎中心。
2. steer link 和 wheel link 的 visual/collision origin 设置为 `0 -0.05 0`，保持零位外观不变。
3. wheel inertial origin 的 Y 减去 `0.05 m`，保持原轮胎质心在世界中的位置不变。
4. steer/wheel joint 之间保持 `xyz="0 0 0"`，使两轴相交。

修正后的 steer joint origin（仍以 `base_link` 的 CAD 坐标表达）：

| Joint | XYZ (m) |
| --- | --- |
| `f1_steer_joint` | `0.1730292170 -0.2330001837 0.2282201851` |
| `f2_steer_joint` | `-0.1730223636 -0.2330000000 0.2216458300` |
| `r1_steer_joint` | `0.1704875673 -0.2330000000 -0.2217400588` |
| `r2_steer_joint` | `-0.1704888362 -0.2329998822 -0.2283146451` |

### 6.1 steer link 质心

原四个 steer inertial origin 经 joint 变换后都落在 `base_link` 中近似同一点：

```text
[-0.03274, 0.26074, -0.04975] m
```

这证明 SolidWorks 导出的是装配全局质心，不是各 steer link 的局部质心。现已按居中后的统一
steer mesh 包围盒设置临时参数：

```xml
<inertial>
  <origin xyz="0 0.009266275 0.072000005" rpy="0 0 0"/>
  <mass value="15.0"/>
  <inertia ixx="0.2244577768" ixy="0" ixz="0"
           iyy="0.2240913810" iyz="0"
           izz="0.1893391739"/>
</inertial>
```

这是以 15 kg 和 mesh 包围盒计算的稳定仿真近似值，不是最终 CAD 质量属性。新版 SolidWorks
模型应按每个 link 的局部坐标重新导出质心和惯性，并替换这里的近似值。
