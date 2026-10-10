# 参数化番茄田：准备、构建、打开与复现

维护者：fsy；更新日期：2026-10-07。

目标：在 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 中加载 22×14 m 番茄田，
并可选择在入口生成 agri_robot。当前启动默认是 6 行×10 株，共 60 株；行距 2 m，
株距 0.7 m，六行以 `x=0` 对称排列。原 10 行×15 株世界仍作为兼容基线保留。
本功能已与远端 10.6.2 的 MID-360/D405 启动框架集成。

## 1. 文件与下载来源

| 内容 | 位置或来源 | 是否需要再次下载 |
| --- | --- | --- |
| 场地包 | `sim_ws/src/agri_greenhouse_worlds` | 随本次代码一起提供 |
| 番茄网格、纹理、材质 | 上述包的 `models/tomato_0` | 已内置，无需联网下载模型 |
| 场地生成器 | `scripts/generate_tomato_field.py`（位于场地包内） | 已内置；仅用 Python 标准库，无额外 pip 依赖 |
| 生成后的默认世界 | 场地包内 `worlds/tomato_field_default.sdf` 及同名 JSON | 已内置；可直接运行 |
| 兼容基线世界 | 场地包内 `worlds/tomato_field_22x14.sdf` | 旧 10 行×15 株回归使用 |
| 一键入口 | `sim_ws/src/agri_sim_bringup/launch/tomato_field.launch.py` | 已内置 |
| 原始番茄项目 | https://github.com/LCAS/aoc_tomato_farm | 仅追溯或修改原始资产时需要 |
| 另一调研项目 | https://github.com/javadibrahimli/tomato_agribot_ros2 | 仅学习参考；本入口不依赖它 |

番茄资产取自 `d8243e48c92377fbd9754400e0ddb1fcffded9d7`，目录为
`tomato_farm_generator/generated_examples_/farm_22mx14m/models/22mx14m/tomato_0/`。
Apache-2.0 许可证来自上游后续提交 `0f4bc757348923dff618ca3b3eac7ce9aa1b4d35`。
本包保留 LICENSE 和 NOTICE，调整资源路径并重新生成布局，不依赖上游相机演示窗口。
模型网格 SHA-256：`3d58113beff9717e9c55eb45be41d2e9650b9ed35fe6d937292834f347549fa1`。

只有需要查看上游来源时才执行下面命令，普通复现跳过：

```bash
git clone https://github.com/LCAS/aoc_tomato_farm.git
git -C aoc_tomato_farm checkout d8243e48c92377fbd9754400e0ddb1fcffded9d7
```

第二个命令切换到资产对应版本。该版本尚无后续新增的根 LICENSE，许可证来源见上面的独立提交。

## 2. 软件包与准备条件

必须先有 Ubuntu 24.04 和 ROS 2 Jazzy 软件源及安装环境。
如果 `/opt/ros/jazzy/setup.bash` 不存在，先按 [ROS 2 官方安装指南](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 完成基础安装。

下列是新电脑的依赖准备命令；本次打包时电脑已有这些基础组件，没有重复安装：

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-yaml ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
```

| 软件包 | 用途 |
| --- | --- |
| `git` | 获取和更新源码 |
| `python3-colcon-common-extensions` | 构建工作空间 |
| `python3-rosdep` | 根据各 `package.xml` 补齐依赖 |
| `python3-pytest` | 执行 Python 测试 |
| `python3-yaml` | 读取控制与传感器配置 |
| `ros-jazzy-ros-gz` | Gazebo/ROS 桥接与实体生成入口，包含相应 vendor 依赖 |
| `ros-jazzy-xacro`、`robot-state-publisher` | 展开机器人描述并发布坐标变换 |
| `joint-state-publisher-gui`、`rviz2` | 单独查看机器人与关节 |
| `gz-ros2-control`、`ros2-controllers` | 可选的仿真关节控制；也满足包声明的依赖 |

测试机器版本：Gazebo Sim 8.15.0，Python 3.12 系列；colcon extensions 0.3.0、rosdep 0.27.0、
ros_gz 1.0.24、gz_ros2_control 1.2.20、ros2_controllers 4.42.1。这是已安装版本记录，
不要求强行降级到完全相同补丁号；主要兼容基线是 Jazzy + Harmonic。

首次使用 rosdep：

```bash
sudo rosdep init
rosdep update
```

`init` 每台机器通常只运行一次；提示 sources list 已存在时直接进行 `rosdep update`，不要删除已有配置。

## 3. 获取代码并确定当前目录

本功能合并进主仓库后，新电脑可以：

```bash
git clone https://github.com/HGZQ1/Agricultural-Bot.git
cd Agricultural-Bot/Agricultural_Bot
```

如果主仓库尚未合并本次提交，先获取贡献分支：

```bash
git clone --branch fsy/tomato-field-reproduction https://github.com/fsy132/Agricultural-Bot.git
cd Agricultural-Bot/Agricultural_Bot
```

两种方式二选一。已有仓库且存在未提交修改时，不要覆盖目录；先用 `git status --short` 检查。

下面统一从同时含有 `ros2_ws` 和 `sim_ws` 的 `Agricultural_Bot` 目录操作。
当前 fsy 电脑可以直接进入：

```bash
cd /home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot
pwd
```

这条个人路径只是示例，其他电脑应换成自己的克隆位置。

## 4. 安装声明依赖并构建

在一个新终端执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ros2_ws/src sim_ws/src --ignore-src --skip-keys ament_python -r -y
```

`--from-paths` 扫描两个源码目录；`--ignore-src` 避免安装本仓库已经提供的包。
`--skip-keys ament_python` 只跳过仓库清单中这个不可解析的构建类型名；
Python包构建由已安装的colcon扩展和setuptools提供，本次干净构建已验证。
不要用它随意跳过其他未安装依赖。
如果报告无法解析的依赖，保留完整输出，解决后再继续，不应把失败当成功。

先构建机器人层：

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

再构建仿真层：

```bash
cd sim_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

顺序原因：仿真包需要找到机器人描述和运动学包。
正常结果是 `Summary: ... packages finished`，没有 failed/aborted。
先检查资源安装是否正确：

```bash
ros2 pkg prefix --share agri_greenhouse_worlds
ros2 launch agri_sim_bringup tomato_field.launch.py --show-args
```

应得到 share 路径，以及 `paused`、`spawn_x/y/z/yaw`、`use_control`、`use_lidar`、`use_camera` 等参数。

## 5. 以后每次打开新终端的准备

从 `Agricultural_Bot` 目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source sim_ws/install/setup.bash
```

三次 source 让当前终端依次找到 ROS、机器人包和仿真包。
它们不会启动 Gazebo，也不需要每次重新构建；修改源码或新增文件后才重新构建相应工作空间。

## 6. 只打开场地（不加载机器人）

```bash
export GZ_PARTITION=agricultural_bot_field_only
field_share="$(ros2 pkg prefix --share agri_greenhouse_worlds)"
export GZ_SIM_RESOURCE_PATH="$field_share/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim "$field_share/worlds/tomato_field_default.sdf"
```

- `GZ_PARTITION` 给本次 Gazebo 通信设置独立名称。
- `field_share` 保存 ROS 查到的已安装资源位置，避免写死用户名。
- `GZ_SIM_RESOURCE_PATH` 告诉 Gazebo 到哪里寻找 `model://tomato_0`。
- `gz sim` 打开世界；这里没有 `-r`，默认暂停，点击左下角播放才运行物理。

应看到褐色地面、60 株番茄、太阳；Entity Tree 中有 `ground_plane` 和 `tomato_0` 至
`tomato_59`，以及随机果实模型。
这是三维仿真世界，不是 SLAM 生成的二维导航地图。
此模式没有机器人，属于预期行为。完成后在启动终端按 `Ctrl+C`，等待退出。

## 7. 加载番茄田与机器人

关闭上一节的 Gazebo，在已完成 source 的终端运行：

```bash
ros2 launch agri_sim_bringup tomato_field.launch.py
```

默认：暂停，控制器/运动学/相机/雷达关闭，机器人位姿为
`x=0, y=-6, z=0.40 m, yaw=1.5708 rad`（约 90°）。
world 名是 `field`，机器人实体名是 `agri_robot`。
先观察装配和接地，再点击播放；默认关闭传感器，因此没有相机图像是预期现象。

在另一个终端只加载 ROS 基础环境后查询：

```bash
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION=agricultural_bot_tomato_field
gz model --list
gz model -m agri_robot -p
```

分区必须与启动端相同。结果应包含 `agri_robot`，暂停状态位置接近上面的生成位姿。
正常落地后高度可以略变；持续大幅负值才需要排查碰撞和初始位姿。

可选，开启控制和原生 GPU 传感器：

```bash
ros2 launch agri_sim_bringup tomato_field.launch.py \
  use_control:=true use_lidar:=true use_camera:=true rviz:=true
```

先退出上一次启动再执行；启动仍默认暂停，点击播放后传感器才按仿真时间更新。
若只要后台测试，加 `gui:=false rviz:=false`；要自动开始物理，加 `paused:=false`。
D405 默认有效距离是 0.07–0.50 m，远处番茄不一定有有效深度。
新传感器的专用验收和限制分别见 [MID-360](mid360_simulation.md) 和 [D405](d405_simulation.md)。

## 8. 可选新下载：RGL（普通场地复现不需要）

远端 10.6.2 新增的 `agri_sim_sensors`、`agri_sim_tests` 是仓库内源码包，随第4节构建。
默认 `gpu_lidar` 使用 Gazebo 原生能力，无需 RGL。只有需要 RGL 雷达后端时才在工程目录执行：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/setup_mid360_rgl.py
```

该脚本下载并构建第三方依赖到 `.cache/rgl`（已被 Git 忽略），版本/下载校验在脚本中固定：
RobotecGPULidar 核心 0.21.0，插件包 0.2.0。具体来源和系统依赖以
[MID-360 安装文档](mid360_simulation.md) 及脚本为准，需要支持的 NVIDIA GPU/驱动。
安装成功后使用 `lidar_mode:=rgl use_lidar:=true`。本次场地上传未执行 RGL 安装，也不包含其二进制。

## 9. 修改场地参数并重新生成

正式生成器位于 `sim_ws/src/agri_greenhouse_worlds/scripts/`，先做临时小规模实验：

```bash
python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py \
  --rows 2 --plants-per-row 3 --output /tmp/tomato_demo.sdf
```

应输出 `Generated 6 plants`，不会覆盖正式场地。
`--rows` 控制行数、`--plants-per-row` 控制每行株数；其他参数见 `--help`。
默认固定 `--seed 42`，使同样参数生成同样的轻微随机朝向。

从工程目录重新生成正式基线（会覆盖该源码世界文件）：

```bash
python3 sim_ws/src/agri_greenhouse_worlds/scripts/generate_tomato_field.py
cd sim_ws
colcon build --symlink-install --packages-select agri_greenhouse_worlds
source install/setup.bash
cd ..
```

重新启动 Gazebo 才会载入新场景；已打开的世界不会自动重读文件。
使用安装后的 `ros2 run agri_greenhouse_worlds generate_tomato_field.py` 时务必指定 `--output`，
不要让默认路径落在安装空间。改行数/间距时还应检查植株是否超出地面边界。

逐株果实数量和高度可通过 `--randomize-fruits` 参数化；完整示例、范围含义和限制见
[果实随机化指南](tomato_fruit_randomization.md)。自定义文件可直接用
`tomato_field.launch.py world:=/绝对路径/场景.sdf` 加载，世界名称自动读取。

### 9.1 布局变化后更新导航地图

改变 `--rows`、`--plants-per-row`、`--row-spacing`、`--plant-spacing` 或植株冠幅后，
旧静态地图、禁行遮罩和停车点都应视为失效。将新 SDF 与 `--metadata` JSON 保存到新的
`FIELD_ID` 目录，用该世界重新运行 `agri_navigation mapping.launch.py`，完成通道覆盖后
保存新的 YAML/PGM 和 pose graph。导航时把 `navigation.launch.py` 的 `map` 参数改成
新 YAML；局部代价地图会继续通过实时 `/scan` 更新。仅改变果实数量、高度、直径或成熟
比例且植株碰撞边界不变时，通常可以复用导航地图。完整命令和 keepout mask 生成方式见
[阶段四导航说明](navigation_stage4.md)。

当前底盘转向测试若采用 3 m 行距，建议在默认 22×14 m 地面内使用 6 行而不是继续
放置 10 行：`origin-x=-7.5`、`rows=6`、`plants-per-row=15`，行坐标为
`-7.5…7.5`，机器人默认出生点 `(0,-6,1.5708)` 位于中央通道。可同时传
`--plant-collision-width 0.06` 缩小简化茎杆碰撞盒，枝叶视觉冠幅仍由
`--plant-width` 控制。生成命令、启动命令及重建地图步骤见
[果实随机化指南中的 3 m 兼容场景](tomato_fruit_randomization.md)。

## 10. 常见故障与限制

| 现象 | 先检查什么 |
| --- | --- |
| `Package ... not found` | 是否成功构建、是否 source 两层 install、新开的终端是否加载环境 |
| `Unable to find uri model://tomato_0` | 场地包是否安装、只开场地时是否设置 models 路径 |
| 只看到网格，机器人穿下去 | 检查实际世界是否有地面 collision；网格线本身没有碰撞 |
| 播放/暂停快速闪烁或查不到实体 | 是否同一分区启动多个服务器；先在对应启动终端 Ctrl+C，再只启动一份 |
| 查询不到 `agri_robot` | 是否只是场地模式、分区是否一致、生成日志是否成功；先保留不带过滤的完整输出 |
| `world_name` 不匹配 | 参数必须等于 SDF 的 world name；通用入口留空会自动读取 |
| 修改文件无效 | 是否误改 install 副本、是否构建并退出旧进程后重启 |
| 图形驱动/EGL 报警 | 结合最终画面和渲染失败日志判断，不能仅凭警告认定机器人碰撞有问题 |

每次退出优先在所属终端按 Ctrl+C，避免无差别结束其他人的仿真。
原项目的相机示例与本世界是不同入口，本包不依赖那些演示摄像头。
当前植株静态、碰撞为简化盒体，番茄是 visual 网格的一部分；尚不支持逐果采摘/脱落、叶片物理或完整温室。
当前 DART 后端还会报告不支持夹爪 mimic constraint；实体可生成，但不能据此宣称双指物理联动已通过验收。

## 11. 上传前验证记录

本轮在独立安装目录构建成功（机器人层2包、仿真层6包），237项 Python 回归测试通过。
`rosdep check --from-paths ros2_ws/src sim_ws/src --ignore-src --skip-keys ament_python` 通过。
这里的历史记录覆盖旧基线世界逐字节再生成、150株计数、地面碰撞、资源引用、传感器处理后保留场地、
Launch无重复参数及安全默认值。无界面暂停启动创建实体成功，查询到
`agri_robot` 位姿 `[0,-6,0.40]`、yaw `1.5708`，列表包含150株和机器人。
45秒后测试主动发送 SIGINT 退出（timeout 返回124，Gazebo日志退出码-2）；不是启动失败。
执行环境限制了默认 `.gz` 日志目录写入，故出现日志写入提示，未妨碍实体查询。
GUI、开启物理后的长时间稳定性和新传感器联动没有在此次上传验证中重新验收。
具体本轮测试结果见仓库根目录 `开发日志.md` 的 2026-10-07 fsy 条目。
旧基线验收至少覆盖：生成可复现、150个实例与地面 collision、资源路径、双工作空间构建、
launch 参数、机器人实体生成和同分区查询。原 GUI 成果见 2026-10-06 记录；
未经本轮重测的图形/传感器效果不应写成“新版本已完整验收”。

## 12. 合并后的底盘实测

2026-10-07 已在独立实例开启物理和全部控制器，完成 305.398 仿真秒的运动测试。
五轮 1 m 前进/倒车及显式停车通过，停稳终点最大误差 3.585 mm。
整圈、小弧、累计轮式里程计航向与断流停车时间仍有未达标指标，整体测试未通过。
本轮未启用 GUI、LiDAR、相机，也未采集 Contact；不能替代传感器共存或完整碰撞验收。
启动、遥控、自动复测、各项结果及失败证据见
[番茄田底盘运动测试](chassis_field_motion.md)。
