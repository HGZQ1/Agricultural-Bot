# 项目脚本

在 `Agricultural_Bot` 根目录执行；先加载 ROS 2 Jazzy 和已构建的工作空间。

## setup_mid360_rgl.py

下载校验固定版本输入，编译 RGLGazeboPlugin，并把插件、RobotecGPULidar 核心库、
官方 MID-360 图样及许可证保存到项目内 `.cache/rgl`。不会安装到系统目录，也不需要
修改上游 C++。成功后记录 manifest 和编译日志，并实际执行 CUDA/OptiX 单射线探针。

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/setup_mid360_rgl.py
python3 scripts/setup_mid360_rgl.py --verify-only
```

可用 `--jobs 2` 限制编译并行度。`--cache-dir` 必须位于本项目内，改变后需要把脚本
输出的 `rgl_install_prefix` 和 `rgl_patterns_dir` 传给 launch。`--skip-gpu-check`
仅用于无 GPU 机器准备编译产物，不能作为后端已可运行的证据。

固定版本：RGLGazeboPlugin `d4bf3cf36fe4a363a56df1bec2ce3809720db563`（包版本
0.2.0），RobotecGPULidar v0.21.0。命名图样 `Livox Mid360` 为 40 段轮换、每段
20,000 射线；不能把完整 800,000 射线文件当成每帧重复图样。

## check_mid360_scene.py

验证静止机器人在 `mid360_validation.sdf` 中的点云：把传感器点转换到世界坐标，
与四墙内表面 `x/y=±2.95 m` 和地面 `z=0` 比较。Gazebo 真值仅用于本验收脚本。

先启动验证世界，在第二个已 source 环境的终端执行：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false world:=mid360_validation.sdf
```

```bash
python3 scripts/check_mid360_scene.py --backend gpu_lidar \
  --output artifacts/mid360_20261006/gpu_geometry.json
```

脚本采集最近三帧，排除距传感器不足 1.5 m 的近处机器人回波；至少需要 1,000 个环境
回波，98% 以上的环境点距墙/地面小于 30 mm 才通过。检查前保持机器人静止。
RGL 场景使用 `lidar_mode:=rgl` 并设置 `--backend rgl` 和对应输出文件名。

连续接口验收使用 ROS 包入口，而不是本目录脚本：

```bash
ros2 run agri_sim_tests check_mid360 --duration 60 --timeout 90
```

详细依赖命令、版本出处和当前验收结果见
[mid360_simulation.md](../docs/mid360_simulation.md)。

## D405 场景准备与验收

`setup_d405_validation_scene.py` 在 `d405_validation.sdf` 验收世界中读取当前机器人
真值和腕部 TF，放置 visual-only 红绿球及近黄/远蓝靶标；不改变机器人控制。
靶标由相机前向和世界 +Z 定向，不沿相机 roll 旋转，以便独立识别视野倒置。
`check_d405_scene.py` 使用 RGB、对齐深度和 CameraInfo 反投影，与测试靶标比较，
验证光学坐标、对齐、几何和近远裁剪。Gazebo 真值仅用于验收，不提供给算法节点。
在已 source 两个工作空间的终端，从项目根目录执行：

```bash
ros2 launch agri_sim_bringup simulation.launch.py gui:=false world:=d405_validation.sdf
```

另一个终端在控制器启动后执行：

```bash
python3 scripts/setup_d405_validation_scene.py
python3 scripts/check_d405_scene.py --require-upright
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120 --require-valid-depth --sync-ms 0
```

准备脚本可用 `--output-dir` 指定输出目录；检查脚本用 `--scene <目录>/scene.json`。
`--require-upright` 用于机械臂零位，要求世界上/左的红球出现在画面上/左，下/右的
绿球出现在下/右；合法腕部滚转后使用不带该参数的普通几何验收。
默认产物在 `artifacts/d405_scene_validation/`。三组几何帧、近远裁剪验收通过，红/绿球
P95 误差 0.462/0.508 mm。空视场允许无有效回波；`--sync-ms 0` 检查原生相同 stamp。
投影、编码、中央虚拟
光心和规格来源见 [d405_simulation.md](../docs/d405_simulation.md)。
