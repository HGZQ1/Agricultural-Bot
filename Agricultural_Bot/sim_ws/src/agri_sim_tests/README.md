# agri_sim_tests

提供 MID-360 和 D405 的可执行接口验收。仿真运行后，在另一个已加载两个工作空间
的终端运行；验收器只订阅传感器、TF 和仿真时钟，不发布运动指令。
输出 JSON 报告，退出码 0 表示通过、1 表示失败。两个验收器均已启用 `use_sim_time`。

## MID-360

```bash
ros2 run agri_sim_tests check_mid360 --duration 5 --timeout 90
ros2 run agri_sim_tests check_mid360 --duration 60 --timeout 90
```

默认检查 `mid360_sensor_frame`、浮点 `x/y/z/intensity`、非零严格递增 stamp、
至少 5 仿真秒、9–11 Hz、每云有限回波、量程和 `base_footprint` 到扫描 frame 的 TF。
传感器局部 XYZ 范数应为 0.1–40 m，允许 1 mm 舍入容差；NaN/Inf XYZ 单独计无回波，
有限 XYZ 的 intensity 必须有限。局部俯仰 `atan2(z,hypot(x,y))` 为 −7° 至 +52°，
允许 `1e-6 rad` 容差，报告实测极值与越界点数。解析支持字段偏移、行 padding 和大小端。

可用 `--topic`、`--frame`、`--base-frame`、`--min-range`、`--max-range`
（别名 `--maxrange`）、`--min-hz`、`--max-hz` 改变接口；垂直边界参数使用度：

```bash
ros2 run agri_sim_tests check_mid360 --vertical-min-deg -10 --vertical-max-deg 45
```

## D405

```bash
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120
```

默认订阅 `/d405/color/image_raw`、`/d405/aligned_depth_to_color/image_raw`、
`/d405/color/camera_info`，要求三路 848×480、`camera_optical_frame`、30 Hz
（通过范围 27–33 Hz）。彩色支持 `rgb8/bgr8`；深度支持 `32FC1` 米与 `16UC1` 毫米，
NumPy 向量化处理大小端、padding、0/NaN/Inf。有限正深度应为 0.07–0.50 m，
算法工作区为 0.10–0.50 m；范围容差 1 µm。CameraInfo 检查 K/P/R/D、正焦距、主点、
旋转矩阵及畸变模型。全无回波图允许，负值或越界有限深度判失败。

三路 stamp 必须非零且严格递增，通过有界缓存单次消费配组，整组跨度 ≤33 ms。
每组按彩色图 stamp 查询 `base_footprint -> camera_optical_frame` 动态 TF，
以墙钟 100 ms 非阻塞重试；同步且及时 TF 的组数占收到最多消息流的比例必须 ≥99%。
报告三路消息数、丢弃、未配对、同步跨度、TF 等待与联合覆盖率，防止丢帧被配组比例隐藏。

有近处靶标的验收世界可要求至少一个有效深度样本：

```bash
ros2 run agri_sim_tests check_d405 --duration 60 --timeout 120 --require-valid-depth
```

可通过 `--color-topic`、`--depth-topic`、`--info-topic`、`--frame`、`--base-frame`、
`--width`、`--height`、`--min-depth`、`--max-depth`、`--sync-ms`、`--tf-ms` 和
`--min-coverage` 等参数配置其他接口。`depth_metres` helper 将两种编码转成 float32 米，
无效值统一为 NaN，可供后续算法编码适配参考；其边界测试不需要仿真进程。

## 二维导航扫描

启动 `use_scan:=true` 后，运行：

```bash
ros2 run agri_sim_tests check_scan --duration 8 --timeout 60
```

检查器订阅 `/scan` 和 TF，要求默认 `mid360_scan_frame`、非零递增仿真时间戳、合法
角度/量程元数据、正无穷空 beam、9–11 Hz，以及扫描 frame 与
`base_footprint -> mid360_sensor_frame` 共光心且水平。`--allow-empty` 可用于没有障碍物
的空场景；番茄田验收应保留默认的有限回波要求。标准
`pointcloud_to_laserscan` 对 360°/1° 请求输出 360 个 bin，检查器允许其上边界约定。

## 时间与检查范围

两个验收器使用 SensorDataQoS（Best Effort、Volatile、Keep Last 5）。`duration` 是
有效 sensor stamp 的仿真跨度，`timeout` 是墙钟上限。sim Hz 使用递增 stamp，wall Hz
使用接收时间；较慢实时因子可降低 wall Hz，但不会单独导致失败。重复、倒退或无效
stamp 仍判失败，且不污染有效频率统计。暂停、采集不足、超时或中断不能返回通过。

验收器不代替场景几何、扫描真实性、IMU、地图或实机外参标定测试。独立场景脚本和
指标见 [MID-360 文档](../../../docs/mid360_simulation.md) 与
[D405 文档](../../../docs/d405_simulation.md)。

番茄田底盘评测在工作空间的 `scripts/check_chassis_motion.py`，启动与指标说明见
[底盘运动测试](../../../docs/chassis_field_motion.md)。该脚本显式使用 `--execute`
才发布速度；需独立 ROS 域/Gazebo 分区且退出其他速度发布者。Gazebo 位姿仅用于评测，
不会注入算法；报告区分命令结束和停稳后的终点，并输出真值 CSV。
