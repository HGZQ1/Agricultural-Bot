# agri_greenhouse_worlds

Gazebo Harmonic tomato-field assets and deterministic world generation for the
Agricultural_Bot simulation workspace.

The initial tomato mesh and textures were adapted from
[`LCAS/aoc_tomato_farm`](https://github.com/LCAS/aoc_tomato_farm), commit
`d8243e48c92377fbd9754400e0ddb1fcffded9d7`, under the repository's Apache-2.0
license. The generated baseline reproduces the reference layout with 10 rows,
15 plants per row, 2.0 m row spacing, and 0.7 m plant spacing. A single plant
asset is instanced to keep the repository and Gazebo resource load manageable.

Regenerate the baseline world from this package directory:

```bash
python3 scripts/generate_tomato_field.py
```

Use `--help` to inspect parameters. Generated worlds contain no robot; the
robot is spawned by `agri_sim_bringup` so its pose remains configurable.

完整中文步骤（依赖、构建、只打开场地、导入机器人、参数及排错）：
[番茄田复现指南](../../../docs/tomato_field_reproduction.md)。
底盘控制启动、低速行间路线与量化评测见
[番茄田底盘运动测试](../../../docs/chassis_field_motion.md)。
第三方来源和本地修改记录见 [NOTICE](NOTICE)。
通过 `ros2 run` 执行生成器时必须用 `--output` 指定输出位置，避免写入安装空间。

Use `--randomize-fruits` to generate independent static fruit targets with
configurable count, height, diameter, horizontal placement, maturity probability
and a reproducible seed. The derived `tomato_plant` asset contains only stems and
leaves; the original mesh and baseline world remain available. Optional
`--metadata` writes offline world-frame annotations.
The default `--fruit-visual mesh` uses a single body extracted from the original
`Fruit1` mesh, its original UV/normals and existing red/green albedo textures.
`--fruit-visual sphere` restores the plain sphere appearance. Both modes retain
the same conservative sphere collision and seeded target positions.
实际尺寸待测，示例参数与模型限制见
[果实随机化指南](../../../docs/tomato_fruit_randomization.md)。
