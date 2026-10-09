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

`--plant-height` changes the branch/leaf crown-top height above ground, using
the original 1.298104 m height as its Z-scale reference. `--plant-width` changes
the maximum local X/Y branch/leaf span before yaw, using the original
0.866695 m width as its X/Y-scale reference. Height and width are independent.
Explicit dimensions inline a resized plant model into the output SDF and scale
its grounded stem collision; shared model, mesh and texture assets are unchanged.
Omitting both options retains the original world-generation path. Without fruit
randomization, this also preserves the baseline's original collision box.
Use `--plant-collision-width` to override the square X/Y footprint of that
simplified stem box without shrinking the visual canopy.  `--plant-collision-height`
can independently override its Z extent; when either option is omitted, that
dimension is derived from the corresponding plant scale.  These options affect
Gazebo physical collision and fruit clearance only; LiDAR/GPU rendering still
sees the visual branch and leaf meshes.
The generator rejects row/column layouts whose conservative yawed-canopy bounds
leave the configured ground box; change the row count, plant count, origin, or
ground dimensions together.

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

Fruit diameter, world-frame center height and horizontal placement radius do
not scale with the branches or leaves. Wider plants have wider derived stem
collision boxes unless `--plant-collision-width` overrides them, and require adequate
`--fruit-radius-min`; rejection sampling may change
positions in crowded layouts. Without `--randomize-fruits`, dimension options
still resize the foliage, but existing fruit and blossom meshes retain their
original sizes and positions. The optional JSON includes `plant_geometry`
(`height`, `width`, `mesh_scale`, `stem_size`, `stem_center_z`,
`collision_width`, `collision_height`); the collision fields report the actual
stem box dimensions and center Z.

Example values, pending real greenhouse measurements:

```bash
python3 scripts/generate_tomato_field.py \
  --randomize-fruits --plant-height 2.0 --plant-width 1.0 \
  --fruit-diameter-min 0.06 --fruit-diameter-max 0.09 \
  --fruit-height-min 0.80 --fruit-height-max 1.60 \
  --output /tmp/tomato_dimensions.sdf --metadata /tmp/tomato_dimensions.json
```

After updating the package, rebuild `agri_greenhouse_worlds`, regenerate the
world, and restart its Gazebo instance. Loading an existing SDF does not apply
new generator parameters retroactively.
实际尺寸待测，示例参数与模型限制见
[植株尺寸与果实参数化指南](../../../docs/tomato_fruit_randomization.md)。
