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
