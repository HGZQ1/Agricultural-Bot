# Single tomato fruit mesh

`meshes/fruit.dae` contains one connected fruit body extracted from the existing
Apache-2.0 tomato asset, retaining its original vertex normals and UV mapping.
It uses the third connected body of `Fruit1`, rather than copying the three-fruit
cluster. `SOURCE.json` records the source hashes, component and transformation.
The original `tomato_0` asset and textures are unchanged.

The mesh is centered on its axis-aligned bounding box and uniformly normalized
so its furthest vertex has radius 0.5. For a fruit diameter `D` metres, use mesh
scale `D D D` and a conservative spherical collision radius `D / 2`. This
retains the original shape and keeps every visual vertex inside the collision
sphere; `D` is the diameter of that bounding sphere, not a separate axis length.

Like the original COLLADA, this file has no embedded material. Set the SDF PBR
albedo map to the original `model://tomato_0/materials/textures/AG15frt1.png` for
a ripe fruit or `AG15frt4.png` for an unripe fruit. Use white RGBA for diffuse
and ambient so the texture supplies its original color.

Rebuild the default asset from the workspace root:

```bash
python3 Agricultural_Bot/sim_ws/src/agri_greenhouse_worlds/scripts/extract_tomato_fruit.py
```

The extraction uses only the Python standard library and produces deterministic
bytes. Texture files stay in `tomato_0`; this derived mesh is covered by the same
package `LICENSE` and upstream attribution in `NOTICE`.

The original body has a small open boundary at its top; this extraction preserves
it and its triangle topology. Close views can show that opening. The closed,
conservative sphere collider still encloses the complete visible mesh.
