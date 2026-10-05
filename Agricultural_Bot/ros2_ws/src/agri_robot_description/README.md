# agri_robot_description

Canonical ROS 2 description package for the `Agricultural_Bot` robot.

The canonical entry point is now:

```text
urdf/agri_robot.urdf.xacro
```

It composes the audited model from these component modules:

- `xacro/base.xacro`: root transform and chassis;
- `xacro/wheel_modules.xacro`: four steering and wheel assemblies;
- `xacro/cr5.xacro`: CR5 arm chain;
- `xacro/gripper.xacro`: wrist, tool frame and two fingers;
- `xacro/sensors.xacro`: MID-360 and D405 frames;
- `xacro/basket.xacro`: fixed collection basket.

Generate a standalone URDF with:

```bash
xacro urdf/agri_robot.urdf.xacro > /tmp/agri_robot.urdf
```

The supported top-level arguments are:

- `mesh_package:=agri_robot_description`
- `use_gazebo:=false` (`true` enables the Gazebo overlay include)
- `use_ros2_control:=false` (`true` removes the URDF finger mimic relation so
  simulation can explicitly command both fingers)

The actual Gazebo control system and controller YAML are owned by
`agri_sim_description` and `agri_sim_control`; this package therefore has no
dependency on `sim_ws`.

The RViz display launch expands the Xacro automatically.  A legacy `.urdf`
path can still be supplied through `model:=...` for regression comparisons.

The component modules were split from the minimally normalized
`robot_pick_robot11` snapshot. They are a validation milestone, not the final
calibrated model.

The source CAD export is intentionally left unchanged at
`../../../../robot_pick_robot11`. The ROS 2 copy in `urdf/robot_pick_robot11.urdf`
contains the requested migration edits:

- all four steering links use a `15.0 kg` mass;
- `r2_wheel_joint` uses the same velocity limit as `r1_wheel_joint`;
- the right finger joint axis is `0 0 -1` and its name is unique;
- current URDF dimensions and link-local installation data are retained;
- the root fixed joint applies the CAD-to-ROS axis conversion required by
  Gazebo (`rpy="1.5707963267949 0 1.5707963267949"`).

The conversion makes the CAD Y-up assembly use ROS Z-up and places the four
wheel centers on one horizontal plane. Inertial origins are still provisional
and must be recomputed from local link frames before dynamic controller tuning.

Validate component equivalence before changing joint names or model values:

```bash
python3 test/compare_urdf_structure.py \
  urdf/robot_pick_robot11.urdf urdf/agri_robot.urdf.xacro
```

The comparison covers every named link/joint subtree, including origins,
axes, limits, inertials, mesh references, materials, dynamics and mimic data.

Known follow-up work:

- Rebuild local link origins and inertias from the SolidWorks coordinate systems.
- Confirm the CR5 sixth-joint interpretation, flange, TCP, and joint limits.
- Replace full-resolution collision meshes with simplified meshes.
- Add the final ROS 2 control and Gazebo sensor overlay in `sim_ws`.
