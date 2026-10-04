# agri_robot_description

Canonical ROS 2 description package for the `Agricultural_Bot` robot.

The current URDF is an imported and minimally normalized `robot_pick_robot11`
snapshot. It is a validation milestone, not the final calibrated model.

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

Validate the installed copy with `check_urdf` and `gz sdf -p` before changing
the joint names in downstream controllers.

Known follow-up work:

- Rebuild local link origins and inertias from the SolidWorks coordinate systems.
- Confirm the CR5 sixth-joint interpretation, flange, TCP, and joint limits.
- Replace full-resolution collision meshes with simplified meshes.
- Add the final ROS 2 control and Gazebo sensor overlay in `sim_ws`.
