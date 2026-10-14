# Mechanical-arm harvest worker

This package adapts the collaborator's 2036 grasp sequence to the existing
`HarvestWork` transaction. The original three files were copied into
`reference/collaborator_2036` for traceability; they are not imported at runtime.

Run this worker only after the project Gazebo controllers, MoveIt backend,
harvest scene bridge, perception and parking handshake are running:

```bash
ros2 launch agri_arm_control harvest_arm_worker.launch.py
```

The worker starts with a false stow lease until six arm positions and velocities
are fresh, match the configured stow pose, and remain settled. It accepts goals
only with matching route/station nonces, the executor's `STARTED` acknowledgement,
fresh motion-lock feedback and continuously measured zero base velocity.
In Gazebo mode, the scene bridge must additionally return `SCENE_READY` after
applying the complete current plant/fruit/ground geometry to MoveIt. Missing
scene acknowledgement keeps readiness false and prevents the scan itself.
Only observational `ready` calls retry scene-lock `BUSY` or transport timeout,
within one 30-second wall deadline. Contact selection and attach/release commands
are sent once because a lost reply can conceal a physical change. During work,
the owner refreshes the scene before each plan; background health does not race
the scene transaction.
The worker also requires real status snapshots from both trajectory controllers;
an empty local goal handle after restart never proves the old controller stopped.
On a cold start, a controller may never have published its first status. With
the simulator running and the base locked and stopped, initialize real status:

```bash
ros2 service call /harvest_arm_worker/initialize_sim_hold std_srvs/srv/Trigger '{}'
```

This explicit Gazebo-only step validates hardware, controller ownership, current
collision scene and fresh stationary joints. It cancels all previous goals,
waits for any reported canceling goal's real terminal status, then submits two
bounded holds at the current measured positions. It never moves to stow or
publishes a synthetic status. A stationary arm starting away from stow enters
`FAULT`; use guarded `recover_stow` before route startup. Missing or ambiguous
terminal evidence keeps the lease false.

At startup, a fresh stationary arm whose measured joints exceed the stow
position tolerance enters `FAULT` and requires `recover_stow`. A pose already
within tolerance waits for its at-pose settlement timer; an unstarted timer
does not establish a non-stow pose. The `/harvest_arm_worker/status`
`stow_evidence` field reports actual joints, ROS/wall feedback ages, position
and velocity tolerances, settlement progress and the last rejected update.

Worker heartbeat callbacks run on a steady timer, but their stamps are the real
ROS sample time. A timer callback and action-finally callback can therefore
observe the same simulation time. The worker serializes publication and
suppresses only identical stamped state messages. A change in measured stow,
phase or work identity is published even when `/clock` has not advanced; the
base gate then revokes the lease. Status and state use the same sampled stamp.
No synthetic timestamp increments or longer freshness windows are used.

`scan_before_selection=true` moves to the configured side's scan joints inside
the same locked transaction, then selects a new mature, stable observation
captured after scan settlement. Empty scan joints reject this mode. A scan pose
passing IK is not evidence that the wrist camera actually sees a fruit.

All work and basket poses use `base_footprint` (+X forward, +Y left, +Z up).
TF rotates them into the planning frame, including the CAD `base_link` axes.
The new `grasp_tcp` is a geometric candidate; its offset and the nominal 25 mm
surface-to-centre correction still require calibration. Finger positions are
metres, with zero open and positive travel closing both independent fingers.

Closure uses at most 50 small steps of 1 mm over 0.25 simulation seconds,
within a 120 wall-second budget. Every step requires the real controller's
successful terminal result and fresh stopped feedback. `check_grip` queries
the selected simulator fruit's actual collision contacts, identity and jaw
enclosure. A contacting finger holds its measured position while the other
continues; their final positions may differ for an off-centre fruit.
`CONTACT_SETTLING` stops further closure and holds the measured positions,
then waits at most five wall seconds for `CONTACT_CONFIRMED`. A final successful
current-position hold and a new double-contact acknowledgement are required
before `attach`, which independently checks the same physical conditions.
Any controller abort, missing contact evidence, contact loss or exhausted
budget faults the work. The fixed `gripper_close_m` value remains a reference
parameter and is not the grasp success condition. `grip_closure` in worker
status reports the step and measured positions.

Every plan has a deadline. Trajectory execution requires accepted goals,
successful terminal results and fresh settled joint feedback. Approach,
retreat, basket descent and lift require complete collision-checked Cartesian
paths. Timeout/cancellation cancels the actual action and waits for its terminal
result; a late accepted submission is canceled as soon as its handle arrives.
Grasp candidates require both collision-aware IK and a successful pregrasp path
before selection; an IK solution with a blocked pregrasp path is skipped.
Controller trajectories measure `time_from_start` in ROS/simulation seconds.
Execution therefore allows the trajectory duration × 1.25 + 2 simulation seconds
(capped at 30 for the arm, 4 for the gripper), with separate hard wall limits
(120/20 seconds) and a five-second stopped-clock guard. The complete work action
has a 400-second wall deadline. Status publishes the actual planned duration,
elapsed simulation/wall time, observed real-time factor and terminal confirmation.

Success requires the simulator to confirm attachment, release and stable
`IN_BASKET`, followed by measured six-joint `STOWED`. The result explicitly
marks `sim_fruit_verified`; it does not claim actual field harvesting.

On any failure the worker enters `FAULT` and keeps its stow lease false. After
checking the stopped simulation, request guarded retraction:

```bash
ros2 service call /harvest_arm_worker/recover_stow std_srvs/srv/Trigger '{}'
```

Recovery requires fresh locked/stopped feedback and a confirmed terminal
trajectory. It collision-plans the configured stow pose and confirms measured
settlement. The route reset remains an explicit operation. If a fruit is still
attached or the path is blocked, resolve the simulator/scene condition first.

The default `backend_mode=gazebo` rejects physical hardware plugins. Explicit
`backend_mode=mock` permits only `mock_components/GenericSystem`, for isolated
controller tests. No mock mode can claim a fruit was physically harvested.
Jazzy Gazebo may leave `ListHardwareComponents.plugin_name` empty. Only that
Gazebo case may use the live RSP `robot_description`: component names, the exact
Gazebo hardware plugin, active lifecycle state and complete available interface
sets must agree. Active arm/gripper controllers must own the configured joints,
and the scene bridge must still acknowledge `SCENE_READY`. An empty mock plugin
or a nonempty foreign plugin is always rejected.

Required services: `/plan_kinematic_path`, `/compute_cartesian_path`,
`/compute_ik`, `/compute_fk`, `/controller_manager/list_controllers`,
`/controller_manager/list_hardware_components`, `/harvest_scene/contact`, and
`/harvest_sim/command`. Required actions: `/arm_controller/follow_joint_trajectory`
and the configured dual-finger `/gripper_controller/follow_joint_trajectory`.
The worker sends MoveIt-planned trajectories directly to the controller so that
the exact controller goal can be canceled and observed terminal.
`/robot_state_publisher/get_parameters` is also required for the Gazebo empty
plugin fallback (`robot_description_service` parameter).
