# Harvest observation and work interfaces

`TomatoTargetArray` keeps each observation's class, confidence, track ID,
depth quality and position together. Its header carries the RGB capture time
and the coordinate frame of all points. `depth_stamp` records the paired depth
time. A target's `detection_index` refers to the same RGB frame's complete 2D
detection array; invalid depth must never shift class/position associations.

Positions are sampled surface proxies in metres. They are not calibrated fruit
centres or grasp poses. Track IDs are local to one detector process and a short
tracking interval, rather than Gazebo fruit IDs or persistent plant identities.
`temporally_stable` describes a bounded observation history, not IK feasibility.
An empty or invalid observation invalidates old candidates for work selection.

`HarvestWork` binds a selected target to a run, waypoint and unique work request.
The worker publishes periodic `HarvestWorkerState` with the same identity.
Completion requires a successful result and fresh matching stowed feedback;
action success or stale stow feedback alone must not release the base lock.
Initial idle readiness may use empty identity, but cannot complete active work.

`scan_before_selection=false` uses the supplied observation. When true, the goal
contains only the request timestamp and station identity; the guarded arm worker
first deploys the wrist camera, then selects a fresh `TomatoTargetArray` captured
after scan arrival. Selection and grasp stay within the same acknowledged parked
transaction. The returned `target_id` records the observation actually selected.
`sim_fruit_verified` refers only to checked Gazebo attachment/release/basket state.
It never asserts real-world detachment or visual plant identity.

`SimFruitCommand` carries a frozen visual observation and work/plant identity.
The planning scene exposes select/clear contact operations; the Gazebo gateway
exposes attach/release/status. Scene truth supplies obstacles and simulation
validation, never a replacement target observation.

The mock worker exercises this transport contract without commanding any arm
joint. The Gazebo integration in `agri_arm_control` and `agri_harvest_sim` uses
collision-checked MoveIt motion and verifies simulated fruit detachment and
basket state. Physical TCP calibration and field harvesting still require
independent hardware validation.
