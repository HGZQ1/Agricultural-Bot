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

The mock worker exercises this transport contract without commanding any arm
joint. Actual TCP, tool orientation, IK, trajectory collision and physical fruit
detachment remain part of the arm integration.
