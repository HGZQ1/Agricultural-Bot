# agri_base_adapter

Navigation-facing safety and compatibility interfaces for the mobile base.

- `/cmd_vel` (`TwistStamped`): manual/legacy input, highest priority.
- `/cmd_vel_nav` (`TwistStamped`): future Nav2 input, lower priority than manual.
- `/cmd_vel_safe` (`TwistStamped`): sole guarded output consumed by four-wheel kinematics.
- `/base_motion/lock_cmd` (`Bool`) and `/base_motion/set_lock` (`SetBool`): motion lock.
- `/base_motion/locked` and `/base_motion/active_source`: latched status.
- `/wheel/odom`: source odometry input; `/odom`: validated navigation-facing output.

The gate publishes zero after a source timeout and immediately while locked.
Locking or unlocking clears all buffered commands, so an old command cannot
resume. Stage one disables lateral velocity until physical lateral motion has
its own acceptance test.

The default limits are `linear.x=0.30 m/s`, `linear.y=0`, and
`angular.z=0.50 rad/s`. Input frames must be `base_footprint` and timestamps
must be current simulation timestamps. The manual and navigation source
timeouts are 0.35 s; the lower four-wheel node keeps its independent 0.5 s
watchdog. Lock or unlock with:

```bash
ros2 service call /base_motion/set_lock std_srvs/srv/SetBool "{data: true}"
ros2 service call /base_motion/set_lock std_srvs/srv/SetBool "{data: false}"
```

For the stage-seven parking/work handshake, start this gate with
`require_harvest_interlock:=true`. The parameter defaults to `false` to preserve
standalone mapping/navigation operation. The simulation bringup forwards the
same argument to its existing gate; do not start a second gate alongside it.
An enabled gate starts locked and accepts an explicit unlock only while
`/harvest_worker/state` (`agri_harvest_interfaces/HarvestWorkerState`) reports
`arm_stowed=true` and phase `IDLE` or `STOWED`. New observations must have positive,
strictly advancing stamps, be at most 0.5 s old and no more than 0.05 s in the future.
The receipt lease is 0.75 s on a monotonic wall clock, so it also expires when
simulation `/clock` is paused. These windows are exposed as
`harvest_worker_max_age_s`, `harvest_worker_future_skew_s` and
`harvest_worker_timeout_s`; `harvest_worker_state_topic` can remap the worker.

An exact duplicate (including frame, run, waypoint and work identity) is harmless
only while the original lease remains valid; it never renews the receipt time.
A state or identity change at the same stamp immediately revokes permission.
Replaying either message cannot restore it: a fresh advancing stowed report and
an explicit unlock are required. Old timestamps and clock rollback still revoke.

Deployment, unknown state, lost feedback or a backwards clock jump immediately
locks the final output and clears pending commands. Both the unlock service and
the unlock topic are checked, and the periodic output continues checking the
lease independently of the task executor. A fresh stowed heartbeat after a
failure permits a new explicit unlock; it does not automatically resume motion.
Orderly shutdown publishes locked zero output and rejects further unlocks.

```bash
ros2 launch agri_base_adapter base_adapter.launch.py \
  use_sim_time:=true require_harvest_interlock:=true
```

The mock worker reports the contract for interface testing. This gate does not
validate mechanical joints, controller health, the worker's physical identity
or grasp completion; the real arm integration must provide truthful periodic
state and retain its own motion-stop safeguards.

When robot_localization is introduced later, stop `odom_adapter` and disable
the wheel node TF so the EKF is the only `/odom` and `odom -> base_footprint`
publisher.
