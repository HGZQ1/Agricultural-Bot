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

When robot_localization is introduced later, stop `odom_adapter` and disable
the wheel node TF so the EKF is the only `/odom` and `odom -> base_footprint`
publisher.
