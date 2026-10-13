# Periodic, short-lived feedback. A latched/stale true is not permission to unlock.
std_msgs/Header header
# Empty identity is only suitable for initial idle/stowed worker readiness.
string work_id
string run_id
string waypoint_id
bool arm_stowed
string phase
