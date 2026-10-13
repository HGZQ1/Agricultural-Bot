# Capture time and frame of every point, including empty observations.
std_msgs/Header header
builtin_interfaces/Time depth_stamp
# False observations clear previously usable targets; never retain old targets.
bool observation_valid
string rejection_reason
TomatoTarget[] targets
