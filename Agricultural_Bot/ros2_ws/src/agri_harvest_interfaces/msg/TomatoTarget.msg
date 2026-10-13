# Stable within a detector process/short observation window; not a fruit truth ID.
string target_id
string class_name
float32 confidence
# Index in the SAME image's unfiltered Detection2DArray, not a PoseArray index.
uint32 detection_index
# Sampled depth surface proxy in the parent array's frame, in metres.
# A fruit centre, tool orientation and grasp pose are not inferred here.
geometry_msgs/Point position
bool mature
uint32 depth_sample_count
float32 depth_valid_fraction
float32 depth_mad_m
bool segmentation_used
bool temporally_stable
uint32 stable_frame_count
