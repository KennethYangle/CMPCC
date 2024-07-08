Header header

time start_time
time final_time

# the trajectory id, starts from "1".
uint32 trajectory_id

# Number of control points per segment
int32[] K
int32   K_max

# Control points
geometry_msgs/Point[] pts

# Evaluating time
float64[] time

# the order of trajectory.
uint32 num_order
uint32 num_segment

string debug_info