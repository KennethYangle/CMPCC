Header header

# the trajectory id, starts from "0".
uint32 trajectory_id

# initial state
geometry_msgs/Point x0
geometry_msgs/Point v0

# acceleration constraint, both positive
geometry_msgs/Point umax
geometry_msgs/Point umin

# phase1: [0, t1], phase2: [t1, t2], phase3: [t2, T]
float64 t1
float64 t2
float64 T
# 0: ↗(→)↘, 1: ↘(→)↗
uint32 case_idx
