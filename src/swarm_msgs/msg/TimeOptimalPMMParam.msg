Header header

# the trajectory id, starts from "0".
uint32 trajectory_id

# initial state
geometry_msgs/Vector3 x0
geometry_msgs/Vector3 v0
# terminal state
geometry_msgs/Vector3 xT
geometry_msgs/Vector3 vT

# acceleration constraint, both positive
geometry_msgs/Vector3 umax
geometry_msgs/Vector3 umin

# phase1: [0, t1], phase2: [t1, t2], phase3: [t2, T]
geometry_msgs/Vector3 t1
geometry_msgs/Vector3 t2
float32 T
# 0: ↗(→)↘, 1: ↘(→)↗, 2:↗or↘
geometry_msgs/Vector3 case_idx

# sampling
geometry_msgs/Vector3[] t_all
geometry_msgs/Vector3[] x_all
geometry_msgs/Vector3[] v_all
geometry_msgs/Vector3[] u_all