# Represents a single point on a discrete trajectory

float64 time_from_start  # Time elapsed since the beginning of the entire trajectory (seconds)
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration