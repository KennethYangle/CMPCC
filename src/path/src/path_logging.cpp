#include "path_logging.h" // Include the header if you created one

// Constructor
PathLoggingNode::PathLoggingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
    // Get Parameters
    pnh_.param<std::string>("world_frame_id", world_frame_id_, "world");
    pnh_.param<int>("max_history_size", max_history_size_, 2000);
    pnh_.param<double>("min_viz_velocity", min_viz_vel_, 0.0);
    pnh_.param<double>("max_viz_velocity", max_viz_vel_, 5.0);

    if (max_history_size_ <= 0) {
        ROS_WARN("max_history_size must be positive, setting to 2000.");
        max_history_size_ = 2000;
    }
    if (min_viz_vel_ >= max_viz_vel_) {
        ROS_WARN("min_viz_velocity >= max_viz_velocity, adjusting max_viz_velocity.");
        max_viz_vel_ = min_viz_vel_ + 1.0;
    }

    ROS_INFO("Path Logging Node initialized:");
    ROS_INFO("  World Frame ID: %s", world_frame_id_.c_str());
    ROS_INFO("  Max Drone History Size: %d", max_history_size_);
    ROS_INFO("  Velocity Color Range: [%.2f, %.2f] m/s", min_viz_vel_, max_viz_vel_);

    // Publishers
    drone_path_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/drone_path", 1);

    // Subscribers
    // Drone pose
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &PathLoggingNode::poseCallback, this, ros::TransportHints().tcpNoDelay());
    // Drone velocity (local frame assumed)
    vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &PathLoggingNode::velocityCallback, this, ros::TransportHints().tcpNoDelay());

    ROS_INFO("Path Logging Node Ready.");
}

// --- Callbacks ---
void PathLoggingNode::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    latest_velocity_ = msg->twist; // Store the twist part
    has_vel_ = true;
}

void PathLoggingNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Check if we have velocity data yet
    if (!has_vel_) {
        return; // Can't calculate speed without velocity
    }

    double current_speed = 0.0;
    { // Lock scope for reading velocity
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        // Calculate speed (magnitude of linear velocity)
        current_speed = std::sqrt(std::pow(latest_velocity_.linear.x, 2) +
                                  std::pow(latest_velocity_.linear.y, 2) +
                                  std::pow(latest_velocity_.linear.z, 2));
    } // Release velocity lock

    { // Lock scope for modifying drone path history
        std::lock_guard<std::mutex> lock(drone_path_mutex_);

        // Create new point
        DronePathPoint new_point;
        new_point.position = msg->pose.position;
        new_point.speed = current_speed;
        new_point.timestamp = msg->header.stamp; // Use pose timestamp

        // Add to history
        drone_path_history_.push_back(new_point);

        // Maintain history size limit
        while (drone_path_history_.size() > static_cast<size_t>(max_history_size_)) {
            drone_path_history_.pop_front(); // Remove oldest point
        }
    } // Release drone path lock

    // Update the drone path visualization
    publishDronePathVisualization();
}

// --- Visualization Functions ---
void PathLoggingNode::publishDronePathVisualization() {
    visualization_msgs::Marker path_marker;
    path_marker.header.stamp = ros::Time::now();
    path_marker.header.frame_id = world_frame_id_;
    path_marker.ns = "drone_path_colored";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;

    // Pose: Identity (points are in world frame)
    path_marker.pose.orientation.w = 1.0;

    // Scale: Line width
    path_marker.scale.x = 0.05; // Adjust as needed

    // Color: Will be set per point

    { // Lock scope for reading drone path history
        std::lock_guard<std::mutex> lock(drone_path_mutex_);

        if (drone_path_history_.size() < 2) {
            // Cannot draw a line with less than 2 points, maybe clear previous?
             path_marker.action = visualization_msgs::Marker::DELETE; // Clear previous marker
             drone_path_viz_pub_.publish(path_marker);
            return;
        }

        path_marker.points.reserve(drone_path_history_.size());
        path_marker.colors.reserve(drone_path_history_.size());

        for (const auto& drone_pt : drone_path_history_) {
            path_marker.points.push_back(drone_pt.position);
            path_marker.colors.push_back(velocityToColor(drone_pt.speed));
        }
    } // Release drone path lock

    drone_path_viz_pub_.publish(path_marker);
}

// --- Color Mapping Helper ---
std_msgs::ColorRGBA PathLoggingNode::velocityToColor(double speed) {
    std_msgs::ColorRGBA color;
    color.a = 1.0; // Fully opaque

    // Normalize speed to [0, 1] based on min/max viz parameters
    double normalized_speed = 0.0;
    if (max_viz_vel_ > min_viz_vel_) { // Avoid division by zero
        normalized_speed = (speed - min_viz_vel_) / (max_viz_vel_ - min_viz_vel_);
    }
    // Clamp to [0, 1]
    normalized_speed = fmax(0.0, fmin(1.0, normalized_speed));

    // Simple Blue -> Green -> Red colormap
    color.r = fmin(1.0, normalized_speed * 2.0);
    color.b = fmax(0.0, 1.0 - normalized_speed * 2.0);
    color.g = fmax(0.0, 1.0 - std::abs(normalized_speed - 0.5) * 2.0);

    return color;
}


// --- Main Function ---
int main(int argc, char **argv) {
    ros::init(argc, argv, "path_logging_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Private node handle for parameters

    PathLoggingNode node(nh, pnh);

    ros::spin(); // Process callbacks

    return 0;
}