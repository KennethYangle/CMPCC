#ifndef PATH_LOGGING_H
#define PATH_LOGGING_H

#include <ros/ros.h>
#include <vector>
#include <deque>
#include <mutex>
#include <limits>
#include <cmath>

// ROS Messages
// #include <swarm_msgs/DiscreteTrajectory.h> // No longer needed
// #include <swarm_msgs/DiscreteTrajectoryPoint.h> // No longer needed
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
// #include <nav_msgs/Path.h> // No longer needed for ref path
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

class PathLoggingNode {
private:
    // Struct to store drone path history points
    struct DronePathPoint {
        geometry_msgs::Point position;
        double speed; // Velocity magnitude
        ros::Time timestamp;
    };

    // Callbacks
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    // Visualization Update
    void publishDronePathVisualization();

    // Helper for color mapping
    std_msgs::ColorRGBA velocityToColor(double speed);

    // ROS Handles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; // Private Node Handle for parameters

    // Publishers & Subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_;
    ros::Publisher drone_path_viz_pub_;

    // State Variables
    std::deque<DronePathPoint> drone_path_history_; // Use deque for efficient removal
    geometry_msgs::Twist latest_velocity_; // Store latest velocity Twist

    // Flags & Config
    bool has_vel_ = false; // Only need velocity flag, pose callback triggers update
    std::string world_frame_id_ = "world"; // Default frame
    int max_history_size_ = 500; // Max number of drone points to store/display
    double min_viz_vel_ = 0.0;   // Minimum velocity for color mapping
    double max_viz_vel_ = 5.0;   // Maximum velocity for color mapping

    // Synchronization
    std::mutex drone_path_mutex_;
    std::mutex velocity_mutex_; // Mutex for latest_velocity_

public:
    PathLoggingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~PathLoggingNode() = default;
};

#endif // PATH_LOGGING_H