#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <limits>

// Required ROS Message Headers
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h> // For drone pose
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <swarm_msgs/MassPoints.h>
#include <swarm_msgs/TimeOptimalPMMPieces.h>
#include <swarm_msgs/TimeOptimalPMMParam.h>
#include <swarm_msgs/DiscreteTrajectory.h>
#include <swarm_msgs/DiscreteTrajectoryPoint.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Include PMM Header
#include "time_optimal_PMM.h"

// --- Helper Functions ---

// a + b
inline geometry_msgs::Vector3 add_Vector3(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    geometry_msgs::Vector3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

// a - b
inline geometry_msgs::Vector3 subtract_Vector3(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    geometry_msgs::Vector3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

// norm(a)
inline double norm_Vector3(const geometry_msgs::Vector3& a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

// normalize(a)
inline geometry_msgs::Vector3 normalize_Vector3(const geometry_msgs::Vector3& a) {
    double norm = norm_Vector3(a);
    geometry_msgs::Vector3 unit_vector;
    // Avoid division by zero
    if (norm > 1e-9) { // Use a small epsilon for comparison
        unit_vector.x = a.x / norm;
        unit_vector.y = a.y / norm;
        unit_vector.z = a.z / norm;
    } else {
        // Return zero vector if norm is too small
        unit_vector.x = 0.0;
        unit_vector.y = 0.0;
        unit_vector.z = 0.0;
    }
    return unit_vector;
}

// scalar * v
inline geometry_msgs::Vector3 scalar_multiply_Vector3(const geometry_msgs::Vector3& v, double scalar) {
    geometry_msgs::Vector3 result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    result.z = v.z * scalar;
    return result;
}

// Floating point comparison helper
inline bool isEqualFloat(double a, double b, double epsilon = 1e-6) {
    return std::fabs(a - b) < epsilon;
}


// --- Path Planning Node Class ---

class PathPlanningNode {
public:
    // Constructor
    PathPlanningNode() : nh_("~") {
        // Load parameters
        loadConstraints();
        nh_.param<double>("freeze_duration", freeze_duration_, 1.0);
        nh_.param<int>("initial_sampling_points", initial_sampling_points_, 100);
        nh_.param<int>("replanned_sampling_points", replanned_sampling_points_, 50);
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world"); // Default frame

        if (initial_sampling_points_ < 2) initial_sampling_points_ = 2;
        if (replanned_sampling_points_ < 2) replanned_sampling_points_ = 2;

        ROS_INFO("PathPlanningNode initialized:");
        ROS_INFO("  freeze_duration: %.2f s", freeze_duration_);
        ROS_INFO("  initial_sampling_points: %d", initial_sampling_points_);
        ROS_INFO("  replanned_sampling_points: %d", replanned_sampling_points_);
        ROS_INFO("  world_frame_id: %s", world_frame_id_.c_str());

        // Publishers
        // Publishes the (potentially stitched) trajectory for the controller
        discrete_trajectory_pub_ = nh_.advertise<swarm_msgs::DiscreteTrajectory>("/stitched_trajectory", 5); // Topic name changed
        // Optional: Publisher for visualization
        stitched_path_viz_pub_ = nh_.advertise<nav_msgs::Path>("visualization/stitched_path", 1);

        // Subscribers
        path_points_sub_ = nh_.subscribe("/path_points", 5, &PathPlanningNode::pathPointsCallback, this);
        // Subscribe to drone pose (adjust topic name as needed)
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &PathPlanningNode::poseCallback, this, ros::TransportHints().tcpNoDelay());


        ROS_INFO("PathPlanningNode ready.");
    }

    // Callbacks
    void pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    // ROS Communication Handles
    ros::NodeHandle nh_;
    ros::Publisher discrete_trajectory_pub_; // Publishes the final stitched trajectory
    ros::Publisher stitched_path_viz_pub_; // Visualization publisher
    ros::Subscriber path_points_sub_;
    ros::Subscriber pose_sub_; // Subscriber for drone pose

    // Constraint Parameters
    std::vector<double> v_max_, v_min_, u_max_, u_min_;

    // Stitching Parameters
    double freeze_duration_;
    int initial_sampling_points_;   // Sampling for the very first trajectory
    int replanned_sampling_points_; // Sampling for the replanned PMM segment
    std::string world_frame_id_;

    // State Variables for Stitching
    std::vector<swarm_msgs::DiscreteTrajectoryPoint> current_published_trajectory_;
    geometry_msgs::PoseStamped current_drone_pose_;
    bool has_published_trajectory_ = false;
    bool has_received_pose_ = false;
    std::mutex pose_mutex_;       // Protects current_drone_pose_ access
    std::mutex trajectory_mutex_; // Protects current_published_trajectory_ access

    // Intermediate variables (used during calculations)
    geometry_msgs::Vector3 current_velocity_; // From input msg
    double current_V_;                      // From input msg
    geometry_msgs::Vector3 p1_, p2_, p3_, v1_, v2_, v_dir_, v_; // For smoothing

    // Sampling Constants
    const double sampling_epsilon_ = 1e-9;

    // --- Helper Functions ---
    void loadConstraints();

    // PMM State Calculation
    struct PMMState { double pos = 0.0; double vel = 0.0; double acc = 0.0; };
    PMMState calculatePMMState(double p0, double v0, double pT, double vT, double umax, double umin,
                               double t1, double t2, double T, int case_idx, double t);

    // Find nearest time on a trajectory (needed for stitching)
    double findNearestTimeOnTrajectory(
        const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& trajectory,
        const Eigen::Vector3d& current_position);

    // Interpolate state on a trajectory (needed for stitching)
    bool interpolateState(
        const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& trajectory,
        double time_from_start,
        swarm_msgs::DiscreteTrajectoryPoint& interpolated_point);

    // Sample a *single* PMM segment (needed for stitching replanned part)
    bool samplePMMTrajectorySegment(
        const geometry_msgs::Vector3& p0_msg, const geometry_msgs::Vector3& v0_msg,
        const geometry_msgs::Vector3& pT_msg, const geometry_msgs::Vector3& vT_msg,
        const geometry_msgs::Vector3& t1, const geometry_msgs::Vector3& t2, double T,
        const geometry_msgs::Vector3& case_idx,
        const geometry_msgs::Vector3& umax, const geometry_msgs::Vector3& umin,
        double start_time_offset,
        int num_samples,
        std::vector<swarm_msgs::DiscreteTrajectoryPoint>& sampled_points);

    // Publish visualization helper
    void publishVisualization(const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& trajectory_to_visualize);

};

#endif // PATH_PLANNING_H