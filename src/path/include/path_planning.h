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
#include <exception>
#include <map> // For Dijkstra

// Required ROS Message Headers
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h> // For drone pose
#include <geometry_msgs/TwistStamped.h>
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
    // --- Graph Search Helper Structs ---
    // Represents a node in the graph search (Layer + Velocity Index)
    struct GraphNode {
        int layer; // Corresponds to waypoint index in the replan sequence (0 is freeze_point)
        int vel_idx; // Index of the candidate velocity at this waypoint

        // Comparison for priority queue or map keys
        bool operator<(const GraphNode& other) const {
            if (layer != other.layer) return layer < other.layer;
            return vel_idx < other.vel_idx;
        }
    };

    // Stores info needed for Dijkstra/forward pass
    struct NodeInfo {
        double min_time = std::numeric_limits<double>::infinity(); // Min time to reach this node
        int prev_layer_vel_idx = -1; // Velocity index in the previous layer that led here
        // Store PMM results for the optimal incoming edge (optional, can recalculate later)
        // swarm_msgs::TimeOptimalPMMParam optimal_incoming_param;
    };
    // --- End Helper Structs ---


    // Constructor
    PathPlanningNode() : nh_("~") {
        // Load parameters
        loadConstraints();
        nh_.param<double>("freeze_duration", freeze_duration_, 2.0);
        nh_.param<int>("initial_sampling_points", initial_sampling_points_, 100);
        nh_.param<int>("replanned_sampling_points", replanned_sampling_points_, 50);
        nh_.param<std::string>("world_frame_id", world_frame_id_, "world");

        // Graph search parameters
        nh_.param<int>("num_velocity_directions", num_v_dir_, 21); // Number of direction candidates
        nh_.param<int>("num_velocity_magnitudes", num_v_mag_, 9); // Number of speed candidates
        nh_.param<double>("smoothing_base_velocity", base_velocity_, 4.0); // Base speed for candidates
        nh_.param<double>("max_angle_deviation_deg", max_angle_dev_deg_, 15.0); // Cone angle for direction sampling
        nh_.param<double>("speed_deviation_ratio", speed_dev_ratio_, 0.5); // +/- ratio for speed sampling

        if (initial_sampling_points_ < 2) initial_sampling_points_ = 2;
        if (replanned_sampling_points_ < 2) replanned_sampling_points_ = 2;
        if (num_v_dir_ < 1) num_v_dir_ = 1;
        if (num_v_mag_ < 1) num_v_mag_ = 1;
        if (max_angle_dev_deg_ < 0.0) max_angle_dev_deg_ = 0.0;
        if (speed_dev_ratio_ < 0.0) speed_dev_ratio_ = 0.0;

        ROS_INFO("PathPlanningNode initialized:");
        ROS_INFO("  freeze_duration: %.2f s", freeze_duration_);
        ROS_INFO("  initial_sampling_points: %d", initial_sampling_points_);
        ROS_INFO("  replanned_sampling_points: %d", replanned_sampling_points_);
        ROS_INFO("  world_frame_id: %s", world_frame_id_.c_str());
        ROS_INFO("  Graph Search Params: dirs=%d, mags=%d, base_vel=%.1f, angle_dev=%.1f deg, speed_dev=%.2f",
                 num_v_dir_, num_v_mag_, base_velocity_, max_angle_dev_deg_, speed_dev_ratio_);


        // Publishers
        discrete_trajectory_pub_ = nh_.advertise<swarm_msgs::DiscreteTrajectory>("/stitched_trajectory", 5);
        stitched_path_viz_pub_ = nh_.advertise<nav_msgs::Path>("visualization/stitched_path", 1);

        // Subscribers
        path_points_sub_ = nh_.subscribe("/path_points", 5, &PathPlanningNode::pathPointsCallback, this);
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &PathPlanningNode::poseCallback, this, ros::TransportHints().tcpNoDelay());
        vel_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &PathPlanningNode::velocityCallback, this, ros::TransportHints().tcpNoDelay());


        ROS_INFO("PathPlanningNode ready.");
    }

    // Callbacks
    void pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

private:
    // ROS Communication Handles
    ros::NodeHandle nh_;
    ros::Publisher discrete_trajectory_pub_;
    ros::Publisher stitched_path_viz_pub_;
    ros::Subscriber path_points_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber vel_sub_; // Velocity subscriber

    // Constraint Parameters
    std::vector<double> v_max_, v_min_, u_max_, u_min_;

    // Stitching & Graph Search Parameters
    double freeze_duration_;
    int initial_sampling_points_;   // Sampling for the very first trajectory
    int replanned_sampling_points_; // Sampling for the replanned PMM segment
    std::string world_frame_id_;
    int num_v_dir_;
    int num_v_mag_;
    double base_velocity_;
    double max_angle_dev_deg_;
    double speed_dev_ratio_;

    // State Variables for Stitching
    std::vector<swarm_msgs::DiscreteTrajectoryPoint> current_published_trajectory_;
    geometry_msgs::PoseStamped current_drone_pose_;
    bool has_published_trajectory_ = false;
    bool has_received_pose_ = false;
    std::mutex pose_mutex_;       // Protects current_drone_pose_ access
    std::mutex trajectory_mutex_; // Protects current_published_trajectory_ access

    // Intermediate variables
    geometry_msgs::Vector3 current_drone_velocity_; // Store current drone velocity
    double current_V_; // Speed used for smoothing (can be drone speed or fixed base)
    geometry_msgs::Vector3 p1_, p2_, p3_, v1_, v2_, v_dir_, v_;

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

    // ** NEW Graph Search Helpers **
    /**
     * @brief Generates candidate velocities for a waypoint.
     * @param prev_pos Position of the previous waypoint (or freeze point).
     * @param current_pos Position of the current waypoint where candidates are generated.
     * @param next_pos Position of the next waypoint.
     * @param base_vel Base velocity magnitude.
     * @param num_dirs Number of direction samples.
     * @param num_mags Number of magnitude samples.
     * @param angle_dev_rad Max angle deviation for direction sampling (radians).
     * @param speed_dev_ratio +/- ratio for speed sampling.
     * @return Vector of candidate velocity vectors.
     */
    std::vector<geometry_msgs::Vector3> generateCandidateVelocities(
        const geometry_msgs::Vector3& prev_pos,
        const geometry_msgs::Vector3& current_pos,
        const geometry_msgs::Vector3& next_pos,
        double base_vel,
        int num_dirs, int num_mags,
        double angle_dev_rad, double speed_dev_ratio);

    /**
     * @brief Performs the forward pass (Dijkstra-like) to find minimum times.
     * @param waypoints Sequence of waypoints for replanning (starts with freeze point pos).
     * @param start_vel Velocity at the freeze point.
     * @param candidate_velocities candidate_velocities[i] holds candidates for waypoints[i+1].
     * @param final_target_vel Fixed velocity at the final waypoint.
     * @param[out] node_infos Map storing NodeInfo (min_time, predecessor) for each graph node.
     * @param[out] final_min_time Minimum time to reach the final waypoint.
     * @param[out] final_pred_vel_idx Index of the best velocity at the second-to-last waypoint.
     * @return True if a path to the end was found, false otherwise.
     */
     bool findOptimalVelocityPath(
         const std::vector<geometry_msgs::Vector3>& waypoints, // Waypoints[0] = freeze_pos
         const geometry_msgs::Vector3& start_vel, // Vel at freeze_point
         const std::vector<std::vector<geometry_msgs::Vector3>>& candidate_velocities, // candidates[i] is for waypoint i+1
         const geometry_msgs::Vector3& final_target_vel, // Vel at last waypoint
         std::map<GraphNode, NodeInfo>& node_infos,
         double& final_min_time,
         int& final_pred_vel_idx);

     /**
      * @brief Helper to calculate PMM time between two states. Returns infinity on failure.
      */
     double calculatePMMTime(const geometry_msgs::Vector3& p0, const geometry_msgs::Vector3& v0,
                             const geometry_msgs::Vector3& pT, const geometry_msgs::Vector3& vT);


};

#endif // PATH_PLANNING_H