#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <cmath>     // For sqrt, fabs
#include <algorithm> // For std::max, std::min

// Required ROS Message Headers
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <swarm_msgs/MassPoints.h>
#include <swarm_msgs/TimeOptimalPMMPieces.h>
#include <swarm_msgs/TimeOptimalPMMParam.h>
#include <swarm_msgs/DiscreteTrajectory.h>
#include <swarm_msgs/DiscreteTrajectoryPoint.h>

#include "time_optimal_PMM.h"

// --- Helper Functions (moved inline definitions here for clarity) ---

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
    PathPlanningNode() : nh_("~") { // Use private node handle "~"
        // Initialize Publishers
        // refer_path_cps_pub_ = nh_.advertise<swarm_msgs::TimeOptimalPMMPieces>("refer_path_params", 1); // Keep if needed for debug
        discrete_trajectory_pub_ = nh_.advertise<swarm_msgs::DiscreteTrajectory>("/discrete_trajectory", 1); // New publisher

        // Initialize Subscriber
        // Use global topic "/path_points" unless specifically namespaced elsewhere
        path_points_sub_ = nh_.subscribe("/path_points", 1, &PathPlanningNode::pathPointsCallback, this);

        // Load constraints from YAML file during construction
        loadConstraints();

        ROS_INFO("PathPlanningNode initialized and ready.");
    }

    // Callback function for incoming path points
    void pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg);

private:
    // ROS Communication Handles
    ros::NodeHandle nh_; // Use private NodeHandle for parameters/namespacing
    // ros::Publisher refer_path_cps_pub_; // Old publisher (optional)
    ros::Publisher discrete_trajectory_pub_; // New publisher for discrete points
    ros::Subscriber path_points_sub_;

    // Constraint Parameters (loaded from YAML)
    std::vector<double> v_max_, v_min_, u_max_, u_min_; // Use trailing underscore convention

    // Intermediate variables (consider making local in callback if not needed across calls)
    geometry_msgs::Vector3 current_velocity_;
    double current_V_;
    geometry_msgs::Vector3 p1_, p2_, p3_, v1_, v2_, v_dir_, v_; // Temporary variables for smoothing

    // Parameters for trajectory sampling (NEW)
    // Consider making these ROS parameters for runtime configuration
    const int num_points_per_segment_ = 100; // Number of points to sample per trajectory segment
    const double sampling_epsilon_ = 1e-9; // Small value for time comparisons during sampling

    // Helper function to load constraints
    void loadConstraints();

    // Helper struct for calculating PMM state during sampling (NEW)
    struct PMMState {
        double pos = 0.0;
        double vel = 0.0;
        double acc = 0.0;
    };

    // Helper function for calculating PMM state at a specific time t within a segment (NEW)
    // Calculates pos, vel, acc for a single axis based on PMM parameters
    PMMState calculatePMMState(double p0, double v0, double pT, double vT, double umax, double umin,
                               double t1, double t2, double T, int case_idx, double t);
};

#endif // PATH_PLANNING_H