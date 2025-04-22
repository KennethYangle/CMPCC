#ifndef PROJECT_MAP_H
#define PROJECT_MAP_H

#include <ros/package.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <vector> // Include vector
#include <limits> // Include limits for numeric_limits
#include <algorithm> // Include for std::lower_bound, std::min_element, std::distance

// #include <yaml-cpp/yaml.h> // Not directly used in the modified parts
// #include "corridor.h"       // Assumed not needed for this functionality
// #include "bezier_base.h"    // Assumed not needed for this functionality
// #include <quadrotor_msgs/PiecewiseBezier.h> // Not needed
// #include <swarm_msgs/TimeOptimalPMMPieces.h> // Replaced
// #include <swarm_msgs/TimeOptimalPMMParam.h>   // Replaced
#include <swarm_msgs/DiscreteTrajectory.h>     // New dependency
#include <swarm_msgs/DiscreteTrajectoryPoint.h>// New dependency

#include <mutex>
#include <condition_variable>
// #include <matplotlibcpp.h> // Removed dependency if not essential
#define EPS 1e-4
// namespace plt = matplotlibcpp; // Removed dependency

namespace ft {

    class Map {
    private:
        // --- Removed old members related to Bezier/PMM params ---
        // int num_order, num_segment, traj_order, K_max;
        // double s_step, global_traj_time;
        // std::vector<int> K_data_my;
        // std::vector<double> range, K_data;
        // Eigen::MatrixXd coef, time, time_acc, a_data, b_data, s_data;
        // std::vector<Eigen::MatrixXd> control_points;
        // Bernstein bezier_basis = Bernstein(3.0);

        // --- New members to store discrete trajectory ---
        std::vector<swarm_msgs::DiscreteTrajectoryPoint> trajectory_points_;
        double total_trajectory_time_ = 0.0; // Renamed from thetaMax for clarity

        // --- Synchronization ---
        std::mutex mtx_; // Use trailing underscore convention
        // std::condition_variable cv_; // Might not be needed if reads/writes are separated
        // bool can_modify = true; // Logic likely changes or becomes simpler

        // --- Helper: Linear Interpolation ---
        swarm_msgs::DiscreteTrajectoryPoint interpolate(
            const swarm_msgs::DiscreteTrajectoryPoint& p1,
            const swarm_msgs::DiscreteTrajectoryPoint& p2,
            double t);


    public:
        // Constructor
        Map();

        // --- New function to set trajectory from discrete points ---
        void setTrajectory(const swarm_msgs::DiscreteTrajectory::ConstPtr& msg);

        // --- Modified query functions (using time instead of theta) ---
        // Find the time on the trajectory corresponding to the spatially closest point
        double findNearestTime(const Eigen::Vector3d& current_position);

        // Get state (PVA) at a specific time using interpolation
        bool getStateAtTime(double time_from_start,
                            Eigen::Vector3d& position,
                            Eigen::Vector3d& velocity,
                            Eigen::Vector3d& acceleration);

        // Get only position at a specific time
        bool getStateAtTime(double time_from_start,
                            Eigen::Vector3d& position);

        // Get position and velocity at a specific time
        bool getStateAtTime(double time_from_start,
                            Eigen::Vector3d& position,
                            Eigen::Vector3d& velocity);

        // --- Yaw calculation based on velocity at time ---
        double getYawAtTime(double time_from_start);

        // --- Accessor for total time ---
        double getTotalTime() const;

        // --- Get the stored trajectory points (e.g., for visualization) ---
        const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& getTrajectoryPoints() const;

        // Utility function (can be kept if needed elsewhere, or made static/global)
        bool isEqualFloat(double a, double b);
    };
}

#endif //PROJECT_MAP_H