#include "map.h"
#include <Eigen/Geometry> // Required for vector operations

using namespace std;
using namespace Eigen;

namespace ft {

    Map::Map() : total_trajectory_time_(0.0) {} // Initialize total time

    // Helper function for linear interpolation between two trajectory points
    swarm_msgs::DiscreteTrajectoryPoint Map::interpolate(
        const swarm_msgs::DiscreteTrajectoryPoint& p1,
        const swarm_msgs::DiscreteTrajectoryPoint& p2,
        double t)
    {
        swarm_msgs::DiscreteTrajectoryPoint result;
        result.time_from_start = t;

        if (std::fabs(p2.time_from_start - p1.time_from_start) < EPS) {
            // Avoid division by zero if times are identical
            result.position = p1.position;
            result.velocity = p1.velocity;
            result.acceleration = p1.acceleration;
            return result;
        }

        double ratio = (t - p1.time_from_start) / (p2.time_from_start - p1.time_from_start);
        ratio = std::max(0.0, std::min(1.0, ratio)); // Clamp ratio to [0, 1]

        // Interpolate Position
        result.position.x = p1.position.x + ratio * (p2.position.x - p1.position.x);
        result.position.y = p1.position.y + ratio * (p2.position.y - p1.position.y);
        result.position.z = p1.position.z + ratio * (p2.position.z - p1.position.z);

        // Interpolate Velocity
        result.velocity.x = p1.velocity.x + ratio * (p2.velocity.x - p1.velocity.x);
        result.velocity.y = p1.velocity.y + ratio * (p2.velocity.y - p1.velocity.y);
        result.velocity.z = p1.velocity.z + ratio * (p2.velocity.z - p1.velocity.z);

        // Interpolate Acceleration
        result.acceleration.x = p1.acceleration.x + ratio * (p2.acceleration.x - p1.acceleration.x);
        result.acceleration.y = p1.acceleration.y + ratio * (p2.acceleration.y - p1.acceleration.y);
        result.acceleration.z = p1.acceleration.z + ratio * (p2.acceleration.z - p1.acceleration.z);

        return result;
    }


    // Function to store the received discrete trajectory
    void Map::setTrajectory(const swarm_msgs::DiscreteTrajectory::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_); // Lock for writing

        if (msg->points.empty()) {
            ROS_WARN("Received empty discrete trajectory.");
            trajectory_points_.clear();
            total_trajectory_time_ = 0.0;
            return;
        }

        trajectory_points_ = msg->points; // Copy the points
        total_trajectory_time_ = trajectory_points_.back().time_from_start; // Get total duration

        ROS_INFO("Map: Set new trajectory with %zu points, duration: %.3f s",
                 trajectory_points_.size(), total_trajectory_time_);
    }

    // Find the time on the trajectory corresponding to the spatially closest point
    double Map::findNearestTime(const Eigen::Vector3d& current_position) {
        std::lock_guard<std::mutex> lock(mtx_); // Lock for reading

        if (trajectory_points_.empty()) {
            ROS_WARN_THROTTLE(1.0, "findNearestTime called on empty trajectory.");
            return 0.0; // Return start time if no trajectory
        }

        double min_dist_sq = std::numeric_limits<double>::max();
        double nearest_time = 0.0;
        size_t nearest_idx = 0;

        // --- Method 1: Find closest *vertex* (simpler, less accurate for sparse points) ---
        for (size_t i = 0; i < trajectory_points_.size(); ++i) {
            double dx = current_position.x() - trajectory_points_[i].position.x;
            double dy = current_position.y() - trajectory_points_[i].position.y;
            double dz = current_position.z() - trajectory_points_[i].position.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;

            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                nearest_time = trajectory_points_[i].time_from_start;
                nearest_idx = i; // Keep track of index for potential refinement
            }
        }

        // --- Method 2: Find closest point *on segments* (more complex, more accurate) ---
        // This involves projecting current_position onto each line segment.
        // For simplicity, we'll stick with Method 1 for now, but Method 2 is better
        // if the drone can be significantly far from the trajectory vertices.
        // Let's add a basic refinement near the closest vertex found by Method 1.

        if (trajectory_points_.size() > 1) {
             // Check segment before and after the closest vertex
             size_t start_check_idx = (nearest_idx > 0) ? nearest_idx - 1 : 0;
             size_t end_check_idx = std::min(nearest_idx + 1, trajectory_points_.size() - 1);

             for(size_t i = start_check_idx; i < end_check_idx; ++i) {
                  const auto& p1 = trajectory_points_[i];
                  const auto& p2 = trajectory_points_[i+1];
                  Eigen::Vector3d p1_eig(p1.position.x, p1.position.y, p1.position.z);
                  Eigen::Vector3d p2_eig(p2.position.x, p2.position.y, p2.position.z);
                  Eigen::Vector3d segment_vec = p2_eig - p1_eig;
                  double segment_len_sq = segment_vec.squaredNorm();

                  if (segment_len_sq < EPS * EPS) continue; // Skip zero-length segments

                  // Project current_position onto the line defined by the segment
                  double t_param = (current_position - p1_eig).dot(segment_vec) / segment_len_sq;

                  Eigen::Vector3d closest_point_on_line;
                  double time_on_segment;

                  if (t_param < 0.0) { // Closest point is p1
                       closest_point_on_line = p1_eig;
                       time_on_segment = p1.time_from_start;
                  } else if (t_param > 1.0) { // Closest point is p2
                       closest_point_on_line = p2_eig;
                       time_on_segment = p2.time_from_start;
                  } else { // Closest point is within the segment
                       closest_point_on_line = p1_eig + t_param * segment_vec;
                       // Interpolate time based on t_param
                       time_on_segment = p1.time_from_start + t_param * (p2.time_from_start - p1.time_from_start);
                  }

                  double dist_sq = (current_position - closest_point_on_line).squaredNorm();
                  if (dist_sq < min_dist_sq) {
                       min_dist_sq = dist_sq;
                       nearest_time = time_on_segment;
                  }
             }
        }


        // Clamp the result to the valid time range
        nearest_time = std::max(0.0, std::min(nearest_time, total_trajectory_time_));

        return nearest_time;
    }


    // Get state (P, V, A) at a specific time using interpolation
    bool Map::getStateAtTime(double time_from_start,
                             Vector3d& position,
                             Vector3d& velocity,
                             Vector3d& acceleration)
    {
        std::lock_guard<std::mutex> lock(mtx_); // Lock for reading

        if (trajectory_points_.empty()) {
            ROS_WARN_THROTTLE(1.0, "getStateAtTime called on empty trajectory.");
            return false; // Indicate failure
        }

        // Clamp time to be within the trajectory duration
        time_from_start = std::max(0.0, std::min(time_from_start, total_trajectory_time_));

        // Find the segment containing the requested time using binary search
        auto it = std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(), time_from_start,
                                   [](const swarm_msgs::DiscreteTrajectoryPoint& p, double t) {
                                       return p.time_from_start < t;
                                   });

        // Determine indices for interpolation
        size_t idx2 = std::distance(trajectory_points_.begin(), it);
        size_t idx1;

        if (idx2 == 0) {
            // Time is before or exactly at the first point
            idx1 = 0;
            idx2 = (trajectory_points_.size() > 1) ? 1 : 0; // Use first segment if available
        } else if (idx2 == trajectory_points_.size()) {
            // Time is after or exactly at the last point
            idx1 = trajectory_points_.size() - (trajectory_points_.size() > 1 ? 2 : 1); // Second to last
            idx2 = trajectory_points_.size() - 1; // Last point
        } else {
            // Time is between idx1 and idx2
            idx1 = idx2 - 1;
        }

        // Perform interpolation
        const auto& p1 = trajectory_points_[idx1];
        const auto& p2 = trajectory_points_[idx2];
        swarm_msgs::DiscreteTrajectoryPoint interpolated_point = interpolate(p1, p2, time_from_start);

        // Convert to Eigen::Vector3d
        position << interpolated_point.position.x, interpolated_point.position.y, interpolated_point.position.z;
        velocity << interpolated_point.velocity.x, interpolated_point.velocity.y, interpolated_point.velocity.z;
        acceleration << interpolated_point.acceleration.x, interpolated_point.acceleration.y, interpolated_point.acceleration.z;

        return true; // Indicate success
    }

    // Overload for getting only position
     bool Map::getStateAtTime(double time_from_start, Vector3d& position) {
         Vector3d vel, acc; // Dummy variables
         return getStateAtTime(time_from_start, position, vel, acc);
     }

    // Overload for getting position and velocity
     bool Map::getStateAtTime(double time_from_start, Vector3d& position, Vector3d& velocity) {
         Vector3d acc; // Dummy variable
         return getStateAtTime(time_from_start, position, velocity, acc);
     }

    // Calculate Yaw based on interpolated velocity at the given time
    double Map::getYawAtTime(double time_from_start) {
        Vector3d pos, vel, acc;
        if (getStateAtTime(time_from_start, pos, vel, acc)) {
            // Calculate yaw from the interpolated velocity's X and Y components
            if (vel.norm() < EPS) { // Handle zero velocity case
                // Option 1: Return 0 yaw
                // return 0.0;
                // Option 2: Try to get yaw from previous non-zero velocity (more complex)
                // For simplicity, let's return 0 or the yaw of the *next* point if possible
                // Find the next point with non-zero velocity if current is zero
                 size_t start_idx = 0;
                  // Find index corresponding to time_from_start (approximate)
                  auto it = std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(), time_from_start,
                                   [](const swarm_msgs::DiscreteTrajectoryPoint& p, double t) { return p.time_from_start < t; });
                 start_idx = std::distance(trajectory_points_.begin(), it);
                 start_idx = std::min(start_idx, trajectory_points_.size() -1); // Ensure valid index

                 for (size_t i = start_idx; i < trajectory_points_.size(); ++i) {
                     double vx = trajectory_points_[i].velocity.x;
                     double vy = trajectory_points_[i].velocity.y;
                     if (std::sqrt(vx*vx + vy*vy) > EPS) {
                          return std::atan2(vy, vx);
                     }
                 }
                 // If no future non-zero velocity found, return 0
                return 0.0;
            }
            return std::atan2(vel.y(), vel.x());
        } else {
            // Could not get state, return 0 or handle error
            return 0.0;
        }
    }

    // Accessor for total trajectory time
    double Map::getTotalTime() const {
        // No lock needed if total_trajectory_time_ is updated atomically with points_
        // Or add lock if concerned about read during write: std::lock_guard<std::mutex> lock(mtx_);
        return total_trajectory_time_;
    }

    // Accessor for the trajectory points (const reference)
    const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& Map::getTrajectoryPoints() const {
        // No lock needed for const access if writes are protected
        return trajectory_points_;
    }


    // Utility function
    bool Map::isEqualFloat(double a, double b) {
        return fabs(a - b) < EPS;
    }

} //namespace ft