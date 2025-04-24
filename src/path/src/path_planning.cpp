#include "path_planning.h"

// Helper function implementation for calculating PMM state at time t
PathPlanningNode::PMMState PathPlanningNode::calculatePMMState(
    double p0, double v0, double pT, double vT, double umax, double umin,
    double t1, double t2, double T, int case_idx, double t)
{
    PMMState state;
    double u1 = 0.0, u2 = 0.0, u3 = 0.0; // Acceleration in phase 1, 2, 3

    // Clamp t to be within [0, T]
    t = std::max(0.0, std::min(t, T));

    if (T < sampling_epsilon_) { // Handle near-zero duration segments
        state.acc = 0.0; // Assume zero acceleration if duration is negligible
        state.vel = v0;  // State remains at the start state
        state.pos = p0; 
        return state; 
    }
    

    if (case_idx == 2) { // Constant velocity (straight line)
        // For case 2, the velocity should ideally be constant between v0 and vT.
        // If v0 and vT differ in case 2, the PMM model assumption might be violated.
        // Assuming v0 is the intended constant velocity for this segment.
        state.acc = 0.0;
        state.vel = (pT - p0) / T;
        state.pos = p0 + state.vel * t;
    } else {
        if (case_idx == 0) { // ↗(→)↘
            u1 = umax;
            u2 = 0.0;
            u3 = -umin; // umin is the magnitude of deceleration limit
        } else { // case_idx == 1: ↘(→)↗
            u1 = -umin; // umin is the magnitude of deceleration limit
            u2 = 0.0;
            u3 = umax;
        }

        // Adjust phase times slightly for safer comparison due to potential float inaccuracies
        double t1_comp = t1 - sampling_epsilon_;
        double t2_comp = t2 + sampling_epsilon_; // Use +epsilon for t2 comparison start


        if (t >= -sampling_epsilon_ && t < t1) { // Phase 1: [0, t1)
            state.acc = u1;
            state.vel = v0 + u1 * t;
            state.pos = p0 + v0 * t + 0.5 * u1 * t * t;
        } else if (t >= t1 && t < t2) { // Phase 2: [t1, t2)
            double v_at_t1 = v0 + u1 * t1;
            double p_at_t1 = p0 + v0 * t1 + 0.5 * u1 * t1 * t1;
            state.acc = u2; // Should be 0
            double dt_phase2 = t - t1;
            state.vel = v_at_t1 + u2 * dt_phase2;
            state.pos = p_at_t1 + v_at_t1 * dt_phase2 + 0.5 * u2 * dt_phase2 * dt_phase2;
        } else { // Phase 3: [t2, T]
            double v_at_t1 = v0 + u1 * t1;
            double p_at_t1 = p0 + v0 * t1 + 0.5 * u1 * t1 * t1;
            double v_at_t2 = v_at_t1 + u2 * (t2 - t1); // v at end of phase 2
            double p_at_t2 = p_at_t1 + v_at_t1 * (t2 - t1) + 0.5 * u2 * (t2 - t1) * (t2 - t1); // p at end of phase 2
            state.acc = u3;
            double dt_phase3 = t - t2;
            // Clamp dt_phase3 just in case t slightly exceeds T due to float issues
            dt_phase3 = std::max(0.0, dt_phase3);
            state.vel = v_at_t2 + u3 * dt_phase3;
            state.pos = p_at_t2 + v_at_t2 * dt_phase3 + 0.5 * u3 * dt_phase3 * dt_phase3;
        }
    }
    return state;
}

// --- Pose Callback ---
void PathPlanningNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_drone_pose_ = *msg;
    if (!has_received_pose_) {
        // Optionally update world_frame_id_ from first pose if not set by param
         if (world_frame_id_.empty()) { // Check default value
             world_frame_id_ = msg->header.frame_id;
             ROS_INFO("PathPlanningNode using frame_id '%s' from pose message.", world_frame_id_.c_str());
         }
        has_received_pose_ = true;
    }
}

void PathPlanningNode::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_V_ = norm_Vector3(msg->twist.linear);
}

// --- Helper: Generate Candidate Velocities ---
std::vector<geometry_msgs::Vector3> PathPlanningNode::generateCandidateVelocities(
    const geometry_msgs::Vector3& prev_pos, const geometry_msgs::Vector3& current_pos,
    const geometry_msgs::Vector3& next_pos, double base_vel,
    int num_dirs, int num_mags, double angle_dev_rad, double speed_dev_ratio)
{
    std::vector<geometry_msgs::Vector3> candidates;
    candidates.reserve(num_dirs * num_mags);

    // --- Calculate Central Direction (Smoothed Forward Direction) ---
    geometry_msgs::Vector3 incoming_dir, outgoing_dir, central_dir;
    double norm_in = 0.0, norm_out = 0.0;

    incoming_dir = subtract_Vector3(current_pos, prev_pos);
    norm_in = norm_Vector3(incoming_dir);
    if (norm_in > sampling_epsilon_) {
        incoming_dir = normalize_Vector3(incoming_dir);
    } else {
        // If previous and current are same, use outgoing direction
        norm_in = 0.0;
    }

    outgoing_dir = subtract_Vector3(next_pos, current_pos);
    norm_out = norm_Vector3(outgoing_dir);
     if (norm_out > sampling_epsilon_) {
        outgoing_dir = normalize_Vector3(outgoing_dir);
    } else {
        // If current and next are same, use incoming direction (if valid)
        norm_out = 0.0;
    }

    // Determine central direction based on available vectors
    if (norm_in > sampling_epsilon_ && norm_out > sampling_epsilon_) {
        // Bisector if both exist
        geometry_msgs::Vector3 sum_dirs = add_Vector3(incoming_dir, outgoing_dir);
        if (norm_Vector3(sum_dirs) > sampling_epsilon_) {
            central_dir = normalize_Vector3(sum_dirs);
        } else { // Vectors are opposite -> use outgoing as default? Or zero velocity? Let's use outgoing.
            central_dir = outgoing_dir;
        }
    } else if (norm_out > sampling_epsilon_) {
        central_dir = outgoing_dir; // Only outgoing available
    } else if (norm_in > sampling_epsilon_) {
        central_dir = incoming_dir; // Only incoming available
    } else {
        // No direction info (coincident points) - generate candidates around zero? Or skip?
        // Let's generate a single zero velocity candidate in this edge case.
        ROS_WARN("Cannot determine central direction for velocity generation (coincident points?). Using zero velocity.");
        candidates.push_back(geometry_msgs::Vector3()); // Add zero vector
        return candidates;
    }

    // --- Generate Speed Candidates ---
    std::vector<double> speeds;
    if (num_mags <= 1) {
        speeds.push_back(base_vel);
    } else {
        double min_speed = base_vel * (1.0 - speed_dev_ratio);
        double max_speed = base_vel * (1.0 + speed_dev_ratio);
        min_speed = std::max(0.0, min_speed); // Ensure non-negative speed
        for (int i = 0; i < num_mags; ++i) {
            double speed = min_speed + (max_speed - min_speed) * static_cast<double>(i) / (num_mags - 1);
            speeds.push_back(speed);
        }
    }

    // --- Generate Direction Candidates (Sampling on a Sphere Cap) ---
    std::vector<geometry_msgs::Vector3> directions;
    if (num_dirs <= 1 || angle_dev_rad < sampling_epsilon_) {
        directions.push_back(central_dir);
    } else {
        directions.push_back(central_dir); // Always include central direction

        // Create an orthonormal basis around central_dir (v1)
        geometry_msgs::Vector3 v1 = central_dir;
        geometry_msgs::Vector3 v2, v3;
        // Find a vector not parallel to v1
        geometry_msgs::Vector3 helper_vec;
        if (std::abs(v1.x) < 0.9) { helper_vec.x = 1; helper_vec.y = 0; helper_vec.z = 0; }
        else { helper_vec.x = 0; helper_vec.y = 1; helper_vec.z = 0; }
        // v3 = normalize(cross(v1, helper_vec))
        v3.x = v1.y * helper_vec.z - v1.z * helper_vec.y;
        v3.y = v1.z * helper_vec.x - v1.x * helper_vec.z;
        v3.z = v1.x * helper_vec.y - v1.y * helper_vec.x;
        v3 = normalize_Vector3(v3);
        // v2 = cross(v3, v1) (already normalized if v1, v3 are orthonormal)
        v2.x = v3.y * v1.z - v3.z * v1.y;
        v2.y = v3.z * v1.x - v3.x * v1.z;
        v2.z = v3.x * v1.y - v3.y * v1.x;

        // Sample directions within the cone using spherical coordinates relative to basis
        int dirs_to_sample = num_dirs -1; // Since central dir is already added
        // Simple uniform sampling in angle for now (more sophisticated needed for uniform sphere cap)
        for (int i = 0; i < dirs_to_sample; ++i) {
             double sample_angle = angle_dev_rad * static_cast<double>(i + 1) / dirs_to_sample; // Spread angles up to max deviation
             double sample_azimuth = 2.0 * M_PI * static_cast<double>(i) / dirs_to_sample; // Spread azimuths around

             // Convert spherical deviation (sample_angle from v1, sample_azimuth around v1) to vector
             // v = cos(angle)*v1 + sin(angle)*cos(azimuth)*v2 + sin(angle)*sin(azimuth)*v3
             double cos_a = std::cos(sample_angle);
             double sin_a = std::sin(sample_angle);
             double cos_az = std::cos(sample_azimuth);
             double sin_az = std::sin(sample_azimuth);

             geometry_msgs::Vector3 dir;
             dir.x = cos_a * v1.x + sin_a * (cos_az * v2.x + sin_az * v3.x);
             dir.y = cos_a * v1.y + sin_a * (cos_az * v2.y + sin_az * v3.y);
             dir.z = cos_a * v1.z + sin_a * (cos_az * v2.z + sin_az * v3.z);
             directions.push_back(dir); // Already normalized (approx)
        }
    }

    // --- Combine Directions and Speeds ---
    for (const auto& dir : directions) {
        for (double speed : speeds) {
            candidates.push_back(scalar_multiply_Vector3(dir, speed));
        }
    }

    return candidates;
}

// --- Helper: Calculate PMM Time (Edge Weight) ---
double PathPlanningNode::calculatePMMTime(const geometry_msgs::Vector3& p0, const geometry_msgs::Vector3& v0,
                                          const geometry_msgs::Vector3& pT, const geometry_msgs::Vector3& vT)
{
     // Use local constraints loaded in the node
    geometry_msgs::Vector3 umax_vec, umin_vec, vmax_vec, vmin_vec;
    umax_vec.x = u_max_[0]; umax_vec.y = u_max_[1]; umax_vec.z = u_max_[2];
    umin_vec.x = u_min_[0]; umin_vec.y = u_min_[1]; umin_vec.z = u_min_[2];
    vmax_vec.x = v_max_[0]; vmax_vec.y = v_max_[1]; vmax_vec.z = v_max_[2];
    vmin_vec.x = v_min_[0]; vmin_vec.y = v_min_[1]; vmin_vec.z = v_min_[2];

     try {
         TimeOptimalPMM3D pmm3d(p0, v0, pT, vT, umax_vec, umin_vec, vmax_vec, vmin_vec);
         auto [t1, t2, T, case_idx] = pmm3d.compute_times();
         if (T >= 0) {
             return T;
         } else {
             ROS_DEBUG_THROTTLE(1.0,"PMM time calculation failed (T=%.3f) between (%f,%f,%f) and (%f,%f,%f)", T, p0.x, p0.y, p0.z, pT.x, pT.y, pT.z);
             return std::numeric_limits<double>::infinity();
         }
     } catch (...) {
         ROS_DEBUG_THROTTLE(1.0, "Exception during PMM time calculation between (%f,%f,%f) and (%f,%f,%f)", p0.x, p0.y, p0.z, pT.x, pT.y, pT.z);
         return std::numeric_limits<double>::infinity();
     }
}


// --- Helper: Find Optimal Velocity Path (Forward Pass) ---
 bool PathPlanningNode::findOptimalVelocityPath(
     const std::vector<geometry_msgs::Vector3>& waypoints, // Waypoints[0] = freeze_pos
     const geometry_msgs::Vector3& start_vel,
     const std::vector<std::vector<geometry_msgs::Vector3>>& candidate_velocities, // candidates[i] is for waypoint i+1
     const geometry_msgs::Vector3& final_target_vel,
     std::map<GraphNode, NodeInfo>& node_infos,
     double& final_min_time,
     int& final_pred_vel_idx)
 {
     node_infos.clear();
     final_min_time = std::numeric_limits<double>::infinity();
     final_pred_vel_idx = -1;

     int num_layers = waypoints.size(); // Number of waypoints = number of layers
     if (num_layers < 2) { // Need at least start (freeze) and end point
         ROS_WARN("findOptimalVelocityPath requires at least 2 waypoints.");
         return false;
     }

     int num_intermediate_layers = num_layers - 2; // Layers with candidate velocities
     if (candidate_velocities.size() != num_intermediate_layers) {
         ROS_ERROR("Mismatch between candidate velocity layers (%zu) and intermediate waypoints (%d)",
                   candidate_velocities.size(), num_intermediate_layers);
         return false;
     }


     // --- Layer 1: From Freeze Point to First Intermediate Waypoint ---
     if (num_layers == 2) { // Direct connection from start to end
          double time = calculatePMMTime(waypoints[0], start_vel, waypoints[1], final_target_vel);
          if (time != std::numeric_limits<double>::infinity()) {
              final_min_time = time;
              final_pred_vel_idx = -2; // Special value indicating direct connection from start
              return true;
          } else {
              return false; // Cannot reach end directly
          }
     }
     // Else (num_layers > 2), process layer 1
     int layer1_idx = 1; // Index of first intermediate waypoint
     const auto& candidates1 = candidate_velocities[0]; // Candidates for waypoint[1]
     for (int k = 0; k < candidates1.size(); ++k) {
         double time = calculatePMMTime(waypoints[0], start_vel, waypoints[layer1_idx], candidates1[k]);
         if (time != std::numeric_limits<double>::infinity()) {
             GraphNode current_node = {layer1_idx, k};
             node_infos[current_node] = {time, -2}; // Predecessor is the start node (use special index -2)
         }
     }

     // --- Layers 2 to N-1: Intermediate to Intermediate ---
     for (int i = 2; i < num_layers - 1; ++i) { // Current layer index (waypoint index)
         int prev_layer_idx = i - 1;
         const auto& candidates_curr = candidate_velocities[i - 1]; // Candidates for waypoint[i]
         const auto& candidates_prev = candidate_velocities[i - 2]; // Candidates for waypoint[i-1]

         for (int k = 0; k < candidates_curr.size(); ++k) { // Iterate over nodes in current layer
             GraphNode current_node = {i, k};
             double min_time_to_curr = std::numeric_limits<double>::infinity();
             int best_prev_idx = -1;

             for (int j = 0; j < candidates_prev.size(); ++j) { // Iterate over nodes in previous layer
                 GraphNode prev_node = {prev_layer_idx, j};
                 if (node_infos.count(prev_node)) { // If previous node is reachable
                     double time_prev = node_infos[prev_node].min_time;
                     double time_edge = calculatePMMTime(waypoints[prev_layer_idx], candidates_prev[j],
                                                         waypoints[i], candidates_curr[k]);

                     if (time_edge != std::numeric_limits<double>::infinity()) {
                         double total_time = time_prev + time_edge;
                         if (total_time < min_time_to_curr) {
                             min_time_to_curr = total_time;
                             best_prev_idx = j;
                         }
                     }
                 }
             } // End loop over previous layer nodes (j)

             if (best_prev_idx != -1) { // If a path was found to this node
                 node_infos[current_node] = {min_time_to_curr, best_prev_idx};
             }
         } // End loop over current layer nodes (k)
     } // End loop over layers (i)


     // --- Final Layer: From Last Intermediate to Final Waypoint ---
     int last_intermediate_layer_idx = num_layers - 2;
     int final_layer_idx = num_layers - 1;
     const auto& candidates_last_interm = candidate_velocities.back(); // Candidates for waypoint[num_layers-2]

     for (int j = 0; j < candidates_last_interm.size(); ++j) {
         GraphNode prev_node = {last_intermediate_layer_idx, j};
         if (node_infos.count(prev_node)) {
             double time_prev = node_infos[prev_node].min_time;
             double time_edge = calculatePMMTime(waypoints[last_intermediate_layer_idx], candidates_last_interm[j],
                                                 waypoints[final_layer_idx], final_target_vel);

             if (time_edge != std::numeric_limits<double>::infinity()) {
                 double total_time = time_prev + time_edge;
                 if (total_time < final_min_time) {
                     final_min_time = total_time;
                     final_pred_vel_idx = j; // Index of the best velocity at the last intermediate waypoint
                 }
             }
         }
     }

     return final_pred_vel_idx != -1; // Success if we found a path to the end
 }


// --- Main Path Points Callback ---
void PathPlanningNode::pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    if (msg->points.size() < 2) {
        ROS_WARN("Received path with less than 2 points. Cannot generate trajectory.");
        return;
    }

    // --- Prepare constraints ---
    geometry_msgs::Vector3 umax_vec, umin_vec, vmax_vec, vmin_vec;
    umax_vec.x = u_max_[0]; umax_vec.y = u_max_[1]; umax_vec.z = u_max_[2];
    umin_vec.x = u_min_[0]; umin_vec.y = u_min_[1]; umin_vec.z = u_min_[2];
    vmax_vec.x = v_max_[0]; vmax_vec.y = v_max_[1]; vmax_vec.z = v_max_[2];
    vmin_vec.x = v_min_[0]; vmin_vec.y = v_min_[1]; vmin_vec.z = v_min_[2];

    // --- Decide whether to stitch ---
    bool perform_stitch = false;
    {
        std::lock_guard<std::mutex> lock_pose(pose_mutex_);
        std::lock_guard<std::mutex> lock_traj(trajectory_mutex_);
        if (has_published_trajectory_ && has_received_pose_ && !current_published_trajectory_.empty()) {
            perform_stitch = true;
        }
    }

    std::vector<swarm_msgs::DiscreteTrajectoryPoint> final_trajectory_points;
    double final_total_time = 0.0;

    // ==================================================
    // --- BRANCH 1: PERFORM STITCHING ---
    // ==================================================
    if (perform_stitch) {
        ROS_DEBUG("Attempting trajectory stitching.");
        std::vector<swarm_msgs::DiscreteTrajectoryPoint> frozen_segment;
        swarm_msgs::DiscreteTrajectoryPoint freeze_point; // State (P,V,A) at t_freeze
        double t_freeze = 0.0;

        // --- Step 1: Calculate Freeze Point and Extract Frozen Segment ---
        { // Scope for reading shared data
            std::lock_guard<std::mutex> lock_pose(pose_mutex_);
            std::lock_guard<std::mutex> lock_traj(trajectory_mutex_);

            Eigen::Vector3d drone_pos(current_drone_pose_.pose.position.x,
                                      current_drone_pose_.pose.position.y,
                                      current_drone_pose_.pose.position.z);
            double t_proj = findNearestTimeOnTrajectory(current_published_trajectory_, drone_pos);
            double current_total_active_time = current_published_trajectory_.back().time_from_start;
            t_freeze = std::min(t_proj + freeze_duration_, current_total_active_time);
            t_freeze = std::max(t_freeze, 0.0);

            ROS_DEBUG("Stitching: t_proj=%.3f, t_freeze=%.3f", t_proj, t_freeze);

            auto it_end = std::lower_bound(current_published_trajectory_.begin(), current_published_trajectory_.end(), t_freeze,
                                          [](const swarm_msgs::DiscreteTrajectoryPoint& p, double t){ return p.time_from_start < t; });
            frozen_segment.assign(current_published_trajectory_.begin(), it_end);

            if (!interpolateState(current_published_trajectory_, t_freeze, freeze_point)) {
                ROS_ERROR("Failed interpolate freeze point t=%.3f. Fallback to new trajectory.", t_freeze);
                perform_stitch = false; // Go to fallback branch
            } else {
                 if (frozen_segment.empty() || std::abs(frozen_segment.back().time_from_start - t_freeze) > sampling_epsilon_) {
                     frozen_segment.push_back(freeze_point);
                 } else if(!frozen_segment.empty()){ // Update last point if times match
                      frozen_segment.back() = freeze_point;
                 }
            }
        } // Locks released

        if(perform_stitch && frozen_segment.empty()){
             ROS_WARN("Frozen segment empty (t_freeze=%.3f?). Fallback to new trajectory.", t_freeze);
             perform_stitch = false; // Go to fallback branch
        }

        // --- Step 2: Identify Relevant Waypoints from New Request (`msg->points`) ---
        int first_replan_target_idx = 1;
        for (size_t i = 1; i < msg->points.size(); ++i) {
            Eigen::Vector3d p0(msg->points[0].position.x, msg->points[0].position.y, msg->points[0].position.z);
            Eigen::Vector3d pi(msg->points[i].position.x, msg->points[i].position.y, msg->points[i].position.z);
            Eigen::Vector3d pf(freeze_point.position.x, freeze_point.position.y, freeze_point.position.z);
            if ((pi - p0).dot(pi - pf) > 0) {
                first_replan_target_idx = i;
                break;
            }
        }

        // --- Step 3: Construct Waypoint Sequence for Graph Search ---
        std::vector<geometry_msgs::Vector3> replan_waypoints; // Waypoints for graph search P0, P1, ... Pn
        geometry_msgs::Vector3 replan_start_vel;
        geometry_msgs::Vector3 replan_final_vel;
        if (perform_stitch && first_replan_target_idx != -1) {
            // P0 is the freeze point position
            geometry_msgs::Vector3 p_freeze_vec;
            p_freeze_vec.x = freeze_point.position.x;
            p_freeze_vec.y = freeze_point.position.y;
            p_freeze_vec.z = freeze_point.position.z;
            replan_waypoints.push_back(p_freeze_vec);
            replan_start_vel = freeze_point.velocity; // V0 is freeze point velocity

            // Add subsequent waypoints from msg
            for (size_t i = first_replan_target_idx; i < msg->points.size(); ++i) {
                replan_waypoints.push_back(msg->points[i].position);
            }
            replan_final_vel = msg->points.back().velocity; // Final velocity is fixed from msg
        }

        // --- Step 4: Generate Candidate Velocities ---
        std::vector<std::vector<geometry_msgs::Vector3>> candidate_velocities; // candidate_velocities[i] is for replan_waypoints[i+1]
        if (perform_stitch && replan_waypoints.size() >= 3) { // Need at least P0, P1, P2 for candidates at P1
            double angle_dev_rad = max_angle_dev_deg_ * M_PI / 180.0;
            for (size_t i = 0; i < replan_waypoints.size() - 2; ++i) { // Generate for P1 to Pn-1
                candidate_velocities.push_back(
                    generateCandidateVelocities(replan_waypoints[i],   // Prev (Pi)
                                                replan_waypoints[i+1], // Current (Pi+1)
                                                replan_waypoints[i+2], // Next (Pi+2)
                                                base_velocity_, num_v_dir_, num_v_mag_,
                                                angle_dev_rad, speed_dev_ratio_)
                );
            }
        }


        // --- Step 5: Perform Graph Search (Forward Pass) ---
        std::map<GraphNode, NodeInfo> node_infos;
        double final_min_time_replan = std::numeric_limits<double>::infinity();
        int final_pred_vel_idx_replan = -1; // Index of best vel at waypoint n-1
        bool path_found = false;
        if (perform_stitch && !replan_waypoints.empty()) {
            if (replan_waypoints.size() == 1) { // Only freeze point, no replan
                 path_found = true; // Trivial path
                 final_min_time_replan = 0.0;
            } else { // At least 2 waypoints (freeze + end or more)
                 path_found = findOptimalVelocityPath(replan_waypoints, replan_start_vel, candidate_velocities,
                                                     replan_final_vel, node_infos,
                                                     final_min_time_replan, final_pred_vel_idx_replan);
            }
        }

        if (perform_stitch && !path_found) {
            ROS_ERROR("Graph search failed to find a path to the end. Fallback.");
            perform_stitch = false;
        }

        // --- Step 6: Backtrack and Reconstruct Optimal Path Params ---
        std::vector<swarm_msgs::TimeOptimalPMMParam> optimal_replan_params;
        if (perform_stitch && !replan_waypoints.empty()) {
            if (replan_waypoints.size() > 1) { // Need to reconstruct if more than just freeze point
                std::vector<int> optimal_vel_indices(replan_waypoints.size() - 2, -1); // Store optimal indices for P1 to Pn-1
                int current_pred_idx = final_pred_vel_idx_replan;

                // Backtrack from Pn-1 up to P1
                for (int layer = replan_waypoints.size() - 2; layer >= 1; --layer) {
                    if (current_pred_idx == -1) { // Error in backtracking
                        ROS_ERROR("Error backtracking optimal path. Fallback.");
                        perform_stitch = false; break;
                    }
                    optimal_vel_indices[layer - 1] = current_pred_idx; // Store index for waypoint layer
                    GraphNode current_node = {layer, current_pred_idx};
                    if (!node_infos.count(current_node)) { ROS_ERROR("Error backtracking - node missing. Fallback."); perform_stitch = false; break; }
                    current_pred_idx = node_infos[current_node].prev_layer_vel_idx; // Move to previous layer's index
                    if (current_pred_idx == -2) break; // Reached the start node connection
                }

                if (perform_stitch) {
                    // Build the final param list using optimal velocities
                    geometry_msgs::Vector3 v_start = replan_start_vel;
                    for (size_t i = 0; i < replan_waypoints.size() - 1; ++i) {
                        swarm_msgs::TimeOptimalPMMParam p;
                        p.trajectory_id = i;
                        p.x0 = replan_waypoints[i];
                        p.v0 = v_start;
                        p.xT = replan_waypoints[i+1];
                        if (i < replan_waypoints.size() - 2) { // Intermediate waypoint
                            int optimal_idx = optimal_vel_indices[i];
                            if (optimal_idx < 0 || optimal_idx >= candidate_velocities[i].size()) { 
                                ROS_ERROR("Invalid optimal index. Fallback.");
                                perform_stitch = false;
                                break;
                            }
                            p.vT = candidate_velocities[i][optimal_idx];
                        } else { // Last segment
                            p.vT = replan_final_vel;
                        }

                        // Recalculate PMM params for this optimal segment
                        try {
                            TimeOptimalPMM3D pmm3d(p.x0, p.v0, p.xT, p.vT, umax_vec, umin_vec, vmax_vec, vmin_vec);
                            auto [t1, t2, T, case_idx] = pmm3d.compute_times();
                            if (T < 0.0) { ROS_ERROR("PMM failed final param calc seg %zu. Fallback.", i); perform_stitch = false; break; }
                            if (T < sampling_epsilon_) { T = 0.0; t1.x=t1.y=t1.z=0.0; t2.x=t2.y=t2.z=0.0; }
                            p.umax.x=pmm3d.pmm_x.umax; p.umax.y=pmm3d.pmm_y.umax; p.umax.z=pmm3d.pmm_z.umax;
                            p.umin.x=pmm3d.pmm_x.umin; p.umin.y=pmm3d.pmm_y.umin; p.umin.z=pmm3d.pmm_z.umin;
                            p.t1 = t1; p.t2 = t2; p.T = T; p.case_idx = case_idx;
                            optimal_replan_params.push_back(p);
                            v_start = p.vT; // Update start velocity for next segment
                        } catch (...) {
                            ROS_ERROR("Exception PMM final param calc seg %zu. Fallback.", i);
                            perform_stitch = false;
                            break;
                        }
                    }
                    if (!perform_stitch) optimal_replan_params.clear(); // Clear if error occurred
                }
            } else {
                // Only frozen segment, no replan params needed
                ROS_DEBUG("Only frozen segment remains after stitching check.");
            }
        }


        // --- Step 7: Sample Optimal Replan Segments ---
        std::vector<swarm_msgs::DiscreteTrajectoryPoint> new_sampled_segment_combined;
        if (perform_stitch && !optimal_replan_params.empty()) {
            double accumulated_replan_time = t_freeze;
            for (const auto& piece : optimal_replan_params) {
                std::vector<swarm_msgs::DiscreteTrajectoryPoint> segment_points;
                if (!samplePMMTrajectorySegment(piece.x0, piece.v0, piece.xT, piece.vT,
                                                piece.t1, piece.t2, piece.T, piece.case_idx,
                                                piece.umax, piece.umin,
                                                accumulated_replan_time, replanned_sampling_points_,
                                                segment_points))
                { ROS_ERROR("Failed sampling optimal segment. Fallback."); perform_stitch = false; break; }
                 if (!new_sampled_segment_combined.empty() && !segment_points.empty()) { if (std::abs(new_sampled_segment_combined.back().time_from_start - segment_points.front().time_from_start) < sampling_epsilon_) { segment_points.erase(segment_points.begin()); } }
                 new_sampled_segment_combined.insert(new_sampled_segment_combined.end(), std::make_move_iterator(segment_points.begin()), std::make_move_iterator(segment_points.end()));
                 accumulated_replan_time += piece.T;
            }
        }


        // --- Step 8: Combine ---
        if (perform_stitch) {
            final_trajectory_points = frozen_segment;
            final_trajectory_points.insert(final_trajectory_points.end(),
                                        std::make_move_iterator(new_sampled_segment_combined.begin()),
                                        std::make_move_iterator(new_sampled_segment_combined.end()));
            if (!final_trajectory_points.empty()) { final_total_time = final_trajectory_points.back().time_from_start; }
            else { final_total_time = 0.0; }
            // ROS_INFO("Graph search stitching successful. Final duration: %.3f s", final_total_time);
        }

    } // END if (perform_stitch)

    // ==================================================
    // --- BRANCH 2: NO STITCHING (or Fallback) ---
    // ==================================================
    if (!perform_stitch)
    {
        ROS_DEBUG("Not stitching or fallback. Generating new trajectory from raw points.");
        final_trajectory_points.clear(); // Start fresh
        double accumulated_time = 0.0;
        // --- Recalculate PMM params for the *entire* raw path ---
        swarm_msgs::TimeOptimalPMMPieces fallback_params;
        // ... (Code to recalculate fallback_params - same as in previous answer) ...
        // Basic info
        fallback_params.header.stamp = ros::Time::now();
        fallback_params.num_segment = msg->points.size() - 1;
        fallback_params.T_tatal = 0.0;
        for (size_t i = 0; i < msg->points.size() - 1; ++i)
        {
            const auto &s = msg->points[i];
            const auto &e = msg->points[i + 1];
            swarm_msgs::TimeOptimalPMMParam p;
            p.trajectory_id = i;
            p.x0 = s.position;
            p.v0 = s.velocity;
            p.xT = e.position;
            p.vT = e.velocity;
            fallback_params.pieces.push_back(p);
        }
        // Smooth vel
        for (size_t i = 0; i < fallback_params.pieces.size() - 1; ++i)
        { /* ... smooth ... */
            p1_ = fallback_params.pieces[i].x0;
            p2_ = fallback_params.pieces[i].xT;
            p3_ = fallback_params.pieces[i + 1].xT;
            v1_ = subtract_Vector3(p2_, p1_);
            v2_ = subtract_Vector3(p3_, p2_);
            if (norm_Vector3(v1_) < 1e-6 || norm_Vector3(v2_) < 1e-6)
            {
                v_ = fallback_params.pieces[i].vT;
            }
            else
            {
                v1_ = normalize_Vector3(v1_);
                v2_ = normalize_Vector3(v2_);
                auto v1p2 = add_Vector3(v1_, v2_);
                if (norm_Vector3(v1p2) < 1e-6)
                {
                    v_.x = v_.y = v_.z = 0;
                }
                else
                {
                    v_dir_ = normalize_Vector3(v1p2);
                    v_ = scalar_multiply_Vector3(v_dir_, base_velocity_);
                }
            }
            fallback_params.pieces[i].vT = v_;
            fallback_params.pieces[i + 1].v0 = v_;
        }
        // Compute PMM
        bool fallback_ok = true;
        for (size_t i = 0; i < fallback_params.pieces.size(); ++i)
        {
            try
            {
                TimeOptimalPMM3D pmm3d(fallback_params.pieces[i].x0, fallback_params.pieces[i].v0, fallback_params.pieces[i].xT, fallback_params.pieces[i].vT, umax_vec, umin_vec, vmax_vec, vmin_vec);
                auto [t1, t2, T, cid] = pmm3d.compute_times();
                if (T < 0)
                {
                    ROS_ERROR("PMM failed fallback seg %zu", i);
                    fallback_ok = false;
                    break;
                }
                if (T < sampling_epsilon_)
                {
                    T = 0;
                    t1.x = t1.y = t1.z = 0;
                    t2.x = t2.y = t2.z = 0;
                } /* ... copy params ... */
                fallback_params.pieces[i].umax.x = pmm3d.pmm_x.umax;
                fallback_params.pieces[i].umax.y = pmm3d.pmm_y.umax;
                fallback_params.pieces[i].umax.z = pmm3d.pmm_z.umax;
                fallback_params.pieces[i].umin.x = pmm3d.pmm_x.umin;
                fallback_params.pieces[i].umin.y = pmm3d.pmm_y.umin;
                fallback_params.pieces[i].umin.z = pmm3d.pmm_z.umin;
                fallback_params.pieces[i].t1 = t1;
                fallback_params.pieces[i].t2 = t2;
                fallback_params.pieces[i].T = T;
                fallback_params.pieces[i].case_idx = cid;
                fallback_params.T_tatal += T;
            }
            catch (...)
            {
                ROS_ERROR("Exception PMM fallback seg %zu", i);
                fallback_ok = false;
                break;
            }
        }
        if (!fallback_ok)
        {
            ROS_ERROR("Fallback PMM failed. Aborting.");
            return;
        }
        // --- End Recalculate ---

        // Sample using fallback_params
        for (size_t i = 0; i < fallback_params.pieces.size(); ++i)
        {
            const auto &piece = fallback_params.pieces[i];
            std::vector<swarm_msgs::DiscreteTrajectoryPoint> segment_points;
            if (!samplePMMTrajectorySegment(piece.x0, piece.v0, piece.xT, piece.vT, piece.t1, piece.t2, piece.T, piece.case_idx, piece.umax, piece.umin, accumulated_time, initial_sampling_points_, segment_points))
            {
                ROS_ERROR("Failed sample raw segment %zu. Aborting.", i);
                return;
            }
            if (!final_trajectory_points.empty() && !segment_points.empty())
            {
                if (std::abs(final_trajectory_points.back().time_from_start - segment_points.front().time_from_start) < sampling_epsilon_)
                {
                    segment_points.erase(segment_points.begin());
                }
            }
            final_trajectory_points.insert(final_trajectory_points.end(), std::make_move_iterator(segment_points.begin()), std::make_move_iterator(segment_points.end()));
            accumulated_time += piece.T;
        }
        if (!final_trajectory_points.empty())
        {
            final_total_time = final_trajectory_points.back().time_from_start;
        }
        else
        {
            final_total_time = 0.0;
        }
    }

    // --- Stage 5: Update State and Publish ---
    if (!final_trajectory_points.empty()) {
        { // Lock trajectory mutex for writing
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            current_published_trajectory_ = final_trajectory_points; // Update the stored trajectory
            if (!has_published_trajectory_) has_published_trajectory_ = true;
        } // Mutex released

        // Publish the final trajectory
        swarm_msgs::DiscreteTrajectory final_msg;
        final_msg.header.stamp = ros::Time::now();
        final_msg.header.frame_id = world_frame_id_;
        final_msg.points = final_trajectory_points; // Copy for publishing
        discrete_trajectory_pub_.publish(final_msg);

        // Publish visualization
        publishVisualization(final_trajectory_points);

        // ROS_INFO("Published final trajectory (%s) with %zu points, duration: %.3f s",
        //          perform_stitch ? "stitched" : "new",
        //          final_trajectory_points.size(), final_total_time);
    } else {
        ROS_WARN("Final trajectory empty. Nothing published.");
    }
}


// --- Helper: Sample Single PMM Segment ---
bool PathPlanningNode::samplePMMTrajectorySegment(
    const geometry_msgs::Vector3& p0_msg, const geometry_msgs::Vector3& v0_msg,
    const geometry_msgs::Vector3& pT_msg, const geometry_msgs::Vector3& vT_msg,
    const geometry_msgs::Vector3& t1, const geometry_msgs::Vector3& t2, double T,
    const geometry_msgs::Vector3& case_idx,
    const geometry_msgs::Vector3& umax, const geometry_msgs::Vector3& umin,
    double start_time_offset,
    int num_samples,
    std::vector<swarm_msgs::DiscreteTrajectoryPoint>& sampled_points)
{
    // (Implementation is identical to the one in the separate stitching node answer)
     sampled_points.clear();
     if (num_samples < 2 || T < 0) return false;
     double dt = (T > sampling_epsilon_) ? T / (num_samples - 1) : 0.0;
     for (int k = 0; k < num_samples; ++k) {
        double t_in_segment = (k == num_samples - 1) ? T : k * dt; // Ensure last point is exactly T
        PMMState sx = calculatePMMState(p0_msg.x, v0_msg.x, pT_msg.x, vT_msg.x, umax.x, umin.x, t1.x, t2.x, T, static_cast<int>(case_idx.x), t_in_segment);
        PMMState sy = calculatePMMState(p0_msg.y, v0_msg.y, pT_msg.y, vT_msg.y, umax.y, umin.y, t1.y, t2.y, T, static_cast<int>(case_idx.y), t_in_segment);
        PMMState sz = calculatePMMState(p0_msg.z, v0_msg.z, pT_msg.z, vT_msg.z, umax.z, umin.z, t1.z, t2.z, T, static_cast<int>(case_idx.z), t_in_segment);
        swarm_msgs::DiscreteTrajectoryPoint point;
        point.time_from_start = start_time_offset + t_in_segment;
        point.position.x = sx.pos; point.position.y = sy.pos; point.position.z = sz.pos;
        point.velocity.x = sx.vel; point.velocity.y = sy.vel; point.velocity.z = sz.vel;
        point.acceleration.x = sx.acc; point.acceleration.y = sy.acc; point.acceleration.z = sz.acc;
        sampled_points.push_back(point);
     }
     return true;
}

// --- Helper: Find Nearest Time ---
double PathPlanningNode::findNearestTimeOnTrajectory(
    const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& trajectory,
    const Eigen::Vector3d& current_position)
{
    // (Implementation is identical to the one in the separate stitching node answer)
    if (trajectory.empty()) { return 0.0; }
    double min_dist_sq = std::numeric_limits<double>::max();
    double nearest_time = trajectory.front().time_from_start; size_t nearest_idx = 0;
    // Find closest vertex
    for (size_t i = 0; i < trajectory.size(); ++i) {
        double dx = current_position.x() - trajectory[i].position.x; double dy = current_position.y() - trajectory[i].position.y; double dz = current_position.z() - trajectory[i].position.z;
        double dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq < min_dist_sq) { min_dist_sq = dist_sq; nearest_idx = i; }
    }
    nearest_time = trajectory[nearest_idx].time_from_start;
    // Refine on segments
    if (trajectory.size() > 1) {
        size_t start_idx = (nearest_idx > 0) ? nearest_idx - 1 : 0; size_t end_idx = std::min(nearest_idx + 1, trajectory.size() - 1);
        for(size_t i = start_idx; i < end_idx; ++i) {
            const auto& p1 = trajectory[i]; const auto& p2 = trajectory[i+1];
            Eigen::Vector3d p1e(p1.position.x, p1.position.y, p1.position.z); Eigen::Vector3d p2e(p2.position.x, p2.position.y, p2.position.z);
            Eigen::Vector3d seg = p2e - p1e; double seg_len_sq = seg.squaredNorm();
            if (seg_len_sq < sampling_epsilon_ * sampling_epsilon_) continue;
            double t_param = (current_position - p1e).dot(seg) / seg_len_sq; Eigen::Vector3d closest_p; double time_seg;
            if (t_param <= 0.0) { closest_p = p1e; time_seg = p1.time_from_start; }
            else if (t_param >= 1.0) { closest_p = p2e; time_seg = p2.time_from_start; }
            else { closest_p = p1e + t_param * seg; time_seg = p1.time_from_start + t_param * (p2.time_from_start - p1.time_from_start); }
            double dist_sq = (current_position - closest_p).squaredNorm();
            if (dist_sq < min_dist_sq) { min_dist_sq = dist_sq; nearest_time = time_seg; }
        }
    }
    double total_time = trajectory.back().time_from_start;
    return std::max(0.0, std::min(nearest_time, total_time));
}

// --- Helper: Interpolate State ---
bool PathPlanningNode::interpolateState(
    const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& trajectory,
    double time_from_start,
    swarm_msgs::DiscreteTrajectoryPoint& interpolated_point)
{
    // (Implementation is identical to the one in the separate stitching node answer)
    if (trajectory.empty()) { return false; }
    double total_time = trajectory.back().time_from_start;
    time_from_start = std::max(0.0, std::min(time_from_start, total_time));
    auto it = std::lower_bound(trajectory.begin(), trajectory.end(), time_from_start,
                               [](const swarm_msgs::DiscreteTrajectoryPoint& p, double t) { return p.time_from_start < t; });
    size_t idx2 = std::distance(trajectory.begin(), it); size_t idx1;
    if (idx2 == 0) { interpolated_point = trajectory.front(); interpolated_point.time_from_start = time_from_start; return true; }
    else if (idx2 == trajectory.size()) { interpolated_point = trajectory.back(); interpolated_point.time_from_start = time_from_start; return true; }
    else { idx1 = idx2 - 1; }
    const auto& p1 = trajectory[idx1]; const auto& p2 = trajectory[idx2];
    if (std::abs(p2.time_from_start - p1.time_from_start) < sampling_epsilon_) { interpolated_point = p1; interpolated_point.time_from_start = time_from_start; }
    else {
        double r = (time_from_start - p1.time_from_start) / (p2.time_from_start - p1.time_from_start); r = std::max(0.0, std::min(1.0, r));
        interpolated_point.time_from_start = time_from_start;
        interpolated_point.position.x = p1.position.x + r * (p2.position.x - p1.position.x); /* y, z */ interpolated_point.position.y = p1.position.y + r * (p2.position.y - p1.position.y); interpolated_point.position.z = p1.position.z + r * (p2.position.z - p1.position.z);
        interpolated_point.velocity.x = p1.velocity.x + r * (p2.velocity.x - p1.velocity.x); /* y, z */ interpolated_point.velocity.y = p1.velocity.y + r * (p2.velocity.y - p1.velocity.y); interpolated_point.velocity.z = p1.velocity.z + r * (p2.velocity.z - p1.velocity.z);
        interpolated_point.acceleration.x = p1.acceleration.x + r * (p2.acceleration.x - p1.acceleration.x); /* y, z */ interpolated_point.acceleration.y = p1.acceleration.y + r * (p2.acceleration.y - p1.acceleration.y); interpolated_point.acceleration.z = p1.acceleration.z + r * (p2.acceleration.z - p1.acceleration.z);
    }
    return true;
}

// --- Helper: Publish Visualization ---
void PathPlanningNode::publishVisualization(const std::vector<swarm_msgs::DiscreteTrajectoryPoint>& trajectory_to_visualize) {
    // (Implementation is identical to the one in the separate stitching node answer)
    nav_msgs::Path viz_msg;
    viz_msg.header.stamp = ros::Time::now(); viz_msg.header.frame_id = world_frame_id_;
    if (trajectory_to_visualize.empty()) { stitched_path_viz_pub_.publish(viz_msg); return; }
    viz_msg.poses.reserve(trajectory_to_visualize.size());
    for(const auto& pt : trajectory_to_visualize) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(pt.time_from_start); pose_stamped.header.frame_id = world_frame_id_;
        pose_stamped.pose.position = pt.position; pose_stamped.pose.orientation.w = 1.0;
        viz_msg.poses.push_back(pose_stamped);
    }
    stitched_path_viz_pub_.publish(viz_msg);
}


// --- loadConstraints() ---

void PathPlanningNode::loadConstraints() {
    try {
        // Define default values in case loading fails
        // Use member variables with underscore
        v_max_ = {10.0, 10.0, 10.0};
        v_min_ = {-10.0, -10.0, -10.0}; // Min velocity limits
        u_max_ = {5.0, 5.0, 5.0};  // Max acceleration limits
        u_min_ = {5.0, 5.0, 5.0};  // Max *deceleration* limits (magnitude)

        std::string package_path_str = ros::package::getPath("params"); // Ensure 'params' package exists or adjust name
        std::string config_path;

        if (package_path_str.empty()) {
             ROS_WARN("Could not find package 'params'. Using default constraints. Specify path via ROS param '~config_path' if needed.");
             // Optionally, try loading from a ROS parameter path
             nh_.param<std::string>("config_path", config_path, ""); // Check for private param first
             if (config_path.empty()) {
                 ros::param::param<std::string>("/config_path", config_path, ""); // Check for global param
             }
        } else {
            config_path = package_path_str + "/path_constraint.yaml";
        }


        if (!config_path.empty()) {
             ROS_INFO("Attempting to load constraints from: %s", config_path.c_str());
             YAML::Node node = YAML::LoadFile(config_path);

             // Load values if they exist in the YAML file
             if (node["v_max"]) v_max_ = node["v_max"].as<std::vector<double>>();
             if (node["v_min"]) v_min_ = node["v_min"].as<std::vector<double>>();
             if (node["u_max"]) u_max_ = node["u_max"].as<std::vector<double>>();
             if (node["u_min"]) u_min_ = node["u_min"].as<std::vector<double>>(); // Ensure YAML contains positive values for decel limit

             // Basic validation
             if (v_max_.size() != 3 || v_min_.size() != 3 || u_max_.size() != 3 || u_min_.size() != 3) {
                 ROS_ERROR("Invalid constraint dimensions loaded from YAML. Expecting 3 elements per vector. Using defaults.");
                 // Reset to defaults if dimensions are wrong
                 v_max_ = {10.0, 10.0, 10.0};
                 v_min_ = {-10.0, -10.0, -10.0};
                 u_max_ = {5.0, 5.0, 5.0};
                 u_min_ = {5.0, 5.0, 5.0};
             } else {
                ROS_INFO("Successfully loaded constraints from YAML file.");
             }
        } else {
             ROS_WARN("Constraint file path is empty. Using default constraints.");
        }

        // Log the constraints being used
        ROS_INFO_STREAM("Using constraints:");
        ROS_INFO_STREAM("  v_max: [" << v_max_[0] << ", " << v_max_[1] << ", " << v_max_[2] << "]");
        ROS_INFO_STREAM("  v_min: [" << v_min_[0] << ", " << v_min_[1] << ", " << v_min_[2] << "]");
        ROS_INFO_STREAM("  u_max (accel limit): [" << u_max_[0] << ", " << u_max_[1] << ", " << u_max_[2] << "]");
        ROS_INFO_STREAM("  u_min (decel limit magnitude): [" << u_min_[0] << ", " << u_min_[1] << ", " << u_min_[2] << "]");


    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("Failed to load or parse constraints YAML: " << e.what() << ". Using default values.");
        // Ensure defaults are set if loading failed mid-way (already done at the start of try block)
    } catch (const std::exception& e) {
         ROS_ERROR_STREAM("Error during constraint loading: " << e.what() << ". Using default values.");
         // Ensure defaults are set
    } catch (...) {
         ROS_ERROR("Unknown error during constraint loading. Using default values.");
         // Ensure defaults are set
    }
}


// --- main() ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");
    PathPlanningNode node;
    ros::spin();
    return 0;
}