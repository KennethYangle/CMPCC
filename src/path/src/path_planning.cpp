#include "path_planning.h"
#include <exception> // Include for std::exception

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


void PathPlanningNode::pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    if (msg->points.size() < 2) {
        ROS_WARN("Received path with less than 2 points. Cannot generate trajectory.");
        return;
    }

    // --- Stage 1 & 2: Calculate TimeOptimalPMM Parameters ---
    swarm_msgs::TimeOptimalPMMPieces optimal_paths_params; // Internal data structure
    optimal_paths_params.header.stamp = ros::Time::now(); // Use calculation time
    optimal_paths_params.num_segment = msg->points.size() - 1;
    optimal_paths_params.T_tatal = 0.0;

    // Use member variables with underscore
    current_velocity_ = msg->points[0].velocity;
    current_V_ = norm_Vector3(current_velocity_);
    // Avoid zero velocity, ensure some minimum speed for smoothing logic
    current_V_ = std::max(2.0, current_V_); // Adjusted minimum speed slightly

    // 1. Construct basic information
    for (size_t i = 0; i < msg->points.size() - 1; ++i) {
        const auto& pt_start = msg->points[i];
        const auto& pt_end = msg->points[i + 1];
        swarm_msgs::TimeOptimalPMMParam path_param;
        path_param.trajectory_id = i;
        path_param.x0 = pt_start.position;
        path_param.v0 = pt_start.velocity;
        path_param.xT = pt_end.position;
        path_param.vT = pt_end.velocity;
        optimal_paths_params.pieces.push_back(path_param);
    }

    // 2. Smooth the speed of the intermediate points
    for (size_t i = 0; i < optimal_paths_params.pieces.size() - 1; ++i) { // Loop up to second-to-last piece
        // Use member variables with underscore for temporary calculation storage
        p1_ = optimal_paths_params.pieces[i].x0;     // Point i
        p2_ = optimal_paths_params.pieces[i].xT;     // Point i+1 (intermediate point)
        p3_ = optimal_paths_params.pieces[i+1].xT;   // Point i+2

        v1_ = subtract_Vector3(p2_, p1_);
        v2_ = subtract_Vector3(p3_, p2_);

        // Handle cases where points might be coincident
        if (norm_Vector3(v1_) < 1e-6 || norm_Vector3(v2_) < 1e-6) {
             ROS_WARN("Skipping velocity smoothing for segment %zu due to coincident points.", i);
             // Keep original velocity from initial setup
             v_ = optimal_paths_params.pieces[i].vT; // Use temporary v_
        } else {
            v1_ = normalize_Vector3(v1_);
            v2_ = normalize_Vector3(v2_);
             geometry_msgs::Vector3 v1_plus_v2 = add_Vector3(v1_, v2_); // Calculate sum once
            // Check for sharp turns (vectors pointing opposite directions)
            if (norm_Vector3(v1_plus_v2) < 1e-6) { // Check if sum is close to zero vector
                 // Handle 180 degree turn - velocity should ideally be zero
                 v_.x = v_.y = v_.z = 0.0; // Use temporary v_
                 ROS_WARN("Sharp 180-degree turn detected at point %zu. Setting intermediate velocity to zero.", i+1);
            } else {
                v_dir_ = normalize_Vector3(v1_plus_v2); // Bisector direction (Use temporary v_dir_)
                v_ = scalar_multiply_Vector3(v_dir_, current_V_); // Assign smoothed velocity (Use temporary v_)
            }
        }

        // Assign the calculated smoothed velocity (v_) to the PMM parameters
        optimal_paths_params.pieces[i].vT = v_;     // Set velocity *at* intermediate point (end of piece i)
        optimal_paths_params.pieces[i+1].v0 = v_;   // Set velocity *at* intermediate point (start of piece i+1)
    }
     // Note: Initial v0 and final vT are taken directly from the input msg->points


    // 3. Construct the trajectory parameters
    for (size_t i = 0; i < optimal_paths_params.pieces.size(); ++i) {
        geometry_msgs::Vector3 umax_vec, umin_vec, vmax_vec, vmin_vec;
        // Use member variables with underscore for constraints
        umax_vec.x = u_max_[0]; umax_vec.y = u_max_[1]; umax_vec.z = u_max_[2];
        // umin_ represents the magnitude of the max *deceleration*
        umin_vec.x = u_min_[0]; umin_vec.y = u_min_[1]; umin_vec.z = u_min_[2];
        vmax_vec.x = v_max_[0]; vmax_vec.y = v_max_[1]; vmax_vec.z = v_max_[2];
        vmin_vec.x = v_min_[0]; vmin_vec.y = v_min_[1]; vmin_vec.z = v_min_[2];

        try {
            // Create TimeOptimalPMM instance (Make sure class name matches your PMM library)
            TimeOptimalPMM3D pmm3d(optimal_paths_params.pieces[i].x0,
                                   optimal_paths_params.pieces[i].v0,
                                   optimal_paths_params.pieces[i].xT,
                                   optimal_paths_params.pieces[i].vT,
                                   umax_vec, umin_vec, vmax_vec, vmin_vec);

            // Debug print (optional)
            // ROS_DEBUG_STREAM("Segment " << i << ":");
            // ROS_DEBUG_STREAM("  x0: [" << optimal_paths_params.pieces[i].x0.x << "," << optimal_paths_params.pieces[i].x0.y << "," << optimal_paths_params.pieces[i].x0.z << "]"
            //             << " v0: [" << optimal_paths_params.pieces[i].v0.x << "," << optimal_paths_params.pieces[i].v0.y << "," << optimal_paths_params.pieces[i].v0.z << "]");
            // ROS_DEBUG_STREAM("  xT: [" << optimal_paths_params.pieces[i].xT.x << "," << optimal_paths_params.pieces[i].xT.y << "," << optimal_paths_params.pieces[i].xT.z << "]"
            //             << " vT: [" << optimal_paths_params.pieces[i].vT.x << "," << optimal_paths_params.pieces[i].vT.y << "," << optimal_paths_params.pieces[i].vT.z << "]");


            auto [t1, t2, T, case_idx] = pmm3d.compute_times();

            // Check if computation was successful (T should be non-negative)
            // Allow T=0 for coincident start/end points with zero velocity difference
            if (T < 0.0) {
                 ROS_ERROR("TimeOptimalPMM computation failed for segment %zu (T=%f). Skipping trajectory generation.", i, T);
                 return; // Abort trajectory generation
            }
             // Handle case where T is extremely small (effectively zero)
             if (T < sampling_epsilon_){
                 ROS_WARN("Segment %zu has near-zero duration (T=%f). Treating as instantaneous.", i, T);
                 T = 0.0; // Clamp to zero
                 // Ensure t1 and t2 are also zero if T is zero
                 t1.x = t1.y = t1.z = 0.0;
                 t2.x = t2.y = t2.z = 0.0;
                 // Case index might become irrelevant, but keep what PMM returned.
                 // Verify v0 and vT are very close if T is near zero.
                 geometry_msgs::Vector3 delta_v = subtract_Vector3(optimal_paths_params.pieces[i].vT, optimal_paths_params.pieces[i].v0);
                 if (norm_Vector3(delta_v) > 1e-3) { // Check if velocities are consistent
                     ROS_WARN("Segment %zu has near-zero duration but significant velocity change.", i);
                 }
             }


            // Synchronized u (Get the actual constraints used by PMM, especially if adjusted internally)
            umax_vec.x = pmm3d.pmm_x.umax; umax_vec.y = pmm3d.pmm_y.umax; umax_vec.z = pmm3d.pmm_z.umax;
            umin_vec.x = pmm3d.pmm_x.umin; umin_vec.y = pmm3d.pmm_y.umin; umin_vec.z = pmm3d.pmm_z.umin;

            // Update TimeOptimalPMMParam message (internal structure)
            optimal_paths_params.pieces[i].umax = umax_vec;
            optimal_paths_params.pieces[i].umin = umin_vec; // Store the deceleration limit magnitude
            optimal_paths_params.pieces[i].t1 = t1;
            optimal_paths_params.pieces[i].t2 = t2;
            optimal_paths_params.pieces[i].T = T;
            optimal_paths_params.pieces[i].case_idx = case_idx;
            optimal_paths_params.T_tatal += T;

            // Debug print (optional)
            // ROS_DEBUG_STREAM("  T: " << T << ", t1: [" << t1.x << "," << t1.y << "," << t1.z << "]"
            //             << ", t2: [" << t2.x << "," << t2.y << "," << t2.z << "]"
            //             << ", case: [" << case_idx.x << "," << case_idx.y << "," << case_idx.z << "]");


        } catch (const std::exception& e) {
            ROS_ERROR("Exception during TimeOptimalPMM computation for segment %zu: %s", i, e.what());
            return; // Abort trajectory generation
        } catch (...) {
            ROS_ERROR("Unknown exception during TimeOptimalPMM computation for segment %zu.", i);
            return; // Abort trajectory generation
        }
    }

    // --- Stage 4: Sample Trajectory Points and Publish ---

    swarm_msgs::DiscreteTrajectory discrete_trajectory_msg;
    discrete_trajectory_msg.header.stamp = ros::Time::now(); // Publish time

    // Set frame_id:
    discrete_trajectory_msg.header.frame_id = "map"; // Or "world", or get from ROS param
    // ROS_WARN_ONCE("Using fixed frame_id '%s' for DiscreteTrajectory. Verify MassPoints header.", discrete_trajectory_msg.header.frame_id.c_str());


    double accumulated_time = 0.0;

    // Loop through each calculated segment parameter set
    for (size_t i = 0; i < optimal_paths_params.pieces.size(); ++i) {
        const auto& piece = optimal_paths_params.pieces[i];
        double T_segment = piece.T;

        // --- Sampling Logic ---
        // Always include the start point of the first segment,
        // or the transition point for subsequent segments.
        // Use k=0 logic for this.

        // Determine the number of steps for sampling
        int num_steps = 0;
        if (T_segment > sampling_epsilon_ && num_points_per_segment_ > 1) {
             num_steps = num_points_per_segment_ -1; // e.g., 100 points -> 99 steps
        } else {
             num_steps = 0; // Only sample start/end point if duration is zero or 1 sample requested
        }
        double dt = (num_steps > 0) ? T_segment / num_steps : 0.0;


        // Sample points within the segment using num_steps + 1 points (k from 0 to num_steps)
        for (int k = 0; k <= num_steps; ++k) {
            // Calculate time 't' within the current segment [0, T_segment]
            double t_in_segment = k * dt;
            // Ensure the last point is exactly at T_segment
            if (k == num_steps) {
                t_in_segment = T_segment;
            }

            // Avoid adding duplicate points at segment transitions
            // Add point k=0 only for the very first segment (i=0)
            // For i > 0, the point k=0 corresponds to the *previous* segment's end point (k=num_steps)
            // which was already added.
            if (i > 0 && k == 0) {
                continue;
            }

            // Calculate state (pos, vel, acc) for each axis at t_in_segment using the helper function
            PMMState state_x = calculatePMMState(piece.x0.x, piece.v0.x, piece.xT.x, piece.vT.x, piece.umax.x, piece.umin.x, piece.t1.x, piece.t2.x, T_segment, static_cast<int>(piece.case_idx.x), t_in_segment);
            PMMState state_y = calculatePMMState(piece.x0.y, piece.v0.y, piece.xT.y, piece.vT.y, piece.umax.y, piece.umin.y, piece.t1.y, piece.t2.y, T_segment, static_cast<int>(piece.case_idx.y), t_in_segment);
            PMMState state_z = calculatePMMState(piece.x0.z, piece.v0.z, piece.xT.z, piece.vT.z, piece.umax.z, piece.umin.z, piece.t1.z, piece.t2.z, T_segment, static_cast<int>(piece.case_idx.z), t_in_segment);

            // Create the trajectory point message
            swarm_msgs::DiscreteTrajectoryPoint point_msg;
            point_msg.time_from_start = accumulated_time + t_in_segment;

            point_msg.position.x = state_x.pos;
            point_msg.position.y = state_y.pos;
            point_msg.position.z = state_z.pos;

            point_msg.velocity.x = state_x.vel;
            point_msg.velocity.y = state_y.vel;
            point_msg.velocity.z = state_z.vel;

            point_msg.acceleration.x = state_x.acc;
            point_msg.acceleration.y = state_y.acc;
            point_msg.acceleration.z = state_z.acc;

            // Add point to the message
            discrete_trajectory_msg.points.push_back(point_msg);
        }

        // Accumulate time for the next segment start time
        accumulated_time += T_segment;
    }

    // Publish the discrete trajectory
    if (!discrete_trajectory_msg.points.empty()) {
         discrete_trajectory_pub_.publish(discrete_trajectory_msg);
         ROS_INFO("Published discrete trajectory with %zu points, total time: %.3f s",
                  discrete_trajectory_msg.points.size(), optimal_paths_params.T_tatal);
    } else if (optimal_paths_params.pieces.empty()) {
         ROS_WARN("No segments generated (input points < 2?), not publishing trajectory.");
    } else {
         ROS_WARN("Generated trajectory has no points (all segments zero duration?). Not publishing.");
    }

    // Optionally, still publish the parameters if needed elsewhere
    // refer_path_cps_pub_.publish(optimal_paths_params); // Make sure publisher is uncommented if used
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
    PathPlanningNode node; // Node object creation triggers constructor, including loadConstraints()
    ros::spin();
    return 0;
}