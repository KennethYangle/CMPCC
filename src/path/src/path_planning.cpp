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

// --- Main Path Points Callback ---
void PathPlanningNode::pathPointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    if (msg->points.size() < 2) {
        ROS_WARN("Received path with less than 2 points. Cannot generate trajectory.");
        return;
    }

    // --- Stage 1 & 2: Calculate PMM Parameters for the *entire new raw path* ---
    // This calculates the parameters as if we were generating a completely new trajectory.
    swarm_msgs::TimeOptimalPMMPieces new_raw_params; // Parameters for the trajectory defined by msg
    new_raw_params.header.stamp = ros::Time::now();
    new_raw_params.num_segment = msg->points.size() - 1;
    new_raw_params.T_tatal = 0.0;

    current_velocity_ = msg->points[0].velocity;
    current_V_ = norm_Vector3(current_velocity_);
    current_V_ = std::max(2.0, current_V_);

    // 1. Basic info
    for (size_t i = 0; i < msg->points.size() - 1; ++i) {
        const auto& pt_start = msg->points[i];
        const auto& pt_end = msg->points[i + 1];
        swarm_msgs::TimeOptimalPMMParam p;
        p.trajectory_id = i; p.x0 = pt_start.position; p.v0 = pt_start.velocity;
        p.xT = pt_end.position; p.vT = pt_end.velocity;
        new_raw_params.pieces.push_back(p);
    }
    // 2. Smooth intermediate velocities
    for (size_t i = 0; i < new_raw_params.pieces.size() - 1; ++i) {
        p1_ = new_raw_params.pieces[i].x0; p2_ = new_raw_params.pieces[i].xT; p3_ = new_raw_params.pieces[i+1].xT;
        v1_ = subtract_Vector3(p2_, p1_); v2_ = subtract_Vector3(p3_, p2_);
        if (norm_Vector3(v1_) < 1e-6 || norm_Vector3(v2_) < 1e-6) { v_ = new_raw_params.pieces[i].vT; }
        else {
            v1_ = normalize_Vector3(v1_); v2_ = normalize_Vector3(v2_);
            geometry_msgs::Vector3 v1_plus_v2 = add_Vector3(v1_, v2_);
            if (norm_Vector3(v1_plus_v2) < 1e-6) { v_.x = v_.y = v_.z = 0.0; }
            else { v_dir_ = normalize_Vector3(v1_plus_v2); v_ = scalar_multiply_Vector3(v_dir_, current_V_); }
        }
        new_raw_params.pieces[i].vT = v_; new_raw_params.pieces[i+1].v0 = v_;
    }
    // 3. Compute PMM times/cases for all raw segments
    geometry_msgs::Vector3 umax_vec, umin_vec, vmax_vec, vmin_vec; // Local temps for constraints
    umax_vec.x = u_max_[0]; umax_vec.y = u_max_[1]; umax_vec.z = u_max_[2];
    umin_vec.x = u_min_[0]; umin_vec.y = u_min_[1]; umin_vec.z = u_min_[2];
    vmax_vec.x = v_max_[0]; vmax_vec.y = v_max_[1]; vmax_vec.z = v_max_[2];
    vmin_vec.x = v_min_[0]; vmin_vec.y = v_min_[1]; vmin_vec.z = v_min_[2];
    for (size_t i = 0; i < new_raw_params.pieces.size(); ++i) {
         try {
            TimeOptimalPMM3D pmm3d(new_raw_params.pieces[i].x0, new_raw_params.pieces[i].v0,
                                   new_raw_params.pieces[i].xT, new_raw_params.pieces[i].vT,
                                   umax_vec, umin_vec, vmax_vec, vmin_vec);
            auto [t1, t2, T, case_idx] = pmm3d.compute_times();
            if (T < 0.0) { ROS_ERROR("PMM failed for raw segment %zu (T=%f). Aborting.", i, T); return; }
            if (T < sampling_epsilon_) { T = 0.0; t1.x=t1.y=t1.z=0.0; t2.x=t2.y=t2.z=0.0; }
            // Update parameters in new_raw_params
            new_raw_params.pieces[i].umax.x = pmm3d.pmm_x.umax; 
            new_raw_params.pieces[i].umax.y = pmm3d.pmm_y.umax; new_raw_params.pieces[i].umax.z = pmm3d.pmm_z.umax;
            new_raw_params.pieces[i].umin.x = pmm3d.pmm_x.umin;
            new_raw_params.pieces[i].umin.y = pmm3d.pmm_y.umin; new_raw_params.pieces[i].umin.z = pmm3d.pmm_z.umin;
            new_raw_params.pieces[i].t1 = t1; new_raw_params.pieces[i].t2 = t2;
            new_raw_params.pieces[i].T = T; new_raw_params.pieces[i].case_idx = case_idx;
            new_raw_params.T_tatal += T;
        } catch (const std::exception& e) {
            ROS_ERROR("Exception during initial PMM computation segment %zu: %s. Aborting.", i, e.what()); return;
        } catch (...) {
            ROS_ERROR("Unknown exception during initial PMM computation segment %zu. Aborting.", i); return;
        }
    }
    // Now new_raw_params holds the info for the desired trajectory *if* we started from msg->points[0]


    // --- Stage 4: Stitching Decision and Execution ---
    std::vector<swarm_msgs::DiscreteTrajectoryPoint> final_trajectory_points;
    double final_total_time = 0.0;

    // --- Check if we should perform stitching ---
    bool perform_stitch = false;
    { // Scope for locks
        std::lock_guard<std::mutex> lock_pose(pose_mutex_);
        std::lock_guard<std::mutex> lock_traj(trajectory_mutex_); // Lock both before checking flags
        if (has_published_trajectory_ && has_received_pose_ && !current_published_trajectory_.empty()) {
            perform_stitch = true;
        }
    } // Locks released here

    if (perform_stitch) {
        ROS_DEBUG("Performing trajectory stitching.");
        std::vector<swarm_msgs::DiscreteTrajectoryPoint> frozen_segment;
        swarm_msgs::DiscreteTrajectoryPoint freeze_point;
        geometry_msgs::Vector3 goal_pos;
        geometry_msgs::Vector3 goal_vel;
        double t_freeze = 0.0;

        { // Scope for reading shared data under locks
            std::lock_guard<std::mutex> lock_pose(pose_mutex_);
            std::lock_guard<std::mutex> lock_traj(trajectory_mutex_);

            Eigen::Vector3d drone_pos(current_drone_pose_.pose.position.x,
                                      current_drone_pose_.pose.position.y,
                                      current_drone_pose_.pose.position.z);

            // Find projection on the *currently published* trajectory
            double t_proj = findNearestTimeOnTrajectory(current_published_trajectory_, drone_pos);
            double current_total_active_time = current_published_trajectory_.back().time_from_start;
            t_freeze = std::min(t_proj + freeze_duration_, current_total_active_time);
            t_freeze = std::max(t_freeze, 0.0);

            ROS_DEBUG("Stitching: t_proj=%.3f, t_freeze=%.3f", t_proj, t_freeze);

            // Extract frozen part
            auto it_end = std::lower_bound(current_published_trajectory_.begin(), current_published_trajectory_.end(), t_freeze,
                                          [](const swarm_msgs::DiscreteTrajectoryPoint& p, double t){ return p.time_from_start < t; });
            frozen_segment.assign(current_published_trajectory_.begin(), it_end);

            if (!interpolateState(current_published_trajectory_, t_freeze, freeze_point)) {
                ROS_ERROR("Failed to interpolate freeze point at t=%.3f during stitching. Publishing new raw trajectory instead.", t_freeze);
                perform_stitch = false; // Fallback to non-stitching case
            } else {
                 if (frozen_segment.empty() || std::abs(frozen_segment.back().time_from_start - t_freeze) > sampling_epsilon_) {
                     frozen_segment.push_back(freeze_point);
                 } else if(!frozen_segment.empty()){ // Update last point if times match
                      frozen_segment.back() = freeze_point;
                 }
            }
        } // Locks released

        if(perform_stitch && frozen_segment.empty()){
             ROS_WARN("Frozen segment became empty after interpolation (t_freeze=%.3f?). Publishing new raw trajectory instead.", t_freeze);
             perform_stitch = false; // Fallback
        }


        if (perform_stitch) {
            // Get goal state from the *end* of the calculated raw parameters
            goal_pos = new_raw_params.pieces.back().xT;
            goal_vel = new_raw_params.pieces.back().vT;

            // --- Replan PMM from freeze_point to goal ---
            geometry_msgs::Vector3 start_pos;
            start_pos.x = freeze_point.position.x;
            start_pos.y = freeze_point.position.y;
            start_pos.z = freeze_point.position.z;
            geometry_msgs::Vector3 start_vel = freeze_point.velocity;
            std::vector<swarm_msgs::DiscreteTrajectoryPoint> new_sampled_segment;

            try {
                TimeOptimalPMM3D pmm3d_replan(start_pos, start_vel, goal_pos, goal_vel,
                                              umax_vec, umin_vec, vmax_vec, vmin_vec); // Constraints loaded earlier
                auto [t1_re, t2_re, T_re, case_idx_re] = pmm3d_replan.compute_times();

                if (T_re < 0.0) {
                    ROS_ERROR("PMM replanning failed from freeze point (T=%.3f). Publishing new raw trajectory instead.", T_re);
                    perform_stitch = false; // Fallback
                }
                else {
                     if (T_re < sampling_epsilon_){ T_re = 0.0; }
                     geometry_msgs::Vector3 umax_re, umin_re; // Get sync'd constraints
                     umax_re.x = pmm3d_replan.pmm_x.umax; /* y, z */ umax_re.y=pmm3d_replan.pmm_y.umax; umax_re.z=pmm3d_replan.pmm_z.umax;
                     umin_re.x = pmm3d_replan.pmm_x.umin; /* y, z */ umin_re.y=pmm3d_replan.pmm_y.umin; umin_re.z=pmm3d_replan.pmm_z.umin;

                    // Sample the replanned segment
                    if (!samplePMMTrajectorySegment(start_pos, start_vel, goal_pos, goal_vel,
                                                    t1_re, t2_re, T_re, case_idx_re,
                                                    umax_re, umin_re,
                                                    t_freeze, // Time offset
                                                    replanned_sampling_points_, // Use specific sampling number
                                                    new_sampled_segment))
                    {
                        ROS_ERROR("Failed to sample replanned PMM segment. Publishing new raw trajectory instead.");
                        perform_stitch = false; // Fallback
                    }
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Exception during PMM replanning: %s. Publishing new raw trajectory instead.", e.what());
                perform_stitch = false; // Fallback
            } catch (...) {
                 ROS_ERROR("Unknown exception during PMM replanning. Publishing new raw trajectory instead.");
                 perform_stitch = false; // Fallback
            }

             if(perform_stitch) {
                 // --- Combine ---
                 final_trajectory_points = frozen_segment;
                 // Avoid duplicate point at connection
                 if (!new_sampled_segment.empty() && !frozen_segment.empty()) {
                     if (std::abs(new_sampled_segment.front().time_from_start - frozen_segment.back().time_from_start) < sampling_epsilon_) {
                         new_sampled_segment.erase(new_sampled_segment.begin());
                     }
                 }
                 final_trajectory_points.insert(final_trajectory_points.end(),
                                                std::make_move_iterator(new_sampled_segment.begin()),
                                                std::make_move_iterator(new_sampled_segment.end()));
                 if (!final_trajectory_points.empty()) {
                    final_total_time = final_trajectory_points.back().time_from_start;
                 }
                 ROS_INFO("Stitching successful. Final trajectory duration: %.3f s", final_total_time);
             }
        }
    } // End of stitching block

    // --- If stitching was not performed or failed, sample the new raw trajectory directly ---
    if (!perform_stitch) {
        ROS_DEBUG("Not stitching. Sampling new raw trajectory directly.");
        double accumulated_time = 0.0;
        final_trajectory_points.clear(); // Ensure it's empty before filling

        for (size_t i = 0; i < new_raw_params.pieces.size(); ++i) {
            const auto& piece = new_raw_params.pieces[i];
            std::vector<swarm_msgs::DiscreteTrajectoryPoint> segment_points;
            if (!samplePMMTrajectorySegment(piece.x0, piece.v0, piece.xT, piece.vT,
                                            piece.t1, piece.t2, piece.T, piece.case_idx,
                                            piece.umax, piece.umin,
                                            accumulated_time, // Time offset for this segment
                                            initial_sampling_points_, // Use initial sampling number
                                            segment_points))
            {
                 ROS_ERROR("Failed to sample raw segment %zu. Aborting trajectory generation.", i);
                 return; // Cannot generate full trajectory
            }

            // Combine segments, avoiding duplicates
             if (!final_trajectory_points.empty() && !segment_points.empty()) {
                 if (std::abs(final_trajectory_points.back().time_from_start - segment_points.front().time_from_start) < sampling_epsilon_) {
                     segment_points.erase(segment_points.begin());
                 }
             }
             final_trajectory_points.insert(final_trajectory_points.end(),
                                            std::make_move_iterator(segment_points.begin()),
                                            std::make_move_iterator(segment_points.end()));

            accumulated_time += piece.T;
        }
         if (!final_trajectory_points.empty()) {
            final_total_time = final_trajectory_points.back().time_from_start;
         } else {
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

        ROS_INFO("Published final trajectory (%s) with %zu points, duration: %.3f s",
                 perform_stitch ? "stitched" : "new",
                 final_trajectory_points.size(), final_total_time);
    } else {
        ROS_WARN("Final trajectory is empty after processing. Nothing published.");
        // Optionally clear the current published trajectory if it should be empty
        // {
        //     std::lock_guard<std::mutex> lock(trajectory_mutex_);
        //     current_published_trajectory_.clear();
        // }
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