#include "map.h"

using namespace std;
using namespace Eigen;
namespace ft{
Map::Map(){}

void Map::setPathPts(const swarm_msgs::TimeOptimalPMMPieces::ConstPtr& msg) {
    std::vector<double> x_all, y_all, z_all;
    {
    std::unique_lock<std::mutex> lock(mtx);
    // 等待直到 can_modify 为 true
    cv.wait(lock, [this]{ return can_modify; });

    theta_sample.clear();
    pos_sample.clear();
    vel_sample.clear();
    acc_sample.clear();
    // Clear new members
    segment_end_times_.clear();
    segment_end_positions_.clear();


    num_segment = msg->num_segment;
    thetaMax = msg->T_tatal;
    double T_accumulated = 0.;

    // 分段循环
    for (int i = 0; i < num_segment; i++) {
        auto piece = msg->pieces[i];
        auto case_idx = piece.case_idx;
        double T_ = piece.T;

        double v_line_x = (piece.xT.x - piece.x0.x) / T_;
        double v_line_y = (piece.xT.y - piece.x0.y) / T_;
        double v_line_z = (piece.xT.z - piece.x0.z) / T_;
        double x = piece.x0.x, y = piece.x0.y, z = piece.x0.z;
        double vx = piece.v0.x, vy = piece.v0.y, vz = piece.v0.z;
        double ux = 0, uy = 0, uz = 0;

        // 采1000个点循环
        for (int tt = 0; tt < num_points; tt++) {
            double theta_ = T_ * tt / (num_points - 1);
             // Ensure dt calculation is robust, especially if num_points is 1
            double dt = (num_points > 1) ? (T_ / (num_points -1)) : 0; // dt between samples
            if (tt == 0) dt = 0; // For the first point, effective dt for integration is 0 or use initial state

            Eigen::Vector3d thetaPoint, thetaDot, thetaDotDot;

            // Recalculate state based on exact formula for each tt if possible,
            // or ensure iterative update starts fresh for each segment for x,y,z,vx,vy,vz,ux,uy,uz
            // The current code reuses x,y,z,vx,vy,vz from the previous 'tt' step, which is integration.
            // This is fine for sampling the segment.
            // For the very first point (tt=0) of a segment, ensure x,y,z,vx,vy,vz are piece.x0, piece.v0
            // And ux,uy,uz are determined by theta_ = 0.

            // The provided code for calculating thetaPoint, thetaDot, thetaDotDot based on iterative integration for non-case_idx=2 seems fine for sampling.
            // The critical part is that x,y,z,vx,vy,vz are reset based on piece.x0, piece.v0 at the start of each segment.
            // This is already happening because x,y,z,vx,vy,vz are initialized from piece.x0 and piece.v0 before the tt loop.

            // x轴
            if (isEqualFloat(case_idx.x, 2)) {
                thetaPoint[0] = piece.x0.x + theta_ * v_line_x;
                thetaDot[0] = v_line_x;
                thetaDotDot[0] = 0;
            }
            else {
                // Re-initialize x, vx for each sample point for analytical calculation if possible
                // Or ensure the iterative update is correct.
                // The current iterative update means x,vx at step tt depend on tt-1.
                // For tt=0:
                if (tt == 0) { // Reset state for the first sample point of the segment
                    x = piece.x0.x;
                    vx = piece.v0.x;
                    // Determine initial ux based on theta_ = 0
                    if (isEqualFloat(case_idx.x, 0)) {
                        ux = 0.0 < piece.t1.x ? piece.umax.x : (0.0 < piece.t2.x ? 0 : -piece.umin.x);
                    } else {
                        ux = 0.0 < piece.t1.x ? -piece.umin.x : (0.0 < piece.t2.x ? 0 : piece.umax.x);
                    }
                } else { // For tt > 0, dt is T_ / (num_points -1)
                     double prev_dt = T_ / (num_points -1); // dt used in previous step to reach current state
                     // ux, uy, uz used for previous step (tt-1) need to be calculated based on (tt-1)*dt
                     double prev_theta = T_ * (tt-1) / (num_points -1);
                     double prev_ux, prev_uy, prev_uz;
                     if (isEqualFloat(case_idx.x, 0)) {
                         prev_ux = prev_theta < piece.t1.x ? piece.umax.x : (prev_theta < piece.t2.x ? 0 : -piece.umin.x);
                     } else {
                         prev_ux = prev_theta < piece.t1.x ? -piece.umin.x : (prev_theta < piece.t2.x ? 0 : piece.umax.x);
                     }
                    x = x + vx * prev_dt + 0.5 * prev_ux * prev_dt * prev_dt; // x at current theta_
                    vx = vx + prev_ux * prev_dt; // vx at current theta_
                    // Now determine u for current theta_ to be stored as acc_sample
                    if (isEqualFloat(case_idx.x, 0)) {
                        ux = theta_ < piece.t1.x ? piece.umax.x : (theta_ < piece.t2.x ? 0 : -piece.umin.x);
                    } else {
                        ux = theta_ < piece.t1.x ? -piece.umin.x : (theta_ < piece.t2.x ? 0 : piece.umax.x);
                    }
                }
                thetaPoint[0] = x;
                thetaDot[0] = vx;
                thetaDotDot[0] = ux;
            }
            // y轴 (similar logic for tt=0 and iterative update)
            if (isEqualFloat(case_idx.y, 2)) {
                thetaPoint[1] = piece.x0.y + theta_ * v_line_y;
                thetaDot[1] = v_line_y;
                thetaDotDot[1] = 0;
            }
            else {
                if (tt == 0) {
                    y = piece.x0.y;
                    vy = piece.v0.y;
                    if (isEqualFloat(case_idx.y, 0)) {
                        uy = 0.0 < piece.t1.y ? piece.umax.y : (0.0 < piece.t2.y ? 0 : -piece.umin.y);
                    } else {
                        uy = 0.0 < piece.t1.y ? -piece.umin.y : (0.0 < piece.t2.y ? 0 : piece.umax.y);
                    }
                } else {
                    double prev_dt = T_ / (num_points -1);
                    double prev_theta = T_ * (tt-1) / (num_points -1);
                    double prev_uy;
                     if (isEqualFloat(case_idx.y, 0)) {
                         prev_uy = prev_theta < piece.t1.y ? piece.umax.y : (prev_theta < piece.t2.y ? 0 : -piece.umin.y);
                     } else {
                         prev_uy = prev_theta < piece.t1.y ? -piece.umin.y : (prev_theta < piece.t2.y ? 0 : piece.umax.y);
                     }
                    y = y + vy * prev_dt + 0.5 * prev_uy * prev_dt * prev_dt;
                    vy = vy + prev_uy * prev_dt;
                    if (isEqualFloat(case_idx.y, 0)) {
                        uy = theta_ < piece.t1.y ? piece.umax.y : (theta_ < piece.t2.y ? 0 : -piece.umin.y);
                    } else {
                        uy = theta_ < piece.t1.y ? -piece.umin.y : (theta_ < piece.t2.y ? 0 : piece.umax.y);
                    }
                }
                thetaPoint[1] = y;
                thetaDot[1] = vy;
                thetaDotDot[1] = uy;
            }
            // z轴 (similar logic for tt=0 and iterative update)
            if (isEqualFloat(case_idx.z, 2)) {
                thetaPoint[2] = piece.x0.z + theta_ * v_line_z;
                thetaDot[2] = v_line_z;
                thetaDotDot[2] = 0;
            }
            else {
                 if (tt == 0) {
                    z = piece.x0.z;
                    vz = piece.v0.z;
                     if (isEqualFloat(case_idx.z, 0)) {
                        uz = 0.0 < piece.t1.z ? piece.umax.z : (0.0 < piece.t2.z ? 0 : -piece.umin.z);
                    } else {
                        uz = 0.0 < piece.t1.z ? -piece.umin.z : (0.0 < piece.t2.z ? 0 : piece.umax.z);
                    }
                } else {
                    double prev_dt = T_ / (num_points -1);
                    double prev_theta = T_ * (tt-1) / (num_points -1);
                    double prev_uz;
                    if (isEqualFloat(case_idx.z, 0)) {
                         prev_uz = prev_theta < piece.t1.z ? piece.umax.z : (prev_theta < piece.t2.z ? 0 : -piece.umin.z);
                     } else {
                         prev_uz = prev_theta < piece.t1.z ? -piece.umin.z : (prev_theta < piece.t2.z ? 0 : piece.umax.z);
                     }
                    z = z + vz * prev_dt + 0.5 * prev_uz * prev_dt * prev_dt;
                    vz = vz + prev_uz * prev_dt;
                    if (isEqualFloat(case_idx.z, 0)) {
                        uz = theta_ < piece.t1.z ? piece.umax.z : (theta_ < piece.t2.z ? 0 : -piece.umin.z);
                    } else {
                        uz = theta_ < piece.t1.z ? -piece.umin.z : (theta_ < piece.t2.z ? 0 : piece.umax.z);
                    }
                }
                thetaPoint[2] = z;
                thetaDot[2] = vz;
                thetaDotDot[2] = uz;
            }

            theta_sample.push_back(theta_ + T_accumulated);
            pos_sample.push_back(thetaPoint);
            vel_sample.push_back(thetaDot);
            acc_sample.push_back(thetaDotDot);
        }
        T_accumulated += T_;
        segment_end_times_.push_back(T_accumulated);
        segment_end_positions_.push_back(Eigen::Vector3d(piece.xT.x, piece.xT.y, piece.xT.z));
    }
    } // unique_lock releases
    can_modify = false;
}

// ... (findNearestTheta functions remain unchanged) ...
double Map::findNearestTheta(double theta, Eigen::Vector3d & position){
    int index = 0;
    int left = 0;
    int right = theta_sample.size()-1;
    double distance = 0;
    double nearestTheta = 0;
    double distanceMin = 1000;
    double error = 10;
    // 二分法找传入的theta对应的下标
    if (theta_sample.empty()) return 0.0; // Guard against empty samples

    while(fabs(error) > 0.005 && (right - left > 1)){ // Add boundary check for binary search
        if (index < 0 || index >= theta_sample.size()) index = (left+right)/2; // Ensure index is valid
        error = theta_sample[index] - theta;
        if(error > 0){
            right = index;
        }
        else{
            left = index;
        }
        index = (left+right)/2; // Update index based on new left/right
    }
    // Fallback index if binary search didn't converge well or range is too small
    if (fabs(theta_sample[index] - theta) > fabs(theta_sample[left] - theta)) index = left;
    if (fabs(theta_sample[index] - theta) > fabs(theta_sample[right] - theta)) index = right;


    // 依次以0.1, 0.01, 0.001为间隔找最接近position的theta
    // Ensure pos_sample is not empty and index is valid
    if (pos_sample.empty()) return 0.0;

    for (int i=((index-1000)>0? index-1000:0); i<(index+1000<theta_sample.size()? index+1000:theta_sample.size()-1); i+=100){
        if (i < 0 || i >= pos_sample.size()) continue;
        distance = (position - pos_sample[i]).squaredNorm();
        if(distance < distanceMin){
            distanceMin = distance;
            nearestTheta = theta_sample[i];
            index = i; // Update index to keep search local
        }
    }
    for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
        if (i < 0 || i >= pos_sample.size()) continue;
        distance = (position - pos_sample[i]).squaredNorm();
        if(distance < distanceMin){
            distanceMin = distance;
            nearestTheta = theta_sample[i];
            index = i; // Update index
        }
    }
    for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
        if (i < 0 || i >= pos_sample.size()) continue;
        distance = (position - pos_sample[i]).squaredNorm();
        if(distance < distanceMin){
            distanceMin = distance;
            nearestTheta = theta_sample[i];
        }
    }
    return nearestTheta;
};

double Map::findNearestTheta(Eigen::Vector3d & position){
    int index = 0; // Initialize index
    double distance = 0;
    double nearestTheta = 0;
    double distanceMin = 10000; // Increased initial distanceMin

    if (theta_sample.empty() || pos_sample.empty()) return 0.0; // Guard

    // sampling method for finding the nearest point on the trajectory
    // 依次以0.1, 0.01, 0.001为间隔找最接近position的theta
    for (int i=0; i<theta_sample.size(); i+=100){
        distance = (position - pos_sample[i]).squaredNorm();
        if(distance < distanceMin){
            distanceMin = distance;
            nearestTheta = theta_sample[i];
            index = i;
        }
    }
    // Ensure index is valid before proceeding to finer searches
    if (theta_sample.size() == 0) return nearestTheta; // if no samples, return 0 or last found

    for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
         if (i < 0 || i >= pos_sample.size()) continue; // bounds check
        distance = (position - pos_sample[i]).squaredNorm();
        if(distance < distanceMin){
            distanceMin = distance;
            nearestTheta = theta_sample[i];
            index = i;
        }
    }
    for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
        if (i < 0 || i >= pos_sample.size()) continue; // bounds check
        distance = (position - pos_sample[i]).squaredNorm();
        if(distance < distanceMin){
            distanceMin = distance;
            nearestTheta = theta_sample[i];
        }
    }
    // cout << "position: " << position(0) << ", " << position(1) << ", " << position(2) << ".   nearestTheta: " << nearestTheta << endl;
    return nearestTheta;
}


void Map::getGlobalCommand(double t, Vector3d & position){
    {
    std::lock_guard<std::mutex> lock(mtx);
    if (theta_sample.empty() || pos_sample.empty()) { // Guard for empty samples
        position.setZero(); // Default value
        can_modify = true;
        cv.notify_all();
        return;
    }
    // Binary search for the closest index
    auto it = lower_bound(theta_sample.begin(), theta_sample.end(), t);
    int idx = distance(theta_sample.begin(), it);

    // Check if we need to adjust idx to ensure it's within bounds
    if (idx >= theta_sample.size()) { // Use >= to handle t > last element
        idx = theta_sample.size() - 1;
    }
    if (idx < 0) idx = 0; // Should not happen if not empty

    position = pos_sample[idx];
    }
    can_modify = true;
    cv.notify_all(); // 通知所有等待的线程
}

void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity){
    {
    std::lock_guard<std::mutex> lock(mtx);
    if (theta_sample.empty() || pos_sample.empty() || vel_sample.empty()) { // Guard
        position.setZero();
        velocity.setZero();
        can_modify = true;
        cv.notify_all();
        return;
    }
    // Binary search for the closest index
    auto it = lower_bound(theta_sample.begin(), theta_sample.end(), t);
    int idx = distance(theta_sample.begin(), it);

    // Check if we need to adjust idx to ensure it's within bounds
    if (idx >= theta_sample.size()) {
        idx = theta_sample.size() - 1;
    }
     if (idx < 0) idx = 0;

    position = pos_sample[idx];
    velocity = vel_sample[idx];
    }
    can_modify = true;
    cv.notify_all(); // 通知所有等待的线程
}

void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity, Vector3d & acceleration){
    {
    std::lock_guard<std::mutex> lock(mtx);
     if (theta_sample.empty() || pos_sample.empty() || vel_sample.empty() || acc_sample.empty()) { // Guard
        position.setZero();
        velocity.setZero();
        acceleration.setZero();
        can_modify = true;
        cv.notify_all();
        return;
    }
    // Binary search for the closest index
    auto it = lower_bound(theta_sample.begin(), theta_sample.end(), t);
    int idx = distance(theta_sample.begin(), it);

    // Check if we need to adjust idx to ensure it's within bounds
    if (idx >= theta_sample.size()) {
        idx = theta_sample.size() - 1;
    }
    if (idx < 0) idx = 0;

    position = pos_sample[idx];
    velocity = vel_sample[idx];
    acceleration = acc_sample[idx];
    }
    can_modify = true;
    cv.notify_all(); // 通知所有等待的线程
}

double Map::getYaw(double t){
    Eigen::Vector3d current_position;
    Eigen::Vector3d target_segment_end_position;
    Eigen::Vector3d velocity_at_t; // For fallback if current_pos is at target_pos
    bool data_is_valid = false;

    { // Lock scope for reading shared data
        std::lock_guard<std::mutex> lock(mtx);

        // Check if necessary data structures are populated
        if (theta_sample.empty() || pos_sample.empty() || vel_sample.empty() ||
            segment_end_times_.empty() || segment_end_positions_.empty()) {
            // Data not sufficiently initialized, will return 0.0 yaw later
        } else {
            data_is_valid = true;

            // 1. Find current state (position and velocity) at time t from samples
            auto it_sample = lower_bound(theta_sample.begin(), theta_sample.end(), t);
            int sample_idx = distance(theta_sample.begin(), it_sample);

            if (sample_idx >= theta_sample.size()) { // If t is >= last sampled time
                sample_idx = theta_sample.size() - 1;
            }
            // Ensure sample_idx is non-negative (it should be if theta_sample is not empty)
            if (sample_idx < 0) sample_idx = 0; 
            
            current_position = pos_sample[sample_idx];
            velocity_at_t = vel_sample[sample_idx];

            // 2. Determine the current segment and its target end position
            // segment_end_times_ stores accumulated end times: [T_seg0_end, T_seg1_end, ..., T_total]
            auto it_seg_end_time = lower_bound(segment_end_times_.begin(), segment_end_times_.end(), t);
            int current_segment_idx = distance(segment_end_times_.begin(), it_seg_end_time);

            // If t is beyond the total trajectory time, or if lower_bound returns end(),
            // use the end position of the very last segment.
            if (current_segment_idx >= segment_end_positions_.size()) {
                current_segment_idx = segment_end_positions_.size() - 1;
            }
            // Ensure index is valid (it should be if segment_end_positions_ is not empty)
            if (current_segment_idx < 0) current_segment_idx = 0; 


            target_segment_end_position = segment_end_positions_[current_segment_idx];
        }
    } // Mutex scope ends, lock is released.

    // Signal that modification can occur, regardless of data validity for this read
    can_modify = true;
    cv.notify_all();

    if (!data_is_valid) {
        return 0.0; // Return default yaw (0) if data was not ready
    }

    // 3. Calculate yaw angle
    Eigen::Vector3d direction_to_end = target_segment_end_position - current_position;

    // Check if the 2D projection of direction_to_end is very small
    // (i.e., current XY position is very close to target XY position)
    double dx = direction_to_end.x();
    double dy = direction_to_end.y();

    if (std::sqrt(dx * dx + dy * dy) < EPS) {
        // Current XY position is (nearly) at the target XY.
        // Fallback to yaw from current velocity's XY components, if velocity is significant.
        double vx = velocity_at_t.x();
        double vy = velocity_at_t.y();
        if (std::sqrt(vx * vx + vy * vy) > EPS) {
            return std::atan2(vy, vx);
        } else {
            // Velocity in XY plane is also negligible. Yaw is ill-defined.
            // Return 0.0 or a previously known good yaw. For simplicity, 0.0.
            return 0.0;
        }
    }

    return std::atan2(dy, dx);
}

// return true if a == b in double, else false
bool Map::isEqualFloat(double a, double b) {
    return fabs(a - b) < EPS ? true : false;
}


} //namespace ft