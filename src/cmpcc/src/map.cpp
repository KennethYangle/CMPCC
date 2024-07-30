#include "map.h"

using namespace std;
using namespace Eigen;
namespace ft{
    Map::Map(){}

    void Map::setPathPts(const swarm_msgs::TimeOptimalPMMPieces::ConstPtr& msg) {
        {
        std::unique_lock<std::mutex> lock(mtx);
        // 等待直到 can_modify 为 true
        cv.wait(lock, [this]{ return can_modify; });

        theta_sample.clear();
        pos_sample.clear();
        vel_sample.clear();
        acc_sample.clear();

        num_segment = msg->num_segment;
        thetaMax = msg->T_tatal;
        double T_accumulated = 0;

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
                double dt = T_ / num_points;
                Eigen::Vector3d thetaPoint, thetaDot, thetaDotDot;

                // x轴
                if (isEqualFloat(case_idx.x, 2)) {
                    thetaPoint[0] = theta_ * v_line_x;
                    thetaDot[0] = v_line_x;
                    thetaDotDot[0] = 0;
                }
                else {
                    x = x + vx * dt + 0.5 * ux * dt * dt;
                    vx = vx + ux * dt;
                    //                     phase 1,                             phase 2, phase 3
                    ux = theta_ < piece.t1.x ? piece.umax.x : (theta_ < piece.t2.x ? 0 : -piece.umin.x);
                    thetaPoint[0] = x;
                    thetaDot[0] = vx;
                    thetaDotDot[0] = ux;
                }
                // y轴
                if (isEqualFloat(case_idx.y, 2)) {
                    thetaPoint[1] = theta_ * v_line_y;
                    thetaDot[1] = v_line_y;
                    thetaDotDot[1] = 0;
                }
                else {
                    y = y + vy * dt + 0.5 * uy * dt * dt;
                    vy = vy + uy * dt;
                    //                     phase 1,                             phase 2, phase 3
                    uy = theta_ < piece.t1.y ? piece.umax.y : (theta_ < piece.t2.y ? 0 : -piece.umin.y);
                    thetaPoint[1] = y;
                    thetaDot[1] = vy;
                    thetaDotDot[1] = uy;
                }
                // z轴
                if (isEqualFloat(case_idx.z, 2)) {
                    thetaPoint[2] = theta_ * v_line_z;
                    thetaDot[2] = v_line_z;
                    thetaDotDot[2] = 0;
                }
                else {
                    z = z + vz * dt + 0.5 * uz * dt * dt;
                    vz = vz + uz * dt;
                    //                     phase 1,                             phase 2, phase 3
                    uz = theta_ < piece.t1.z ? piece.umax.z : (theta_ < piece.t2.z ? 0 : -piece.umin.z);
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
        }
        }
        can_modify = false;
    }

    double Map::findNearestTheta(double theta, Eigen::Vector3d & position){
        int index = 0;
        int left = 0;
        int right = theta_sample.size()-1;
        double distance = 0;
        double nearestTheta = 0;
        double distanceMin = 1000;
        double error = 10;
        // 二分法找传入的theta对应的下标
        while(fabs(error) > 0.005){
            error = theta_sample[index] - theta;
            if(error > 0){
                right = index;
                index = (index+left)/2;
            }
            else{
                left = index;
                index = (index+right)/2;
            }
        }
        // 依次以0.1, 0.01, 0.001为间隔找最接近position的theta
        for (int i=((index-1000)>0? index-1000:0); i<(index+1000<theta_sample.size()? index+1000:theta_sample.size()-1); i+=100){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
            }
        }
        return nearestTheta;
    };

    // 通过p找theta
    double Map::findNearestTheta(Eigen::Vector3d & position){
        int index;
        double distance = 0;
        double nearestTheta = 0;
        double distanceMin = 10;
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
        for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
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
        t = std::min(t, thetaMax);     // 保护不越界

        {
        std::lock_guard<std::mutex> lock(mtx);
        // Binary search for the closest index
        auto it = lower_bound(theta_sample.begin(), theta_sample.end(), t);
        int idx = distance(theta_sample.begin(), it);

        // Check if we need to adjust idx to ensure it's within bounds
        if (idx == theta_sample.size()) {
            idx = theta_sample.size() - 1;
        } else if (idx > 0 && t - theta_sample[idx - 1] < theta_sample[idx] - t) {
            idx--;
        }

        position = pos_sample[idx];
        }
        can_modify = true;
        cv.notify_all(); // 通知所有等待的线程
    }

    void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity){
        t = std::min(t, thetaMax);     // 保护不越界

        {
        std::lock_guard<std::mutex> lock(mtx);
        // Binary search for the closest index
        auto it = lower_bound(theta_sample.begin(), theta_sample.end(), t);
        int idx = distance(theta_sample.begin(), it);

        // Check if we need to adjust idx to ensure it's within bounds
        if (idx == theta_sample.size()) {
            idx = theta_sample.size() - 1;
        } else if (idx > 0 && t - theta_sample[idx - 1] < theta_sample[idx] - t) {
            idx--;
        }

        position = pos_sample[idx];
        velocity = vel_sample[idx];
        }
        can_modify = true;
        cv.notify_all(); // 通知所有等待的线程
    }
    
    void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity, Vector3d & acceleration){
        t = std::min(t, thetaMax);     // 保护不越界

        {
        std::lock_guard<std::mutex> lock(mtx);
        // Binary search for the closest index
        auto it = lower_bound(theta_sample.begin(), theta_sample.end(), t);
        int idx = distance(theta_sample.begin(), it);

        // Check if we need to adjust idx to ensure it's within bounds
        if (idx == theta_sample.size()) {
            idx = theta_sample.size() - 1;
        } else if (idx > 0 && t - theta_sample[idx - 1] < theta_sample[idx] - t) {
            idx--;
        }

        position = pos_sample[idx];
        velocity = vel_sample[idx];
        acceleration = acc_sample[idx];
        }
        can_modify = true;
        cv.notify_all(); // 通知所有等待的线程
    }

    double Map::getYaw(double t){
        t = std::min(t, thetaMax);     // 保护不越界
        Eigen::Vector3d velocity;

        {
        std::lock_guard<std::mutex> lock(mtx);
        // Binary search for the closest index
        auto it = lower_bound(theta_sample.begin(), theta_sample.end(), t);
        int idx = distance(theta_sample.begin(), it);

        // Check if we need to adjust idx to ensure it's within bounds
        if (idx == theta_sample.size()) {
            idx = theta_sample.size() - 1;
        } else if (idx > 0 && t - theta_sample[idx - 1] < theta_sample[idx] - t) {
            idx--;
        }

        velocity = vel_sample[idx];
        }
        can_modify = true;
        cv.notify_all(); // 通知所有等待的线程
        return std::atan2(velocity[1], velocity[0]);
    }

    // return true if a == b in double, else false
    bool Map::isEqualFloat(double a, double b) {
        return fabs(a - b) < EPS ? true : false;
    }

} //namespace ft
