#ifndef PROJECT_MAP_H
#define PROJECT_MAP_H

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "corridor.h"
#include "bezier_base.h"
#include <quadrotor_msgs/PiecewiseBezier.h>

namespace ft {
//    class Map
    class Map {
    private:
        int num_order, num_segment, traj_order, K_max;
        double s_step, global_traj_time;
        std::vector<int> K_data_my;
        std::vector<double> range, K_data;
        std::vector<double> theta_sample;
        std::vector<Eigen::Vector3d> pos_sample;
        Eigen::MatrixXd coef, time, time_acc, a_data, b_data, s_data;
        std::vector<Eigen::MatrixXd> control_points;

        Bernstein bezier_basis = Bernstein(3.0);
        
    public:
        double thetaMax;
        Corridor corridor;

        Map();
        void setPathPts(const quadrotor_msgs::PiecewiseBezier::ConstPtr& msg);
        double findNearestTheta(Eigen::Vector3d & position);
        double findNearestTheta(double theta, Eigen::Vector3d & position);
        
        void getGlobalCommand(double t, 
            Eigen::Vector3d & position);
        void getGlobalCommand(double t, 
            Eigen::Vector3d & position, 
            Eigen::Vector3d & velocity);
        void getGlobalCommand(double t,
            Eigen::Vector3d & position, 
            Eigen::Vector3d & velocity, 
            Eigen::Vector3d & acceleration);
        double getYaw(double theta);
    };
}

#endif //PROJECT_MAP_H
