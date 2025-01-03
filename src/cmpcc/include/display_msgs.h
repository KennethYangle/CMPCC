#ifndef PROJECT_DISPLAT_MSGS_H
#define PROJECT_DISPLAT_MSGS_H

#include "map.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_msgs/Polyhedron.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Sparse>

namespace ft{
    class DisplayMsgs
    {
    private:
        int horizon;
        geometry_msgs::Point pt, nm;
    public:
        ft::Map &map;
        nav_msgs::Path refTraj_msg;
        nav_msgs::Path trajPred_msg;
        visualization_msgs::Marker drone_msg;
        visualization_msgs::Marker theta_msg;

        DisplayMsgs(Map &map_, int horizon_);
        void displayRefTraj();
        void displayDrone(Eigen::SparseMatrix<double> &state);
        void displayTheta(Eigen::SparseMatrix<double> &state);
        void displayPredict(Eigen::SparseMatrix<double> &statePredict);
    };
    
}//namespace ft

#endif