#include "display_msgs.h"
#include "model.h"
#include "Eigen/Dense"
#include "ros/ros.h"

using namespace std;
namespace ft{
    DisplayMsgs::DisplayMsgs(Map &map_, int horizon_):map(map_), horizon(horizon_){
        // initialize msgs:
        refTraj_msg.header.frame_id = "world";
        trajPred_msg.header.frame_id = "world";
        drone_msg.header.frame_id = "world";
        drone_msg.type = visualization_msgs::Marker::ARROW;
        drone_msg.action = visualization_msgs::Marker::ADD;
        drone_msg.scale.x = 0.06;
        drone_msg.scale.y = 0.1;
        drone_msg.scale.z = 0;
        drone_msg.color.a = 1;
        drone_msg.color.r = 1;
        drone_msg.color.g = 0;
        drone_msg.color.b = 0;
        drone_msg.pose.orientation.w = 1;
        theta_msg = drone_msg;
        theta_msg.color.r = 0;
        theta_msg.color.b = 1;

        // display unchanged msgs
        displayRefTraj();

        // pre malloc for trajPred_msg
        trajPred_msg.poses.resize(horizon);
    }   

    void DisplayMsgs::displayRefTraj(){
        geometry_msgs::PoseStamped tmpPose;
        double theta = 0;
        refTraj_msg.poses.clear();
        while (theta < map.thetaMax)
        {
            Eigen::Vector3d pos;
            map.getGlobalCommand(theta, pos);
            tmpPose.pose.position.x = pos(0);
            tmpPose.pose.position.y = pos(1);
            tmpPose.pose.position.z = pos(2);
            refTraj_msg.poses.push_back(tmpPose);
            theta += 0.01;
        }
    }
    void DisplayMsgs::displayDrone(Eigen::SparseMatrix<double> &state){
        drone_msg.points.clear();
        pt.x = state.coeffRef(0,0);
        pt.y = state.coeffRef(Model::numOrder,0);
        pt.z = state.coeffRef(2*Model::numOrder,0);
        drone_msg.points.push_back(pt);
        pt.x += state.coeffRef(1,0);
        pt.y += state.coeffRef(1+Model::numOrder,0);
        pt.z += state.coeffRef(1+2*Model::numOrder,0);
        drone_msg.points.push_back(pt);
        // drone_msg.header.stamp = ros::Time::now();
    }
    void DisplayMsgs::displayTheta(Eigen::SparseMatrix<double> &state){
        double theta = state.coeffRef(Model::numState-Model::numOrder,0);
        theta_msg.points.clear();
        Eigen::Vector3d pos, vel;
        map.getGlobalCommand(theta, pos, vel);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        theta_msg.points.push_back(pt);
        pt.x += vel(0);
        pt.y += vel(1);
        pt.z += vel(2);
        theta_msg.points.push_back(pt);      
    }
    void DisplayMsgs::displayPredict(Eigen::SparseMatrix<double> &statePredict){
        geometry_msgs::PoseStamped tmpPose;
        for (unsigned int i=0; i<horizon; ++i){
            tmpPose.pose.position.x = statePredict.coeffRef(0,i);
            tmpPose.pose.position.y = statePredict.coeffRef(Model::numOrder,i);
            tmpPose.pose.position.z = statePredict.coeffRef(2*Model::numOrder,i);
            trajPred_msg.poses[i] = tmpPose;
        }
    }
}
