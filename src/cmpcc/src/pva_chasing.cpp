#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PiecewiseBezier.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "map.h"

ros::Publisher cmd_pub, drone_pub, cmd_vis_pub, refer_pub;
ft::Map map;
double cmdT = 0.02;
Eigen::Vector3d pDrone, vDrone;
quadrotor_msgs::PositionCommand cmdMsg;
bool is_get_pos, is_get_path;
geometry_msgs::Point tmpPoint;
geometry_msgs::Vector3 tmpVector;
nav_msgs::Path refTraj_msg;

void displayRefTraj(){
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

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    is_get_pos = true;
    pDrone << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vDrone << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
}

void refer_path_params_callback(const swarm_msgs::TimeOptimalPMMPieces::ConstPtr& msg) {
    is_get_path = true;
    map.setPathPts(msg);
    displayRefTraj();
    refer_pub.publish(refTraj_msg);
}

void cmd_callback(const ros::TimerEvent& event) {
    if (!is_get_path || !is_get_pos) {
        std::cout << "is_get_path: " << is_get_path << ", is_get_pos: " << is_get_pos << std::endl;
        return;
    }

    double theta_now = map.findNearestTheta(pDrone);
    double theta_chase = theta_now + cmdT;
    Eigen::Vector3d position_d, velocity_d, acceleration_d;
    map.getGlobalCommand(theta_chase, position_d, velocity_d, acceleration_d);
    
    tmpPoint.x = position_d[0];
    tmpPoint.y = position_d[1];
    tmpPoint.z = position_d[2];
    cmdMsg.position = tmpPoint;
    tmpVector.x = velocity_d[0];
    tmpVector.y = velocity_d[1];
    tmpVector.z = velocity_d[2];
    cmdMsg.velocity = tmpVector;
    tmpVector.x = acceleration_d[0];
    tmpVector.y = acceleration_d[1];
    tmpVector.z = acceleration_d[2];
    cmdMsg.acceleration = tmpVector;
    cmdMsg.yaw = map.getYaw(theta_chase);
    cmdMsg.header.stamp = ros::Time::now();
    cmd_pub.publish(cmdMsg);

    // Visualization of drone position and velocity
    visualization_msgs::Marker drone_marker;
    drone_marker.header.frame_id = "world";
    drone_marker.header.stamp = ros::Time::now();
    drone_marker.ns = "drone";
    drone_marker.id = 0;
    drone_marker.type = visualization_msgs::Marker::ARROW;
    drone_marker.action = visualization_msgs::Marker::ADD;
    drone_marker.pose.orientation.w = 1.0;  // No rotation, aligned with world axis
    drone_marker.scale.x = 0.1;  // Arrow length
    drone_marker.scale.y = 0.02; // Arrow shaft diameter
    drone_marker.scale.z = 0.02; // Arrow head diameter
    drone_marker.color.a = 1.0;  // Alpha
    drone_marker.color.r = 1.0;  // Red for drone velocity
    drone_marker.color.g = 0.0;
    drone_marker.color.b = 0.0;
    geometry_msgs::Point start_point;
    start_point.x = pDrone.x();
    start_point.y = pDrone.y();
    start_point.z = pDrone.z();
    drone_marker.points.push_back(start_point);  // Start of the arrow (drone position)
    geometry_msgs::Point end_point;
    end_point.x = pDrone.x() + vDrone[0];
    end_point.y = pDrone.y() + vDrone[1];
    end_point.z = pDrone.z() + vDrone[2];
    drone_marker.points.push_back(end_point);  // End of the arrow (drone position + velocity)
    drone_pub.publish(drone_marker);

    // Visualization of control command position and velocity
    visualization_msgs::Marker cmd_marker;
    cmd_marker.header.frame_id = "world";
    cmd_marker.header.stamp = ros::Time::now();
    cmd_marker.ns = "cmd";
    cmd_marker.id = 1;
    cmd_marker.type = visualization_msgs::Marker::ARROW;
    cmd_marker.action = visualization_msgs::Marker::ADD;
    cmd_marker.pose.orientation.w = 1.0;  // No rotation, aligned with world axis
    cmd_marker.scale.x = 0.1;  // Arrow length
    cmd_marker.scale.y = 0.02; // Arrow shaft diameter
    cmd_marker.scale.z = 0.02; // Arrow head diameter
    cmd_marker.color.a = 1.0;  // Alpha
    cmd_marker.color.r = 0.0;
    cmd_marker.color.g = 0.0;
    cmd_marker.color.b = 1.0;  // Blue for command velocity
    geometry_msgs::Point cmd_start_point;
    cmd_start_point.x = position_d[0];
    cmd_start_point.y = position_d[1];
    cmd_start_point.z = position_d[2];
    cmd_marker.points.push_back(cmd_start_point);  // Start of the arrow (command position)
    geometry_msgs::Point cmd_end_point;
    cmd_end_point.x = position_d[0] + velocity_d[0];
    cmd_end_point.y = position_d[1] + velocity_d[1];
    cmd_end_point.z = position_d[2] + velocity_d[2];
    cmd_marker.points.push_back(cmd_end_point);  // End of the arrow (command position + velocity)
    cmd_vis_pub.publish(cmd_marker);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pva_chasing_node");
    ros::NodeHandle nodeHandle;
    std::cout << "Init node." << std::endl;

    // Publisher
    cmd_pub = nodeHandle.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);
    drone_pub = nodeHandle.advertise<visualization_msgs::Marker>("drone_pose", 1);
    cmd_vis_pub = nodeHandle.advertise<visualization_msgs::Marker>("cmd_vis", 1);
    refer_pub = nodeHandle.advertise<nav_msgs::Path>("refer_path", 1);

    // Subscriber
    ros::Subscriber sub_path = nodeHandle.subscribe("/refer_path_params", 5, refer_path_params_callback);
    ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber local_vel_sub = nodeHandle.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, local_vel_cb, ros::TransportHints().tcpNoDelay());

    // Timer
    ros::Timer timer_cmd = nodeHandle.createTimer(ros::Duration(cmdT), cmd_callback);
    std::cout << "Publisher and Subscriber." << std::endl;

    ros::MultiThreadedSpinner spinner(4);
    cmdMsg.header.frame_id = "world";
    refTraj_msg.header.frame_id = "world";

    // init position: 
    tmpPoint.x = 0; tmpPoint.y = 0; tmpPoint.z = 0; cmdMsg.position = tmpPoint;
    tmpVector.x = 0; tmpVector.y = 0; tmpVector.z = 0; cmdMsg.velocity = tmpVector;
    tmpVector.x = 0; tmpVector.y = 0; tmpVector.z = 0; cmdMsg.acceleration = tmpVector;
    cmdMsg.yaw_dot = 0;

    ros::Rate loopRate(10);
    ros::Time startT = ros::Time::now();
    while (ros::ok()) {
        ros::Time nowT = ros::Time::now();
        cmd_pub.publish(cmdMsg);
        if ((nowT - startT).toSec() > 3) {
            break;
        }
        loopRate.sleep();
    }

    spinner.spin();
    return 0;
}
