#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
// #include <quadrotor_msgs/PiecewiseBezier.h> // Not needed
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>        // Keep for cmdMsg
#include <geometry_msgs/Vector3.h>      // Keep for cmdMsg
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "map.h" // Include the modified map header
#include <swarm_msgs/DiscreteTrajectory.h> // Include the new message type

ros::Publisher cmd_pub, drone_pub, cmd_vis_pub, refer_pub;
ft::Map map; // Use the modified Map class
double cmdT = 0.02; // Lookahead time
Eigen::Vector3d pDrone, vDrone;
quadrotor_msgs::PositionCommand cmdMsg;
bool is_get_pos = false; // Initialize to false
bool is_get_path = false; // Initialize to false
geometry_msgs::Point tmpPoint;
geometry_msgs::Vector3 tmpVector;
nav_msgs::Path refTraj_msg; // Keep for visualization

// Function to update the visualization path from stored discrete points
void displayRefTraj(){
    const auto& points = map.getTrajectoryPoints(); // Get points from map
    if (points.empty()) {
        refTraj_msg.poses.clear();
        return;
    }

    refTraj_msg.header.stamp = ros::Time::now();
    // Assuming frame_id is consistent, set during initialization
    // refTraj_msg.header.frame_id = points[0]. ? No direct frame_id in point msg

    refTraj_msg.poses.clear();
    refTraj_msg.poses.reserve(points.size()); // Reserve space

    for(const auto& pt : points) {
        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header.stamp = ros::Time(pt.time_from_start); // Optional: use point time
        tmpPose.pose.position = pt.position;
        // Set orientation to default (or calculate from velocity if needed)
        tmpPose.pose.orientation.w = 1.0;
        tmpPose.pose.orientation.x = 0.0;
        tmpPose.pose.orientation.y = 0.0;
        tmpPose.pose.orientation.z = 0.0;
        refTraj_msg.poses.push_back(tmpPose);
    }
     refer_pub.publish(refTraj_msg); // Publish updated path
}

// --- local_pos_cb remains the same ---
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    is_get_pos = true;
    pDrone << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
}

// --- local_vel_cb remains the same ---
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vDrone << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
}

// --- MODIFIED CALLBACK: Receive Discrete Trajectory ---
void discrete_trajectory_callback(const swarm_msgs::DiscreteTrajectory::ConstPtr& msg) {
    ROS_INFO_ONCE("Received first discrete trajectory.");
    map.setTrajectory(msg); // Use the new function in Map class
    is_get_path = true; // Set flag only after successfully processing
    // Update visualization path immediately after receiving a new trajectory
    displayRefTraj();
}

// --- MODIFIED TIMER CALLBACK: Use Time-based Lookahead ---
void cmd_callback(const ros::TimerEvent& event) {
    if (!is_get_path || !is_get_pos) {
        // Add throttle to avoid flooding console if waiting
        ROS_WARN_THROTTLE(2.0, "Waiting for path (%d) or pose (%d)...", is_get_path, is_get_pos);
        return;
    }

    // 1. Find the time on the reference trajectory closest to the drone's current position
    double time_now = map.findNearestTime(pDrone);

    // 2. Calculate the target time by looking ahead
    double time_chase = time_now + cmdT;

    // 3. Clamp the target time to the valid trajectory duration
    double total_time = map.getTotalTime();
    if (time_chase > total_time) {
        time_chase = total_time;
        ROS_WARN_THROTTLE(1.0, "Lookahead time exceeds trajectory duration. Clamping to end.");
    }
    if (time_chase < 0.0) {
         time_chase = 0.0; // Should not happen with positive cmdT, but good practice
    }


    // 4. Get the desired state (P, V, A) at the target time using interpolation
    Eigen::Vector3d position_d, velocity_d, acceleration_d;
    if (!map.getStateAtTime(time_chase, position_d, velocity_d, acceleration_d)) {
        ROS_ERROR("Failed to get state at time %.3f from map.", time_chase);
        // What to do on failure? Hold previous command? Hover?
        // For now, just return and don't publish a new command.
        return;
    }

    // 5. Get the desired yaw at the target time
    double yaw_d = map.getYawAtTime(time_chase);

    // 6. Populate and publish the PositionCommand message
    cmdMsg.header.stamp = ros::Time::now();
    // cmdMsg.header.frame_id = "world"; // Set in main

    tmpPoint.x = position_d.x();
    tmpPoint.y = position_d.y();
    tmpPoint.z = position_d.z();
    cmdMsg.position = tmpPoint;

    tmpVector.x = velocity_d.x();
    tmpVector.y = velocity_d.y();
    tmpVector.z = velocity_d.z();
    cmdMsg.velocity = tmpVector;

    tmpVector.x = acceleration_d.x();
    tmpVector.y = acceleration_d.y();
    tmpVector.z = acceleration_d.z();
    cmdMsg.acceleration = tmpVector;

    cmdMsg.yaw = yaw_d;
    // cmdMsg.yaw_dot = 0; // Assuming yaw dot is handled by controller or not needed

    cmd_pub.publish(cmdMsg);

    // --- Visualization code remains largely the same ---
    // (Uses pDrone, vDrone for drone marker and position_d, velocity_d for cmd marker)
    visualization_msgs::Marker drone_marker;
    drone_marker.header.frame_id = "world";
    drone_marker.header.stamp = ros::Time::now();
    drone_marker.ns = "drone";
    drone_marker.id = 0;
    drone_marker.type = visualization_msgs::Marker::ARROW;
    drone_marker.action = visualization_msgs::Marker::ADD;
    drone_marker.pose.orientation.w = 1.0;
    drone_marker.scale.x = 0.1;
    drone_marker.scale.y = 0.02;
    drone_marker.scale.z = 0.02;
    drone_marker.color.a = 1.0;
    drone_marker.color.r = 1.0;
    drone_marker.color.g = 0.0;
    drone_marker.color.b = 0.0;
    geometry_msgs::Point start_point;
    start_point.x = pDrone.x();
    start_point.y = pDrone.y();
    start_point.z = pDrone.z();
    drone_marker.points.push_back(start_point);
    geometry_msgs::Point end_point;
    // Scale velocity for visualization if needed
    double viz_vel_scale = 0.2;
    end_point.x = pDrone.x() + vDrone.x() * viz_vel_scale;
    end_point.y = pDrone.y() + vDrone.y() * viz_vel_scale;
    end_point.z = pDrone.z() + vDrone.z() * viz_vel_scale;
    drone_marker.points.push_back(end_point);
    drone_pub.publish(drone_marker);

    visualization_msgs::Marker cmd_marker;
    cmd_marker.header.frame_id = "world";
    cmd_marker.header.stamp = ros::Time::now();
    cmd_marker.ns = "cmd";
    cmd_marker.id = 1;
    cmd_marker.type = visualization_msgs::Marker::ARROW;
    cmd_marker.action = visualization_msgs::Marker::ADD;
    cmd_marker.pose.orientation.w = 1.0;
    cmd_marker.scale.x = 0.1;
    cmd_marker.scale.y = 0.02;
    cmd_marker.scale.z = 0.02;
    cmd_marker.color.a = 1.0;
    cmd_marker.color.r = 0.0;
    cmd_marker.color.g = 0.0;
    cmd_marker.color.b = 1.0;
    geometry_msgs::Point cmd_start_point;
    cmd_start_point.x = position_d.x();
    cmd_start_point.y = position_d.y();
    cmd_start_point.z = position_d.z();
    cmd_marker.points.push_back(cmd_start_point);
    geometry_msgs::Point cmd_end_point;
    cmd_end_point.x = position_d.x() + velocity_d.x() * viz_vel_scale;
    cmd_end_point.y = position_d.y() + velocity_d.y() * viz_vel_scale;
    cmd_end_point.z = position_d.z() + velocity_d.z() * viz_vel_scale;
    cmd_marker.points.push_back(cmd_end_point);
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
    ros::Subscriber sub_path = nodeHandle.subscribe<swarm_msgs::DiscreteTrajectory>(
                                    "/discrete_trajectory",
                                    5,
                                    discrete_trajectory_callback);
    ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber local_vel_sub = nodeHandle.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, local_vel_cb, ros::TransportHints().tcpNoDelay());

    // Timer (no changes needed, uses cmdT)
    nodeHandle.param<double>("command_lookahead_time", cmdT, 0.02); // Make lookahead time a parameter
    ros::Timer timer_cmd = nodeHandle.createTimer(ros::Duration(cmdT), cmd_callback);
    ROS_INFO("Publishers, Subscribers, and Timer initialized. Lookahead time: %.3f s", cmdT);

    // Set fixed frame_id for messages
    cmdMsg.header.frame_id = "world"; // Or get from parameter
    refTraj_msg.header.frame_id = "world"; // Or get from parameter

    // --- Initial hover command can be removed or kept as fallback ---
    /*
    tmpPoint.x = 0; tmpPoint.y = 0; tmpPoint.z = 1.0; // Start hover higher?
    cmdMsg.position = tmpPoint;
    tmpVector.x = 0; tmpVector.y = 0; tmpVector.z = 0; cmdMsg.velocity = tmpVector;
    tmpVector.x = 0; tmpVector.y = 0; tmpVector.z = 0; cmdMsg.acceleration = tmpVector;
    cmdMsg.yaw = 0.0;
    cmdMsg.yaw_dot = 0;
    ros::Rate init_rate(10);
    ros::Time startT = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - startT).toSec() < 1.0) { // Hover briefly
        cmdMsg.header.stamp = ros::Time::now();
        cmd_pub.publish(cmdMsg);
        ros::spinOnce(); // Process callbacks during init hover
        init_rate.sleep();
    }
    ROS_INFO("Initial hover complete. Starting trajectory chasing.");
    */

    // Use a MultiThreadedSpinner if callbacks need parallel processing,
    // otherwise SingleThreadedSpinner is usually sufficient.
    ros::spin(); // Process callbacks

    return 0;
}