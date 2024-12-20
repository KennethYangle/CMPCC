#pragma once
#include "MavDataNode.h" 
#include "MassPointsDataNode.h" 


// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include "swarm_msgs/MassPoint.h"
#include "swarm_msgs/MassPoints.h"


#include <vector>
#include <thread>
#include <sstream>




// 接收从dds来的数据
std::vector<mav_data_struct> mavDataVec;
std::vector<swarm_msgs::MassPoints> masspointsDataVec;




ros::V_Publisher posPubs;
ros::V_Publisher velPubs;
ros::V_Publisher masspointsPubs;
mavros_msgs::State mav_state;
mavros_msgs::HomePosition home_pos;
int count_home_req = 0;

// ros callback本地的状态
void stateCallback(const mavros_msgs::State::ConstPtr& msg) { mav_state = *msg; }
void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, int mav_id) { mavDataVec[mav_id].t = *msg; }
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int mav_id) { mavDataVec[mav_id].p = *msg; }
void globalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg, int mav_id) { 
    mavDataVec[mav_id].g = *msg;
}
void homeCallback(const mavros_msgs::HomePosition::ConstPtr& msg, int mav_id) { 
    count_home_req++;
    if (count_home_req > 10) {
        return;
    }
    home_pos = *msg; 
}

void masspointsCallback(const swarm_msgs::MassPoints::ConstPtr& msg, int mav_id) { masspointsDataVec[mav_id] = *msg; }

// ROS message --> DDS message
void constructMavData(MavData& data,int mav_id)
{
    mav_data_struct tmp = mavDataVec[mav_id];
    data.system_ID(mav_id);

    data.pose_x(tmp.p.pose.position.x);
    data.pose_y(tmp.p.pose.position.y);
    data.pose_z(tmp.p.pose.position.z);

    data.quat_w(tmp.p.pose.orientation.w);
    data.quat_x(tmp.p.pose.orientation.x);
    data.quat_y(tmp.p.pose.orientation.y);
    data.quat_z(tmp.p.pose.orientation.z);


    data.latitude(tmp.g.latitude);
    data.longitude(tmp.g.longitude);
    data.altitude(tmp.g.altitude);


    data.vel_x(tmp.t.twist.linear.x);
    data.vel_y(tmp.t.twist.linear.y);
    data.vel_z(tmp.t.twist.linear.z);

    data.ang_x(tmp.t.twist.angular.x);
    data.ang_y(tmp.t.twist.angular.y);
    data.ang_z(tmp.t.twist.angular.z);

}

// ROS message --> DDS message
MassPointData constructMassPoint(const swarm_msgs::MassPoint msg)
{
    MassPointData data;

    data.pos_x(msg.position.x);
    data.pos_y(msg.position.y);
    data.pos_z(msg.position.z);

    data.vel_x(msg.velocity.x);
    data.vel_y(msg.velocity.y);
    data.vel_z(msg.velocity.z);

    data.volume(msg.volume);
    return data;
}

// ROS message --> DDS message
void constructMassPoints(MassPointsData& data,int mav_id)
{
    swarm_msgs::MassPoints tmp = masspointsDataVec[mav_id];
    data.system_ID(mav_id);

    std::vector<MassPointData> masspointVec;
    for (int i=0; i < tmp.points.size(); i++)
    {
        masspointVec.push_back(constructMassPoint(tmp.points[i]));
    }

    data.points(masspointVec);

}


// dds推送当前的状态
void MavDataUpdate(MavDataNode* dds_node,int mav_id)
{
    while (true)
    {
        MavData data;
        constructMavData(data,mav_id);
        dds_node->publish(&data);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/40));
    }
}

// dds推送当前的状态
void MassPointsDataUpdate(MassPointsDataNode* dds_node,int mav_id)
{
    while (true)
    {
        MassPointsData data;
        constructMassPoints(data, mav_id);
        dds_node->publish(&data);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/25));
    }
}