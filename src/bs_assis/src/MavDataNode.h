#pragma once
/*
    本文件是MavData的头文件定义
*/

// 生成的消息类型头文件
#include "MavDataPubSubTypes.h"

#include "DDS_NodeBase.h"

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

extern ros::V_Publisher velPubs;
extern ros::V_Publisher posPubs;
extern mavros_msgs::HomePosition home_pos;

struct mav_data_struct
{
    geometry_msgs::PoseStamped p;
    geometry_msgs::TwistStamped t;
    sensor_msgs::NavSatFix g;
};

const double EARTH_RADIUS = 6371000; // 地球半径，单位：米

geometry_msgs::Point calculateRelativePosition(const float& global_pos_latitude, const float& global_pos_longitude, const float& global_pos_altitude, const mavros_msgs::HomePosition& hp);
geometry_msgs::Point calculateRelativePositionGeoLib(const double& global_pos_latitude, const double& global_pos_longitude, const double& global_pos_altitude, const mavros_msgs::HomePosition& hp);

class MavDataSubscriber : public DDS_Subscriber
{
    // 接收，数据可用的时候
    void on_data_available(DataReader* reader) override
    {
        SampleInfo info;
        if (reader->take_next_sample(&recvData, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {
                geometry_msgs::PoseStamped posMsg;
                geometry_msgs::TwistStamped velMsg;
                // geometry_msgs::Point relative_pos = calculateRelativePosition(
                //     recvData.latitude(), recvData.longitude(), recvData.altitude(), home_pos);
                // std::cout << "relative_pos_ori: " << relative_pos.x << ", " << relative_pos.y << ", " << relative_pos.z << std::endl;
                geometry_msgs::Point relative_pos = calculateRelativePositionGeoLib(
                    recvData.latitude(), recvData.longitude(), recvData.altitude(), home_pos);
                // std::cout << "relative_pos_GeoLib: " << relative_pos.x << ", " << relative_pos.y << ", " << relative_pos.z << std::endl;
                
                posMsg.pose.position.x  = relative_pos.x;
                posMsg.pose.position.y  = relative_pos.y;
                posMsg.pose.position.z  = relative_pos.z;


                posMsg.pose.orientation.w  = recvData.quat_w();
                posMsg.pose.orientation.x  = recvData.quat_x();
                posMsg.pose.orientation.y  = recvData.quat_y();
                posMsg.pose.orientation.z  = recvData.quat_z();

                velMsg.twist.linear.x   = recvData.vel_x();
                velMsg.twist.linear.y   = recvData.vel_y();
                velMsg.twist.linear.z   = recvData.vel_z();
                velMsg.twist.angular.x  = recvData.ang_x();
                velMsg.twist.angular.y  = recvData.ang_y();
                velMsg.twist.angular.z  = recvData.ang_z();
                                
                // samples_++;
                // std::cout << " recv system_ID: " << recvData.system_ID() << std::endl;
                posPubs.at(recvData.system_ID()).publish(posMsg);
                velPubs.at(recvData.system_ID()).publish(velMsg);
            }
        }
    }

    MavData recvData;
};




class MavDataNode
{
public:

    MavDataNode();
    ~MavDataNode();

    bool init();
    bool publish(MavData* sendData);

private:
    DomainParticipant* participant_ = nullptr;

    Topic* topic_;
    TypeSupport type_;

    Subscriber* subscriber_;
    DataReader* reader_;

    Publisher* publisher_;
    DataWriter* writer_;
    
    DDS_Publisher dwListener_;
    MavDataSubscriber drListener_;

};
