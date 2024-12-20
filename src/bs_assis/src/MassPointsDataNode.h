#pragma once
/*
    本文件是MassPoints的头文件定义
*/

// 生成的消息类型头文件
#include "MassPointsDataPubSubTypes.h"

#include "DDS_NodeBase.h"

// ros
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "swarm_msgs/MassPoint.h"
#include "swarm_msgs/MassPoints.h"

#include <string>


extern ros::V_Publisher masspointsPubs;


// DDS message --> ROS message
class MassPointsDataSubscriber : public DDS_Subscriber
{
    // 接收，数据可用的时候
    void on_data_available(DataReader* reader) override
    {
        SampleInfo info;
        if (reader->take_next_sample(&recvData, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (info.valid_data)
            {
                swarm_msgs::MassPoints masspointsMsg;
                
                for (size_t i = 0; i < recvData.points().size(); i++)
                {
                    swarm_msgs::MassPoint masspointMsg;
                    auto masspoint_data = recvData.points()[i];

                    masspointMsg.position.x = masspoint_data.pos_x();
                    masspointMsg.position.y = masspoint_data.pos_y();
                    masspointMsg.position.z = masspoint_data.pos_z();

                    masspointMsg.velocity.x = masspoint_data.vel_x();
                    masspointMsg.velocity.y = masspoint_data.vel_y();
                    masspointMsg.velocity.z = masspoint_data.vel_z();

                    masspointMsg.volume = masspoint_data.volume();

                    masspointsMsg.points.push_back(masspointMsg);
                }
                masspointsPubs[recvData.system_ID()].publish(masspointsMsg);
            }
        }
    }

    MassPointsData  recvData;
};




class MassPointsDataNode
{
public:

    MassPointsDataNode();
    ~MassPointsDataNode();

    bool init();
    bool publish(MassPointsData* sendData);

private:
    DomainParticipant* participant_;

    Topic* topic_;
    TypeSupport type_;

    Subscriber* subscriber_;
    DataReader* reader_;

    Publisher* publisher_;
    DataWriter* writer_;
    
    DDS_Publisher dwListener_;
    MassPointsDataSubscriber drListener_;

};
