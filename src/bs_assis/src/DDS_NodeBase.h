#pragma once
/*
    本文件是dds接收和传输类的头文件定义
*/



#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>


// publisher特有的文件
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>


// subscriber特有的文件
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>


#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>



using namespace eprosima::fastdds::dds;


class DDS_Publisher : public DataWriterListener
{
public:

    DDS_Publisher() : matched_(0) {}

    ~DDS_Publisher() override {}

    // 可以定义一些action当新的listener被检测到
    void on_publication_matched( DataWriter*, const PublicationMatchedStatus& info) override
    {
        if (info.current_count_change == 1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher matched." << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher unmatched." << std::endl;
        }
        else
        {
            std::cout << info.current_count_change << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
        }
    }

    std::atomic_int matched_;

};


class DDS_Subscriber : public DataReaderListener
{
public:

    DDS_Subscriber() : samples_(0) { }

    ~DDS_Subscriber() override { }

    void on_subscription_matched( DataReader*, const SubscriptionMatchedStatus& info) override
    {
        if (info.current_count_change == 1)
        {
            std::cout << "Subscriber matched." << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            std::cout << "Subscriber unmatched." << std::endl;
        }
        else
        {
            std::cout << info.current_count_change << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
        }
    }


    std::atomic_int samples_;

};



