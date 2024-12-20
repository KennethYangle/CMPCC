#include "MavDataNode.h"


MavDataNode::MavDataNode() : participant_(nullptr) , subscriber_(nullptr) , topic_(nullptr) , reader_(nullptr) , type_(new MavDataPubSubType())
{}



MavDataNode::~MavDataNode()
{
    if (reader_ != nullptr)
    {
        subscriber_->delete_datareader(reader_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}


bool MavDataNode::init()
{
    DomainParticipantQos participantQos;
    participantQos.name("MavDataNode");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

    if (participant_ == nullptr) { return false; }

    // Register the Type
    // Create the subscriptions Topic
    type_.register_type(participant_);
    topic_ = participant_->create_topic("MavDataTopic", "MavData", TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr) { return false; }


    // Create the Subscriber
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
    if (subscriber_ == nullptr) { return false; }
    reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &drListener_);
    if (reader_ == nullptr) { return false; }

    // Create the Publisher
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (publisher_ == nullptr) { return false; }
    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &dwListener_);
    if (writer_ == nullptr) { return false; }

    return true;
}


bool MavDataNode::publish(MavData* sendData)
{
    if (dwListener_.matched_ > 0)
    {
        writer_->write(sendData);
        return true;
    }
    return false;
}

geometry_msgs::Point calculateRelativePosition(const float& global_pos_latitude, const float& global_pos_longitude, const float& global_pos_altitude,
                                               const mavros_msgs::HomePosition& hp) {
    double lat_diff = global_pos_latitude - hp.geo.latitude;
    double lon_diff = global_pos_longitude - hp.geo.longitude;
    double alt_diff = global_pos_altitude - hp.geo.altitude;

    // 纬度和经度差异转换为米
    double lat_dist = lat_diff * (M_PI / 180) * EARTH_RADIUS;
    double lon_dist = lon_diff * (M_PI / 180) * EARTH_RADIUS * cos(hp.geo.latitude * M_PI / 180);

    geometry_msgs::Point relative_position;
    relative_position.x = lon_dist; // 东/西方向
    relative_position.y = lat_dist; // 北/南方向
    relative_position.z = alt_diff; // 高度差

    return relative_position;
}