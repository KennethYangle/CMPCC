#include "dds_node.hpp" 



/*
    本文件是dds接收和传输节点主程序
    MavData*文件是框架生成的文件
*/




// 先把从mavros拿到的东西，存到向量中，并且从DDS发出去
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dds_node");
    ros::NodeHandle nh("~");



    int mav_num = 1;
    int mav_id = 1;
    nh.param<int>("mav_num",mav_num , 6);
    nh.param<int>("mav_id",mav_id , 1);

    std::cout<<"mav_num:"<<mav_num<<std::endl;
    std::cout<<"mav_id:"<<mav_id<<std::endl;
    
    // index start 0 in C++
    mav_id = mav_id-1; 
    mavDataVec.reserve(mav_num);
    masspointsDataVec.reserve(mav_num);


    // rospy.Subscriber("mavros/state", State, state_cb)
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10,
        boost::bind(&poseCallback, _1, mav_id)
    );
    ros::Subscriber global_sub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/global", 10,
        boost::bind(&globalCallback, _1, mav_id)
    );
    ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>(
        "/mavros/home_position/home", 10,
        boost::bind(&homeCallback, _1, mav_id)
    );
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped> (
        "/mavros/local_position/velocity_local", 10,
        boost::bind(&velCallback, _1, mav_id)
    );
    ros::Subscriber masspoints_sub = nh.subscribe<swarm_msgs::MassPoints> (
        "/balloons/masspoint", 10,
        boost::bind(&masspointsCallback, _1, mav_id)
    );


    




    // 接收从DDS来的数据并推到ros的话题中
    for (size_t i = 0; i < mav_num; i++)
    {
        // drone id 从1开始
        std::stringstream fmt;
        fmt << "/drone_"<<(i+1);
        
        mav_data_struct temp_mavdata;
        swarm_msgs::MassPoints temp_masspointsMsg;
        mavDataVec.push_back(temp_mavdata);
        masspointsDataVec.push_back(temp_masspointsMsg);

        // /drone_i/mavros/local_position/pose 所有飞机相对于当前飞机home点的相对位置姿态
        posPubs.push_back(nh.advertise<geometry_msgs::PoseStamped>(fmt.str()+"/mavros/local_position/pose", 10));
        velPubs.push_back(nh.advertise<geometry_msgs::TwistStamped>(fmt.str()+"/mavros/local_position/velocity_local", 10));
        masspointsPubs.push_back(nh.advertise<swarm_msgs::MassPoints>(fmt.str()+"/balloons/masspoint", 10));
    }
    
    std::cout << "Starting DDS pub and sub" << std::endl;

    MavDataNode* mav_data_node = new MavDataNode();
    if(!mav_data_node->init())
    {
        std::cout<< "DDS init fail!" <<std::endl;
        return 1;
    }

    MassPointsDataNode* masspoints_data_node = new MassPointsDataNode();
    if(!masspoints_data_node->init())
    {
        std::cout<< "DDS init fail!" <<std::endl;
        return 1;
    }

    std::cout<< "DDS init successful!" <<std::endl;

    std::thread ddsPub_thread(MavDataUpdate, mav_data_node, mav_id);
    std::thread dds_masspoint_thread(MassPointsDataUpdate, masspoints_data_node, mav_id);


    ros::spin();

    return 0;
}
