#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <ros/time.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pos, ref_pose;
geometry_msgs::TwistStamped local_vel;
quadrotor_msgs::PositionCommand mpcc_cmd;
ros::Subscriber state_sub, local_pos_sub, local_vel_sub, mpcc_cmd_sub;
ros::Publisher local_pos_pub, local_pva_pub, predict_pos_pub, ref_pos_pub;
ros::ServiceClient arming_client, set_mode_client;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel = *msg;
}
void mpcc_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
    mpcc_cmd = *msg;
}


void local_acc_target(const double ax, const double ay, const double az)
{
    mavros_msgs::PositionTarget pva_target;
    pva_target.coordinate_frame = 1;
    pva_target.header.stamp = ros::Time::now();

    pva_target.acceleration_or_force.x = ax;
    pva_target.acceleration_or_force.y = ay;
    pva_target.acceleration_or_force.z = az;

    pva_target.type_mask = pva_target.IGNORE_YAW + 
                pva_target.IGNORE_VX + pva_target.IGNORE_VY + pva_target.IGNORE_VZ+
                pva_target.IGNORE_PX + pva_target.IGNORE_PY + pva_target.IGNORE_PZ ;

    pva_target.yaw_rate = 0;

    local_pva_pub.publish(pva_target);
}

void local_vel_target(const double vx, const double vy, const double vz)
{
    mavros_msgs::PositionTarget pva_target;
    pva_target.coordinate_frame = 1;
    pva_target.header.stamp = ros::Time::now();

    pva_target.velocity.x = vx;
    pva_target.velocity.y = vy;
    pva_target.velocity.z = vz;

    pva_target.type_mask = pva_target.IGNORE_YAW + pva_target.FORCE + 
                pva_target.IGNORE_AFX + pva_target.IGNORE_AFY + pva_target.IGNORE_AFZ +
                pva_target.IGNORE_PX + pva_target.IGNORE_PY + pva_target.IGNORE_PZ ;

    pva_target.yaw_rate = 0;

    local_pva_pub.publish(pva_target);
}

void local_pva_target(const double px, const double py, const double pz,
        const double vx, const double vy, const double vz, const double ax,
        const double ay, const double az)
{
    mavros_msgs::PositionTarget pva_target;
    pva_target.coordinate_frame = 1;
    pva_target.header.stamp = ros::Time::now();

    pva_target.position.x = px;
    pva_target.position.y = py;
    pva_target.position.z = pz;
    pva_target.velocity.x = vx;
    pva_target.velocity.y = vy;
    pva_target.velocity.z = vz;
    pva_target.acceleration_or_force.x = ax;
    pva_target.acceleration_or_force.y = ay;
    pva_target.acceleration_or_force.z = az;

    pva_target.type_mask = pva_target.IGNORE_YAW;

    pva_target.yaw_rate = 0;

    local_pva_pub.publish(pva_target);
}

void takeoff(ros::NodeHandle &nh)
{
    ros::Rate rate(20.0);

    int cnt = 0;
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        if(cnt % 100 == 0)
            std::cout << "Waiting for FCU connection..." << std::endl;
        cnt++;
    }

    //send a few setpoints before starting
    ref_pose.pose.position.z = 4;
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(ref_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while((current_state.mode != "OFFBOARD") || (!arm_cmd.response.success)) {
        if( current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(5.0))) {
            if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else {
            if( !current_state.armed &&(ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(ref_pose);

        ros::spinOnce();
        rate.sleep();
    }

    double vx = - (local_pos.pose.position.x - ref_pose.pose.position.x);
    double vy = - (local_pos.pose.position.y - ref_pose.pose.position.y);
    double vz = - (local_pos.pose.position.z - ref_pose.pose.position.z);

    double dis = sqrt((local_pos.pose.position.x - ref_pose.pose.position.x)*(local_pos.pose.position.x - ref_pose.pose.position.x)
    + (local_pos.pose.position.y - ref_pose.pose.position.y)*(local_pos.pose.position.y - ref_pose.pose.position.y)
    + (local_pos.pose.position.z - ref_pose.pose.position.z)*(local_pos.pose.position.z - ref_pose.pose.position.z));
    while(dis > 0.4)
    {
        local_vel_target(vx, vy, vz);
        vx = - (local_pos.pose.position.x - ref_pose.pose.position.x);
        vy = - (local_pos.pose.position.y - ref_pose.pose.position.y);
        vz = - (local_pos.pose.position.z - ref_pose.pose.position.z);
        ros::spinOnce();
        rate.sleep();
        dis = sqrt((local_pos.pose.position.x - ref_pose.pose.position.x)*(local_pos.pose.position.x - ref_pose.pose.position.x)
            + (local_pos.pose.position.y - ref_pose.pose.position.y)*(local_pos.pose.position.y - ref_pose.pose.position.y)
            + (local_pos.pose.position.z - ref_pose.pose.position.z)*(local_pos.pose.position.z - ref_pose.pose.position.z));
    }
}


void move_to_init()
{
    ros::Rate rate(20.0);
    double vx = - (local_pos.pose.position.x - ref_pose.pose.position.x);
    double vy = - (local_pos.pose.position.y - ref_pose.pose.position.y);
    double vz = - (local_pos.pose.position.z - ref_pose.pose.position.z);

    double dis = sqrt((local_pos.pose.position.x - ref_pose.pose.position.x)*(local_pos.pose.position.x - ref_pose.pose.position.x)
    + (local_pos.pose.position.y - ref_pose.pose.position.y)*(local_pos.pose.position.y - ref_pose.pose.position.y)
    + (local_pos.pose.position.z - ref_pose.pose.position.z)*(local_pos.pose.position.z - ref_pose.pose.position.z));
    while(dis > 0.1)
    {
        local_vel_target(vx, vy, vz);
        vx = - (local_pos.pose.position.x - ref_pose.pose.position.x);
        vy = - (local_pos.pose.position.y - ref_pose.pose.position.y);
        vz = - (local_pos.pose.position.z - ref_pose.pose.position.z);
        ros::spinOnce();
        rate.sleep();
        dis = sqrt((local_pos.pose.position.x - ref_pose.pose.position.x)*(local_pos.pose.position.x - ref_pose.pose.position.x)
            + (local_pos.pose.position.y - ref_pose.pose.position.y)*(local_pos.pose.position.y - ref_pose.pose.position.y)
            + (local_pos.pose.position.z - ref_pose.pose.position.z)*(local_pos.pose.position.z - ref_pose.pose.position.z));
    }
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    ros::Rate rate(50);

    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, local_vel_cb);
    mpcc_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd",2, mpcc_cmd_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_pva_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",10);

    // takeoff
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    std::cout << "begin takeoff" << std::endl;
    takeoff(nh);
    std::cout << "takeoff success" << std::endl; 
    
    // std::cout << "move to init pose" << std::endl; 
    // move_to_init();


    while (ros::ok())
    {
        local_pva_target(mpcc_cmd.position.x, mpcc_cmd.position.y, mpcc_cmd.position.z, 
                mpcc_cmd.velocity.x, mpcc_cmd.velocity.y, mpcc_cmd.velocity.z,
                mpcc_cmd.acceleration.x, mpcc_cmd.acceleration.y, mpcc_cmd.acceleration.z);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
