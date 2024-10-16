#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PiecewiseBezier.h>
#include <swarm_msgs/TimeOptimalPMMPieces.h>
#include <swarm_msgs/TimeOptimalPMMParam.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "mpc_solver.h"

using namespace std;
using namespace Eigen;
using namespace ft;

ros::Publisher cmd_pub, refer_pub, drone_pub, global_pub, predict_pub, globalOdom_pub;

ft::MpcSolver simSolver;
quadrotor_msgs::PositionCommand cmdMsg;
nav_msgs::Odometry globalOdom;
// Eigen::SparseMatrix<double,Eigen::RowMajor> stateOdom(Model::numState,1);
Eigen::SparseMatrix<double> stateCmd(Model::numState,1);
Eigen::SparseMatrix<double> stateMpc(Model::numState,1);
Eigen::SparseMatrix<double> stateOdom(Model::numState,1);
Eigen::SparseMatrix<double> stateOdomPrevious(Model::numState,1);
double mpcT = 0.05;
double odomT;
Eigen::Vector3d pDrone, vDrone, aDrone;
bool mpcInit, odomRead=true, pathRead=true;
geometry_msgs::Point tmpPoint;
geometry_msgs::Vector3 tmpVector;
ros::Time tStart, tFinal, tOdom, tMpc;

bool reset_now = false;
int fail_times = 0;

Eigen::SparseMatrix<double> stateL;
Eigen::SparseMatrix<double> stateR;
bool state_predict_done = false;

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    while(ros::ok() && pathRead)
        ros::Duration(0.001).sleep();
    odomRead = true;
    odomT = (msg->header.stamp - tOdom).toSec();
    tOdom = msg->header.stamp;
    pDrone << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    stateOdomPrevious = stateOdom;
    stateOdom.coeffRef(0,0) = pDrone(0);
    stateOdom.coeffRef(3,0) = pDrone(1);
    stateOdom.coeffRef(6,0) = pDrone(2);
    stateOdom.coeffRef(9,0) = simSolver.map.findNearestTheta(pDrone);
    odomRead = false;
}
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    vDrone << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    stateOdom.coeffRef(1,0) = vDrone(0);
    stateOdom.coeffRef(4,0) = vDrone(1);
    stateOdom.coeffRef(7,0) = vDrone(2);
}

void refer_path_params_callback(const swarm_msgs::TimeOptimalPMMPieces::ConstPtr& msg) {
    pathRead = true;
    simSolver.map.setPathPts(msg);
    simSolver.updateSolver();
    // cout<< "refer_path_callback" << endl;
    pathRead = false;
}

// state = [px, vx, ax, py, vy, ay, pz, vz, az, theta(t), vtheta, atheta]
// u = [jx, jy, jz, jtheta]
void mpc_callback(const ros::TimerEvent& event){
    while(ros::ok() && odomRead && pathRead)
        ros::Duration(0.001).sleep();
    mpcInit = true;
    tMpc = ros::Time::now();
    stateL = simSolver.statePredict.col(0); // 第0列就是传入的状态
    stateR = simSolver.statePredict.col(1); // 第1列是预测1步的状态
    // calculate stateMPC
    stateMpc = (stateOdom - stateOdomPrevious)/odomT*(tMpc-tOdom).toSec() + stateOdom;  // 插值。根据最近时刻Odom状态和上次Odom状态，以及时间戳插值到当前时刻
    // update some state from last horizon
    if (fail_times > 3){
        // 计算失败清空加速度项
        stateMpc.coeffRef( 2, 0) = 0; // ax
        stateMpc.coeffRef( 5, 0) = 0; // ay
        stateMpc.coeffRef( 8, 0) = 0; // az
        stateMpc.coeffRef(10, 0) = 0; // vtheta
        stateMpc.coeffRef(11, 0) = 0; // atheta
        simSolver.initStatus = true;
        mpcInit = false;
        if( simSolver.solveMpcQp(stateMpc) ){
            ROS_ERROR("MPCQP Solver failure!");
            // ros::shutdown();
        }
        else{
            fail_times = 0;
        }
    }
    else{
        // 计算成果使用1步预测的结果
        stateMpc.coeffRef(2, 0) = simSolver.statePredict.coeffRef(2, 1); // ax
        stateMpc.coeffRef(5, 0) = simSolver.statePredict.coeffRef(5, 1); // ay
        stateMpc.coeffRef(8, 0) = simSolver.statePredict.coeffRef(8, 1); // az
        stateMpc.coeffRef(10,0) = simSolver.statePredict.coeffRef(10,1); // vtheta
        stateMpc.coeffRef(11,0) = simSolver.statePredict.coeffRef(11,1); // atheta
        mpcInit = false;
        if( simSolver.solveMpcQp(stateMpc) ){
            fail_times ++;
        }
        else{
            fail_times = 0;
        }
    }

    state_predict_done = true;
    
    refer_pub.publish(simSolver.displayPtr->refTraj_msg);
    global_pub.publish(simSolver.displayPtr->theta_msg);
    drone_pub.publish(simSolver.displayPtr->drone_msg);
    predict_pub.publish(simSolver.displayPtr->trajPred_msg);
}

void cmd_callback(const ros::TimerEvent& event){
    if (state_predict_done){
        while(ros::ok() && mpcInit);
        double deltaT = (ros::Time::now() - tMpc).toSec();
        stateCmd = stateL + (stateR-stateL)*deltaT/mpcT;
        tmpPoint.x = stateCmd.coeffRef(0,0);
        tmpPoint.y = stateCmd.coeffRef(3,0);
        tmpPoint.z = stateCmd.coeffRef(6,0);
        cmdMsg.position = tmpPoint;
        tmpVector.x = stateCmd.coeffRef(1,0);
        tmpVector.y = stateCmd.coeffRef(4,0);
        tmpVector.z = stateCmd.coeffRef(7,0);
        cmdMsg.velocity = tmpVector;
        tmpVector.x = stateCmd.coeffRef(2,0);
        tmpVector.y = stateCmd.coeffRef(5,0);
        tmpVector.z = stateCmd.coeffRef(8,0);
        cmdMsg.acceleration = tmpVector;
        cmdMsg.yaw = simSolver.map.getYaw(stateCmd.coeffRef(9, 0));
        // cout << "stateCmd.coeffRef(9, 0): " << stateCmd.coeffRef(9, 0) << endl;
        cmdMsg.header.stamp = ros::Time::now();
        cmd_pub.publish(cmdMsg);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fly_node");
    ros::NodeHandle nodeHandle;
    cout<< "Init node." << endl;
    cmd_pub = nodeHandle.advertise<quadrotor_msgs::PositionCommand>("position_cmd",1);
    refer_pub = nodeHandle.advertise<nav_msgs::Path>("refer_path", 1);
    drone_pub = nodeHandle.advertise<visualization_msgs::Marker>("drone_pose", 1);
    global_pub = nodeHandle.advertise<visualization_msgs::Marker>("global_pose", 1);
    ros::Subscriber sub_path = nodeHandle.subscribe("/refer_path_params", 5 , refer_path_params_callback);
    ros::Subscriber local_pos_sub = nodeHandle.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb, ros::TransportHints().tcpNoDelay());
    ros::Subscriber local_vel_sub = nodeHandle.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, local_vel_cb, ros::TransportHints().tcpNoDelay());
    predict_pub = nodeHandle.advertise<nav_msgs::Path>("predict_path", 1);
    globalOdom_pub = nodeHandle.advertise<nav_msgs::Odometry>("globalOdom", 1);
    cout<< "Publisher and Subscriber." << endl;

    ros::Timer timer_mpc = nodeHandle.createTimer(ros::Duration(mpcT), mpc_callback);
    ros::Timer timer_cmd = nodeHandle.createTimer(ros::Duration(0.01), cmd_callback);

    ros::MultiThreadedSpinner spinner(4);

    cmdMsg.header.frame_id = "world";
    
    // init position: 
    tmpPoint.x = 0; tmpPoint.y = 0; tmpPoint.z = 0; cmdMsg.position = tmpPoint;
    tmpVector.x = 0; tmpVector.y = 0; tmpVector.z = 0; cmdMsg.velocity = tmpVector;
    tmpVector.x = 0; tmpVector.y = 0; tmpVector.z = 0; cmdMsg.acceleration = tmpVector;
    cmdMsg.yaw_dot = 0;
    ros::Rate loopRate(10);
    ros::Time startT = ros::Time::now();
    while(ros::ok()){
        ros::Time nowT = ros::Time::now();
        cmd_pub.publish(cmdMsg);
        if ((nowT-startT).toSec() > 3){
            break;
        }
        loopRate.sleep();
    }

    spinner.spin();
    return 0;
}
