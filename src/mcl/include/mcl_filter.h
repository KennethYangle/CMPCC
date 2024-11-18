//Compile with: g++ solution.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
#include "matplotlibcpp.h" //Graph Library
#include <iostream>
#include <string>
#include <math.h>
#include <stdexcept> // throw errors
#include <random> //C++ 11 Random Numbers
#include <vector>
#include <deque>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <tf/tf.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <algorithm>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <swarm_msgs/BoundingBoxes.h>
#include <swarm_msgs/CenterPoints.h>

#include "sensor_msgs/image_encodings.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

#include <yaml-cpp/yaml.h>

namespace plt = matplotlibcpp;
using namespace std;
using namespace Eigen;


Vector2d img0(640, 360);//图像中心
double img_f = 370;
double img_distance = 20.0;
double img_width, img_height;
std::vector<Vector3d> target_pos;
std::vector<Vector3d> target_vel;
std::vector<Vector3d> target_pos_hat;   // estimated target pos
Vector3d mav_pos;  // 相对于home点的位置
Vector3d mav_vel;
geometry_msgs::Quaternion mav_quad;
double mav_yaw;
double mav_roll;
double mav_pitch;
Eigen::Matrix3d mav_R;
Eigen::Quaterniond mav_quad_eigen;
Eigen::Matrix3d R_cb;
std::vector<Vector3d> p_s_Vec;   // 飞机检测到的所有目标的集合
geometry_msgs::PoseArray mav_pos_Array;
swarm_msgs::CenterPoints expect_target_pos;
swarm_msgs::CenterPoints target_cps;
ros::Time timestamp;

bool get_img = false;
bool get_pose = false;
bool get_vel = false;
bool is_show_image = false;
int image_failed_cnt = 0;
const int image_failed_max_cnt = 5;
int ch7=0;
int ch8=0;

int particle_num = 1000;
int target_num = 1;
int max_iterations = 100;
XmlRpc::XmlRpcValue target_params;
std::map<string, double> params;

// Random Generators
random_device rd;
mt19937 gen(rd());


double gen_real_random();
double gen_unit_random();
double angle_diff(double a, double b);
double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

void target_resampling();
void target_clustering();
void target_balance();
void publishInterceptors();
void publishTargetMarkers();
void publishBarycenter();
void publishTargetBarycenter();
void publishCenterMarkers();
void publishDetectedFeatures();
void publishExpectedTarget();


class Robot {
public:
    Robot()
    {
        // Constructor
        position(0) = gen_unit_random() * init_pos_conv;  // robot's coordinate
        position(1) = gen_unit_random() * init_pos_conv;
        position(2) = gen_unit_random() * init_pos_conv;
        velocity(0) = gen_unit_random() * init_vel_conv; // robot's velocity
        velocity(1) = gen_unit_random() * init_vel_conv;
        velocity(2) = gen_unit_random() * init_vel_conv;
    }

    Robot(const std::vector<Vector3d>& init_pos, const std::vector<Vector3d>& init_vel)
    {
        // Constructor
        int target_num = init_pos.size();
        int idx = rand() % target_num;
        position(0) = init_pos[idx](0) + gen_unit_random() * init_pos_conv; // robot's coordinate
        position(1) = init_pos[idx](1) + gen_unit_random() * init_pos_conv;
        position(2) = init_pos[idx](2) + gen_unit_random() * init_pos_conv;
        velocity(0) = init_vel[idx](0) + gen_unit_random() * init_vel_conv; // robot's velocity
        velocity(1) = init_vel[idx](1) + gen_unit_random() * init_vel_conv;
        velocity(2) = init_vel[idx](2) + gen_unit_random() * init_vel_conv;
    }

    Robot(const std::vector<Vector3d>& init_pos, const std::vector<Vector3d>& init_vel, std::map<string, double>& params)
    {
        // Constructor
        set_params(params);

        int target_num = init_pos.size();
        int idx = rand() % target_num;
        position(0) = init_pos[idx](0) + gen_unit_random() * init_pos_conv; // robot's coordinate
        position(1) = init_pos[idx](1) + gen_unit_random() * init_pos_conv;
        position(2) = init_pos[idx](2) + gen_unit_random() * init_pos_conv;
        velocity(0) = init_vel[idx](0) + gen_unit_random() * init_vel_conv; // robot's velocity
        velocity(1) = init_vel[idx](1) + gen_unit_random() * init_vel_conv;
        velocity(2) = init_vel[idx](2) + gen_unit_random() * init_vel_conv;
    }

    void set_params(std::map<string, double>& params)
    {
        this->alpha_1 = params["alpha_1"];
        this->alpha_2 = params["alpha_2"];
        this->alpha_3 = params["alpha_3"];
        this->alpha_4 = params["alpha_4"];
        this->init_pos_conv = params["init_pos_conv"];
        this->init_vel_conv = params["init_vel_conv"];
        this->sigma_p = params["sigma_p"];
    }

    // Prediction Step, 预测粒子（目标）的运动
    void target_prediction(double dt)
    {
        // ROS_INFO_STREAM("Enter target_prediction.");
        position = position + velocity * dt;

        // ROS_INFO_STREAM("position: " << position << ", velocity: " << velocity << ", vel_noise: " << vel_noise);
        velocity(0) += gen_gauss_random(0., alpha_1);
        velocity(1) += gen_gauss_random(0., alpha_1);
        velocity(2) += gen_gauss_random(0., alpha_1);
    }

    // Update Step, 根据观测为粒子赋予权重
    void target_update()
    {
        // ROS_INFO_STREAM("Enter target_update.");
        q = 0.01;
        Vector3d p_mav2target = position - mav_pos;
        p_s_hat = p_mav2target.normalized();
        std::vector<double> weight_stash={0.};
        for (int t=0; t<p_s_Vec.size(); t++) {
            weight_stash.push_back( gaussian(0, sigma_p, 1.0 - p_s_hat.dot(p_s_Vec[t])) );
        }
        q += *std::max_element(weight_stash.begin(), weight_stash.end());
    }

    void landmark_model_likelyhood_simple(Vector3d p_s, Vector3d n_o, double img_distance)
    {
        Vector3d p = -position;
        p.normalize();
        p_s_hat = p;

        q = gaussian(0, sigma_p, 1.0 - p_s_hat.dot(p_s));

        if (p_s.norm() < 0.01) {
            if (judge_in_FOV(n_o, img_distance)) {
                q = 1e-100;
            }
        }
        else {
            if (!judge_in_FOV(n_o, img_distance+5.0)) {
                q = 1e-100;
            }
        }

        // ROS_INFO_STREAM("p_s_hat: " << p_s_hat << ", p_s: " << p_s);
        // ROS_INFO_STREAM("q: " << q);
    }

    bool judge_in_FOV(Vector3d n_o, double img_distance)
    {
        Vector3d p = -position;
        double p_norm = p.norm();
        if (p_s_hat.dot(n_o) > sqrt(2)/2 && p_norm < img_distance) {
            return true;
        }
        else {
            return false;
        }
    }


    Vector3d position;
    Vector3d velocity;
    double mav_yaw_truth;
    double delta_rot1, delta_rot2, delta_rot3, delta_trans;
    double delta_rot1_hat, delta_rot2_hat, delta_rot3_hat, delta_trans_hat;
    double alpha_1 = 0.0;   // uncertainty weight
    double alpha_2 = 0.0;
    double alpha_3 = 0.0;
    double alpha_4 = 0.0;
    double init_pos_conv = 1.0;
    double init_vel_conv = 1.0;
    double init_orient_conv = 0.1;
    double sigma_p = 0.5;
    Vector3d p_s_hat;
    double q;               // particle's weight
    int cluster = 1;

private:
    // Gaussian random
    double gen_gauss_random(double mean, double variance)
    {
        if (variance < numeric_limits<double>::min())   variance = numeric_limits<double>::min();
        normal_distribution<double> gauss_dist(mean, variance);
        return gauss_dist(gen);
    }

    // Probability of x for 1-dim Gaussian with mean mu and var. sigma
    double gaussian(double mu, double sigma, double x)
    {
        return exp(-(pow((mu - x), 2)) / (pow(sigma, 2)) / 2.0) / sqrt(2.0 * M_PI * (pow(sigma, 2)));
    }
};

// Generate real random between 0 and 1
double gen_real_random()
{
    uniform_real_distribution<double> real_dist(0.0, 1.0); //Real
    return real_dist(gen);
}

// Generate real random between -1 and 1
double gen_unit_random()
{
    uniform_real_distribution<double> real_dist(-1.0, 1.0); //Real
    double rt = real_dist(gen);
    // cout << "gen_unit_random: " << rt << endl;
    return rt;
}

double angle_diff(double a, double b)
{
    double diff = a - b;
    if (diff < 0)
        diff += 2 * M_PI;
    if (diff < M_PI)
        return diff;
    else
        return diff - 2 * M_PI;
}

// 计算两个点之间的距离
double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return (a - b).norm();
}