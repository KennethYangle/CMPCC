#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <swarm_msgs/MassPoint.h>
#include <swarm_msgs/MassPoints.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

struct Balloon {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

double calculateMeetingTime(const Eigen::Vector3d &Pc, const double &V, const Eigen::Vector3d &Pb, const Eigen::Vector3d &Vb) {
    Eigen::Vector3d dP = Pb - Pc;
    double a = Vb.dot(Vb) - V*V;
    double b = 2 * dP.dot(Vb);
    double c = dP.dot(dP);

    double Delta = b*b - 4*a*c;
    double t = (-b - sqrt(Delta)) / a / 2;
    return t;
}


double distance(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
    return (p1 - p2).norm();
}

std::pair<std::vector<Balloon>, double> calculateTotalDistance(const std::vector<Balloon> &path, const Eigen::Vector3d &currentPosition, double mavVel) {
    double time = 0, time_stash = 0;
    double totalDistance = 0;
    std::vector<Balloon> pathAfterMotion;
    Eigen::Vector3d currentPos = currentPosition;

    for (const auto &balloon : path) {
        Eigen::Vector3d nextPosition = balloon.position + balloon.velocity * time_stash;
        time = calculateMeetingTime(currentPos, mavVel, nextPosition, balloon.velocity);
        // std::cout << time << std::endl;
        nextPosition += balloon.velocity * time;
        double dist = distance(currentPos, nextPosition);

        Balloon balloonAfterMotion;
        balloonAfterMotion.position = nextPosition;
        balloonAfterMotion.velocity = balloon.velocity;
        pathAfterMotion.push_back(balloonAfterMotion);
        totalDistance += dist;
        currentPos = nextPosition;
        time_stash += time;
    }

    return {pathAfterMotion, totalDistance};
}

std::pair<std::vector<Balloon>, double> findShortestPath(const Eigen::Vector3d &currentPosition, std::vector<Balloon> &balloons, double mavVel) {
    std::vector<Balloon> shortestPath;
    double shortestDistance = std::numeric_limits<double>::infinity();

    std::sort(balloons.begin(), balloons.end(), [](const Balloon &a, const Balloon &b) { return a.position.norm() < b.position.norm(); });

    do {
        auto result = calculateTotalDistance(balloons, currentPosition, mavVel);
        double totalDistance = result.second;
        if (totalDistance < shortestDistance) {
            shortestDistance = totalDistance;
            shortestPath = result.first;
        }
    } while (std::next_permutation(balloons.begin(), balloons.end(), [](const Balloon &a, const Balloon &b) { return a.position.norm() < b.position.norm(); }));

    return {shortestPath, shortestDistance};
}

class ShortestPathNode {
public:
    ShortestPathNode() {
        path_T = 0.2;
        mav_vel_ = 3.0;  // 平均访问速度
        current_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        current_velocity_ = Eigen::Vector3d(0.0, 0.0, 0.0);

        ros::NodeHandle nh;
        local_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &ShortestPathNode::local_pos_cb, this);
        local_vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &ShortestPathNode::local_vel_cb, this);
        balloon_sub_ = nh.subscribe<swarm_msgs::MassPoints>("/balloons/masspoint", 10, &ShortestPathNode::balloonsCallback, this);
        path_pub_ = nh.advertise<swarm_msgs::MassPoints>("/path_points", 1);
        timer_path = nh.createTimer(ros::Duration(path_T), &ShortestPathNode::pathCallback, this);
    }


    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_position_[0] = msg->pose.position.x;
        current_position_[1] = msg->pose.position.y;
        current_position_[2] = msg->pose.position.z;
    }

    void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
        current_velocity_[0] = msg->twist.linear.x;
        current_velocity_[1] = msg->twist.linear.y;
        current_velocity_[2] = msg->twist.linear.z;
    }

    void balloonsCallback(const swarm_msgs::MassPoints::ConstPtr &msg) {
        while(ros::ok() && pathRead_)
            ros::Duration(0.001).sleep();

        balloons_.clear();
        for (const auto &b : msg->points) {
            Balloon balloon;
            balloon.position = Eigen::Vector3d(b.position.x, b.position.y, b.position.z);
            balloon.velocity = Eigen::Vector3d(b.velocity.x, b.velocity.y, b.velocity.z);
            balloons_.push_back(balloon);
        }
    }

    void pathCallback(const ros::TimerEvent& event) {
        pathRead_ = true;
        if (!balloons_.empty()) {
            auto result = findShortestPath(current_position_, balloons_, mav_vel_);
            auto shortestPath = result.first;

            swarm_msgs::MassPoints pathPointsMsg;
            swarm_msgs::MassPoint point;
            point.position.x = current_position_[0];
            point.position.y = current_position_[1];
            point.position.z = current_position_[2];
            point.velocity.x = current_velocity_[0];
            point.velocity.y = current_velocity_[1];
            point.velocity.z = current_velocity_[2];
            pathPointsMsg.points.push_back(point);

            for (const auto &balloon : shortestPath) {
                swarm_msgs::MassPoint point;
                point.position.x = balloon.position.x();
                point.position.y = balloon.position.y();
                point.position.z = balloon.position.z();
                point.velocity.x = balloon.velocity.x();
                point.velocity.y = balloon.velocity.y();
                point.velocity.z = balloon.velocity.z();
                pathPointsMsg.points.push_back(point);
            }
            path_pub_.publish(pathPointsMsg);
        }
        pathRead_ = false;
    }


private:
    ros::Subscriber local_pos_sub_, local_vel_sub_, balloon_sub_;
    ros::Publisher path_pub_;
    ros::Timer timer_path;
    Eigen::Vector3d current_position_, current_velocity_;
    double path_T, mav_vel_;
    std::vector<Balloon> balloons_;
    bool pathRead_=true;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_finding_node");
    ShortestPathNode node;
    ros::spin();
    return 0;
}
