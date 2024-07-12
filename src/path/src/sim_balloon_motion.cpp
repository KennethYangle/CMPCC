// 读取仿真气球参数文件，模拟气球运动， RflySim中刷新气球

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <rflysim_ros_pkg/Obj.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <swarm_msgs/MassPoint.h>
#include <swarm_msgs/MassPoints.h>

ros::Publisher mass_points_pub, sphere_pub, marker_pub;

struct balloon {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

class SimBalloon {
public:
    SimBalloon() {
        std::string path = ros::package::getPath("params") + "/balloon_motion.yaml";
        YAML::Node node = YAML::LoadFile(path);

        for (const YAML::Node& balloon_node : node["balloons"]) {
            balloon b;
            b.position = Eigen::Vector3d(balloon_node["pos"][0].as<double>(),
                                        balloon_node["pos"][1].as<double>(),
                                        balloon_node["pos"][2].as<double>());
            b.velocity = Eigen::Vector3d(balloon_node["vel"][0].as<double>(),
                                         balloon_node["vel"][1].as<double>(),
                                         balloon_node["vel"][2].as<double>());
            balloons.push_back(b);
            // std::cout << "position: " << b.position << "velocity: " << b.velocity << std::endl;
        }

        init_msgs();
    }

    void init_msgs() {
        obj_msg.type = 152;
        obj_msg.size.x = 0.2;
        obj_msg.size.y = 0.2;
        obj_msg.size.z = 0.2;

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // 设置标记的颜色为红色
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.header.frame_id = "world";
    }

    void update(double dt) {
        for (auto& b : balloons) {
            b.position += b.velocity * dt;
        }
        publishBalloons();

        // Show in RflySim and Rviz
        for (int i=0; i<balloons.size(); i++) {
            rflysim_interface(i);
            rviz_interface(i);
        }
    }

    void publishBalloons() {
        swarm_msgs::MassPoints mass_points_msg;
        for (const auto& balloon : balloons) {
            swarm_msgs::MassPoint point;
            point.position.x = balloon.position[0];
            point.position.y = balloon.position[1];
            point.position.z = balloon.position[2];
            point.velocity.x = balloon.velocity[0];
            point.velocity.y = balloon.velocity[1];
            point.velocity.z = balloon.velocity[2];
            mass_points_msg.points.push_back(point);
        }

        mass_points_pub.publish(mass_points_msg);
    }

    void rflysim_interface(int id) {
        obj_msg.id = 100 + id;
        obj_msg.position.x = balloons[id].position[0];
        obj_msg.position.y = balloons[id].position[1];
        obj_msg.position.z = balloons[id].position[2];

        sphere_pub.publish(obj_msg);
    }

    void rviz_interface(int id) {
        marker.id = 100 + id;
        marker.pose.position.x = balloons[id].position[0];
        marker.pose.position.y = balloons[id].position[1];
        marker.pose.position.z = balloons[id].position[2];

        marker_pub.publish(marker);
    }

private:
    std::vector<balloon> balloons;
    rflysim_ros_pkg::Obj obj_msg;
    visualization_msgs::Marker marker;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_balloon_node");
    ros::NodeHandle nh;
    mass_points_pub = nh.advertise<swarm_msgs::MassPoints>("balloons/masspoint", 10);
    sphere_pub = nh.advertise<rflysim_ros_pkg::Obj>("ue4_ros/obj", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("rviz/obj", 10);
    
    SimBalloon sim;
    ros::Time start_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    ros::Rate loop_rate(30); // 10Hz loop rate
    while (ros::ok()) {
        // 前20s先不动
        ros::Time now_time = ros::Time::now();
        if ((now_time-start_time).toSec() < 20) {
            loop_rate.sleep();
            last_time = now_time;
            continue;
        }
        sim.update((now_time-last_time).toSec());
        last_time = now_time;
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}