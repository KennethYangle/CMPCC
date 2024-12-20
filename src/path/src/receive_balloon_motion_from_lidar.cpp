#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <swarm_msgs/MassPoint.h>
#include <swarm_msgs/MassPoints.h>
#include <tf/transform_datatypes.h>

class BalloonMotionReceiver {
public:
    BalloonMotionReceiver() {
        // 初始化ROS节点句柄和订阅/发布
        ros::NodeHandle nh;

        // 订阅话题
        pose_sub_ = nh.subscribe("/drone_2/mavros/local_position/pose", 10, &BalloonMotionReceiver::poseCallback, this);
        masspoint_sub_ = nh.subscribe("/drone_2/balloons/masspoint", 10, &BalloonMotionReceiver::masspointCallback, this);

        // 发布话题
        masspoint_pub_ = nh.advertise<swarm_msgs::MassPoints>("/balloons/masspoint", 10);
        
        // RViz marker 发布器
        marker_pub = nh.advertise<visualization_msgs::Marker>("/rviz/obj", 10);
        is_receive_lidar_pose_ = false;

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

private:
    ros::Subscriber pose_sub_;       // 订阅激光雷达相对于无人机的姿态
    ros::Subscriber masspoint_sub_;  // 订阅激光雷达检测的目标数据
    ros::Publisher masspoint_pub_;   // 发布转换后的目标数据
    ros::Publisher marker_pub;       // 发布Marker以在RViz中可视化气球

    geometry_msgs::PoseStamped trans_lidar_mav_;  // 激光雷达相对于无人机的位置
    bool is_receive_lidar_pose_;

    // RViz接口函数：将气球数据发布为Marker显示
    void rviz_interface(int id, swarm_msgs::MassPoint point) {
        marker.id = 100 + id;  // 设置Marker ID
        marker.lifetime = ros::Duration(0.1);  // 设置显示时长
        marker.pose.position.x = point.position.x;  // 设置气球位置
        marker.pose.position.y = point.position.y;
        marker.pose.position.z = point.position.z;

        // 发布Marker
        marker_pub.publish(marker);
    }

    // 接收无人机姿态并保存
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        trans_lidar_mav_ = *msg;
        is_receive_lidar_pose_ = true;
    }

    // 接收激光雷达检测的目标并进行坐标系转换
    void masspointCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
        if (!is_receive_lidar_pose_) {
            ROS_WARN("No pose data received yet.");
            return;
        }

        // 创建一个新的MassPoints消息
        swarm_msgs::MassPoints transformed_masspoints;

        // 变换每一个目标点
        for (int i = 0; i < msg->points.size(); ++i) {
            const auto& point = msg->points[i];
            swarm_msgs::MassPoint transformed_point;

            // 将目标位置转换到无人机坐标系
            tf::Vector3 point_pos(point.position.x, point.position.y, point.position.z);
            tf::Vector3 trans_pos(trans_lidar_mav_.pose.position.x,
                                  trans_lidar_mav_.pose.position.y,
                                  trans_lidar_mav_.pose.position.z);

            // 目标位置加上激光雷达相对于无人机的位置
            tf::Vector3 transformed_position = point_pos + trans_pos;

            // 将转换后的位置赋值给目标点
            transformed_point.position.x = transformed_position.x();
            transformed_point.position.y = transformed_position.y();
            transformed_point.position.z = transformed_position.z();

            // 保持目标的速度不变
            transformed_point.velocity = point.velocity;

            // 保持目标的体积不变
            transformed_point.volume = point.volume;

            // 将转换后的目标点添加到结果中
            transformed_masspoints.points.push_back(transformed_point);

            // 显示转换后的气球位置
            rviz_interface(i, transformed_point);
        }

        // 发布转换后的目标点数据
        masspoint_pub_.publish(transformed_masspoints);
    }

    // RViz Marker
    visualization_msgs::Marker marker;
};

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "receive_balloon_node");
    std::cout << "receive_balloon_node" << std::endl;

    // 创建并运行BalloonMotionReceiver实例
    BalloonMotionReceiver receiver;

    // 进入ROS循环
    ros::spin();

    return 0;
}
