#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <swarm_msgs/MassPoint.h>
#include <swarm_msgs/MassPoints.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/State.h>

class BalloonMotionReceiver {
public:
    BalloonMotionReceiver() {
        // 初始化ROS节点句柄和订阅/发布
        ros::NodeHandle nh("~");

        // 订阅话题
        pose_sub_ = nh.subscribe("/drone_2/mavros/local_position/pose", 10, &BalloonMotionReceiver::poseCallback, this);
        mav_pose_sub_ = nh.subscribe("/mavros/local_position/pose", 10, &BalloonMotionReceiver::mavposeCallback, this);
        masspoint_sub_ = nh.subscribe("/drone_2/balloons/masspoint", 10, &BalloonMotionReceiver::masspointCallback, this);
        state_sub_ = nh.subscribe("/mavros/state", 10, &BalloonMotionReceiver::stateCallback, this);

        // 发布话题
        masspoint_pub_ = nh.advertise<swarm_msgs::MassPoints>("/balloons/masspoint", 10);

        // RViz marker 发布器
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rviz/obj", 10);

        is_receive_lidar_pose_ = false;
        initial_z_offset_ = 0.0;
        is_matching_ = false;
        last_matching_offset_.x = last_matching_offset_.y = last_matching_offset_.z = 0.0;

        // 从ROS参数服务器加载参数
        nh.param("matching_horizontal_th", matching_horizontal_th_, 0.5);
        nh.param("matching_vertical_th", matching_vertical_th_, 2.0);
        nh.param("matching_variation_th", matching_variation_th_, 0.5);
        nh.param("is_matching", is_matching_, true);

        std::cout << "matching_horizontal_th: " << matching_horizontal_th_ << "\nmatching_vertical_th: " << matching_vertical_th_ << "\nmatching_variation_th: " << matching_variation_th_ << "\nis_matching: " << is_matching_ << std::endl;

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

        // 设置定时器，30Hz频率
        timer_ = nh.createTimer(ros::Duration(1.0 / 30.0), &BalloonMotionReceiver::timerCallback, this);

        swarm_msgs::MassPoint init_up_point;
        init_up_point.position.z = 10.0;
        latest_transformed_masspoints_.points.push_back(init_up_point);
    }

private:
    ros::Subscriber pose_sub_;
    ros::Subscriber mav_pose_sub_;
    ros::Subscriber masspoint_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher masspoint_pub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;  // 定时器

    geometry_msgs::PoseStamped trans_lidar_mav_;  // 激光雷达相对于无人机的位置
    swarm_msgs::MassPoints latest_transformed_masspoints_;  // 保存最近一次处理的数据
    bool is_receive_lidar_pose_;
    bool is_matching_;  // 控制是否进行匹配操作

    double initial_z_offset_;  // 存储初始的Z偏移量
    double matching_horizontal_th_;  // 匹配的水平偏差阈值
    double matching_vertical_th_;  // 匹配的垂直偏差阈值
    double matching_variation_th_;  // 匹配偏移量变化阈值

    geometry_msgs::Point mav_pos_;
    geometry_msgs::Point last_matching_offset_;  // 上一次匹配的偏移量

    visualization_msgs::Marker marker;

    // RViz接口函数：将气球数据发布为Marker显示
    void rviz_interface(int id, swarm_msgs::MassPoint point) {
        marker.id = 100 + id;  // 设置Marker ID
        marker.lifetime = ros::Duration(0.1);  // 设置显示时长
        marker.pose.position.x = point.position.x;  // 设置气球位置
        marker.pose.position.y = point.position.y;
        marker.pose.position.z = point.position.z;

        // 发布Marker
        marker_pub_.publish(marker);
    }

    // Callback to receive UAV state
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        // Check if the drone is unlocked
        if (!msg->armed && msg->connected) {
            initial_z_offset_ = trans_lidar_mav_.pose.position.z;
        }
    }

    // 接收激光雷达位置姿态并保存
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        trans_lidar_mav_ = *msg;
        is_receive_lidar_pose_ = true;
    }
    
    // 接收无人机姿态并保存
    void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        mav_pos_.x = msg->pose.position.x;
        mav_pos_.y = msg->pose.position.y;
        mav_pos_.z = msg->pose.position.z;
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

            // Apply z-offset correction if the UAV is unlocked
            double corrected_z = trans_lidar_mav_.pose.position.z - initial_z_offset_;

            // Transform the target position to the UAV coordinate system
            tf::Vector3 point_pos(point.position.x, point.position.y, point.position.z);
            tf::Vector3 trans_pos(trans_lidar_mav_.pose.position.x,
                                  trans_lidar_mav_.pose.position.y,
                                  corrected_z);  // Use corrected z value
            tf::Vector3 matching_offset(last_matching_offset_.x, last_matching_offset_.y, last_matching_offset_.z);

            // 目标位置加上激光雷达相对于无人机的位置
            tf::Vector3 transformed_position = point_pos + trans_pos - matching_offset;

            // 将转换后的位置赋值给目标点
            transformed_point.position.x = transformed_position.x();
            transformed_point.position.y = transformed_position.y();
            transformed_point.position.z = transformed_position.z();

            // 如果启用了匹配
            if (is_matching_ && matchWithUAV(transformed_point)) {
                // 如果匹配上了，不需要发布这个目标点
                continue;
            }

            // 保持目标的速度不变
            // transformed_point.velocity = point.velocity;
            transformed_point.velocity.x = transformed_point.velocity.y = transformed_point.velocity.z = 0.0;
            
            // 保持目标的体积不变
            transformed_point.volume = point.volume;

            // 将转换后的目标点添加到结果中
            transformed_masspoints.points.push_back(transformed_point);

            // 显示转换后的气球位置
            rviz_interface(i, transformed_point);
        }

        latest_transformed_masspoints_ = transformed_masspoints;  // 保存最新的处理数据
    }

    // 判断目标点是否与无人机匹配
    bool matchWithUAV(const swarm_msgs::MassPoint& point) {
        double dx = fabs(point.position.x - mav_pos_.x);
        double dy = fabs(point.position.y - mav_pos_.y);
        double dz = fabs(point.position.z - mav_pos_.z);

        // 检查是否满足匹配条件
        if (dx < matching_horizontal_th_ && dy < matching_horizontal_th_ && dz < matching_vertical_th_) {
            // 检查与上次匹配偏移量的变化
            if (fabs(dx) < matching_variation_th_ &&
                fabs(dy) < matching_variation_th_ &&
                fabs(dz) < matching_variation_th_) {

                // 更新上次匹配偏移量
                last_matching_offset_.x += dx;
                last_matching_offset_.y += dy;
                last_matching_offset_.z += dz;
                std::cout << "Matching Success.\n" << "last_matching_offset_: " << last_matching_offset_.x << ", " << last_matching_offset_.y << ", " << last_matching_offset_.z << "\ndx: " << dx << ", dy: " << dy << ", dz: " << dz << std::endl;

                return true;  // 匹配成功
            }
        }

        return false;  // 匹配失败
    }

    void timerCallback(const ros::TimerEvent&) {
        // 定时发布最新的处理数据
        masspoint_pub_.publish(latest_transformed_masspoints_);
    }
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
