#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <swarm_msgs/BoundingBox.h>
#include <swarm_msgs/BoundingBoxes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <tf/tf.h>

// 是否显示图像
bool is_show_rgb = false;
bool is_show_hsv = false;
bool is_show_3D_estimation = false;

// 发布器
ros::Publisher imag_pub, marker_pub;

// 相机参数
cv::Mat R_cam_body(3, 3, CV_64F); // 相机到机体的旋转矩阵
double img_f, target_size;
int img_width, img_height;

// 无人机姿态
geometry_msgs::PoseStamped mav_pose;
cv::Mat R_body_world_cv(3, 3, CV_64F);
cv::Mat T_body_world;

void publishTargetMarkers(swarm_msgs::BoundingBoxes img_pos) {
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < img_pos.bounding_boxes.size(); ++i) {
        const auto& bbox = img_pos.bounding_boxes[i];

        // 创建 Marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_markers";
        marker.id = i;  // 每个目标的唯一 ID
        marker.type = visualization_msgs::Marker::SPHERE;  // 使用球体表示目标
        marker.action = visualization_msgs::Marker::ADD;

        // 设置目标的 3D 位置
        marker.pose.position = bbox.position;

        // 设置默认方向
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 设置大小
        marker.scale.x = 0.5;  // 球体直径
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // 设置颜色
        marker.color.r = 0.0;  // 蓝色
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;  // 完全不透明

        // Marker 不过期
        marker.lifetime = ros::Duration(0);

        marker_array.markers.push_back(marker);
    }

    // 发布 MarkerArray
    marker_pub.publish(marker_array);
}

// 无人机姿态回调函数
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    mav_pose = *msg;

    // 转换到世界坐标系
    tf::Quaternion q(mav_pose.pose.orientation.x, mav_pose.pose.orientation.y, mav_pose.pose.orientation.z, mav_pose.pose.orientation.w);
    tf::Matrix3x3 R_body_world(q);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_body_world_cv.at<double>(i, j) = R_body_world[i][j];

    T_body_world = (cv::Mat_<double>(3, 1) << mav_pose.pose.position.x, mav_pose.pose.position.y, mav_pose.pose.position.z);
}

// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    swarm_msgs::BoundingBoxes img_pos;
    img_pos.header.stamp = ros::Time::now();
    cv_bridge::CvImagePtr cv_ptr;

    try {
        // 将ROS图像消息转为OpenCV图像
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat cv_img = cv_ptr->image;
    cv::Mat hue_image, th1, th2, th;

    // 转换为HSV颜色空间
    cv::cvtColor(cv_img, hue_image, cv::COLOR_BGR2HSV);

    // 定义HSV范围
    cv::inRange(hue_image, cv::Scalar(171, 80, 40), cv::Scalar(180, 256, 256), th1);
    cv::inRange(hue_image, cv::Scalar(0, 80, 40), cv::Scalar(10, 256, 256), th2);
    th = th1 + th2;

    // 图像膨胀
    cv::dilate(th, th, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1), 2);

    // 连通域分析, https://stackoverflow.com/questions/35854197/how-to-use-opencvs-connected-components-with-stats-in-python/35854198#35854198
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(th, labels, stats, centroids);

    int id = 0;
    for (int i = 1; i < num_labels; ++i) { // 0是背景
        int left = stats.at<int>(i, cv::CC_STAT_LEFT);
        int top = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        if (area > 25) {
            // 提取连通域对应的像素
            cv::Mat mask = (labels == i);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 如果轮廓点数足够，拟合椭圆
            if (!contours.empty() && contours[0].size() >= 5) { // 椭圆拟合至少需要5个点
                cv::RotatedRect ellipse = cv::fitEllipse(contours[0]);

                // 排除异常情况：面积太小/太大，或椭圆长宽比异常
                if (ellipse.size.width / ellipse.size.height > 10 || ellipse.size.height / ellipse.size.width > 10) continue;

                // 创建BoundingBox并赋值
                swarm_msgs::BoundingBox bbox;
                bbox.Class = "Ballon";
                bbox.probability = 1.0;
                bbox.xmin = left;
                bbox.ymin = top;
                bbox.xmax = left + width;
                bbox.ymax = top + height;
                bbox.id = id++;
                bbox.a = ellipse.size.width / 2.0;  // 半长轴
                bbox.b = ellipse.size.height / 2.0; // 半短轴

                // 计算目标三维位置
                double diameter = std::min(ellipse.size.width, ellipse.size.height);
                double Z = target_size * img_f / diameter;
                double u = ellipse.center.x, v = ellipse.center.y;

                double X_cam = (u - img_width / 2.0) * Z / img_f;
                double Y_cam = (v - img_height / 2.0) * Z / img_f;

                cv::Mat P_cam = (cv::Mat_<double>(3, 1) << X_cam, Y_cam, Z);
                cv::Mat P_body = R_cam_body * P_cam;
                cv::Mat P_world = R_body_world_cv * P_body + T_body_world;

                bbox.position.x = P_world.at<double>(0, 0);
                bbox.position.y = P_world.at<double>(1, 0);
                bbox.position.z = P_world.at<double>(2, 0);

                img_pos.bounding_boxes.push_back(bbox);
                // std::cout << "left: " << bbox.xmin << ", top: " << bbox.ymin << ", width: " << ellipse.size.width << ", height: " << ellipse.size.height << std::endl;

                // 绘制椭圆和边界框
                cv::rectangle(cv_img, cv::Point(bbox.xmin, bbox.ymin), cv::Point(bbox.xmax, bbox.ymax), cv::Scalar(0, 255, 0), 2);
                cv::ellipse(cv_img, ellipse, cv::Scalar(255, 0, 0), 2);
            }
        }
    }

    // 按横坐标排序
    std::sort(img_pos.bounding_boxes.begin(), img_pos.bounding_boxes.end(), [](const swarm_msgs::BoundingBox& a, const swarm_msgs::BoundingBox& b) {
        return (a.xmin + a.xmax) / 2 < (b.xmin + b.xmax) / 2;
    });

    // 更新ID
    id = 0;
    for (auto& bbox : img_pos.bounding_boxes) {
        bbox.id = id++;
    }

    // 发布检测结果
    imag_pub.publish(img_pos);

    // 显示图像
    if (is_show_rgb) {
        cv::imshow("img", cv_img);
        cv::waitKey(1);
    }
    if (is_show_hsv) {
        cv::imshow("hsv", th);
        cv::waitKey(1);
    }
    if (is_show_3D_estimation) {
        publishTargetMarkers(img_pos);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "iris_fpv_cam");
    ros::NodeHandle nh;

    // 获取参数
    nh.param("/Debug/is_show_rgb", is_show_rgb, false);
    nh.param("/Debug/is_show_hsv", is_show_hsv, false);
    nh.param("/Debug/is_show_3D_estimation", is_show_3D_estimation, false);
    nh.getParam("/camera/img_width", img_width);
    nh.getParam("/camera/img_height", img_height);
    nh.getParam("/camera/img_f", img_f);
    nh.getParam("/target_size", target_size);
    std::vector<double> R_cam_body_vector;
    if (nh.getParam("/camera/R_cam_body", R_cam_body_vector)) {
        // 确保数据大小正确，例如 3x3 的旋转矩阵
        if (R_cam_body_vector.size() == 9) {
            R_cam_body = cv::Mat(3, 3, CV_64F, R_cam_body_vector.data()).clone();
        } else {
            ROS_ERROR("R_cam_body parameter size mismatch! Expected 9 elements for a 3x3 matrix.");
            return -1;
        }
    } else {
        ROS_ERROR("Failed to get R_cam_body from parameter server.");
        return -1;
    }

    ROS_INFO("is_show_rgb: %d", is_show_rgb);
    ROS_INFO("is_show_hsv: %d", is_show_hsv);
    ROS_INFO("is_show_3D_estimation: %d", is_show_3D_estimation);

    // 订阅图像话题
    ros::Subscriber image_sub = nh.subscribe("/camera/left", 1, imageCallback);

    // 订阅无人机姿态话题
    ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 1, poseCallback);

    // 发布检测框话题
    imag_pub = nh.advertise<swarm_msgs::BoundingBoxes>("/tracker/pos_image", 10);

    // Marker 发布器
    if (is_show_3D_estimation) {
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/tracker/3D_estimation_marker", 10);
    }

    // 开启回调
    ros::spin();

    return 0;
}