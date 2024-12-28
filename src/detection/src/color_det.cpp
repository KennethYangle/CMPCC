#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <swarm_msgs/BoundingBox.h>
#include <swarm_msgs/BoundingBoxes.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

// 是否显示图像
bool is_show_rgb = false;
bool is_show_hsv = false;

// 发布器
ros::Publisher imag_pub;

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
            // 创建BoundingBox并赋值
            swarm_msgs::BoundingBox bbox;
            bbox.Class = "Ballon";
            bbox.probability = 1.0;
            bbox.xmin = left;
            bbox.ymin = top;
            bbox.xmax = left + width;
            bbox.ymax = top + height;
            bbox.id = id++;
            bbox.a = width;
            bbox.b = height;
            img_pos.bounding_boxes.push_back(bbox);

            // 绘制椭圆和边界框
            cv::rectangle(cv_img, cv::Point(bbox.xmin, bbox.ymin), cv::Point(bbox.xmax, bbox.ymax), cv::Scalar(0, 255, 0), 2);
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
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "iris_fpv_cam");
    ros::NodeHandle nh;

    // 获取参数
    nh.param("/Debug/is_show_rgb", is_show_rgb, false);
    nh.param("/Debug/is_show_hsv", is_show_hsv, false);
    ROS_INFO("is_show_rgb: %d", is_show_rgb);
    ROS_INFO("is_show_hsv: %d", is_show_hsv);

    // 订阅图像话题
    ros::Subscriber image_sub = nh.subscribe("/camera/left", 1, imageCallback);

    // 发布检测框话题
    imag_pub = nh.advertise<swarm_msgs::BoundingBoxes>("/tracker/pos_image", 10);

    // 开启回调
    ros::spin();

    return 0;
}