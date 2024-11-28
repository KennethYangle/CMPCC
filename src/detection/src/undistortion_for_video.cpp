#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;
using namespace std;

class ImageUndistortionNode {
public:
    ImageUndistortionNode() {
        // 初始化 ROS 发布者
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image_raw/undistorted", 1);

        // 初始化相机内参矩阵
        K_ = (Mat_<double>(3, 3) << 777.8701, 0, 962.2362,
                                    0, 777.8701, 548.3110,
                                    0, 0, 1);
        // 初始化畸变系数
        D_ = (Mat_<double>(1, 4) << -4.4708e-05, 6.5542e-07, -1.3420e-09, 0);

        // 打开视频设备（默认 /dev/video0）
        if (!cap_.open(0)) {
            ROS_ERROR("Failed to open video device.");
            ros::shutdown();
        } else {
            ROS_INFO("Video device opened successfully.");
        }
    }

    void spin() {
        Mat frame;

        // 设置摄像头分辨率（例如 1280x720）
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

        while (ros::ok()) {
            // 从摄像头捕获帧
            cap_ >> frame;
            if (frame.empty()) {
                ROS_ERROR("Captured empty frame.");
                continue;
            }

            // 打印捕获帧信息
            // ROS_INFO_THROTTLE(1, "Captured frame size: %dx%d", frame.cols, frame.rows);
            // 显示捕获的图像
            //if (!frame.empty()) {
            //    cv::imshow("Captured Frame", frame);  // 显示捕获的原始图像
            //    cv::waitKey(1);                       // 确保窗口实时刷新
            //} else {
            //    ROS_WARN_THROTTLE(1, "Captured frame is empty.");
            //}
            
            // 输出图像尺寸（与输入图像相同）
            Size image_size = frame.size();

            Mat new_K;
            K_.copyTo(new_K);

            // 调整视场大小系数（例如 1.0 为原始大小，0.8 为更大视场）
            double scale_factor = 0.4;
            new_K.at<double>(0, 0) *= scale_factor;
            new_K.at<double>(1, 1) *= scale_factor;

            // 去畸变处理
            Mat undistorted_frame;
            fisheye::undistortImage(frame, undistorted_frame, K_, D_, new_K);

            if (undistorted_frame.empty()) {
                ROS_ERROR("Undistorted frame is empty.");
                continue;
            }

            //if (!undistorted_frame.empty()) {
            //    cv::imshow("Captured Frame", undistorted_frame);  // 显示捕获的原始图像
            //    cv::waitKey(1);                       // 确保窗口实时刷新
            //} else {
            //    ROS_WARN_THROTTLE(1, "Undistorted frame is empty.");
            //}

            // 转换为 ROS 图像消息并发布
            sensor_msgs::ImagePtr undistorted_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted_frame).toImageMsg();
            image_pub_.publish(undistorted_msg);

            // 控制发布频率
            ros::Rate(30).sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher image_pub_;
    VideoCapture cap_;  // OpenCV 视频捕获设备
    Mat K_;             // 相机内参矩阵
    Mat D_;             // 畸变系数
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_undistortion_node");
    ImageUndistortionNode undistortion_node;
    undistortion_node.spin();
    return 0;
}
