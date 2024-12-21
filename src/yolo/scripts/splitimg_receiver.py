#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

# 创建 CvBridge 对象
bridge = CvBridge()

# 保存图像的文件夹路径
output_directory = '/home/nvidia/record/extracted_images/'

# 确保输出目录存在
if not os.path.exists(output_directory):
    os.makedirs(output_directory)

# 图像回调函数
def image_callback(msg):
    try:
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 获取图像的时间戳并创建文件名
        timestamp = rospy.get_time()  # 获取当前时间戳
        file_name = output_directory + "split_image_{:.2f}.jpg".format(timestamp)

        # 保存图像
        cv2.imwrite(file_name, cv_image)
        rospy.loginfo("Image saved to: %s", file_name)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)

def main():
    # 初始化 ROS 节点
    rospy.init_node('image_subscriber_node', anonymous=True)

    # 订阅话题 "/split_images"
    rospy.Subscriber('/split_images', Image, image_callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()
