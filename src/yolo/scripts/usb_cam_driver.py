#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""

Copyright (c) 2015 PAL Robotics SL.
Released under the BSD License.

Created on 7/14/15

@author: Sammy Pfeiffer

test_video_resource.py contains
a testing code to see if opencv can open a video stream
useful to debug if video_stream does not work
"""

import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class USBCamDriver:
    def __init__(self):
        rospy.init_node('usb_cam_driver', anonymous=True)

        # ROS parameters
        self.image_width = rospy.get_param("/camera/img_width", 640)  # Default width
        self.image_height = rospy.get_param("/camera/img_height", 480) # Default height
        self.is_show_image = rospy.get_param("/Debug/is_show_image_driver", True)  # Show image?

        # ROS publisher
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

        self.bridge = CvBridge() # Used to convert OpenCV images to ROS Image messages

        # Video Capture setup
        resource = 0
        self.cap = cv2.VideoCapture(resource)
        if not self.cap.isOpened():
            rospy.logerr("Error opening resource: %s", str(resource))
            rospy.logerr("Maybe opencv VideoCapture can't open it")
            rospy.signal_shutdown("Failed to open video resource.")
            return

        rospy.loginfo("Correctly opened resource, starting to show feed.")
        self.run()

    def run(self):
        rval, frame = self.cap.read()
        if not rval:
            rospy.logwarn("Initial frame read failed.  Closing.")
            self.cap.release()
            rospy.signal_shutdown("Failed to read initial frame.")
            return

        while not rospy.is_shutdown() and rval:
            # Resize the frame
            frame = cv2.resize(frame, (self.image_width, self.image_height))

            # Publish the image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8")) #Assuming BGR color
            except CvBridgeError as e:
                rospy.logerr(e)

            if self.is_show_image:
                cv2.imshow("Stream: " + rospy.get_name(), frame)  #Using ROS node name for window
                key = cv2.waitKey(20) # Show image for 20 ms
                if key == 27 or key == 1048603: # ESC key
                    break

            rval, frame = self.cap.read()

        # Clean up
        self.cap.release()
        if self.is_show_image:
            cv2.destroyAllWindows()
        rospy.loginfo("Closing video stream.")


if __name__ == '__main__':
    try:
        usb_cam_driver = USBCamDriver()
        #rospy.spin()  # Handled within the class's run method now.
    except rospy.ROSInterruptException:
        pass