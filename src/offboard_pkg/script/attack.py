#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
import math
import threading
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from swarm_msgs.msg import BoundingBox, BoundingBoxes
from quadrotor_msgs.msg import PositionCommand
from utils_att import Utils


mav_pos = [0, 0, 0]
mav_vel = np.array([0, 0, 0])
mav_yaw = 0
mav_R = np.zeros((3,3))
pos_i = [0, 0, 0, 0, 0]
pos_i_raw = [0, 0, 0, 0, 0]
image_failed_cnt = 0

#attack
command = PositionCommand()
command.position.x = command.position.y = command.position.z = 0
command.velocity.x = command.velocity.y = command.velocity.z = 0

image_failed_max_cnt = 5 #
last_weight = False


def spin():
    rospy.spin()

def mav_pose_cb(msg):
    global mav_pos, mav_yaw, mav_R
    mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    mav_yaw = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    mav_R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

def mav_vel_cb(msg):
    global mav_vel
    mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

def pos_image_cb(msg):
    global pos_i_raw, pos_i, image_failed_cnt, image_failed_max_cnt
    global img_f, img_height, img_width, mav_R
    sphere_num = len(msg.bounding_boxes)
    if sphere_num > 0:
        image_failed_cnt = 0
    else:
        image_failed_cnt += 1
    if image_failed_cnt <= image_failed_max_cnt and image_failed_cnt > 0:
        pass
    else:
        if sphere_num > 0:
            idx = 0
            max_area = -1e10
            for i in range(sphere_num):
                bw = msg.bounding_boxes[i].xmax - msg.bounding_boxes[i].xmin
                bh = msg.bounding_boxes[i].ymax - msg.bounding_boxes[i].ymin
                barea = bw * bh
                if max_area > barea:
                    max_area = barea
                    idx = i

            xmiddle = (msg.bounding_boxes[idx].xmin + msg.bounding_boxes[idx].xmax) / 2
            ymiddle = (msg.bounding_boxes[idx].ymin + msg.bounding_boxes[idx].ymax) / 2
            bw = msg.bounding_boxes[idx].xmax - msg.bounding_boxes[idx].xmin
            bh = msg.bounding_boxes[idx].ymax - msg.bounding_boxes[idx].ymin
            outdata = [xmiddle, ymiddle, bw, bh]

            pos_i_raw = outdata
            pos_i = pos_i_raw
            command.weight = calc_weight(bw, img_width)
        else:
            outdata = [-1, -1, -1, -1]
            pos_i_raw = outdata
            pos_i = pos_i_raw
            command.weight = 0.
    
def calc_weight(w, w_max):
    global last_weight
    if w > 30 or (last_weight and w > 20):
        last_weight = True
        return 1.
    else:
        last_weight = False
        return w / 30.


if __name__=="__main__":
    rospy.init_node('attack_node', anonymous=True)
 
    img_width = rospy.get_param("/camera/img_width")
    img_height = rospy.get_param("/camera/img_height")
    img_f = rospy.get_param("/camera/img_f")
    distortion_coeffs = np.array(rospy.get_param('camera/distortion_coeffs'))
    R_cb = np.array(rospy.get_param('camera/R_cam_body')).reshape(3, 3)
    image_center = [img_width / 2.0, img_height / 2.0]

    params = {"WIDTH": img_width, "HEIGHT": img_height, "FOC": img_f, "DISTORTION": distortion_coeffs, "R_cb": R_cb}
    print(params)
    u = Utils(params)

    spin_thread = threading.Thread(target = spin)
    spin_thread.start()

    rospy.Subscriber("mavros/local_position/pose", PoseStamped, mav_pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, mav_vel_cb)
    rospy.Subscriber("tracker/pos_image", BoundingBoxes, pos_image_cb)
    local_acc_pub = rospy.Publisher('/attack_cmd', PositionCommand, queue_size=5)

    rate = rospy.Rate(150)  # 50
    
    controller_reset = True
    while not rospy.is_shutdown():
        pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_R": mav_R, "R_bc": np.array([[0,0,1], [1,0,0], [0,1,0]]), "mav_yaw": mav_yaw}
        
        if pos_i[0] > 0:
            cmd = u.RotateAttackAccelerationController2(pos_info, pos_i, controller_reset)
            command.header.stamp = rospy.Time.now()
            command.acceleration.x = cmd[0]
            command.acceleration.y = cmd[1]
            command.acceleration.z = cmd[2]
            command.yaw_dot = cmd[3]
            command.velocity.x = cmd[4]
            command.velocity.y = cmd[5]
            command.velocity.z = cmd[6]
        else:
            command.header.stamp = rospy.Time.now()
            command.acceleration.x = 0
            command.acceleration.y = 0
            command.acceleration.z = 0
            command.yaw_dot = 0
            command.velocity.x = 0
            command.velocity.y = 0
            command.velocity.z = 0
        local_acc_pub.publish(command)

        rate.sleep()
    rospy.spin()