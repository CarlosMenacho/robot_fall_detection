#!/usr/bin/env python
# -*- coding: utf-8 -*-
#/mynteye/depth/image_raw             mynt
#/zed_node/depth/depth_registered     zed
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3, Quaternion, Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class depth_processing():

    def __init__(self):

        rospy.init_node('mynteye_depth', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("/mynteye/depth/image_raw", Image, self.callback, queue_size=1)
        rospy.Subscriber("/bounding_box",Quaternion , self.pos_callback, queue_size=1)
        rospy.Subscriber("/follow", Int8, self.callback_fall, queue_size = 1)
        self.cmd_robot = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.follow_enable = Int8()
        self.follow_enable = 0
        self.kp = 0.008
        self.kp_z = 0.5
        self.position = Quaternion()
        self.run_robot = Twist()
        


    def callback(self, depth_data):
        if self.follow_enable == 1:
            if self.position.x and self.position.y != 0:            
                try:
                    depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
                except CvBridgeError, e:
                    print e

                x1 = int(self.position.x)
                y1 = int(self.position.y)

                x2 = int(self.position.z)
                y2 = int(self.position.w)

                x_setPoint = depth_image.shape[1]/2
                x = (self.position.x + self.position.z)/2
                y = (self.position.y + self.position.w)/2
                error = x_setPoint - x
                cmd_z = round(self.kp * error, 2)

                z_setPoint = 1.1

                depth_array = np.array(depth_image, dtype=np.float32)
                area = depth_array[y1:y2, x1:x2]
                dist = np.min(area)/1000

                error_z = z_setPoint - dist

                cmd_x = round(self.kp_z * error_z, 2) 

                if dist<10:

                  self.run_robot.linear.x = cmd_x*-1
                  self.run_robot.angular.z = cmd_z
                  self.cmd_robot.publish(self.run_robot)
                else: 
                  self.run_robot.linear.x = 0
                  self.run_robot.angular.z = 0
                  self.cmd_robot.publish(self.run_robot)

                print("angular: ", cmd_z, "  linear: ", cmd_x, "  dist: ", dist)
            else:
              self.run_robot.linear.x = 0
              self.run_robot.angular.z = 0
              self.cmd_robot.publish(self.run_robot)  
              rospy.logwarn("no detection")
    def callback_fall(self,data):
        self.follow_enable = data.data

    def pos_callback(self, data):
        self.position.x = data.x
        self.position.y = data.y
        self.position.z = data.z
        self.position.w = data.w


if __name__ == '__main__': 
    try:
        detector = depth_processing()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Detector node terminated.")