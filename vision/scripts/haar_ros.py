#!/usr/bin/env python
#/zed_node/right/image_rect_color             Zed
#/mynteye/right/image_raw                     mynt
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8

import numpy as np 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion, Twist
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    rospy.Subscriber("/mynteye/right/image_raw",Image,self.callback, queue_size=1)
    rospy.Subscriber("/follow", Int8,self.callback_follow, queue_size = 1)
    self.detected = rospy.Publisher("/bounding_box",Quaternion, queue_size = 1)
    self.upperbody = cv2.CascadeClassifier('/home/carlos/peopleF_ws/src/vision/scripts/haarcascade_upperbody.xml')
    self.seguir = Int8()
    self.seguir = 0
    self.people_pos = Quaternion()

  def getImage(self,image):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
      return cv_image
    except CvBridgeError as e:
      print(e)

  def callback(self,data):
    if self.seguir==1:
      frame = self.getImage(data)

      gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

      pick= self.upperbody.detectMultiScale(gray,1.3,5)

      if len(pick)>0:

        pick[0][2]=pick[0][2]+pick[0][0]
        pick[0][3]=pick[0][3]+pick[0][1] 
        #cv2.rectangle(frame, (pick[0][0], pick[0][1]), (pick[0][2], pick[0][3]), (0, 255, 255), 2)

        self.people_pos.x = pick[0][0]
        self.people_pos.y = pick[0][1]
        self.people_pos.z = pick[0][2]
        self.people_pos.w = pick[0][3]
        self.detected.publish(self.people_pos)

      else: 
        self.people_pos.x = 0
        self.people_pos.y = 0
        self.people_pos.z = 0
        self.people_pos.w = 0
        self.detected.publish(self.people_pos)

      #cv2.imshow("Image window", frame)
      #cv2.waitKey(1)
  def callback_follow(self, data):
    self.seguir=data.data


def main(args):
  rospy.init_node('people_detector', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
