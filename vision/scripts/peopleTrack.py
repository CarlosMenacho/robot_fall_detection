#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
import numpy as np 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
  

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/mynteye/right/image_raw",Image,self.callback)
    self.upperbody = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

  def getImage(self,image):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
      return cv_image
    except CvBridgeError as e:
      print(e)

  def callback(self,data):
    frame = self.getImage(data)

    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    pick= self.upperbody.detectMultiScale(gray,1.3,5)
    
    if len(pick)>0:
      pick[0][2]=pick[0][2]+pick[0][0]
      pick[0][3]=pick[0][3]+pick[0][1] 
      cv2.rectangle(frame, (pick[0][0], pick[0][1]), (pick[0][2], pick[0][3]), (0, 255, 255), 2)
    cv2.imshow("Image window", frame)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
