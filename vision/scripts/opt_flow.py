#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

class image_converter:

  def __init__(self):
    self.frame1 = None
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/mynteye/left_rect/image_rect",Image,self.callback)
    self.hsv = None

  def callback(self,data):
    try:
      frame2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    frame2 = cv2.resize(frame2,(480,360))

    if self.frame1 is None:
    	print("frame1 is none")
    	self.frame1 = frame2
    	self.frame1 = cv2.cvtColor(self.frame1,cv2.COLOR_BGR2GRAY)
    	self.hsv = np.zeros_like(frame2)
    	self.hsv[...,1] = 255
    	print(self.frame1.shape, frame2.shape, self.hsv.shape)

    frame2 = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    flow = cv2.calcOpticalFlowFarneback(self.frame1,frame2, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
    self.hsv[...,0] = ang*180/np.pi/2
    self.hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
    rgb = cv2.cvtColor(self.hsv,cv2.COLOR_HSV2BGR)

    cv2.imshow("Image window", rgb)
    cv2.waitKey(3)
    self.frame1=frame2

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