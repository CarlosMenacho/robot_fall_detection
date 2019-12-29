#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("template",Image,self.callback)
    self.lk_params = dict(winSize = (15, 15), maxLevel = 4, 
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    self.last_frame = None
    self.old_gray = None
    self.old_point = np.array([[]])
    self.color = (0,255,0)

  def getImage(self,image):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding = "bgr8")
      return cv_image
    except CvBridgeError as e:
      print(e)

  def callback(self,data):

    current_frame = self.getImage(data)
    current_frame = cv2.resize(current_frame,(90,200))

    gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

    i_y,i_x = gray_frame.shape

    cv2.circle( current_frame, (i_x/2,i_y/2), 5, self.color,2 )

    if self.old_gray is None:
      self.old_gray = gray_frame
      self.old_point = np.array([[i_y/2, i_x/2]], dtype = np.float32)
      
      print(self.old_point)

    new_points, status, error = cv2.calcOpticalFlowPyrLK(self.old_gray, gray_frame, self.old_point, None, **self.lk_params)

    self.old_gray = gray_frame.copy()
    self.old_point = new_points
    x , y = new_points.ravel()
    print(x,y)
    cv2.circle(current_frame, (x,y), 5, (0,0,255), -1)

    cv2.imshow("falling detection ", current_frame)
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