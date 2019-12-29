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
    self.image_sub = rospy.Subscriber("/mynteye/depth/image_raw",Image,self.callback,queue_size=1)

  def callback(self,depth_data):
    try:
      depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32SC1")
    except CvBridgeError as e:
      print(e)

    depth_array = np.array(depth_image, dtype=np.float32)

    print('Image size: {width}x{height}'.format(width=depth_data.width,height=depth_data.height))

    u = depth_data.width/2
    v = depth_data.height/2

    print('Center depth: {dist} m'.format(dist=depth_array[u,v]))


    cv2.imshow("Image window", depth_image)
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
