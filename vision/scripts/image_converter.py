#!/usr/bin/env python

import sys, time
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge, CvBridgeError

class image_feature:

    def __init__(self):
        self.image_pub = rospy.Publisher("/image/toFall",CompressedImage,queue_size = 1)
        self.bridge = CvBridge()
        rospy.Subscriber("/enable_fall",Int8,self.fall_callblack, queue_size =1)
        rospy.Subscriber("/mynteye/left/image_raw",Image,self.callback, queue_size = 1)
        
        self.enable = Int8()
        self.enable = 0
        self.cont = 0
        #/zed_node/right/image_rect_color            zed
        #

    def callback(self, data):

    	if self.enable == 1:
	        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
	        #cv2.imshow('cv_img', frame)
	        #cv2.waitKey(2)
	        msg = CompressedImage()
	        msg.header.stamp = rospy.Time.now()
	        msg.format = "jpeg"
	        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
	        self.image_pub.publish(msg)
	        cv2.imwrite("/home/carlos/peopleF_ws/src/vision/imgs/"+str(self.cont)+".png", frame)
	        self.cont+=1

    def fall_callblack(self, data):
    	self.enable = data.data

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
