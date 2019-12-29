#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from imutils.object_detection import non_max_suppression
from cv_bridge import CvBridge, CvBridgeError
import imutils
from kalmanF import KalmanFilter

class detector:

  def __init__(self):

    self.image_pub = rospy.Publisher("template",Image,queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
    self.HOGCascade = cv2.HOGDescriptor()
    self.HOGCascade.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    dt = 1.0/60
    self.F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
    self.H = np.array([1, 0, 0]).reshape(1, 3)
    self.Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
    self.R = np.array([0.5]).reshape(1, 1)
    self.kf = KalmanFilter(F = self.F, H = self.H, Q = self.Q, R = self.R)
    self.kf2 = KalmanFilter(F = self.F, H = self.H, Q = self.Q, R = self.R)
    self.prediction_x = np.array([0],np.uint8)
    self.prediction_y = np.array([0],np.uint8)
    self.horizontal = np.array([80],np.uint8)
    self.vertical = np.array([80],np.uint8)

  def getImage(self,image):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
      return cv_image
    except CvBridgeError as e:
      print(e)

  def pubImage(self,image):
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    
  def callback(self,data):
    
    frame = self.getImage(data)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(gray,(5,5),0)
    #image = cv2.medianBlur(image,5)
    image = imutils.resize(image, width=700)
    clahe = cv2.createCLAHE(clipLimit=15.0,tileGridSize=(8,8))
    image = clahe.apply(image)

    winStride = (8,8)
    padding = (16,16)

    (rects, weights) = self.HOGCascade.detectMultiScale(image, winStride=winStride,
                                            padding=padding,
                                            scale=1.05)

    #for (x, y, w, h) in rects:
    #    cv2.rectangle(frame, (x, y), (x+w, y+h), (0,200,255), 2)

    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    
    if len(pick)>0:
      #print(pick[0][0],pick[0][1],pick[0][2],pick[0][3])
      #cv2.rectangle(frame, (pick[0][0], pick[0][1]), (pick[0][2], pick[0][3]), (0, 255, 255), 2)

      self.horizontal =  pick[0][2] - pick[0][0] 
      self.vertical =  pick[0][3] - pick[0][1]  

      self.prediction_x = (np.dot(self.H, self.kf.predict())[0])
      self.prediction_y = (np.dot(self.H, self.kf2.predict())[0])

      self.kf.update(pick[0][0])
      self.kf2.update(pick[0][1])

      #temp = frame[pick[0][1]:pick[0][3], pick[0][0]:pick[0][2]]
      #cv2.imshow("people",temp)
      #self.pubImage(temp)
      print("detected at: ", pick, "  predicted x at: ", self.prediction_x,"  predicted y at: ",
        self.prediction_y)

    cv2.rectangle(frame,(int(self.prediction_x),int(self.prediction_y)),
      (int(self.prediction_x + self.horizontal),int(self.prediction_y + self.vertical)),(255,0,0),2)
    
    temp = frame[int(self.prediction_y):int(self.prediction_y + self.vertical) , 
    int(self.prediction_x):int(self.prediction_x+self.horizontal)]
    #cv2.imshow("people",temp)
    self.pubImage(temp)

    cv2.imshow("Frame",frame)

    cv2.waitKey(3)
    

def main(args):
  ic = detector()
  rospy.init_node('detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)