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

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import load_model, Sequential
from tensorflow.keras.preprocessing import image
from numpy import loadtxt
from tensorflow.keras.optimizers import RMSprop

class image_converter:
  def __init__(self):

    self._session = tf.Session()
    self.graph = tf.get_default_graph()
    self.model = load_model('/home/carlos/peopleF_ws/src/vision/scripts/custom_model.h5')
    self.model.load_weights('/home/carlos/peopleF_ws/src/vision/scripts/custom_model_weights.h5')
    
    print(self.model.summary())

    self.model.compile(loss='binary_crossentropy',optimizer=RMSprop(lr=0.0001), metrics=['accuracy'])

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.model._make_predict_function()
    
    self.prev_1 = None
    self.prev_2 = None
    self.prev_3 = None
    

  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    frame = cv2.resize(frame,(224,224))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if self.prev_1 is None:
      self.prev_1 = frame
      #break

    if self.prev_2 is None:
      self.prev_2 = self.prev_1
      self.prev_1 = frame
      #break

    if self.prev_3 is None:
      self.prev_3 = self.prev_2
      self.prev_2 = self.prev_1
      self.prev_1 = frame
      #break
    #try:
    new = np.stack((frame, self.prev_1, self.prev_2), axis = -1)
      
    self.prev_3 = self.prev_2
    self.prev_2 = self.prev_1
    self.prev_1 = frame
    fall = np.expand_dims(new,axis=0)
    print(fall.shape)
    #prediction = self.model.predict_classes(fall)
    with self.graph.as_default():
      prediction = self.model.predict(fall)
      print(prediction)
    cv2.imshow("Image window", new)
    cv2.imshow("other", frame)
    #except Exception as e:
    #  print("wait !! ")
      
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


