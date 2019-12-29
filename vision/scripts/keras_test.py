#!/usr/bin/env python

import sys, time
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import load_model, Sequential
from tensorflow.keras.preprocessing import image
from numpy import loadtxt
from tensorflow.keras.optimizers import RMSprop

is_frame_changed = 0
image_data = np.array([])
prev_1 = None
prev_2 = None

def set_frame_changed(val):
    global is_frame_changed
    is_frame_changed = val

def callback(ros_data):
    global image_data 

    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image_data = cv2.resize(image_np,(224,224))
    image_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
    set_frame_changed(1)


if __name__ == '__main__':
    
    rospy.init_node('falling')
    subscriber = rospy.Subscriber("/mynteye/right/image_raw/compressed",CompressedImage, callback,  queue_size = 1)
    model = load_model('/home/carlos/peopleF_ws/src/vision/scripts/custom_model.h5')
    model.load_weights('/home/carlos/peopleF_ws/src/vision/scripts/custom_model_weights.h5')
    model.compile(loss='binary_crossentropy',optimizer=RMSprop(lr=0.0001), metrics=['accuracy'])

    print(model.summary())

    while not rospy.is_shutdown():

        if is_frame_changed==1:
            if prev_1 is None:
                prev_1 = image_data
            if prev_2 is None:
                prev_2 = prev_1
                prev_1 = image_data
            try:
                new = np.stack((image_data, prev_1, prev_2), axis = -1)
                fall = np.expand_dims(new,axis=0)
                preds=model.predict(fall)
                print(preds)
                cv2.imshow("frame", new)
                cv2.waitKey(1)
            except Exception as e:
                print(e)
            

            prev_2 = prev_1
            prev_1 = image_data
            set_frame_changed(0)

