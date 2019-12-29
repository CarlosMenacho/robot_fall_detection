#!/usr/bin/env python

import sys, time
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import load_model, Sequential
from tensorflow.keras.preprocessing import image
from numpy import loadtxt
from tensorflow.keras.optimizers import RMSprop

import time

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
    subscriber = rospy.Subscriber("/image/to_fall",CompressedImage, callback,  queue_size = 1)

    model = load_model('/home/carlos/peopleF_ws/src/vision/scripts/model/vgg_opt_custom_model.h5')
    model.load_weights('/home/carlos/peopleF_ws/src/vision/scripts/model/vgg_opt_custom_model_weights.h5')
    model.compile(loss='binary_crossentropy',optimizer=RMSprop(lr=0.0001), metrics=['accuracy'])

    print(model.summary())

    while not rospy.is_shutdown():
        start_time = time.time()
        if is_frame_changed==1:
            if prev_1 is None:
                print("frame1 is none")
                prev_1 = image_data

            try:
        
                flow = cv2.calcOpticalFlowFarneback(prev_1,image_data, None, 0.5, 3, 15, 3, 5, 1.2, 0)
                
                vert = cv2.normalize(flow[...,1], None, 0, 255, cv2.NORM_MINMAX)
                vert = vert.astype('uint8')
                kernel = np.ones((15,15),np.float32)/225
                smoothed = cv2.filter2D(vert,-1,kernel)

                back = cv2.cvtColor(smoothed,cv2.COLOR_GRAY2RGB)

                cv2.imshow("img", back)

                fall = np.expand_dims(back ,axis=0)
                preds=model.predict(fall)
                if preds[0][0] < 0.5:
                    print("----------------caida--------------------------")
                cv2.imshow("frame", image_data)
                cv2.waitKey(1)
                print(preds," FPS: ", 1.0/(time.time()-start_time))
            except Exception as e:
                print(e)
            
            prev_1 = image_data
            set_frame_changed(0)
