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
from keras.applications.vgg16 import preprocess_input
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array
from keras.models import model_from_json


class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    self.save_model_path = '/home/carlos/peopleF_ws/src/vision/scripts/model/'
    self.name = 'VGG16_V9'
    # load json and create model
    self.json_file = open(self.save_model_path+self.name+'.json', 'r')
    self.loaded_model_json = self.json_file.read()
    self.json_file.close()
    self.loaded_model = model_from_json(self.loaded_model_json)
    # load weights into new model
    self.loaded_model.load_weights(self.save_model_path+self.name+'.h5')
    print("Loaded model from disk")
    print(self.loaded_model.summary())


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # print((cv_image).shape)
    image = cv_image[100:324,]
    image = cv2.resize(image, (224,224), interpolation = cv2.INTER_AREA)
    # convert the image to an array
    img = img_to_array(image)
    # expand dimensions so that it represents a single 'sample'
    img = np.expand_dims(img, axis=0)
    # prepare the image (e.g. scale pixel values for the vgg)
    img = preprocess_input(img)
    print(img.shape)
    print(type(img))
    # get feature map for first hidden layer
    self.loaded_model.predict(img)
    # print(feature_maps)
    # print(label_map[feature_maps.argmax()])

    cv2.imshow("Image", cv_image)
    # cv2.imwrite('/home/israel/dro_ws/src/basic_pkg/scripts/imagenes/img_'+str(self.count)+'.jpg', cv_image)
    # cv2.imshow("Canny Image", edges)
    cv2.waitKey(1)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_cv', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
