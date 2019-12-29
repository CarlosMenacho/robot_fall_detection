#!/usr/bin/env python

from __future__ import print_function

from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QFileDialog
from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import QThread, QObject
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv2
import rospy 
import numpy as np
from std_msgs.msg import String
import time

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('gui_ros.ui', self)

        self.bridge = CvBridge()
        rospy.init_node('gui_ros')
        self.follow_pub = rospy.Publisher("/follow", Int8, queue_size = 1)
        self.fall_pub = rospy.Publisher("/enable_fall", Int8, queue_size = 1)
        self.goto_pub = rospy.Publisher("/robot/nav", String, queue_size = 1)
        self.subscriber = rospy.Subscriber("/mynteye/left/image_raw",Image, self.callback,  queue_size = 1)
        #/mynteye/left/image_raw                 mynt
        #/zed_node/left/image_rect_color         zed
        self.button_follow = self.findChild(QtWidgets.QPushButton, 'follow_button_true')
        self.button_follow.clicked.connect(self.follow_msg)

        self.button_Nofollow = self.findChild(QtWidgets.QPushButton, 'stop_following')
        self.button_Nofollow.clicked.connect(self.leaveFollow)
        self.button_Nofollow.setVisible(False)

        self.quitButton = self.findChild(QtWidgets.QPushButton,'exit_button')
        self.quitButton.clicked.connect(QtWidgets.qApp.quit)

        self.charge = self.findChild(QtWidgets.QPushButton, 'go_toCharge')
        self.charge.clicked.connect(self.goToCharge)

        self.goTo = self.findChild(QtWidgets.QPushButton, 'go_point')
        self.goTo.clicked.connect(self.go_to_point)

        self.fall_b = self.findChild(QtWidgets.QPushButton, 'fall_button')
        self.fall_b.clicked.connect(self.falling)

        self.nofall_b = self.findChild(QtWidgets.QPushButton, 'NoFall_button')
        self.nofall_b.clicked.connect(self.no_falling)
        self.nofall_b.setVisible(False)

        self.listView = self.findChild(QtWidgets.QListView, 'msg_view')
        self.model = QtGui.QStandardItemModel()
        self.listView.setModel(self.model)

        item = QtGui.QStandardItem("Initializing program ...")
        self.model.appendRow(item)

        self.label_img = self.findChild(QtWidgets.QLabel, 'label_image')

        self.thread = QThread()
        self.subs = Subscriber_msgs()
        self.subs.moveToThread(self.thread)
        self.thread.started.connect(self.subs.spinOnce_msgs)
        self.thread.finished.connect(self.thread.deleteLater)

        item = QtGui.QStandardItem("Ready to play")
        self.model.appendRow(item)

        self.show()

    def callback(self, ros_data):
        image_np = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        height, width, channel = image_np.shape
        bytesPerLine = 3 * width
        qImg = QImage(image_np.data, width, height, bytesPerLine, QImage.Format_RGB888)
        qImg = qImg.rgbSwapped()
        qImg = qImg.scaled(640,480,Qt.KeepAspectRatio)
        self.label_image.setPixmap(QPixmap.fromImage(qImg))

    def follow_msg(self):
    	self.follow_pub.publish(1)
    	self.button_follow.setVisible(False)
    	self.button_Nofollow.setVisible(True)
        self.goTo.setEnabled(False)
        self.charge.setEnabled(False)

        #self.fall_b.setEnabled(False)
        #self.nofall_b.setEnabled(False)

        item = QtGui.QStandardItem("Following People! ...")
        self.model.appendRow(item)
    	

    def leaveFollow(self):
        self.follow_pub.publish(0)
    	self.button_Nofollow.setVisible(False)
    	self.button_follow.setVisible(True)
        self.goTo.setEnabled(True)
        self.charge.setEnabled(True)

        #self.fall_b.setEnabled(True)
        #self.nofall_b.setEnabled(True)


        item = QtGui.QStandardItem("Leave Following ...")
        self.model.appendRow(item)

    def goToCharge(self):
        self.goto_pub.publish("charge")
        #self.goTo.setEnabled(False)
        #self.charge.setEnabled(True)
        item = QtGui.QStandardItem("Need bateries ...")
        self.model.appendRow(item)



    def go_to_point(self):
        self.goto_pub.publish("point")
        #self.goTo.setEnabled(True)
        #self.charge.setEnabled(False)

        item = QtGui.QStandardItem("looking point ...")
        self.model.appendRow(item)

    def falling(self):
        self.fall_pub.publish(1)
        self.nofall_b.setVisible(True)
        self.fall_b.setVisible(False)
        
    def no_falling(self):
        self.fall_pub.publish(0)
        self.nofall_b.setVisible(False)
        self.fall_b.setVisible(True)

class Subscriber_msgs(QObject):
    def __init__(self, parent=None):
        QObject.__init__(self, parent=parent)

    def spinOnce_msgs(self):
        rospy.spin()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()

if __name__ == '__main__':
    main()
