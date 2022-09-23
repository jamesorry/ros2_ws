#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import *
from PyQt5.QtCore import QThread, pyqtSignal
import cv2
from Test_01 import Ui_MainWindow
from nav_msgs.msg import Odometry

# TODO: PYQT5 界面可以與ros2 topic 建立關係


class MainWindow_controller(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_control()

        self.thread_a = QThread()   # 建立 Thread()
        self.thread_a.run = self.ros_main_loop
        # self.thread_a.start()       # 啟動執行緒

    def ros_main_loop(self):
        rclpy.init()
        self.node = Node('pyqt5_control_03')
        self.sub_position = self.node.create_subscription(
            Odometry, '/robot1/odom', self.sub_robot_position, 10)
        print("start ros2 UI.... OK!!!!")
        rclpy.spin(self.node)

    def sub_robot_position(self, data: Odometry):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        self.pose_z = data.pose.pose.position.z
        self.ui.position_x.setText('x: ' + str(self.pose_x))
        self.ui.position_y.setText('y: ' + str(self.pose_y))
        self.ui.position_z.setText('z: ' + str(self.pose_z))

    def setup_control(self):
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0
        self.ui.position_x.setText('x: ' + str(self.pose_x))
        self.ui.position_y.setText('y: ' + str(self.pose_y))
        self.ui.position_z.setText('z: ' + str(self.pose_z))
        self.ui.Button_read_position.clicked.connect(self.update_poxition)
        self.ui.Button_clear.clicked.connect(self.clear_poxition)

    def update_poxition(self):
        self.thread_a.start()

    def clear_poxition(self):
        self.thread_a.quit()
        rclpy.shutdown()
