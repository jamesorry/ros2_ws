#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from geometry_msgs.msg import *
from my_robot_interfaces.msg import *
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import *
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, QDate
import cv2
from Test_02 import Ui_MainWindow
from nav_msgs.msg import Odometry, OccupancyGrid

# TODO: PYQT5 界面可以與ros2 topic 建立關係


class MainWindow_controller(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_control()
        self.timer = QTimer()
        self.timer.timeout.connect(self.showtime)  # 這個通過呼叫槽函式來重新整理時間
        self.timer.start(1000)  # 每隔一秒重新整理一次，這裡設定為1000ms

        self.Map_Image = None

        self.thread_a = QThread()   # 建立 Thread()
        self.thread_a.run = self.ros_main_loop

    def ros_main_loop(self):
        rclpy.init()
        self.node = Node('pyqt5_control_04')

        self.sub_robot1_position = self.node.create_subscription(
            Odometry, '/robot1/odom', self.sub_robot_1_position, 10)

        self.sub_robot2_position = self.node.create_subscription(
            Odometry, '/robot2/odom', self.sub_robot_2_position, 10)

        self.sub_map_image = self.node.create_subscription(
            SendMapImage, '/SendMapImageIng', self.loadImage, 10)

        # self.sub_map = self.node.create_subscription(
        #     OccupancyGrid, '/robot1/map', self.opencv_image, 10)

        print("start ros2 UI.... OK!!!!")
        rclpy.spin(self.node)

    def loadImage(self, img: SendMapImage):
        """ This function will load the user selected image
                and set it to label using the setPhoto function
        """
        if(self.Map_Image is None):
            bridge = CvBridge()
            self.Map_Image = bridge.imgmsg_to_cv2(img.map_image, "bgr8")
            print(self.Map_Image.shape) # 384*384
            self.setImage(self.Map_Image)

    def opencv_image(self, map_message: OccupancyGrid):
        print("into opencv image...")
        map_width = map_message.info.width
        map_height = map_message.info.height
        resolution = map_message.info.resolution
        origin = map_message.info.origin.position
        map_data = np.reshape(map_message.data, (map_width, map_height))
        map_rgb = np.zeros((map_width, map_height, 3), np.uint8)
        map_rgb.fill(205)
        for row in range(map_height):
            for col in range(map_width):
                probability = map_data[row, col]
                if probability == -1:
                    continue
                if probability > 0:
                    color = 0
                else:
                    color = 255
                map_rgb[row, col] = (color, color, color)
        map_rgb = cv2.flip(map_rgb, 0)
        # image = map_rgb
        # image = QImage(
        #     image, image.shape[1], image.shape[0], image.strides[0], QImage.Format_RGB888)
        # self.ui.label_Image.setPixmap(QtGui.QPixmap.fromImage(image))

    def setImage(self, image):
        """ This function will take image input and resize it 
                only for display purpose and convert it to QImage
                to set at the label.
        """
        # 要画的点的坐标
        points_list = [(202, 182), (136, 160), (150, 200), (200, 180), (120, 150), (145, 180)]
        # center_point = (192, 192)
        # center_point = (202, 182)
        # center_point = (200, 184) #fix 8 pixel
        point_size = 1
        point_color = (0, 0, 255) # BGR
        thickness = 4 # 可以为 0 、4、8
        
        for center_point in points_list:
            cv2.circle(image, center_point, point_size, point_color, thickness)
        # image = cv2.resize(image, (640, 480))
        # frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # print(image.shape)
        image = QImage(
            image, image.shape[1], image.shape[0], image.strides[0], QImage.Format_RGB888)
        self.ui.label_Image.setPixmap(QtGui.QPixmap.fromImage(image))

    def sub_robot_1_position(self, data: Odometry):
        self.robot_1_position[0] = data.pose.pose.position.x
        self.robot_1_position[1] = data.pose.pose.position.y
        self.robot_1_position[2] = data.pose.pose.position.z
        self.ui.label_robot_1_x.setText(str(self.robot_1_position[0]))
        self.ui.label_robot_1_y.setText(str(self.robot_1_position[1]))
        self.ui.label_robot_1_z.setText(str(self.robot_1_position[2]))

    def sub_robot_2_position(self, data: Odometry):
        self.robot_2_position[0] = data.pose.pose.position.x
        self.robot_2_position[1] = data.pose.pose.position.y
        self.robot_2_position[2] = data.pose.pose.position.z
        self.ui.label_robot_2_x.setText(str(self.robot_2_position[0]))
        self.ui.label_robot_2_y.setText(str(self.robot_2_position[1]))
        self.ui.label_robot_2_z.setText(str(self.robot_2_position[2]))

    def showtime(self):
        # Update Date and Time
        now_ = QDate.currentDate()
        current_date = now_.toString('ddd dd MMMM yyyy')
        current_time = datetime.now().strftime("%I:%M:%S %p")
        self.ui.label_Date.setText(current_date)
        self.ui.label_Time.setText(current_time)

    def setup_control(self):
        self.robot_1_position = [0.0, 0.0, 0.0]
        self.robot_2_position = [0.0, 0.0, 0.0]
        self.ui.label_robot_1_x.setText(str(self.robot_1_position[0]))
        self.ui.label_robot_1_y.setText(str(self.robot_1_position[1]))
        self.ui.label_robot_1_z.setText(str(self.robot_1_position[2]))
        self.ui.label_robot_2_x.setText(str(self.robot_2_position[0]))
        self.ui.label_robot_2_y.setText(str(self.robot_2_position[1]))
        self.ui.label_robot_2_z.setText(str(self.robot_2_position[2]))
        self.ui.pushButton_start.clicked.connect(self.start_thread)
        self.ui.pushButton_stop.clicked.connect(self.stop_thread)

    def start_thread(self):
        self.thread_a.start()  # 啟動執行緒

    def stop_thread(self):
        self.thread_a.quit()
        rclpy.shutdown()
