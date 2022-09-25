#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
import math
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from geometry_msgs.msg import *
from my_robot_interfaces.msg import *
from sensor_msgs.msg import CompressedImage, Image
from tf2_ros import TransformBroadcaster
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import *
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, QDate
import cv2
from Test_02 import Ui_MainWindow
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Quaternion

# TODO: PYQT5 界面可以與ros2 topic 建立關係


def quaternion_to_euler_angles(x, y, z, w):
    # 四元數轉歐拉角
    """w、x、y、z to euler angles"""
    try:
        angles = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*z))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))

        angles['roll'] = r*180/math.pi
        angles['pitch'] = p*180/math.pi
        angles['yaw'] = y*180/math.pi

        return angles
    except:
        pass


def quaternion_from_euler(ai, aj, ak):
    # 歐拉角轉四元數
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q


class MainWindow_controller(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_control()
        self.timer = QTimer()
        self.timer.timeout.connect(self.showtime)  # 這個通過呼叫槽函式來重新整理時間
        self.timer.start(250)  # 每隔一秒重新整理一次，這裡設定為1000ms
        
        self.setup_status = False
        self.Map_Image = None
        self.map_resolution = 0.05
        self.map_center_pixel_x = 200 #fix 8 pixel
        self.map_center_pixel_y = 184 #fix 8 pixel
        self.point_size = 1
        self.point_color_blue = (0, 0, 255)  # robot_1
        self.point_color_green = (0, 255, 0) # robot_2
        self.point_thickness = 4  # 可以为 0 、4、8
        
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
            self.Map_Image = cv2.resize(self.Map_Image, (768, 768))
            self.Init_Map_Image = self.Map_Image
            self.origin_height, self.origin_width, self.origin_channel = self.Init_Map_Image.shape
            # print(self.Map_Image.shape)  # 384*384
            # 要画的点的坐标
            # points_list = [(400, 368)]
            # # center_point = (200, 184) #fix 8 pixel
            # for center_point in points_list:
            #     cv2.circle(self.Map_Image, center_point, self.point_size, self.point_color, self.point_thickness)
            # self.setImage(self.Map_Image)
            self.setup_status = True

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

    # https://answers.ros.org/question/205521/robot-coordinates-in-map/
    def pose_to_pixel_map(self):
        # resolution is 0.05
        # pose.x / resolution + center_point = cell_x
        map = self.Init_Map_Image.copy()
        center_x = self.robot_1_position['x'] / self.map_resolution + self.map_center_pixel_x
        center_y = self.robot_1_position['y'] / self.map_resolution + self.map_center_pixel_y
        center_point = (int(center_x*2.0), int(center_y*(2.0)))
        cv2.circle(map, center_point, self.point_size, self.point_color_blue, self.point_thickness)
        
        center_x = self.robot_2_position['x'] / self.map_resolution + self.map_center_pixel_x
        center_y = self.robot_2_position['y'] / self.map_resolution + self.map_center_pixel_y
        center_point = (int(center_x*2.0), int(center_y*(2.0)))
        cv2.circle(map, center_point, self.point_size, self.point_color_green, self.point_thickness)
        self.setImage(map)

    def setImage(self, image):
        """ This function will take image input and resize it 
                only for display purpose and convert it to QImage
                to set at the label.
        """
        # image = cv2.resize(image, (768, 768))
        # frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = QImage(
            image, image.shape[1], image.shape[0], image.strides[0], QImage.Format_RGB888)
        self.ui.label_Image.setPixmap(QtGui.QPixmap.fromImage(image))

    def sub_robot_1_position(self, data: Odometry):
        self.robot_1_position['x'] = data.pose.pose.position.x
        self.robot_1_position['y'] = -1.0*data.pose.pose.position.y
        self.robot_1_position['z'] = data.pose.pose.position.z
        self.robot_1_quaternion['x'] = data.pose.pose.orientation.x
        self.robot_1_quaternion['y'] = data.pose.pose.orientation.y
        self.robot_1_quaternion['z'] = data.pose.pose.orientation.z
        self.robot_1_quaternion['w'] = data.pose.pose.orientation.w

        self.ui.label_robot_1_x.setText(str(self.robot_1_position['x']))
        self.ui.label_robot_1_y.setText(str(self.robot_1_position['y']))
        self.ui.label_robot_1_z.setText(str(self.robot_1_position['z']))
        
        self.robot_1_euler_angles = quaternion_to_euler_angles(
            self.robot_1_quaternion['x'], self.robot_1_quaternion['y'],
            self.robot_1_quaternion['z'], self.robot_1_quaternion['z']
        )

    def sub_robot_2_position(self, data: Odometry):
        self.robot_2_position['x'] = data.pose.pose.position.x
        self.robot_2_position['y'] = -1.0*data.pose.pose.position.y
        self.robot_2_position['z'] = data.pose.pose.position.z
        self.robot_2_quaternion['x'] = data.pose.pose.orientation.x
        self.robot_2_quaternion['y'] = data.pose.pose.orientation.y
        self.robot_2_quaternion['z'] = data.pose.pose.orientation.z
        self.robot_2_quaternion['w'] = data.pose.pose.orientation.w

        self.ui.label_robot_2_x.setText(str(self.robot_2_position['x']))
        self.ui.label_robot_2_y.setText(str(self.robot_2_position['y']))
        self.ui.label_robot_2_z.setText(str(self.robot_2_position['z']))
        
        self.robot_2_euler_angles = quaternion_to_euler_angles(
            self.robot_2_quaternion['x'], self.robot_2_quaternion['y'],
            self.robot_2_quaternion['z'], self.robot_2_quaternion['z']
        )

    def showtime(self):
        # Update Date and Time
        now_ = QDate.currentDate()
        current_date = now_.toString('ddd dd MMMM yyyy')
        current_time = datetime.now().strftime("%I:%M:%S %p")
        self.ui.label_Date.setText(current_date)
        self.ui.label_Time.setText(current_time)
        # print(self.Init_Map_Image)
        if self.setup_status is True:
            self.pose_to_pixel_map()

    def setup_control(self):
        # position [x, y, z]
        # quaternion [x, y, z, w]
        self.robot_1_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.robot_1_quaternion = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        self.robot_2_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.robot_2_quaternion = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        self.ui.label_robot_1_x.setText(str(self.robot_1_position['x']))
        self.ui.label_robot_1_y.setText(str(self.robot_1_position['y']))
        self.ui.label_robot_1_z.setText(str(self.robot_1_position['z']))
        self.ui.label_robot_2_x.setText(str(self.robot_2_position['x']))
        self.ui.label_robot_2_y.setText(str(self.robot_2_position['y']))
        self.ui.label_robot_2_z.setText(str(self.robot_2_position['z']))
        self.ui.pushButton_start.clicked.connect(self.start_thread)
        self.ui.pushButton_stop.clicked.connect(self.stop_thread)
        self.ui.pushButton_update.clicked.connect(self.pose_to_pixel_map)

    def start_thread(self):
        self.thread_a.start()  # 啟動執行緒

    def stop_thread(self):
        self.thread_a.quit()
        rclpy.shutdown()

    def handle_rpy_to_quaternion(self, msg):
        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

    def handle_quaternion_to_rpy(self):
        self.robot_1_euler_angles = quaternion_to_euler_angles(
            self.robot_1_quaternion['x'], self.robot_1_quaternion['y'],
            self.robot_1_quaternion['z'], self.robot_1_quaternion['z']
        )
        print(self.robot_1_euler_angles)
        self.robot_2_euler_angles = quaternion_to_euler_angles(
            self.robot_2_quaternion['x'], self.robot_2_quaternion['y'],
            self.robot_2_quaternion['z'], self.robot_2_quaternion['z']
        )
        print(self.robot_2_euler_angles)
