#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
import os
from gc import isenabled
import math
from datetime import datetime
from turtle import color
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from geometry_msgs.msg import *
from my_robot_interfaces.msg import *
from tf2_ros import TransformBroadcaster
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5 import QtWidgets, QtGui, QtCore
# from PyQt5.QtCore import QThread, pyqtSignal, QTimer, QDate, QAbstractTableModel, QModelIndex
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

# from PyQt5.QtWidgets import (
#     QApplication,
#     QLabel,
#     QMainWindow,
#     QPushButton,
#     QVBoxLayout,
#     QFileDialog,
#     QWidget,
# )
import cv2
from Test_10 import Ui_MainWindow
from camera_window_01 import Ui_CameraWindow
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Quaternion, PoseStamped
# 基于PyQt的Led（灯）控件显示库(https://blog.csdn.net/lockhou/article/details/113700860)
from pyqt_led import Led

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from my_robot_interfaces.action import ObjectTrack
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
# TODO: PYQT5 界面可以與ros2 topic 建立關係
from sensor_msgs.msg import Image

import pyqtgraph.opengl as gl

from action_msgs.msg import GoalStatus
# import subprocess
import multiprocessing
from enum import Enum

bridge = CvBridge()


class FollowStatusMSG(Enum):
    NONE = ""
    CANCEL = 'Cancel'
    CANCEL_ERROR = 'Cancel Error'
    FOLLOWING = 'Following'
    MISSING = 'Missing'


def quaternion_to_euler_angles(x, y, z, w):
    # 四元數轉歐拉角
    # 此網站才是正確轉換(https://blog.csdn.net/Fzc_Ztt/article/details/116668569)
    # (https://zh.m.wikipedia.org/zh-tw/%E5%BC%A7%E5%BA%A6)
    """w、x、y、z to euler angles"""
    try:
        angles = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))

        # print(f"r: {r}, p: {p}, y: {y}")

        angles['roll'] = r*180/math.pi  # (degree)
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
        self.disabled_ui()
        self.setup_control()
        self.setup_controller()
        self.setup_fix_controller()
        self.setup_camera_window()
        self.setup_TableWidget()
        # self.init_pyqtgraph()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.reflash_thread)  # 這個通過呼叫槽函式來重新整理時間
        self.timer.start(200)  # 每隔一秒重新整理一次，這裡設定為1000ms
        self.indexes = None
        # https://github.com/ros-visualization/rqt_robot_steering/blob/master/src/rqt_robot_steering/robot_steering.py
        # timer to consecutively send twist messages
        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(
            self._on_parameter_changed)
        self._update_parameter_timer.start(100)

        self.setup_status = False
        self.Map_Image = None
        self.map_resolution = 0.05

        self.point_size = 1
        self.point_color_blue = (0, 0, 255)  # robot_1
        self.point_color_green = (0, 255, 0)  # robot_2
        self.point_thickness = 2  # 可以为 0 、4、8

        self.thread_a = QThread()   # 建立 Thread()
        self.thread_a.run = self.ros_main_loop
        self.__debug_function()

    def __debug_function(self):
        self.ui.label_27.setVisible(False)
        self.ui.label_28.setVisible(False)
        self.ui.tableWidget_robot1.setVisible(True)
        self.ui.tableWidget_robot2.setVisible(True)
        self.ui.pushButton_start_yolov5.clicked.connect(
            self.__Start_YoloV5
        )

    def __Start_YoloV5(self):
        self.process_yolo = multiprocessing.Process(target=os.system(
            "/home/james/ros2_ws/src/yolobot/yolobot_recognition/scripts/ros_recognition_yolo_fix_v8.py"))
        if(not self.process_yolo.is_alive()):
            self.process_yolo.start()
            self.process_yolo.join()
        # subprocess.run(
        #     "/home/james/ros2_ws/src/yolobot/yolobot_recognition/scripts/ros_recognition_yolo_fix_v8.py", shell=True)

    def ros_main_loop(self):
        rclpy.init()
        self.node = Node('pyqt5_control_08')

        self.sub_robot1_position = self.node.create_subscription(
            Odometry, '/robot1/odom', self.sub_robot_1_position, 10)
        self.sub_robot1_position

        self.sub_robot2_position = self.node.create_subscription(
            Odometry, '/robot2/odom', self.sub_robot_2_position, 10)
        self.sub_robot2_position

        self.sub_map_image = self.node.create_subscription(
            SendMapImage, '/SendMapImageIng', self.loadImage, 10)
        self.sub_map_image

        self._publisher_twist_robot1 = self.node.create_publisher(
            Twist, "/robot1/cmd_vel", 10)

        self._publisher_twist_robot2 = self.node.create_publisher(
            Twist, "/robot2/cmd_vel", 10)

        # self.pub_robot1_target = self.node.create_publisher(
        #     PoseStamped, "/robot1/goal_pose", 10)

        # self.pub_robot2_target = self.node.create_publisher(
        #     PoseStamped, "/robot2/goal_pose", 10)

        self._action_client_robot1 = ActionClient(
            self.node, NavigateToPose, "/robot1/navigate_to_pose")

        self._action_client_robot2 = ActionClient(
            self.node, NavigateToPose, "/robot2/navigate_to_pose")

        self._action_client_follow = ActionClient(
            self.node, ObjectTrack, "/robot1/object_track")

        self.sub_bounding_boxes_robot1 = self.node.create_subscription(
            BoundingBoxesV2, '/robot1/yolov5/bounding_boxes', self.get_robot1_distance, 10)
        self.sub_bounding_boxes_robot1

        self.sub_bounding_boxes_robot2 = self.node.create_subscription(
            BoundingBoxesV2, '/robot2/yolov5/bounding_boxes', self.get_robot2_distance, 10)
        self.sub_bounding_boxes_robot2

        self.yolo_result_robot1 = {"class_id": [],
                                   "probability": [],
                                   "center_dist": [],
                                   "position": []
                                   }
        self.check_num_robot1 = 0
        self.check_num_robot2 = 0
        self.distance_mean_robot1 = 0.0
        self.distance_mean_robot1 = 0.0
        self.yolo_result_robot2 = {"class_id": [],
                                   "probability": [],
                                   "center_dist": [],
                                   "position": []
                                   }

        print("start ros2 UI.... OK!!!!")
        rclpy.spin(self.node)

    def get_robot1_distance(self, data: BoundingBoxesV2):
        for score in data.bounding_boxes:
            # print(score)
            # print(f"class_id: {score.class_id}")
            # print(f"probability: {score.probability}")
            # print(f"center_dist: {score.center_dist}")
            if score.class_id in self.yolo_result_robot1["class_id"]:
                # self.yolo_result_robot1["class_id"].append(score.class_id)
                list_num = self.yolo_result_robot1["class_id"].index(
                    score.class_id)
                self.yolo_result_robot1["probability"][list_num] = score.probability
                self.yolo_result_robot1["center_dist"][list_num] = score.center_dist
                position = self.get_another_point(float(self.robot_1_position['x']), -1.0*float(
                    self.robot_1_position['y']), float(self.robot_1_euler_angles['yaw']), score.center_dist)
                self.yolo_result_robot1["position"][list_num] = position
                # list_num = len(self.yolo_result_robot1["class_id"])

                self.update_tablewidget_robot1(
                    self.yolo_result_robot1["class_id"][list_num],
                    self.yolo_result_robot1["probability"][list_num],
                    self.yolo_result_robot1["position"][list_num],
                    "robot1", list_num)
                self.update_tablewidget_robot2(
                    self.yolo_result_robot1["class_id"][list_num],
                    self.yolo_result_robot1["probability"][list_num],
                    self.yolo_result_robot1["position"][list_num],
                    "robot1", list_num)

                """ 
                self.check_num_robot1 = self.check_num_robot1 + 1
                self.distance_mean_robot1 = self.distance_mean_robot1 + score.center_dist
                if self.check_num_robot1 > 10:
                    list_num = len(self.yolo_result_robot1["class_id"])
                    self.distance_mean_robot1 = self.distance_mean_robot1 / 10.0
                    position = self.get_another_point(float(self.robot_1_position['x']), -1.0*float(
                        self.robot_1_position['y']), float(self.robot_1_euler_angles['yaw']), self.distance_mean_robot1)
                    if position != self.yolo_result_robot1["position"][list_num-1]:
                        self.yolo_result_robot1["class_id"][list_num -
                                                            1] = score.class_id
                        self.yolo_result_robot1["probability"][list_num -
                                                               1] = score.probability
                        self.yolo_result_robot1["center_dist"][list_num -
                                                               1] = self.distance_mean_robot1
                        self.yolo_result_robot1["position"][list_num - 1] = position
                        self.insert_tablewidget_robot1(
                            self.yolo_result_robot1["class_id"][list_num - 1], self.yolo_result_robot1["probability"][list_num - 1], self.yolo_result_robot1["position"][list_num - 1], "robot1")
                        self.insert_tablewidget_robot2(
                            self.yolo_result_robot1["class_id"][list_num - 1], self.yolo_result_robot1["probability"][list_num - 1], self.yolo_result_robot1["position"][list_num - 1], "robot1")

                    self.check_num_robot1 = 0
                    break
                 """
            else:
                # print("first: " + str(score.center_dist))
                if True:  # score.center_dist < 1.5 and self.ui.label_target_status_robot1.text() == "Goal Finish":
                    self.yolo_result_robot1["class_id"].append(score.class_id)
                    self.yolo_result_robot1["probability"].append(
                        score.probability)
                    self.yolo_result_robot1["center_dist"].append(
                        score.center_dist)
                    position = self.get_another_point(float(self.robot_1_position['x']), -1.0*float(
                        self.robot_1_position['y']), float(self.robot_1_euler_angles['yaw']), score.center_dist)
                    self.yolo_result_robot1["position"].append(position)
                    list_num = len(self.yolo_result_robot1["class_id"])
                    self.insert_tablewidget_robot1(
                        self.yolo_result_robot1["class_id"][list_num - 1], self.yolo_result_robot1["probability"][list_num - 1], self.yolo_result_robot1["position"][list_num - 1], "robot1")
                    self.insert_tablewidget_robot2(
                        self.yolo_result_robot1["class_id"][list_num - 1], self.yolo_result_robot1["probability"][list_num - 1], self.yolo_result_robot1["position"][list_num - 1], "robot1")

        # print(self.yolo_result_robot1)
        # print("========================End line=======================")

    def get_robot2_distance(self, data: BoundingBoxesV2):
        for score in data.bounding_boxes:
            # print(score)
            # print(f"class_id: {score.class_id}")
            # print(f"probability: {score.probability}")
            # print(f"center_dist: {score.center_dist}")
            if score.class_id in self.yolo_result_robot2["class_id"]:
                self.check_num_robot2 = self.check_num_robot2 + 1
                self.distance_mean_robot2 = self.distance_mean_robot2 + score.center_dist
                if self.check_num_robot2 > 10:
                    list_num = len(self.yolo_result_robot2["class_id"])
                    self.distance_mean_robot2 = self.distance_mean_robot2 / 10.0
                    position = self.get_another_point(float(self.robot_2_position['x']), -1.0*float(
                        self.robot_2_position['y']), float(self.robot_2_euler_angles['yaw']), self.distance_mean_robot2)
                    if position != self.yolo_result_robot2["position"][list_num-1]:
                        self.yolo_result_robot2["class_id"][list_num -
                                                            1] = score.class_id
                        self.yolo_result_robot2["probability"][list_num -
                                                               1] = score.probability
                        self.yolo_result_robot2["center_dist"][list_num -
                                                               1] = self.distance_mean_robot2
                        self.yolo_result_robot2["position"][list_num - 1] = position
                    self.check_num_robot2 = 0
                    break
            else:
                # print("first: " + str(score.center_dist))
                if score.center_dist < 1.5 and self.ui.label_target_status_robot2.text() == "Goal Finish":
                    self.yolo_result_robot2["class_id"].append(score.class_id)
                    self.yolo_result_robot2["probability"].append(
                        score.probability)
                    self.yolo_result_robot2["center_dist"].append(
                        score.center_dist)
                    position = self.get_another_point(float(self.robot_2_position['x']), -1.0*float(
                        self.robot_2_position['y']), float(self.robot_2_euler_angles['yaw']), score.center_dist)
                    self.yolo_result_robot2["position"].append(position)
                    list_num = len(self.yolo_result_robot2["class_id"])
                    self.insert_tablewidget_robot1(
                        self.yolo_result_robot2["class_id"][list_num - 1], self.yolo_result_robot2["probability"][list_num - 1], self.yolo_result_robot2["position"][list_num - 1], "robot2")
                    self.insert_tablewidget_robot2(
                        self.yolo_result_robot2["class_id"][list_num - 1], self.yolo_result_robot2["probability"][list_num - 1], self.yolo_result_robot2["position"][list_num - 1], "robot2")

    def loadImage(self, img: SendMapImage):
        """ This function will load the user selected image
                and set it to label using the setPhoto function
        """
        if(self.Map_Image is None):
            self.Map_Image = bridge.imgmsg_to_cv2(img.map_image, "bgr8")
            self.Init_Map_Image = self.Map_Image
            self.origin_height, self.origin_width, self.origin_channel = self.Init_Map_Image.shape
            self.fix_map_center_pixel_x = 20
            self.fix_map_center_pixel_y = 18
            self.map_center_pixel_x = (
                self.origin_width / 2) - self.fix_map_center_pixel_x  # 200  # fix 8 pixel
            # 184  # fix 8 pixel 384/2 = 192
            self.map_center_pixel_y = (
                self.origin_height / 2) - self.fix_map_center_pixel_y
            self.init_map_img(self.Map_Image)
            self.set_img_ratio()
            self.setup_status = True
            self.ui.label_system_status_robot1.setText("ok")
            self.ui.label_system_status_robot2.setText("ok")

    def reflash_thread(self):
        # Update Date and Time
        now_ = QDate.currentDate()
        # current_date = now_.toString('ddd dd MMMM yyyy')
        current_time = datetime.now().strftime("%I:%M:%S %p")
        # self.ui.label_Date.setText(current_date)
        self.ui.label_Time.setText(current_time)

        # 印出選中的行數
        # if(self.indexes != self.ui.tableWidget_robot1.currentRow()):
        #     self.indexes = self.ui.tableWidget_robot1.currentRow()
        #     print("self.indexes: ", self.indexes)
        if self.setup_status is True:
            self.pose_to_pixel_map()
        self.ui.label_follow_status.setText(self.follow_status_msg.value)

    def __pose_to_pixel_cell(self, position, center_pixel):
        # pose.x / resolution + center_point = cell_x
        return (position / self.map_resolution) + center_pixel

    def __pixel_cell_to_pose(self, pixel_cell, center_pixel):
        return (pixel_cell - center_pixel) * self.map_resolution

    # https://answers.ros.org/question/205521/robot-coordinates-in-map/
    def pose_to_pixel_map(self):
        map = self.Init_Map_Image.copy()
        center_x = self.__pose_to_pixel_cell(
            float(self.robot_1_position['x']), self.map_center_pixel_x)
        center_y = self.__pose_to_pixel_cell(
            float(self.robot_1_position['y']), self.map_center_pixel_y)
        center_point = (int(center_x*1.0), int(center_y*(1.0)))

        another_center = self.get_another_point(float(self.robot_1_position['x']), -1.0*float(
            self.robot_1_position['y']), (float(self.robot_1_euler_angles['yaw'])-180.0), 0.5)
        another_center_x = self.__pose_to_pixel_cell(
            float(another_center[0]), self.map_center_pixel_x)
        another_center_y = self.__pose_to_pixel_cell(
            -1.0*float(another_center[1]), self.map_center_pixel_y)
        another_center_point = (int(another_center_x*1.0),
                                int(another_center_y*(1.0)))
        # 修改箭頭方向
        cv2.arrowedLine(map, another_center_point, center_point,
                        self.point_color_blue, 3)
        # cv2.circle(map, center_point, self.point_size,
        #            self.point_color_blue, 2)

        center_x = self.__pose_to_pixel_cell(
            float(self.robot_2_position['x']), self.map_center_pixel_x)
        center_y = self.__pose_to_pixel_cell(
            float(self.robot_2_position['y']), self.map_center_pixel_y)
        center_point = (int(center_x*1.0), int(center_y*(1.0)))

        another_center = self.get_another_point(float(self.robot_2_position['x']), -1.0*float(
            self.robot_2_position['y']), (float(self.robot_2_euler_angles['yaw'])-180.0), 0.5)
        another_center_x = self.__pose_to_pixel_cell(
            float(another_center[0]), self.map_center_pixel_x)
        another_center_y = self.__pose_to_pixel_cell(
            -1.0*float(another_center[1]), self.map_center_pixel_y)
        another_center_point = (int(another_center_x*1.0),
                                int(another_center_y*(1.0)))
        # 修改箭頭方向
        cv2.arrowedLine(map, another_center_point, center_point,
                        self.point_color_green, 3)

        cv2.circle(map, (int(self.mouse_target['robot1'][0]), int(self.mouse_target['robot1'][1])), self.point_size,
                   color=(0, 0, 97), thickness=2)
        cv2.circle(map, (int(self.mouse_target['robot2'][0]), int(self.mouse_target['robot2'][1])), self.point_size,
                   color=(0, 97, 0), thickness=2)

        for pos in self.yolo_result_robot1["position"]:
            # print("pos: ", pos)
            another_center_x = self.__pose_to_pixel_cell(
                float(pos[0]), self.map_center_pixel_x)
            another_center_y = self.__pose_to_pixel_cell(
                -1.0*float(pos[1]), self.map_center_pixel_y)
            if not math.isinf(another_center_x) and not math.isinf(another_center_y):
                another_center_point = (int(another_center_x*1.0),
                                        int(another_center_y*(1.0)))
                cv2.drawMarker(map, another_center_point,
                            (0, 0, 199), markerType=2, markerSize=5)

        # cv2.arrowedLine(输入图像，起始点(x,y)，结束点(x,y)，线段颜色，线段厚度，线段样式，位移因数，箭头因数)

        self.setImage(map)

    def setImage(self, image):
        bytesPerline = 3 * self.origin_width
        qimg = QImage(image, self.origin_width, self.origin_height,
                      bytesPerline, QImage.Format_RGB888).rgbSwapped()
        self.origin_qpixmap = QPixmap.fromImage(qimg)
        self.set_img_ratio()

    def save_recongnition_to_file(self):
        filename, _ = QFileDialog.getSaveFileName(
            self, 'Save Image', 'Image', '*.png *.jpg *.bmp')
        if filename == '':
            return
        map = self.Init_Map_Image.copy()
        for pos in self.yolo_result_robot1["position"]:
            another_center_x = self.__pose_to_pixel_cell(
                float(pos[0]), self.map_center_pixel_x)
            another_center_y = self.__pose_to_pixel_cell(
                -1.0*float(pos[1]), self.map_center_pixel_y)
            another_center_point = (int(another_center_x*1.0),
                                    int(another_center_y*(1.0)))
            cv2.drawMarker(map, another_center_point,
                           (255, 255, 0), markerType=2, markerSize=5)
        for pos in self.yolo_result_robot2["position"]:
            another_center_x = self.__pose_to_pixel_cell(
                float(pos[0]), self.map_center_pixel_x)
            another_center_y = self.__pose_to_pixel_cell(
                -1.0*float(pos[1]), self.map_center_pixel_y)
            another_center_point = (int(another_center_x*1.0),
                                    int(another_center_y*(1.0)))
            cv2.drawMarker(map, another_center_point,
                           (255, 0, 255), markerType=2, markerSize=5)
        cv2.imwrite(filename, map)

    def get_another_point(self, point_x, point_y, angle, bevel):
        # print(f"origin angle: {angle}")
        if angle < 0.0:
            angle = 360.0 + angle
        # print(f"fix angle: {angle}")
        # point_x = 0.0
        # point_y = 0.0
        # angle = 138.0
        # bevel = 0.5
        radian = angle * math.pi / 180.0
        Margin = [0.0, 0.0]
        Margin[0] = (math.cos(radian) * bevel) + point_x
        Margin[1] = (math.sin(radian) * bevel) + point_y
        # print(f"radian: {radian}")
        # print(f"cos: {math.cos(radian)}, sin: {math.sin(radian)}")
        # print(f"point_x: {point_x}, point_y: {point_y}")
        # print(f"Margin[0]: {Margin[0]:.3f}, Margin[1]: {Margin[1]:.3f}")

        return Margin

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

        self.robot_1_euler_angles = quaternion_to_euler_angles(
            self.robot_1_quaternion['x'], self.robot_1_quaternion['y'],
            self.robot_1_quaternion['z'], self.robot_1_quaternion['w']
        )
        self.ui.label_robot_1_z.setText(
            str(self.robot_1_euler_angles['yaw']))  # show angles
        # print(self.robot_1_euler_angles)
        self.robot_1_start_pos = (
            self.robot_1_position['x'], self.robot_1_position['y'])

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

        self.robot_2_euler_angles = quaternion_to_euler_angles(
            self.robot_2_quaternion['x'], self.robot_2_quaternion['y'],
            self.robot_2_quaternion['z'], self.robot_2_quaternion['w']
        )
        self.ui.label_robot_2_z.setText(str(self.robot_2_euler_angles['yaw']))

    def __groupBox_Target_robot1(self):
        self.ui.groupBox_Target_robot2.setChecked(False)

    def __groupBox_Target_robot2(self):
        self.ui.groupBox_Target_robot1.setChecked(False)

    def update_mouse_click_position_robot1(self):
        # x = self.__pixel_cell_to_pose(
        #     self.mouse_target_x, self.map_center_pixel_x)
        # y = self.__pixel_cell_to_pose(
        #     self.mouse_target_y, self.map_center_pixel_y)
        x = self.__pixel_cell_to_pose(
            self.mouse_target['robot1'][0], self.map_center_pixel_x)
        y = self.__pixel_cell_to_pose(
            self.mouse_target['robot1'][1], self.map_center_pixel_y)
        if self.ui.groupBox_Target_robot1.isChecked():
            self.ui.lineEdit_robot1_target_x.setText(f"{x:.3f}")
            self.ui.lineEdit_robot1_target_y.setText(f"{y:.3f}")
            if self.ui.lineEdit_robot1_target_theta.text() == "":
                self.ui.lineEdit_robot1_target_theta.setText("0")
        else:
            print(f"target: {x}, {y}")

    def update_mouse_click_position_robot2(self):
        # x = self.__pixel_cell_to_pose(
        #     self.mouse_target_x, self.map_center_pixel_x)
        # y = self.__pixel_cell_to_pose(
        #     self.mouse_target_y, self.map_center_pixel_y)
        x = self.__pixel_cell_to_pose(
            self.mouse_target['robot2'][0], self.map_center_pixel_x)
        y = self.__pixel_cell_to_pose(
            self.mouse_target['robot2'][1], self.map_center_pixel_y)
        if self.ui.groupBox_Target_robot2.isChecked():
            self.ui.lineEdit_robot2_target_x.setText(f"{x:.3f}")
            self.ui.lineEdit_robot2_target_y.setText(f"{y:.3f}")
            if self.ui.lineEdit_robot2_target_theta.text() == "":
                self.ui.lineEdit_robot2_target_theta.setText("0")
        else:
            print(f"target: {x}, {y}")

    def clear_position_text(self):
        if self.ui.groupBox_Target_robot1.isChecked():
            # self.ui.lineEdit_robot1_target_x.setText("")
            # self.ui.lineEdit_robot1_target_y.setText("")
            # self.ui.lineEdit_robot1_target_theta.setText("")
            self.yolo_result_robot1 = {"class_id": [],
                                       "probability": [],
                                       "center_dist": [],
                                       "position": []
                                       }
            self.check_num = 0
            self.distance_mean = 0.0
            self.cancel_robot1_goal()
        elif self.ui.groupBox_Target_robot2.isChecked():
            # self.ui.lineEdit_robot2_target_x.setText("")
            # self.ui.lineEdit_robot2_target_y.setText("")
            # self.ui.lineEdit_robot2_target_theta.setText("")
            self.yolo_result_robot2 = {"class_id": [],
                                       "probability": [],
                                       "center_dist": [],
                                       "position": []
                                       }
            self.cancel_robot2_goal()

    # def feedback_callback(self, feedback_msg):
        # https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action
        # feedback = feedback_msg.feedback
        # print("Received Feedback :" + str(feedback.current_pose))
        # print("Navigation time :" + str(feedback.navigation_time))
        # print("Distance remaining :" + str(feedback.distance_remaining))
        # pass

    # https://answers.ros.org/question/361666/ros2-action-goal-canceling-problem/
    # https://www.ncnynl.com/archives/202008/3824.html
    def goal_response_robot1_callback(self, future):
        self.robot1_goal_handle = future.result()
        if self.robot1_goal_handle.accepted == False:
            print("Goal Rejected")
            return None
        self.ui.label_target_status_robot1.setText("Goal Accepted")
        self._get_result_future = self.robot1_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_robot1_callback)

    def get_result_robot1_callback(self, future):
        result = future.result().result
        status = future.result().status
        print('robot1 result: ', result)
        print('robot1 status: ', status)

        x = self.__pixel_cell_to_pose(
            self.mouse_target['robot1'][0], self.map_center_pixel_x)
        y = self.__pixel_cell_to_pose(
            self.mouse_target['robot1'][1], self.map_center_pixel_y)
        theta = float(self.ui.lineEdit_robot1_target_theta.text())

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Navigation robot1 succeeded! ')
            self.ui.label_target_status_robot1.setText("Goal Finish")
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info('Navigation robot1 canceled! ')
            self.ui.label_target_status_robot1.setText("Goal Canceled")
        else:
            self.node.get_logger().info(
                'Navigation robot1 failed with status: {0}'.format(status))

        # 設定誤差值，在以下誤差值以內則顯示Goal Finish
        if (self.robot_1_position['x']-0.5) <= x and (self.robot_1_position['x']+0.5) >= x:
            if (self.robot_1_position['y']-0.5) <= y and (self.robot_1_position['y']+0.5) >= y:
                if (self.robot_1_euler_angles['yaw']-4) <= theta and (self.robot_1_euler_angles['yaw']+4) >= theta:
                    self.ui.label_target_status_robot1.setText("Goal Finish")

    def cancel_robot1_goal(self):
        self.node.get_logger().info('robot1 canceling goal')
        future = self.robot1_goal_handle.cancel_goal_async()
        future.add_done_callback(self.goal_canceled_robot1_callback)

    def goal_canceled_robot1_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Cancelling of robot1 goal complete')
        else:
            self.node.get_logger().warning('Goal robot1 failed to cancel')

    def goal_response_robot2_callback(self, future):
        self.robot2_goal_handle = future.result()
        if self.robot2_goal_handle.accepted == False:
            print("Goal Rejected")
            return None
        self.ui.label_target_status_robot2.setText("Goal Accepted")
        self._get_result_future = self.robot2_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_robot2_callback)

    def get_result_robot2_callback(self, future):
        result = future.result().result
        status = future.result().status
        print('robot2 result: ', result)
        print('robot2 status: ', status)
        x = self.__pixel_cell_to_pose(
            self.mouse_target['robot2'][0], self.map_center_pixel_x)
        y = self.__pixel_cell_to_pose(
            self.mouse_target['robot2'][1], self.map_center_pixel_y)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Navigation succeeded! ')
            self.ui.label_target_status_robot2.setText("Goal Finish")
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info('Navigation robot2 canceled! ')
            self.ui.label_target_status_robot2.setText("Goal Canceled")
        else:
            self.node.get_logger().info(
                'Navigation failed with status: {0}'.format(status))

        theta = float(self.ui.lineEdit_robot2_target_theta.text())
        if (self.robot_2_position['x']-0.5) <= x and (self.robot_2_position['x']+0.5) >= x:
            if (self.robot_2_position['y']-0.5) <= y and (self.robot_2_position['y']+0.5) >= y:
                if (self.robot_2_euler_angles['yaw']-4) <= theta and (self.robot_2_euler_angles['yaw']+4) >= theta:
                    self.ui.label_target_status_robot2.setText("Goal Finish")

    def cancel_robot2_goal(self):
        self.node.get_logger().info('robot2 canceling goal')
        future = self.robot2_goal_handle.cancel_goal_async()
        future.add_done_callback(self.goal_canceled_robot2_callback)

    def goal_canceled_robot2_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Cancelling of robot2 goal complete')
        else:
            self.node.get_logger().warning('Goal robot2 failed to cancel')

    def feedback_robot1_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
    def feedback_robot2_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def publish_robot_position_target(self):
        if self.ui.groupBox_Target_robot1.isChecked():
            x = float(self.ui.lineEdit_robot1_target_x.text())
            y = float(self.ui.lineEdit_robot1_target_y.text())
            self.mouse_target['robot1'][0] = self.__pose_to_pixel_cell(
                x, self.map_center_pixel_x)
            self.mouse_target['robot1'][1] = self.__pose_to_pixel_cell(
                y, self.map_center_pixel_y)
            theta = (int(self.ui.lineEdit_robot1_target_theta.text()))
            self.ui.lineEdit_robot1_target_theta.setText(str(theta))
            theta = theta - 0
            if theta > 360:
                theta = theta - 360
            elif theta < 0:
                theta = theta + 360
            # print(f"theta1: {theta}")
            result = self.handle_action_rpy_to_quaternion(x, y, theta)
            self._action_client_robot1.wait_for_server()
            self._send_goal_future_robot1 = self._action_client_robot1.send_goal_async(
                result, feedback_callback=self.feedback_robot1_callback)
            self._send_goal_future_robot1.add_done_callback(
                self.goal_response_robot1_callback)

        elif self.ui.groupBox_Target_robot2.isChecked():
            x = float(self.ui.lineEdit_robot2_target_x.text())
            y = float(self.ui.lineEdit_robot2_target_y.text())
            self.mouse_target['robot2'][0] = self.__pose_to_pixel_cell(
                x, self.map_center_pixel_x)
            self.mouse_target['robot2'][1] = self.__pose_to_pixel_cell(
                y, self.map_center_pixel_y)
            theta = (int(self.ui.lineEdit_robot2_target_theta.text()))
            self.ui.lineEdit_robot2_target_theta.setText(str(theta))
            theta = theta - 0
            if theta > 360:
                theta = theta - 360
            elif theta < 0:
                theta = theta + 360
            # print(f"theta2: {theta}")
            result = self.handle_action_rpy_to_quaternion(x, y, theta)
            self._action_client_robot2.wait_for_server()
            self._send_goal_future_robot2 = self._action_client_robot2.send_goal_async(
                result, feedback_callback=self.feedback_robot2_callback)
            self._send_goal_future_robot2.add_done_callback(
                self.goal_response_robot2_callback)
    # ===========================================================
    # follow controller

    def follow_controller_update(self):
        choose_index = self.ui.tableWidget_robot1.currentRow()
        print("choose_index: ", choose_index)
        self.follow_target_name = self.ui.tableWidget_robot1.item(
            choose_index, 0).text()
        print("target_name: ", self.follow_target_name)
        self.ui.label_follow_object_target.setText(self.follow_target_name)
        self.ui.pushButton_follow_start.setEnabled(True)

    def follow_controller_cancel(self):
        self.node.get_logger().info('Canceling follow goal')
        self.follow_status_msg = FollowStatusMSG.CANCEL
        self.follow_trigger = False
        self.ui.pushButton_follow_cancel.setEnabled(False)
        self.ui.pushButton_follow_start.setEnabled(False)
        # Cancel the goal
        future = self._follow_goal_handle.cancel_goal_async()
        future.add_done_callback(self.follow_cancel_done)

    def follow_cancel_done(self, future):
        cancel_response = future.result()
        print("cancel_response: ", cancel_response)
        if len(cancel_response.goals_canceling) > 0:
            self.follow_status_msg = FollowStatusMSG.CANCEL
            self.node.get_logger().info('Follow Goal successfully canceled')
            # self.ui.label_follow_status.setText("Cancel")
        else:
            self.follow_status_msg = FollowStatusMSG.CANCEL_ERROR
            self.node.get_logger().info('Follow Goal failed to cancel')
            # self.ui.label_follow_status.setText("Cancel error")

    def follow_controller_start_follow(self):
        self.node.get_logger().info('Waiting for action server...')
        self._action_client_follow.wait_for_server()

        goal_msg = ObjectTrack.Goal()
        goal_msg.object_name = self.follow_target_name  # 此處填入須要跟隨的目標名稱（需為YOLO辨識結果中的）
        self.node.get_logger().info(
            'follow target name: {0}'.format(goal_msg.object_name))

        self.node.get_logger().info('Sending follow goal request...')
        self.follow_trigger = True
        self._follow_send_goal_future = self._action_client_follow.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_feedback_callback)

        self._follow_send_goal_future.add_done_callback(
            self.follow_goal_response_callback)

    def follow_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.status == 1 and self.follow_trigger:
            self.follow_status_msg = FollowStatusMSG.FOLLOWING
        if feedback.status == 0 and self.follow_trigger:
            self.follow_status_msg = FollowStatusMSG.MISSING
        # self.node.get_logger().info('Received feedback(elapsed_time): {0}'.format(feedback.elapsed_time))
        # self.node.get_logger().info('Received feedback(status): {0}'.format(feedback.status))

    def follow_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Follow Goal rejected :(')
            return
        self._follow_goal_handle = goal_handle
        self.ui.pushButton_follow_cancel.setEnabled(True)
        self.node.get_logger().info('Follow Goal accepted :)')
        self._follow_get_result_future = self._follow_goal_handle.get_result_async()
        self._follow_get_result_future.add_done_callback(
            self.follow_get_result_callback)

    def follow_get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        print('follow result: ', result)
        print('follow status: ', status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Follow succeeded! ')
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info('Follow canceled! ')
            self.follow_status_msg = FollowStatusMSG.CANCEL
            self.ui.label_follow_status.setText("Cancel")
        else:
            self.node.get_logger().info(
                'Follow failed with status: {0}'.format(status))
    # ===========================================================

    def setup_control(self):
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
        self.ui.groupBox_Target_robot1.clicked.connect(
            self.__groupBox_Target_robot1)
        self.ui.groupBox_Target_robot2.clicked.connect(
            self.__groupBox_Target_robot2)
        self.ui.pushButton_update_robot1.clicked.connect(
            self.update_mouse_click_position_robot1)
        self.ui.pushButton_update_robot2.clicked.connect(
            self.update_mouse_click_position_robot2)
        self.ui.pushButton_robot_1_clear.clicked.connect(
            self.clear_position_text)
        self.ui.pushButton_robot_2_clear.clicked.connect(
            self.clear_position_text)
        self.ui.pushButton_robot_1_send.clicked.connect(
            self.publish_robot_position_target)
        self.ui.pushButton_robot_2_send.clicked.connect(
            self.publish_robot_position_target)

        # ===========================================================
        # follow controller
        self.ui.pushButton_follow_cancel.clicked.connect(
            self.follow_controller_cancel)
        self.ui.pushButton_follow_start.clicked.connect(
            self.follow_controller_start_follow)
        self.ui.pushButton_follow_update.clicked.connect(
            self.follow_controller_update)

        self.ui.pushButton_follow_start.setEnabled(False)
        self.ui.pushButton_follow_cancel.setEnabled(False)
        self.follow_status_msg = FollowStatusMSG.NONE
        self.follow_trigger = False
        # ===========================================================

        self.ui.pushButton_save_recongnition.clicked.connect(
            self.save_recongnition_to_file)

        # self.mouse_target_x = 0.0
        # self.mouse_target_y = 0.0

        self.mouse_target = {'robot1': [0.0, 0.0], 'robot2': [0.0, 0.0]}

        self.ratio_value = 50
        self.ui.Slider_ImageSizeArea.setValue(self.ratio_value)
        self.ui.Slider_ImageSizeArea.valueChanged.connect(
            self.set_slider_value)
        self.ui.pushButton_zoom_in.clicked.connect(self.set_zoom_in)
        self.ui.pushButton_zoom_out.clicked.connect(self.set_zoom_out)
        self.ui.scrollArea.setWidgetResizable(True)
        self.ui.label_Image.setAlignment(
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)  # 將圖片置左上角
        # self.ui.label_Image.setAlignment(
        #     QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)  # 將圖片置中

    def start_thread(self):
        self.thread_a.start()  # 啟動執行緒
        if not self._update_parameter_timer.isActive():
            self._update_parameter_timer.start(100)
        self.enabled_ui()

    def stop_thread(self):
        self.shutdown_controller_plugin()
        self.thread_a.quit()
        rclpy.shutdown()
        self.disabled_ui()
        self.ui.label_system_status_robot1.setText("error")
        self.ui.label_system_status_robot2.setText("error")

    def handle_rpy_to_quaternion(self, x, y, theta):
        t = PoseStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()

        t.pose.position.x = x
        t.pose.position.y = -1.0*y
        t.pose.position.z = 0.0
        # theta to radin
        radin = theta * math.pi / 180.0
        q = quaternion_from_euler(0, 0, radin)
        t.pose.orientation.x = q[0]
        t.pose.orientation.y = q[1]
        t.pose.orientation.z = q[2]
        t.pose.orientation.w = q[3]
        return t

    def handle_action_rpy_to_quaternion(self, x, y, theta):
        t = NavigateToPose.Goal()
        t.pose.header.stamp = self.node.get_clock().now().to_msg()

        t.pose.pose.position.x = x
        t.pose.pose.position.y = -1.0*y
        t.pose.pose.position.z = 0.0
        # theta to radin
        radin = theta * math.pi / 180.0
        q = quaternion_from_euler(0, 0, radin)
        t.pose.pose.orientation.x = q[0]
        t.pose.pose.orientation.y = q[1]
        t.pose.pose.orientation.z = q[2]
        t.pose.pose.orientation.w = q[3]
        return t

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

    # ========================================================================================
    # enable all group box
    def disabled_ui(self):
        self.ui.groupBox_robot_1.setEnabled(False)
        self.ui.groupBox_robot_2.setEnabled(False)
        self.ui.groupBox_Target_robot1.setEnabled(False)
        self.ui.groupBox_Target_robot2.setEnabled(False)
        self.ui.groupBox_controller_robot1.setEnabled(False)
        self.ui.groupBox_controller_robot2.setEnabled(False)
        self.ui.groupBox_fix_controller_robot1.setEnabled(False)
        self.ui.groupBox_fix_controller_robot2.setEnabled(False)
        self.ui.pushButton_save_recongnition.setEnabled(False)
        self.ui.pushButton_zoom_in.setEnabled(False)
        self.ui.pushButton_zoom_out.setEnabled(False)
        self.ui.Slider_ImageSizeArea.setEnabled(False)
        self.ui.pushButton_show_camera_window.setEnabled(False)
        self.ui.groupBox_follow_controller.setEnabled(False)
        # self.ui.pushButton_start_yolov5.setEnabled(False)

    def enabled_ui(self):
        self.ui.groupBox_robot_1.setEnabled(True)
        self.ui.groupBox_robot_2.setEnabled(True)
        self.ui.groupBox_Target_robot1.setEnabled(True)
        self.ui.groupBox_Target_robot2.setEnabled(True)
        self.ui.groupBox_controller_robot1.setEnabled(True)
        self.ui.groupBox_controller_robot2.setEnabled(True)
        self.ui.groupBox_fix_controller_robot1.setEnabled(True)
        self.ui.groupBox_fix_controller_robot2.setEnabled(True)
        self.ui.pushButton_save_recongnition.setEnabled(True)
        self.ui.pushButton_zoom_in.setEnabled(True)
        self.ui.pushButton_zoom_out.setEnabled(True)
        self.ui.Slider_ImageSizeArea.setEnabled(True)
        self.ui.pushButton_show_camera_window.setEnabled(True)
        self.ui.groupBox_follow_controller.setEnabled(True)
        # self.ui.pushButton_start_yolov5.setEnabled(True)
    # ========================================================================================

    def init_map_img(self, img):
        bytesPerline = 3 * self.origin_width
        qimg = QImage(img, self.origin_width, self.origin_height,
                      bytesPerline, QImage.Format_RGB888).rgbSwapped()
        self.origin_qpixmap = QPixmap.fromImage(qimg)

    def set_img_ratio(self):
        self.ratio_rate = pow(10, (self.ratio_value - 50)/50)
        qpixmap_height = self.origin_height * self.ratio_rate
        self.qpixmap = self.origin_qpixmap.scaledToHeight(qpixmap_height)
        self.__update_img()
        self.__update_text_ratio()
        self.__update_text_img_shape()

    def __update_img(self):
        self.ui.label_Image.setPixmap(self.qpixmap)
        self.ui.label_Image.setAlignment(
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
        self.ui.label_Image.mousePressEvent = self.get_clicked_position

    def __update_text_ratio(self):
        self.ui.label_ratio.setText(f"{int(100*self.ratio_rate)} %")

    def __update_text_img_shape(self):
        current_text = f"Current img shape = ({self.qpixmap.width()}, {self.qpixmap.height()})"
        origin_text = f"Origin img shape = ({self.origin_width}, {self.origin_height})"
        self.ui.label_image_origin_shape.setText(origin_text)
        self.ui.label_image_current_shape.setText(current_text)

    def set_zoom_in(self):
        self.ratio_value = max(0, self.ratio_value - 1)
        self.ui.Slider_ImageSizeArea.setValue(self.ratio_value)
        self.set_img_ratio()

    def set_zoom_out(self):
        self.ratio_value = min(100, self.ratio_value + 1)
        self.ui.Slider_ImageSizeArea.setValue(self.ratio_value)
        self.set_img_ratio()

    def set_slider_value(self):
        self.ratio_value = self.ui.Slider_ImageSizeArea.value() + 1
        self.set_img_ratio()

    def get_clicked_position(self, event):
        x = event.pos().x()
        y = event.pos().y()
        self.norm_x = x/self.qpixmap.width()
        self.norm_y = y/self.qpixmap.height()
        # print(f"(x, y) = ({x}, {y}), normalized (x, y) = ({self.norm_x}, {self.norm_y})")
        self.__update_text_clicked_position(x, y)

    def __update_text_clicked_position(self, x, y):
        self.ui.label_click_pos.setText(f"Clicked postion = ({x}, {y})")
        self.ui.label_normal_pos.setText(
            f"Normalized postion = ({self.norm_x:.3f}, {self.norm_y:.3f})")
        self.ui.label_real_pos.setText(
            f"Real postion = ({int(self.norm_x*self.origin_width)}, {int(self.norm_y*self.origin_height)})")
        # Real postion 圖片變大變小，可以抓到圖片相對應的pixel 座標
        # self.mouse_target_x = (self.norm_x*self.origin_width)
        # self.mouse_target_y = (self.norm_y*self.origin_height)
        if self.ui.groupBox_Target_robot1.isChecked():
            self.mouse_target['robot1'][0] = (self.norm_x*self.origin_width)
            self.mouse_target['robot1'][1] = (self.norm_y*self.origin_height)
            # print(self.mouse_target['robot1'])
        elif self.ui.groupBox_Target_robot2.isChecked():
            self.mouse_target['robot2'][0] = (self.norm_x*self.origin_width)
            self.mouse_target['robot2'][1] = (self.norm_y*self.origin_height)
            # print(self.mouse_target['robot2'])

    # ============================================================================
    def setup_controller(self):
        self.zero_cmd_sent = False
        self.slider_factor = 1000.0
        self.slider_middle_value = 500
        self._publisher_twist_robot1 = None
        self._publisher_twist_robot2 = None

        self.ui.groupBox_controller_robot1.setChecked(False)
        self.ui.Slider_z_angular_robot1.setInvertedAppearance(True)
        self.ui.Slider_x_linear_robot1.valueChanged.connect(
            self._on_x_linear_slider_changed_robot1)
        self.ui.Slider_z_angular_robot1.valueChanged.connect(
            self._on_z_angular_slider_changed_robot1)
        self.ui.pushButton_increase_x_linear_robot1.pressed.connect(
            self._on_strong_increase_x_linear_pressed_robot1)
        self.ui.pushButton_reset_x_linear_robot1.pressed.connect(
            self._on_reset_x_linear_pressed_robot1)
        self.ui.pushButton_decrease_x_linear_robot1.pressed.connect(
            self._on_strong_decrease_x_linear_pressed_robot1)
        self.ui.pushButton_increase_z_angular_robot1.pressed.connect(
            self._on_strong_increase_z_angular_pressed_robot1)
        self.ui.pushButton_reset_z_angular_robot1.pressed.connect(
            self._on_reset_z_angular_pressed_robot1)
        self.ui.pushButton_decrease_z_angular_robot1.pressed.connect(
            self._on_strong_decrease_z_angular_pressed_robot1)
        self.ui.pushButton_stop_moving_robot1.pressed.connect(
            self._on_stop_moving_robot1)
        self.ui.groupBox_controller_robot1.clicked.connect(
            self.__groupBox_controller_robot1)

        self.ui.groupBox_controller_robot2.setChecked(False)
        self.ui.Slider_z_angular_robot2.setInvertedAppearance(True)
        self.ui.Slider_z_angular_robot2.valueChanged.connect(
            self._on_z_angular_slider_changed_robot2)
        self.ui.Slider_x_linear_robot2.valueChanged.connect(
            self._on_x_linear_slider_changed_robot2)
        self.ui.pushButton_increase_x_linear_robot2.pressed.connect(
            self._on_strong_increase_x_linear_pressed_robot2)
        self.ui.pushButton_reset_x_linear_robot2.pressed.connect(
            self._on_reset_x_linear_pressed_robot2)
        self.ui.pushButton_decrease_x_linear_robot2.pressed.connect(
            self._on_strong_decrease_x_linear_pressed_robot2)
        self.ui.pushButton_increase_z_angular_robot2.pressed.connect(
            self._on_strong_increase_z_angular_pressed_robot2)
        self.ui.pushButton_reset_z_angular_robot2.pressed.connect(
            self._on_reset_z_angular_pressed_robot2)
        self.ui.pushButton_decrease_z_angular_robot2.pressed.connect(
            self._on_strong_decrease_z_angular_pressed_robot2)
        self.ui.pushButton_stop_moving_robot2.pressed.connect(
            self._on_stop_moving_robot2)
        self.ui.groupBox_controller_robot2.clicked.connect(
            self.__groupBox_controller_robot2)

        self._on_x_linear_slider_changed_robot1()
        self._on_x_linear_slider_changed_robot2()
        self._on_z_angular_slider_changed_robot1()
        self._on_z_angular_slider_changed_robot2()

    def shutdown_controller_plugin(self):
        self._update_parameter_timer.stop()

    def _on_parameter_changed(self):
        self._on_parameter_changed_robot1()
        self._on_parameter_changed_robot2()
        self.fix_controller_send_twist_robot1(
            self.send_x_linear_robot1, self.send_z_angular_robot1)
        self.fix_controller_send_twist_robot2(
            self.send_x_linear_robot2, self.send_z_angular_robot2)

    """ Robot1 ========================================================================================== """

    def _on_stop_moving_robot1(self):
        self._on_reset_x_linear_pressed_robot1()
        self._on_reset_z_angular_pressed_robot1()

    def _on_x_linear_slider_changed_robot1(self):
        self.ui.label_current_x_linear_robot1.setText(
            '%0.2f m/s' % ((self.ui.Slider_x_linear_robot1.value()-self.slider_middle_value) / self.slider_factor))
        self._on_parameter_changed_robot1()

    def _on_z_angular_slider_changed_robot1(self):
        self.ui.label_current_z_angular_robot1.setText(
            '%0.2f rad/s' % ((self.ui.Slider_z_angular_robot1.value()-self.slider_middle_value) / self.slider_factor))
        self._on_parameter_changed_robot1()

    # def _on_increase_x_linear_pressed_robot1(self):
    #     self.ui.Slider_x_linear_robot1.setValue(
    #         self.ui.Slider_x_linear_robot1.value() + self.ui.Slider_x_linear_robot1.singleStep())

    def _on_reset_x_linear_pressed_robot1(self):
        self.ui.Slider_x_linear_robot1.setValue(self.slider_middle_value)
        self.ui.label_current_x_linear_robot1.setText(
            '%0.2f m/s' % ((self.ui.Slider_x_linear_robot1.value()-self.slider_middle_value) / self.slider_factor))

    # def _on_decrease_x_linear_pressed_robot1(self):
    #     self.ui.Slider_x_linear_robot1.setValue(
    #         self.ui.Slider_x_linear_robot1.value() - self.ui.Slider_x_linear_robot1.singleStep())

    # def _on_increase_z_angular_pressed_robot1(self):
    #     self.ui.Slider_z_angular_robot1.setValue(
    #         self.ui.Slider_z_angular_robot1.value() + self.ui.Slider_z_angular_robot1.singleStep())

    def _on_reset_z_angular_pressed_robot1(self):
        self.ui.Slider_z_angular_robot1.setValue(self.slider_middle_value)
        self.ui.label_current_z_angular_robot1.setText(
            '%0.2f rad/s' % ((self.ui.Slider_z_angular_robot1.value()-self.slider_middle_value) / self.slider_factor))
    # def _on_decrease_z_angular_pressed_robot1(self):
    #     self.ui.Slider_z_angular_robot1.setValue(
    #         self.ui.Slider_z_angular_robot1.value() - self.ui.Slider_z_angular_robot1.singleStep())

    def _on_strong_increase_x_linear_pressed_robot1(self):
        self.ui.Slider_x_linear_robot1.setValue(
            self.ui.Slider_x_linear_robot1.value() + self.ui.Slider_x_linear_robot1.pageStep())

    def _on_strong_decrease_x_linear_pressed_robot1(self):
        self.ui.Slider_x_linear_robot1.setValue(
            self.ui.Slider_x_linear_robot1.value() - self.ui.Slider_x_linear_robot1.pageStep())

    def _on_strong_increase_z_angular_pressed_robot1(self):
        self.ui.Slider_z_angular_robot1.setValue(
            self.ui.Slider_z_angular_robot1.value() + self.ui.Slider_z_angular_robot1.pageStep())

    def _on_strong_decrease_z_angular_pressed_robot1(self):
        self.ui.Slider_z_angular_robot1.setValue(
            self.ui.Slider_z_angular_robot1.value() - self.ui.Slider_z_angular_robot1.pageStep())

    def _on_parameter_changed_robot1(self):
        self._send_twist_robot1(
            (self.ui.Slider_x_linear_robot1.value() -
             self.slider_middle_value) / self.slider_factor,
            (self.ui.Slider_z_angular_robot1.value()-self.slider_middle_value) / self.slider_factor)

    def _send_twist_robot1(self, x_linear, z_angular):
        if (self._publisher_twist_robot1 is None) or (not self.ui.groupBox_controller_robot1.isChecked):
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_angular

        # print(x_linear, z_angular)
        # # Only send the zero command once so other devices can take control
        # if x_linear == 0.0 and z_angular == 0.0:
        #     if not self.zero_cmd_sent:
        #         self.zero_cmd_sent = True
        #         self._publisher_twist_robot1.publish(twist)
        # else:
        #     self.zero_cmd_sent = False
        self._publisher_twist_robot1.publish(twist)

    """ Robot2 ========================================================================================== """

    def _on_stop_moving_robot2(self):
        self._on_reset_x_linear_pressed_robot2()
        self._on_reset_z_angular_pressed_robot2()

    def _on_x_linear_slider_changed_robot2(self):
        self.ui.label_current_x_linear_robot2.setText(
            '%0.2f m/s' % ((self.ui.Slider_x_linear_robot2.value()-self.slider_middle_value) / self.slider_factor))
        # print("x_linear_robot2: " +self.ui.label_current_x_linear_robot2.text())
        self._on_parameter_changed_robot2()

    def _on_z_angular_slider_changed_robot2(self):
        self.ui.label_current_z_angular_robot2.setText(
            '%0.2f rad/s' % ((self.ui.Slider_z_angular_robot2.value()-self.slider_middle_value) / self.slider_factor))
        # print("z_angular_robot2: " +self.ui.label_current_z_angular_robot2.text())
        self._on_parameter_changed_robot2()

    # def _on_increase_x_linear_pressed_robot2(self):
    #     self.ui.Slider_x_linear_robot2.setValue(
    #         self.ui.Slider_x_linear_robot2.value() + self.ui.Slider_x_linear_robot2.singleStep())

    def _on_reset_x_linear_pressed_robot2(self):
        self.ui.Slider_x_linear_robot2.setValue(self.slider_middle_value)
        self.ui.label_current_x_linear_robot2.setText(
            '%0.2f m/s' % ((self.ui.Slider_x_linear_robot2.value()-self.slider_middle_value) / self.slider_factor))
    # def _on_decrease_x_linear_pressed_robot2(self):
    #     self.ui.Slider_x_linear_robot2.setValue(
    #         self.ui.Slider_x_linear_robot2.value() - self.ui.Slider_x_linear_robot2.singleStep())

    # def _on_increase_z_angular_pressed_robot2(self):
    #     self.ui.Slider_z_angular_robot2.setValue(
    #         self.ui.Slider_z_angular_robot2.value() + self.ui.Slider_z_angular_robot2.singleStep())

    def _on_reset_z_angular_pressed_robot2(self):
        self.ui.Slider_z_angular_robot2.setValue(self.slider_middle_value)
        self.ui.label_current_z_angular_robot2.setText(
            '%0.2f rad/s' % ((self.ui.Slider_z_angular_robot2.value()-self.slider_middle_value) / self.slider_factor))
    # def _on_decrease_z_angular_pressed_robot2(self):
    #     self.ui.Slider_z_angular_robot2.setValue(
    #         self.ui.Slider_z_angular_robot2.value() - self.ui.Slider_z_angular_robot2.singleStep())

    def _on_strong_increase_x_linear_pressed_robot2(self):
        self.ui.Slider_x_linear_robot2.setValue(
            self.ui.Slider_x_linear_robot2.value() + self.ui.Slider_x_linear_robot2.pageStep())

    def _on_strong_decrease_x_linear_pressed_robot2(self):
        self.ui.Slider_x_linear_robot2.setValue(
            self.ui.Slider_x_linear_robot2.value() - self.ui.Slider_x_linear_robot2.pageStep())

    def _on_strong_increase_z_angular_pressed_robot2(self):
        self.ui.Slider_z_angular_robot2.setValue(
            self.ui.Slider_z_angular_robot2.value() + self.ui.Slider_z_angular_robot2.pageStep())

    def _on_strong_decrease_z_angular_pressed_robot2(self):
        self.ui.Slider_z_angular_robot2.setValue(
            self.ui.Slider_z_angular_robot2.value() - self.ui.Slider_z_angular_robot2.pageStep())

    def _on_parameter_changed_robot2(self):
        self._send_twist_robot2(
            (self.ui.Slider_x_linear_robot2.value() -
             self.slider_middle_value) / self.slider_factor,
            (self.ui.Slider_z_angular_robot2.value()-self.slider_middle_value) / self.slider_factor)

    def _send_twist_robot2(self, x_linear, z_angular):
        if (self._publisher_twist_robot2 is None) or (not self.ui.groupBox_controller_robot2.isChecked()):
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_angular

        # Only send the zero command once so other devices can take control
        # if x_linear == 0.0 and z_angular == 0.0:
        #     if not self.zero_cmd_sent:
        #         self.zero_cmd_sent = True
        #         self._publisher_twist_robot2.publish(twist)
        # else:
        #     self.zero_cmd_sent = False
        self._publisher_twist_robot2.publish(twist)
    """ ========================================================================================== """

    def __groupBox_controller_robot1(self):
        self.ui.groupBox_fix_controller_robot1.setChecked(False)

    def __groupBox_fix_controller_robot1(self):
        self.ui.groupBox_controller_robot1.setChecked(False)

    def __groupBox_controller_robot2(self):
        self.ui.groupBox_fix_controller_robot2.setChecked(False)

    def __groupBox_fix_controller_robot2(self):
        self.ui.groupBox_controller_robot2.setChecked(False)

    # fix controller
    def setup_fix_controller(self):
        self.send_x_linear_robot1 = 0.0
        self.send_z_angular_robot1 = 0.0
        self.fix_controller_x_linear_robot1 = 0.00
        self.fix_controller_z_angular_robot1 = 0.00
        self.ui.pushButton_move_forward_robot1.pressed.connect(
            self.move_forward_robot1)
        self.ui.pushButton_move_forward_robot1.setAutoRepeat(True)
        self.ui.pushButton_move_backward_robot1.pressed.connect(
            self.move_backward_robot1)
        self.ui.pushButton_move_backward_robot1.setAutoRepeat(True)
        self.ui.pushButton_right_rotate_robot1.pressed.connect(
            self.right_rotate_robot1)
        self.ui.pushButton_right_rotate_robot1.setAutoRepeat(True)
        self.ui.pushButton_left_rotate_robot1.pressed.connect(
            self.left_rotate_robot1)
        self.ui.pushButton_left_rotate_robot1.setAutoRepeat(True)
        self.ui.pushButton_fix_stop_moving_robot1.clicked.connect(
            self.stop_moving_robot1)
        self.ui.lineEdit_x_liner_robot1.setPlaceholderText("0.00~0.50")
        self.ui.lineEdit_x_liner_robot1.editingFinished.connect(
            self.x_liner_text_edit_robot1)
        self.ui.lineEdit_x_liner_robot1.setText(
            str(self.fix_controller_x_linear_robot1))
        self.ui.lineEdit_z_angular_robot1.setPlaceholderText("0.00~0.50")
        self.ui.lineEdit_z_angular_robot1.editingFinished.connect(
            self.z_angular_text_edit_robot1)
        self.ui.lineEdit_z_angular_robot1.setText(
            str(self.fix_controller_z_angular_robot1))
        self.ui.groupBox_fix_controller_robot1.clicked.connect(
            self.__groupBox_fix_controller_robot1)

        self.send_x_linear_robot2 = 0.0
        self.send_z_angular_robot2 = 0.0
        self.fix_controller_x_linear_robot2 = 0.00
        self.fix_controller_z_angular_robot2 = 0.00
        self.ui.pushButton_move_forward_robot2.pressed.connect(
            self.move_forward_robot2)
        self.ui.pushButton_move_forward_robot2.setAutoRepeat(True)
        self.ui.pushButton_move_backward_robot2.pressed.connect(
            self.move_backward_robot2)
        self.ui.pushButton_move_backward_robot2.setAutoRepeat(True)
        self.ui.pushButton_right_rotate_robot2.pressed.connect(
            self.right_rotate_robot2)
        self.ui.pushButton_right_rotate_robot2.setAutoRepeat(True)
        self.ui.pushButton_left_rotate_robot2.pressed.connect(
            self.left_rotate_robot2)
        self.ui.pushButton_left_rotate_robot2.setAutoRepeat(True)
        self.ui.pushButton_fix_stop_moving_robot2.clicked.connect(
            self.stop_moving_robot2)
        self.ui.lineEdit_x_liner_robot2.setPlaceholderText("0.00~0.50")
        self.ui.lineEdit_x_liner_robot2.editingFinished.connect(
            self.x_liner_text_edit_robot2)
        self.ui.lineEdit_x_liner_robot2.setText(
            str(self.fix_controller_x_linear_robot2))
        self.ui.lineEdit_z_angular_robot2.setPlaceholderText("0.00~0.50")
        self.ui.lineEdit_z_angular_robot2.editingFinished.connect(
            self.z_angular_text_edit_robot2)
        self.ui.lineEdit_z_angular_robot2.setText(
            str(self.fix_controller_z_angular_robot2))
        self.ui.groupBox_fix_controller_robot2.clicked.connect(
            self.__groupBox_fix_controller_robot2)

    def move_forward_robot1(self):
        self.send_x_linear_robot1 = self.fix_controller_x_linear_robot1
        self.send_z_angular_robot1 = 0.0
        # self.fix_controller_send_twist_robot1(self.fix_controller_x_linear_robot1, 0.0)

    def move_backward_robot1(self):
        self.send_x_linear_robot1 = -self.fix_controller_x_linear_robot1
        self.send_z_angular_robot1 = 0.0
        # self.fix_controller_send_twist_robot1(-self.fix_controller_x_linear_robot1, 0.0)

    def right_rotate_robot1(self):
        self.send_x_linear_robot1 = 0.0
        self.send_z_angular_robot1 = -self.fix_controller_z_angular_robot1
        # self.fix_controller_send_twist_robot1(0.0, -self.fix_controller_z_angular_robot1)

    def left_rotate_robot1(self):
        self.send_x_linear_robot1 = 0.0
        self.send_z_angular_robot1 = self.fix_controller_z_angular_robot1
        # self.fix_controller_send_twist_robot1(0.0, self.fix_controller_z_angular_robot1)

    def stop_moving_robot1(self):
        self.send_x_linear_robot1 = 0.0
        self.send_z_angular_robot1 = 0.0
        # self.fix_controller_send_twist_robot1(0.0, 0.0)

    def move_forward_robot2(self):
        self.send_x_linear_robot2 = self.fix_controller_x_linear_robot2
        self.send_z_angular_robot2 = 0.0
        # self.fix_controller_send_twist_robot2(self.fix_controller_x_linear_robot2, 0.0)

    def move_backward_robot2(self):
        self.send_x_linear_robot2 = -self.fix_controller_x_linear_robot2
        self.send_z_angular_robot2 = 0.0
        # self.fix_controller_send_twist_robot2(-self.fix_controller_x_linear_robot2, 0.0)

    def right_rotate_robot2(self):
        self.send_x_linear_robot2 = 0.0
        self.send_z_angular_robot2 = -self.fix_controller_z_angular_robot2
        # self.fix_controller_send_twist_robot2(0.0, -self.fix_controller_z_angular_robot2)

    def left_rotate_robot2(self):
        self.send_x_linear_robot2 = 0.0
        self.send_z_angular_robot2 = self.fix_controller_z_angular_robot2
        # self.fix_controller_send_twist_robot2(0.0, self.fix_controller_z_angular_robot2)

    def stop_moving_robot2(self):
        self.send_x_linear_robot2 = 0.0
        self.send_z_angular_robot2 = 0.0
        # self.fix_controller_send_twist_robot2(0.0, 0.0)

    def fix_controller_send_twist_robot1(self, x_linear, z_angular):
        if (self._publisher_twist_robot1 is None) or (not self.ui.groupBox_fix_controller_robot1.isChecked()):
            # print("not")
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_angular
        # print(f"x_linear: {x_linear}, z_angular: {z_angular}")
        self._publisher_twist_robot1.publish(twist)
        # print("send twist robot1")

    def fix_controller_send_twist_robot2(self, x_linear, z_angular):
        if (self._publisher_twist_robot2 is None) or (not self.ui.groupBox_fix_controller_robot2.isChecked()):
            # print("not")
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_angular
        # print(f"x_linear: {x_linear}, z_angular: {z_angular}")
        self._publisher_twist_robot2.publish(twist)
        # print("send twist robot2")

    def x_liner_text_edit_robot1(self):
        if self.ui.lineEdit_x_liner_robot1.text() != "":
            if float(self.ui.lineEdit_x_liner_robot1.text()) > 0.5:
                self.fix_controller_x_linear_robot1 = 0.5
                self.ui.lineEdit_x_liner_robot1.setText(
                    str(self.fix_controller_x_linear_robot1))
            elif float(self.ui.lineEdit_x_liner_robot1.text()) < -0.5:
                self.fix_controller_x_linear_robot1 = -0.5
                self.ui.lineEdit_x_liner_robot1.setText(
                    str(self.fix_controller_x_linear_robot1))
            else:
                self.fix_controller_x_linear_robot1 = float(
                    self.ui.lineEdit_x_liner_robot1.text())

    def z_angular_text_edit_robot1(self):
        if self.ui.lineEdit_z_angular_robot1.text() != "":
            if float(self.ui.lineEdit_z_angular_robot1.text()) > 0.5:
                self.fix_controller_z_angular_robot1 = 0.5
                self.ui.lineEdit_z_angular_robot1.setText(
                    str(self.fix_controller_z_angular_robot1))
            elif float(self.ui.lineEdit_z_angular_robot1.text()) < -0.5:
                self.fix_controller_z_angular_robot1 = -0.5
                self.ui.lineEdit_z_angular_robot1.setText(
                    str(self.fix_controller_z_angular_robot1))
            else:
                self.fix_controller_z_angular_robot1 = float(
                    self.ui.lineEdit_z_angular_robot1.text())

    def x_liner_text_edit_robot2(self):
        if self.ui.lineEdit_x_liner_robot2.text() != "":
            if float(self.ui.lineEdit_x_liner_robot2.text()) > 0.5:
                self.fix_controller_x_linear_robot2 = 0.5
                self.ui.lineEdit_x_liner_robot2.setText(
                    str(self.fix_controller_x_linear_robot2))
            elif float(self.ui.lineEdit_x_liner_robot2.text()) < -0.5:
                self.fix_controller_x_linear_robot2 = -0.5
                self.ui.lineEdit_x_liner_robot2.setText(
                    str(self.fix_controller_x_linear_robot2))
            else:
                self.fix_controller_x_linear_robot2 = float(
                    self.ui.lineEdit_x_liner_robot2.text())

    def z_angular_text_edit_robot2(self):
        if self.ui.lineEdit_z_angular_robot2.text() != "":
            if float(self.ui.lineEdit_z_angular_robot2.text()) > 0.5:
                self.fix_controller_z_angular_robot2 = 0.5
                self.ui.lineEdit_z_angular_robot2.setText(
                    str(self.fix_controller_z_angular_robot2))
            elif float(self.ui.lineEdit_z_angular_robot2.text()) < -0.5:
                self.fix_controller_z_angular_robot2 = -0.5
                self.ui.lineEdit_z_angular_robot2.setText(
                    str(self.fix_controller_z_angular_robot2))
            else:
                self.fix_controller_z_angular_robot2 = float(
                    self.ui.lineEdit_z_angular_robot2.text())

    # ==========================================================================================
    # 新增不同風格ui 以及 開啟另一個新視窗
    # 處理重複create_subscription的問題
    def setup_camera_window(self):
        self.camera_window_ui = Ui_CameraWindow()
        self._camera_window = QMainWindow()
        self.camera_window_ui.setupUi(self._camera_window)
        self.ui.pushButton_show_camera_window.clicked.connect(
            self.show_camera_window
        )
        self.camera_window_ui.comboBox_robot_n.addItems(
            ["", "Robot_1", "Robot_2"])
        self.camera_window_ui.comboBox_robot_n.activated.connect(
            self.robot_yolo_image)
        self._update_show_camera_timer = QTimer(self)

        self._update_show_camera_timer.timeout.connect(
            self.reflash_camera_image)
        self._update_show_camera_timer.start(100)
        self.sub_yolov5_image_raw_robot1 = None
        self.sub_yolov5_image_raw_robot2 = None
        self.robot_camera_img = {"Robot_1": None, "Robot_2": None}

    def reflash_camera_image(self):
        try:
            if self.camera_window_ui.comboBox_robot_n.currentIndex() == 1 and self.robot1_yolo_image_is_exist:
                height, width, channel = self.robot_camera_img["Robot_1"].shape
                bytesPerline = 3 * width
                qImg = QImage(self.robot_camera_img["Robot_1"], width,
                              height, bytesPerline, QImage.Format_RGB888).rgbSwapped()
                self.camera_window_ui.label_camera_image.setPixmap(
                    QPixmap.fromImage(qImg))
            elif self.camera_window_ui.comboBox_robot_n.currentIndex() == 2 and self.robot2_yolo_image_is_exist:
                height, width, channel = self.robot_camera_img["Robot_2"].shape
                bytesPerline = 3 * width
                qImg = QImage(self.robot_camera_img["Robot_2"], width,
                              height, bytesPerline, QImage.Format_RGB888).rgbSwapped()
                self.camera_window_ui.label_camera_image.setPixmap(
                    QPixmap.fromImage(qImg))
            else:
                self.camera_window_ui.label_camera_image.setPixmap(QPixmap(""))
        except Exception as e:
            print(e)
            self.camera_window_ui.label_camera_image.setPixmap(QPixmap(""))

    def show_camera_window(self):
        if not self._camera_window.isVisible():
            self._camera_window.show()
            self.camera_window_ui.comboBox_robot_n.setCurrentIndex(0)
            print("show_camera_window")
        self.robot1_yolo_image_is_exist = self.check_topic_is_exist(
            '/robot1/yolov5/image_raw')
        self.robot2_yolo_image_is_exist = self.check_topic_is_exist(
            '/robot2/yolov5/image_raw')

    def get_robot1_yolov5_image(self, image_date):
        img = bridge.imgmsg_to_cv2(image_date, "bgr8")
        self.robot_camera_img["Robot_1"] = img.copy()

    def get_robot2_yolov5_image(self, image_date):
        img = bridge.imgmsg_to_cv2(image_date, "bgr8")
        self.robot_camera_img["Robot_2"] = img.copy()

    def robot_yolo_image(self):
        if self.camera_window_ui.comboBox_robot_n.currentIndex() == 1:
            if self.sub_yolov5_image_raw_robot1 == None:
                self.sub_yolov5_image_raw_robot1 = self.node.create_subscription(
                    Image, '/robot1/yolov5/image_raw', self.get_robot1_yolov5_image, 10)

            if self.sub_yolov5_image_raw_robot2 != None:
                self.node.destroy_subscription(
                    self.sub_yolov5_image_raw_robot2)
                self.sub_yolov5_image_raw_robot2 = None

        elif self.camera_window_ui.comboBox_robot_n.currentIndex() == 2:
            if self.sub_yolov5_image_raw_robot2 == None:
                self.sub_yolov5_image_raw_robot2 = self.node.create_subscription(
                    Image, '/robot2/yolov5/image_raw', self.get_robot2_yolov5_image, 10)

            if self.sub_yolov5_image_raw_robot1 != None:
                self.node.destroy_subscription(
                    self.sub_yolov5_image_raw_robot1)
                self.sub_yolov5_image_raw_robot1 = None
        else:
            if self.sub_yolov5_image_raw_robot1 != None:
                self.node.destroy_subscription(
                    self.sub_yolov5_image_raw_robot1)
                self.sub_yolov5_image_raw_robot1 = None
            if self.sub_yolov5_image_raw_robot2 != None:
                self.node.destroy_subscription(
                    self.sub_yolov5_image_raw_robot2)
                self.sub_yolov5_image_raw_robot2 = None

    # ======================================================================================
    def setup_TableWidget(self):
        # QTableWidget使用方法(https://www.jb51.net/article/181132.htm)
        # https://stackoverflow.com/questions/24044421/how-to-add-a-row-in-a-tablewidget-pyqt
        self.ui.tableWidget_robot1.setColumnCount(4)
        self.ui.tableWidget_robot1.setHorizontalHeaderLabels(
            ['ID', 'PR', 'Pos', 'Check'])
        # self.ui.tableWidget_robot1.horizontalHeader().setSectionResizeMode(
        #     QHeaderView.Stretch)  # 设置表格头为伸缩模式
        self.ui.tableWidget_robot1.setEditTriggers(
            QAbstractItemView.NoEditTriggers)  # 将表格设置为禁止编辑
        self.ui.tableWidget_robot1.setSelectionBehavior(
            QAbstractItemView.SelectRows)  # 表格整行选中
        self.ui.tableWidget_robot1.resizeColumnsToContents()  # 将行与列的宽度高度与文本内容的宽高相匹配
        self.ui.tableWidget_robot1.resizeRowsToContents()  # 将行与列的宽度高度与文本内容的宽高相匹配

        self.ui.tableWidget_robot2.setColumnCount(4)
        self.ui.tableWidget_robot2.setHorizontalHeaderLabels(
            ['ID', 'PR', 'Pos', 'Check'])
        # self.ui.tableWidget_robot2.horizontalHeader().setSectionResizeMode(
        #     QHeaderView.Stretch)  # 设置表格头为伸缩模式
        self.ui.tableWidget_robot2.setEditTriggers(
            QAbstractItemView.NoEditTriggers)  # 将表格设置为禁止编辑
        self.ui.tableWidget_robot2.setSelectionBehavior(
            QAbstractItemView.SelectRows)  # 表格整行选中
        self.ui.tableWidget_robot2.resizeColumnsToContents()  # 将行与列的宽度高度与文本内容的宽高相匹配
        self.ui.tableWidget_robot2.resizeRowsToContents()  # 将行与列的宽度高度与文本内容的宽高相匹配

    def update_tablewidget_robot1(self, class_id, probability, position, check_from, row):
        self.ui.tableWidget_robot1.item(row, 0).setText(f'{class_id}')
        self.ui.tableWidget_robot1.item(row, 1).setText(f'{probability:.2f}')
        self.ui.tableWidget_robot1.item(row, 2).setText(
            f'({position[0]:.2f}, {position[1]:.2f})')
        self.ui.tableWidget_robot1.item(row, 3).setText(f'{check_from}')

    def insert_tablewidget_robot1(self, class_id, probability, position, check_from):
        rowPosition = self.ui.tableWidget_robot1.rowCount()
        self.ui.tableWidget_robot1.insertRow(rowPosition)
        numcols = self.ui.tableWidget_robot1.columnCount()
        numrows = self.ui.tableWidget_robot1.rowCount()
        self.ui.tableWidget_robot1.setRowCount(numrows)
        self.ui.tableWidget_robot1.setColumnCount(numcols)
        probability = "{:.2f}".format(float(probability))
        print("probability: ", probability)
        self.ui.tableWidget_robot1.setItem(
            numrows-1, 0, QTableWidgetItem(f'{class_id}'))
        self.ui.tableWidget_robot1.setItem(
            numrows-1, 1, QTableWidgetItem(probability))
        self.ui.tableWidget_robot1.setItem(
            numrows-1, 2, QTableWidgetItem(f'({position[0]:.2f}, {position[1]:.2f})'))
        self.ui.tableWidget_robot1.setItem(
            numrows-1, 3, QTableWidgetItem(f'{check_from}'))

    def update_tablewidget_robot2(self, class_id, probability, position, check_from, row):
        self.ui.tableWidget_robot2.item(row, 0).setText(f'{class_id}')
        self.ui.tableWidget_robot2.item(row, 1).setText(f'{probability:.2f}')
        self.ui.tableWidget_robot2.item(row, 2).setText(
            f'({position[0]:.2f}, {position[1]:.2f})')
        self.ui.tableWidget_robot2.item(row, 3).setText(f'{check_from}')

    def insert_tablewidget_robot2(self, class_id, probability, position, check_from):
        rowPosition = self.ui.tableWidget_robot2.rowCount()
        self.ui.tableWidget_robot2.insertRow(rowPosition)
        numcols = self.ui.tableWidget_robot2.columnCount()
        numrows = self.ui.tableWidget_robot2.rowCount()
        self.ui.tableWidget_robot2.setRowCount(numrows)
        self.ui.tableWidget_robot2.setColumnCount(numcols)
        self.ui.tableWidget_robot2.setItem(
            numrows-1, 0, QTableWidgetItem(f'{class_id}'))
        self.ui.tableWidget_robot2.setItem(
            numrows-1, 1, QTableWidgetItem(f'{probability:.2f}'))
        self.ui.tableWidget_robot2.setItem(
            numrows-1, 2, QTableWidgetItem(f'({position[0]:.2f}, {position[1]:.2f})'))
        self.ui.tableWidget_robot2.setItem(
            numrows-1, 3, QTableWidgetItem(f'{check_from}'))
    # try pyqtgraph

    def init_pyqtgraph(self):
        self.ui.verticalLayoutWidget_GL = QtWidgets.QWidget(
            self.ui.centralwidget)
        self.ui.verticalLayoutWidget_GL.setGeometry(
            QtCore.QRect(1493, 21, 400, 400))  # QWidget位置
        self.ui.verticalLayoutWidget_GL.setObjectName(
            "verticalLayoutWidget_GL")

        self.ui.GLlayout = QtWidgets.QVBoxLayout(
            self.ui.verticalLayoutWidget_GL)
        self.ui.GLlayout.setContentsMargins(0, 0, 0, 0)
        self.ui.GLlayout.setObjectName("GLlayout")

        self.viewer = gl.GLViewWidget()
        self.viewer.setCameraPosition(distance=40)
        self.ui.GLlayout.addWidget(self.viewer)

        g = gl.GLGridItem()
        g.setSize(100, 100)
        g.setSpacing(5, 5)
        self.viewer.addItem(g)

    def check_topic_is_exist(self, topic_name):
        topic_list = self.node.get_topic_names_and_types()
        # print("topic name: ", str(topic_name))
        for info in topic_list:
            # print(info[0])
            if info[0] == str(topic_name):
                print("topic name: " + str(topic_name) + " is Exist!!!!!")
                return True
        print("topic name: " + str(topic_name) + " can not Find!!!!!")
        return False
