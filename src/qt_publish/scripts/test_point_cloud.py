#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pickletools import stackslice
import sys
import os
import math
from turtle import color
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SendMapImage  # import custom message
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory, get_resources, get_resource, get_search_paths
import sensor_msgs_py.point_cloud2 as point_cloud2
import matplotlib.pyplot as plt
# 可以讀取到z軸的數值,但xy兩個數值需要經過轉換
import statistics

class TestPointCloud(Node):
    def __init__(self):
        super().__init__('TestPointCloud_Node')
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # save_text_dir = get_package_share_directory('qt_publish')
        # self.save_text_file = os.path.join(save_text_dir, 'doc_file', 'for_test_1.txt')
        self.save_text_file = "/home/james/ros2_ws/src/qt_publish/doc_file/for_test_1.txt"
        self.save_image_path = "/home/james/ros2_ws/src/qt_publish/images"
        self._num_count = 0
        self.x_list = []
        self.y_list = []
        self.x_list_fix = []
        self.y_list_fix = []
        self.z_list = []
        # self.subscriber_ = self.create_subscription(
        #     PointCloud2, "/robot1/intel_realsense_r200_depth/points", self.callback_pointcloud_save_to_text, 10)
        self.subscriber_ = self.create_subscription(
            PointCloud2, "/robot1/intel_realsense_r200_depth/points", self.callback_pointcloud_XY_Plot, 10)

    def callback_pointcloud_XY_Plot(self, data):
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(
            data, field_names=("x", "y", "z"), skip_nans=True)
        time.sleep(1)
        for p in gen:
            [(p[0], p[1], p[2]), (p[0], p[1], p[2])]
            # type(p[0]) and type(p[1]) and type(p[2]) is float
            x = p[0]
            y = p[1]
            z = p[2]
            # 除去inf的數值
            if not math.isinf(x) and not math.isinf(y):
                self.x_list.append(x)
                self.y_list.append(y)
                if (x <= 0.01 and x >= -0.01) and (y <= 0.01 and y >= -0.01):
                    self.x_list_fix.append(x)
                    self.y_list_fix.append(y)
                    self.z_list.append(z)
        print('X list max value:', max(self.x_list))
        print('X list min value:', min(self.x_list))
        print('Y list max value:', max(self.y_list))
        print('Y list min value:', min(self.y_list))
        if len(self.z_list) != 0:
            self._z_finial_disrance = statistics.mean(self.z_list)
            print('Z list mean value(m): ', self._z_finial_disrance) # 最後取平均值 得到z軸的距離
        self._num_count = self._num_count + 1
        print("writing.....", self._num_count)
        plt.plot(self.x_list, self.y_list)
        plt.plot(self.x_list_fix, self.y_list_fix, color='red')
        # plt.gca().invert_xaxis() # x軸逆序显示
        plt.gca().invert_yaxis() # y軸逆序显示
        plt.show()
        # plt.savefig(self.save_image_path + "_" + self._num_count + '.png')
        self.x_list_fix.clear()
        self.y_list_fix.clear()
        self.x_list.clear()
        self.y_list.clear()
        self.z_list.clear()

    def callback_pointcloud_save_to_text(self, data):
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(
            data, field_names=("x", "y", "z"), skip_nans=True)
        time.sleep(1)
        self._data_write_file = open(self.save_text_file, 'a')
        print("start print: ", type(gen), file=self._data_write_file)
        for p in gen:
            print(" x : %.3f  y: %.3f  z: %.3f" %
                  (p[0], p[1], p[2]), file=self._data_write_file)
        self._data_write_file.close
        print("writing.....")

    def get_depth(self, x, y):
        gen = point_cloud2.read_points(self.pc, field_names='z', skip_nans=False, uvs=[
                                       (x, y)])  # Questionable
        print(gen)
        return next(gen)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    TestPointCloud_Node = TestPointCloud()
    rclpy.spin(TestPointCloud_Node)
    TestPointCloud_Node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
