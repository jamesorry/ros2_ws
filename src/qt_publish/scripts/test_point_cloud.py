#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SendMapImage #### import custom message
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,PointCloud2
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory
import sensor_msgs_py.point_cloud2 as point_cloud2

class TestPointCloud(Node):
    def __init__(self):
        super().__init__('TestPointCloud_Node')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(PointCloud2, "/robot1/intel_realsense_r200_depth/points", self.callback_pointcloud, 10)
        
    def callback_pointcloud(self, data):
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        time.sleep(1)
        print(type(gen))
        for p in gen:
            print(" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))
    
    def get_depth(self, x, y):
        gen = point_cloud2.read_points(self.pc, field_names='z', skip_nans=False, uvs=[(x, y)]) #Questionable
        print (gen)
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