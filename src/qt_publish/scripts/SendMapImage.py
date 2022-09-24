#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SendMapImage  # import custom message
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory
import argparse


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('send_map_image')
        self.publisher_ = self.create_publisher(
            SendMapImage, 'SendMapImageIng', 10)  # change here
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        images_dir = get_package_share_directory('my_bringup')
        image_path = os.path.join(images_dir, 'maps', 'turtlebot3_world.pgm')

        parser = argparse.ArgumentParser(
            description='Send pgm map file')
        parser.add_argument('-m', '--map', type=str, default=image_path,
                            help='Map path')
        args, unknown = parser.parse_known_args()

        self.cv_image1 = cv2.imread(args.map)
        self.bridge = CvBridge()
        print("start send map image.")

    def timer_callback(self):
        my_msg = SendMapImage()
        my_msg.map_image = self.bridge.cv2_to_imgmsg(
            np.array(self.cv_image1), "bgr8")
        self.publisher_.publish(my_msg)
        # img = self.bridge.imgmsg_to_cv2(my_msg.map_image, "bgr8")
        # cv2.imshow("IMAGE", img)
        # self.get_logger().info('Publishing a batch of images')
        # cv2.waitKey(3)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
