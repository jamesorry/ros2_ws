#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SendMapImage  # import custom message
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, LaserScan
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory
import argparse
from nav_msgs.msg import OccupancyGrid


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('send_map_image')
        self.publisher_ = self.create_publisher(
            SendMapImage, 'SendMapImageIng', 10)  # change here
        timer_period = 1.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # images_dir = get_package_share_directory('my_bringup')
        # image_path = os.path.join(images_dir, 'maps', 'turtlebot3_world.pgm')

        parser = argparse.ArgumentParser(
            description='Send pgm map file')
        parser.add_argument('-m', '--map', type=str, default='/robot1',
                            help='Map path')
        args, unknown = parser.parse_known_args()

        self.cv_image1 = cv2.imread(args.map)
        self.bridge = CvBridge()

        occupancygrid_node = args.map + '/global_costmap/costmap'
        self.occupancygrid_subscription = self.create_subscription(
            OccupancyGrid, occupancygrid_node, self.convert, 10)
        self.occupancygrid_subscription  # 避免被垃圾收集器回收

        # laser_node = args.map + '/scan'
        # self.laser_subscription = self.create_subscription(
        #     LaserScan, laser_node, self.scan_callback, 10)
        # self.laser_subscription  # 避免被垃圾收集器回收

        print("start send map image.")

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        for i in range(len(ranges)):
            degree = i * msg.angle_increment * 180 / 3.14159
            distance = ranges[i]
            print("Degree: {:0.2f}, Distance: {:0.2f}".format(
                degree, distance))

    def convert(self, occupancy_grid_msg: OccupancyGrid):
        # 轉換 OccupancyGrid 訊息到 numpy 陣列
        grid_data = list(occupancy_grid_msg.data)
        grid_data = [grid_data[i:i+occupancy_grid_msg.info.width]
                     for i in range(0, len(grid_data), occupancy_grid_msg.info.width)]
        grid_data = np.asarray(grid_data, dtype=np.int8)

        # 計算 RGB 值並轉換成圖像
        img_data = (grid_data * 255 / 100).astype(np.uint8)
        img_data = np.stack((img_data, img_data, img_data), axis=2)
        img_data = 255 - img_data  # 將 img_data 進行反轉
        img_data = np.flipud(img_data)  # 進行垂直翻轉
        # 將圖像轉換為 ROS 2 訊息
        image_msg = self.bridge.cv2_to_imgmsg(img_data, encoding='rgb8')
        # image_msg.header = occupancy_grid_msg.header

        my_msg = SendMapImage()
        my_msg.map_image = image_msg
        my_msg.origin_position_x = occupancy_grid_msg.info.origin.position.x
        my_msg.origin_position_y = occupancy_grid_msg.info.origin.position.y
        # 發布圖像訊息
        self.publisher_.publish(my_msg)

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
