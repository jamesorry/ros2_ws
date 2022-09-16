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

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SendMapImage, 'SendMapImageIng', 10) #### change here
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # images_dir = get_package_share_directory('qt_publish')
        # image_path = os.path.join(images_dir, 'images')
        # self.cv_image1 = cv2.imread(os.path.join(image_path + '/ToryMotor.jpg'))
        images_dir = get_package_share_directory('my_bringup')
        image_path = os.path.join(images_dir, 'maps')
        self.cv_image1 = cv2.imread(os.path.join(image_path + '/turtlebot3_world.pgm'))
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(PointCloud2, "/robot1/intel_realsense_r200_depth/points", self.callback_pointcloud, 10)
        
    def callback_pointcloud(self, data):
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        time.sleep(1)
        print(type(gen))
        for p in gen:
            print(" x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2]))
    
    def timer_callback(self):
        #### custom message
        my_msg = SendMapImage()
        my_msg.map_image = self.bridge.cv2_to_imgmsg(np.array(self.cv_image1), "bgr8")
        img = self.bridge.imgmsg_to_cv2(my_msg.map_image, "bgr8")
        #####
        self.publisher_.publish(my_msg) ## custom message
        cv2.imshow("IMAGE", img)
        self.get_logger().info('Publishing a batch of images')
        cv2.waitKey(4)

def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
     main()