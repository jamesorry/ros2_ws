#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import TurnCamera

import os
import cv2
from cv_bridge import CvBridge


class TurnCameraClient(Node):
    def __init__(self):
        super().__init__('turn_camera_client')
        self.client = self.create_client(TurnCamera, "turn_camera")
        self.req = TurnCamera.Request()

    def send_request(self, deg_num):
        self.req.degree_turn = float(deg_num)
        self.client.wait_for_service()

        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        self.result = self.future.result()
        return self.result.image

    def display_image(self, image_msg):
        image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        cv2.imshow("Turn Camera Image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()



def main():
    rclpy.init()

    client_node = TurnCameraClient()

    print("Turn Camera Service Client Running ...")

    try:
        user_val = input("Enter in a number (in degrees) to turn robot camera: ")
        res_image_msg = client_node.send_request(user_val)
        client_node.display_image(res_image_msg)
    except KeyboardInterrupt:
        client_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
