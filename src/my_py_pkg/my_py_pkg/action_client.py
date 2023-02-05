#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import Navigate2D

from geometry_msgs.msg import Point

import math

DISTANCE_THRESHOLD = 0.125

class NavigateActionClient(Node):
    def __init__(self):
        super().__init__("navigate_action_client_node")
        self._action_client = ActionClient(self, Navigate2D, "navigate")

    def send_goal(self, x, y):
        goal_msg = Navigate2D.Goal()
        goal_msg.goal_point.x = float(x)
        goal_msg.goal_point.y = float(y)
        goal_msg.goal_point.z = float(0)

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print("Received Feedback :" + str(feedback.distance_to_point))

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle.accepted == False:
            print("Goal Rejected")
            return None

        print("Goal Accepted ")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        print("Result: " + str(result.elapsed_time) + " seconds")
        rclpy.shutdown()



def main():
    rclpy.init()

    my_client = NavigateActionClient()

    print("Action Client Running...")

    try:

        x = input("Enter a X coordinate: ")
        y = input("Enter a Y coordinate: ")

        my_client.send_goal(x, y)

        rclpy.spin(my_client)


    except KeyboardInterrupt:
        my_client._action_client.destroy()
        my_client.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()