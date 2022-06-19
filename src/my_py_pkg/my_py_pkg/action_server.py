#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer
from my_robot_interfaces.action import Navigate2D

from std_msgs.msg import String
from geometry_msgs.msg import Point


class NavigateActionServer(Node):
    def __init__(self):
        super().__init__("action_server")
        self.sub = self.create_subscription(
            Point, "robot_position", self.update_robot_position, 1)
        self._action_server = ActionServer(
            self, Navigate2D, "navigate", self.navigate_callback)
        self.robot_current_position = None
    
    def update_robot_position(self, point):
        self.robot_current_position = [point.x, point.y, point.z]

    def navigate_callback(self, goal_handle):
        print("Received Goal")
        robot_goal_point = [goal_handle.request.goal_point.x,
                            goal_handle.request.goal_point.y,
                            goal_handle.request.goal_point.z]
        print("Goal Point: " + str(robot_goal_point))
        
        while self.robot_current_position == None:
            print("Robot Point Not Detected")
            rclpy.spin_once(self, timeout_sec=3)


def main(args=None):
    rclpy.init(args=args)
    my_server = NavigateActionServer()
    print("Action Server Running...")

    try:
        while rclpy.ok():
            rclpy.spin_once(my_server)
    except KeyboardInterrupt:
        my_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
