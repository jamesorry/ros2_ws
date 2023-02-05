#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_interfaces.action import Navigate2D

from geometry_msgs.msg import Point

import math

DISTANCE_THRESHOLD = 0.125

class NavigateActionServer(Node):
    def __init__(self):
        super().__init__("navigate_action_server_node")
        self.sub = self.create_subscription(Point, "robot_position", self.update_robot_position, 1)
        self._action_server = ActionServer(self, Navigate2D, "navigate", self.navigate_callback)
        self.robot_current_position = None

    def update_robot_position(self, point):
        self.robot_current_position = [point.x, point.y, point.z]

    def navigate_callback(self, goal_handle):
        print("Received Goal")
        start_time = self.get_clock().now().to_msg().sec

        robot_goal_point = [goal_handle.request.goal_point.x,
                            goal_handle.request.goal_point.y,
                            goal_handle.request.goal_point.z]

        print("Goal Point: " + str(robot_goal_point))

        while self.robot_current_position == None:
            print("Robot Point Not Detected")
            rclpy.spin_once(self, timeout_sec=3)

        print("Robot Point Detected")

        distance_to_goal = math.dist(self.robot_current_position, robot_goal_point)

        feedback_msg = Navigate2D.Feedback()

        while distance_to_goal > DISTANCE_THRESHOLD:
            distance_to_goal = math.dist(self.robot_current_position, robot_goal_point)
            feedback_msg.distance_to_point = distance_to_goal

            goal_handle.publish_feedback(feedback_msg)

            rclpy.spin_once(self, timeout_sec=1)

        goal_handle.succeed()
        print("Goal Succeeded")

        result = Navigate2D.Result()
        result.elapsed_time = float(self.get_clock().now().to_msg().sec - start_time)

        return result



def main():
    rclpy.init()

    my_server = NavigateActionServer()

    print("Action Server Running...")

    try:
        while rclpy.ok():
            rclpy.spin_once(my_server)
    except KeyboardInterrupt:
        my_server._action_server.destroy()
        my_server.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()