#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("initial_pose")
        # https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
        self.declare_parameter("robot_num", 1)
        self.declare_parameter("init_pose_array")
        self.robot_num = self.get_parameter("robot_num").value
        self.init_pose_array = self.get_parameter("init_pose_array").value
        # self.topic_name = "/robot" + self.get_parameter("robot_num").value + "/initialpose"
        self.alpha = math.pi/2.0  # radian value
        print("====================================")
        print("robot_num: ", self.robot_num)
        print("init_pose_array: ", self.init_pose_array)

        if self.robot_num == 1:
            self.pose_1_publisher_ = self.create_publisher(
                PoseWithCovarianceStamped, "/robot1/initialpose", 10)
        elif self.robot_num == 2:
            self.pose_1_publisher_ = self.create_publisher(
                PoseWithCovarianceStamped, "/robot1/initialpose", 10)
            self.pose_2_publisher_ = self.create_publisher(
                PoseWithCovarianceStamped, "/robot2/initialpose", 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("initial_pose has been started.")

    def timer_callback(self):
        if self.robot_num == 1:
            self.Publusher_1()
        elif self.robot_num == 2:
            self.Publusher_1()
            self.Publusher_2()

    def Publusher_1(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = self.init_pose_array[0]
        pose_msg.pose.pose.position.y = self.init_pose_array[1]
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        self.pose_1_publisher_.publish(pose_msg)
        self.get_logger().info("robot1 public initial_pose.")

    def Publusher_2(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = self.init_pose_array[2]
        pose_msg.pose.pose.position.y = self.init_pose_array[3]
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[6 * 1 + 1] = 0.25
        pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0
        self.pose_2_publisher_.publish(pose_msg)
        self.get_logger().info("robot2 public initial_pose.")


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    number_of_cycles = 5
    spin_count = 0
    while rclpy.ok() and spin_count < number_of_cycles:
        rclpy.spin_once(node, timeout_sec=1.0)
        spin_count += 1
        print('spin_count: ' + str(spin_count))
    rclpy.shutdown()
    print("publish end")


if __name__ == "__main__":
    main()
