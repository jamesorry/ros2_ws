#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
class GetRobotPoseNode(Node):
    def __init__(self):
        super().__init__("getRobotPose")
        self.subscriber_ = self.create_subscription(
            Odometry, "/robot1/odom", self.callback_robot1_pose, 10)
        self.subscriber_ = self.create_subscription(
            Odometry, "/robot2/odom", self.callback_robot2_pose, 10)
        self.get_logger().info("getRobotPose has been started.")

    def callback_robot1_pose(self, msg):
        print("robot1 position------------")
        print('x: ',msg.pose.pose.position.x)
        print('y: ',msg.pose.pose.position.y)
        print('z: ',msg.pose.pose.position.z)
    
    def callback_robot2_pose(self, msg):
        print("robot2 position------------")
        print('x: ',msg.pose.pose.position.x)
        print('y: ',msg.pose.pose.position.y)
        print('z: ',msg.pose.pose.position.z)

def main(args=None):
    rclpy.init(args=args)
    node = GetRobotPoseNode()
    while rclpy.ok():
        rclpy.spin_once(node)
        time.sleep(0.5)
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
