#!/usr/bin/env python3
import imp
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.subscriber_ = self.create_subscription(
            Int64, "number", self.callbackCounter, 10)
        self.reset_counter_service_ = self.create_service(
            SetBool, "reset_counter", self.callbackResetCounter)
        self.get_logger().info("number_counter has been started.")

    def callbackCounter(self, msg):
        self.counter_ += msg.data
        self.get_logger().info("number_counter: " + str(self.counter_))
        new_data = Int64()
        new_data.data = self.counter_
        self.publisher_.publish(new_data)

    def callbackResetCounter(self, request, response):
        if request.data:
            self.counter_ = 0
            response.success = True
            response.message = "Counter has been reset."
        else:
            response.success = False
            response.message = "Counter has not been reset."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
