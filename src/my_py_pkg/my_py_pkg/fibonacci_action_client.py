# https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
# https://github.com/ros2/examples/blob/foxy/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client_cancel.py
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_robot_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')
        # 啟動計時器，兩秒後觸發取消action
        # Start a 2 second timer
        self._timer = self.create_timer(2.0, self.timer_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
