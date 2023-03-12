#!/usr/bin/python3
import rclpy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist

def move_model():
    rclpy.init()
    node = rclpy.create_node('move_model_node')
    pub = node.create_publisher(ModelState, '/gazebo/set_model_state', 10)
    rate = node.create_rate(10)

    # 设置模型的初始状态
    model_state = ModelState()
    model_state.model_name = 'box_bot_0'
    model_state.pose.position.x = 0.0
    model_state.pose.position.y = 0.0
    model_state.pose.position.z = 0.0
    model_state.pose.orientation.x = 0.0
    model_state.pose.orientation.y = 0.0
    model_state.pose.orientation.z = 0.0
    model_state.pose.orientation.w = 1.0
    model_state.twist.linear.x = 0.0
    model_state.twist.linear.y = 0.0
    model_state.twist.linear.z = 0.0
    model_state.twist.angular.x = 0.0
    model_state.twist.angular.y = 0.0
    model_state.twist.angular.z = 0.0

    while rclpy.ok():
        # 发布模型的新状态
        pub.publish(model_state)

        # 移动模型
        twist = Twist()
        twist.linear.x = 0.1 # 设置线速度
        twist.angular.z = 0.1 # 设置角速度
        model_state.twist = twist

        # 等待一段时间
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    move_model()