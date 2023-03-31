#!/usr/bin/env python3
from nav_msgs.msg import *
from sensor_msgs.msg import *
from my_robot_interfaces.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import rclpy
from rclpy.node import Node
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from builtin_interfaces.msg import Time


class MyNode(Node):

    def __init__(self):
        super().__init__("plot_pid_curve")
        self.get_logger().info("start plot_pid_curve node")
        __sub_pid_curve_name = "/pid_curve"
        self.sub_pid_curve = self.create_subscription(
            CurvePIDarray, __sub_pid_curve_name, self.get_plot_pid_curve_data, 20)
        self.init_pid_plot()
        self.time_received = None
        
    def get_plot_pid_curve_data(self, data: CurvePIDarray):
        # builtin_interfaces/Time time_stamp
        # float64 line_feedback_value
        # float64 line_output
        if self.time_received is not None:
            for num, curvepid in enumerate(data.curvepid):
                time_stamp = curvepid.time_stamp
                seconds_passed = time_stamp.sec - self.time_received.sec #time_stamp.sec + time_stamp.nanosec / 1e9
                nanoseconds_passed = time_stamp.nanosec - self.time_received.nanosec
                total_seconds_passed = seconds_passed + nanoseconds_passed / 1e9
                feedback_value = curvepid.line_feedback_value
                if num == 0:
                    output = 350.0# curvepid.line_output
                elif num == 1:
                    output = 1.05
                # print("num: ", num)
                # print("seconds: ", total_seconds_passed)
                # print("feedback_value: ", feedback_value)
                # print("output: ", output)
                self.update_plot_data(num, feedback_value, output, total_seconds_passed)
        else:
            for num, curvepid in enumerate(data.curvepid):
                self.time_received = curvepid.time_stamp

    def init_pid_plot(self):
        # 设置图表
        plt.title('Real-time PID Curve')
        plt.ion()  # 开启交互模式

        # 创建第一个子图
        self.sub1_x_時間 = deque(maxlen=50)  # 初始化 x 数据(時間)
        self.sub1_y_實際獲取值 = deque(maxlen=50)  # sub1 初始化第一个 y 数据(實際獲取值)
        self.sub1_y_輸出值 = deque(maxlen=50)  # sub1 初始化第二个 y 数据(輸出值)
        self.ax1 = plt.subplot(2, 1, 1)
        #self.ax1.set_xlim([0, 10])
        #self.ax1.set_ylim([-5, 20])
        self.sub1_line1, = self.ax1.plot([], [])
        self.sub1_line2, = self.ax1.plot([], [])

        # 创建第二个子图
        self.sub2_x_時間 = deque(maxlen=30)  # 初始化 x 数据(時間)
        self.sub2_y_實際獲取值 = deque(maxlen=30)  # sub2 初始化第一个 y 数据(實際獲取值)
        self.sub2_y_輸出值 = deque(maxlen=30)  # sub2 初始化第二个 y 数据(輸出值)
        self.ax2 = plt.subplot(2, 1, 2)
        #self.ax2.set_xlim([0, 10])
        #self.ax2.set_ylim([-5, 20])
        self.sub2_line1, = self.ax2.plot([], [])
        self.sub2_line2, = self.ax2.plot([], [])

    def draw_pid_plot(self):
        # 更新第一个子图中的两条线条
        self.sub1_line1.set_data(self.sub1_x_時間, self.sub1_y_實際獲取值)
        self.sub1_line2.set_data(self.sub1_x_時間, self.sub1_y_輸出值)
        self.ax1.relim()
        self.ax1.autoscale_view()
        # 更新第二个子图中的两条线条
        self.sub2_line1.set_data(self.sub2_x_時間, self.sub2_y_實際獲取值)
        self.sub2_line2.set_data(self.sub2_x_時間, self.sub2_y_輸出值)
        self.ax2.relim()
        self.ax2.autoscale_view()
        plt.draw()
        plt.pause(0.005)

    def update_plot_data(self, sub_num, process_variable, output, current_time):
        if sub_num == 0:
            self.sub1_x_時間.append(current_time)
            self.sub1_y_實際獲取值.append(process_variable)
            self.sub1_y_輸出值.append(output)
        elif sub_num == 1:
            self.sub2_x_時間.append(current_time)
            self.sub2_y_實際獲取值.append(process_variable)
            self.sub2_y_輸出值.append(output)
        self.draw_pid_plot()


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
