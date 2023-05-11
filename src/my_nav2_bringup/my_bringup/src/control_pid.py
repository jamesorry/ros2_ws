# https://cloud.tencent.com/developer/article/1456305
import time
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
#  因为X轴和Y轴相互独立。所以使用了两套闭环系统、两套PID。
# x軸是車身左右搖擺，y軸是前後移動
# ! 因為action server 是在多線程中運行的，但matplotlib必須要在主線程中才能運行。
# 因此換個思考方向，可以先將資料push到ros2上，再去訂閱回來即可


class PID:
    def __init__(self, P=[0.0, 0.0], I=[0.0, 0.0], D=[0.0, 0.0]):
        self.Kp係數 = P
        self.Ki係數 = I
        self.Kd係數 = D
        self.sample_time = 0.00
        self.current_time = None
        self.last_time = self.current_time
        self.clear()

    def update_pid(self, P=[0.0, 0.0], I=[0.0, 0.0], D=[0.0, 0.0]):
        self.Kp係數 = P
        self.Ki係數 = I
        self.Kd係數 = D
        self.ITerm = [0.0, 0.0]

    def init_pid_plot(self):
        # 设置图表
        plt.title('Real-time PID Curve')
        plt.ion()  # 开启交互模式

        # 创建第一个子图
        self.sub1_x_時間 = deque(maxlen=30)  # 初始化 x 数据(時間)
        self.sub1_y_實際獲取值 = deque(maxlen=30)  # sub1 初始化第一个 y 数据(實際獲取值)
        self.sub1_y_輸出值 = deque(maxlen=30)  # sub1 初始化第二个 y 数据(輸出值)
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

    def clear(self):
        self.SetPoint = [0.0, 0.0]  # 目標量[畫面中心點, 物體距離]
        self.PTerm = [0.0, 0.0]  # 系统自动使用的变量
        self.ITerm = [0.0, 0.0]
        self.DTerm = [0.0, 0.0]
        self.error偏差量 = [0.0, 0.0]  # 偏差量
        self.last_error = [0.0, 0.0]  # 上一次偏差量
        self.output = [0.0, 0.0]

    '''
    # 將datetime對象轉換成time對象
    start_time = datetime.now().time()

    # 將time對象轉換成rclpy.time.Time對象
    rclpy_start_time = Time.from_time(start_time)
    '''

    def update(self, feedback_value, _time):
        # print("pid_feedback: ", feedback_value)
        self.current_time = _time  # time.time()
        delta_time = self.current_time - self.last_time  # 運行時間
        # print(type(self.current_time))
        # print(type(self.last_time))
        # print(type(delta_time))
        # print(delta_time)
        delta_time = (delta_time.nanoseconds * 1e-9)
        for n in range(0, 2):
            self.error偏差量[n] = self.SetPoint[n] - feedback_value[n]
            # print("error偏差量[", n, "]: ", self.error偏差量[n])
            # print("SetPoint[", n, "]: ", self.SetPoint[n])

            # if (delta_time >= self.sample_time):
            self.PTerm[n] = self.error偏差量[n] * self.Kp係數[n]  # 比例

            self.ITerm[n] = self.ITerm[n] + \
                self.error偏差量[n] * delta_time  # 積分

            # self.DTerm[n] = 0.0
            self.DTerm[n] = (self.error偏差量[n] -
                             self.last_error[n]) / delta_time  # 微分

            self.last_time = self.current_time
            self.last_error[n] = self.error偏差量[n]
            self.output[n] = (self.Kp係數[n] * self.PTerm[n]) + \
                (self.Ki係數[n] * self.ITerm[n]) + \
                (self.Kd係數[n] * self.DTerm[n])
            # print("Kp係數[", n, "]: ", self.Kp係數[n])
            # print("PTerm[", n, "]: ", self.PTerm[n])
            # print("Ki係數[", n, "]: ", self.Ki係數[n])
            # print("ITerm[", n, "]: ", self.ITerm[n])
            # print("Kd係數[", n, "]: ", self.Kd係數[n])
            # print("DTerm[", n, "]: ", self.DTerm[n])
        # print("=============================================")

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time
