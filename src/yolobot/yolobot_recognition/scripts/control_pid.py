# https://cloud.tencent.com/developer/article/1456305
import time

#  因为X轴和Y轴相互独立。所以使用了两套闭环系统、两套PID。
# x軸是車身左右搖擺，y軸是前後移動


class PID:
    def __init__(self, P=[0.0, 0.0], I=[0.0, 0.0], D=[0.0, 0.0]):
        self.Kp係數 = P
        self.Ki係數 = I
        self.Kd係數 = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()

    def clear(self):
        self.SetPoint = [0.0, 0.0]  # 目標量[畫面中心點, 物體距離]
        self.PTerm = [0.0, 0.0]  # 系统自动使用的变量
        self.ITerm = [0.0, 0.0]
        self.DTerm = [0.0, 0.0]
        self.error偏差量 = [0.0, 0.0]  # 偏差量
        self.last_error = [0.0, 0.0]  # 上一次偏差量
        self.output = [0.0, 0.0]

    def update(self, feedback_value=[0.0, 0.0]):
        # print("pid_feedback: ", feedback_value)
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time  # 運行時間
        for n in range(0, 2):
            self.error偏差量[n] = self.SetPoint[n] - feedback_value[n]
            print("error偏差量[", n, "]: ", self.error偏差量[n])
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
