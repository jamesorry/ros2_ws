#https://pysource.com/2021/10/29/kalman-filter-predict-the-trajectory-of-an-object/
import cv2
import numpy as np


class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2, 0) # 4个状态变量，2个观测变量，0个控制变量
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    # kf.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03
    kf.processNoiseCov = np.array([[1e-4, 0, 0, 0], [0, 1e-4, 0, 0], [0, 0, 2.5e-4, 0], [0, 0, 0, 2.5e-4]], np.float32)
    kf.measurementNoiseCov = np.array([[1e-1, 0], [0, 1e-1]], np.float32)
    
    # 初始化Kalman濾波器的狀態
    kf.statePost = np.array([[0], [0], [0], [0]], np.float32)

    def init(self):
        self.kf.statePost = np.array([[0], [0], [0], [0]], np.float32)
    
    def update(self, coordX, coordY):
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
    
    def only_predict(self):
        predicted = self.kf.predict()
        x, y = int(predicted[0]), int(predicted[1])
        return x, y
    
    def predict(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        # 更新 Kalman 滤波器
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x, y = int(predicted[0]), int(predicted[1])
        return x, y

