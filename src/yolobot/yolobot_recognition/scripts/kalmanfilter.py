# https://pysource.com/2021/10/29/kalman-filter-predict-the-trajectory-of-an-object/
import cv2
import numpy as np


class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2, 0)  # 4个状态变量，2个观测变量，0个控制变量
    '''
    這裡的4代表狀態向量的維度，因為我們要預測二維座標，所以狀態向量的維度是4。
    2代表觀察向量的維度，因為我們的觀察向量也是二維的，所以觀察向量的維度是2。
    最後一個0代表控制向量的維度，因為我們這個例子中沒有使用控制向量，所以控制向量的維度是0。
    '''
    # ! 定義測量矩陣
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)

    # ! 定義狀態轉移矩陣
    kf.transitionMatrix = np.array(
        [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    # ! 定義狀態協方差矩陣
    # kf.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03
    kf.processNoiseCov = np.array(
        [[1e-4, 0, 0, 0], [0, 1e-4, 0, 0], [0, 0, 2.5e-4, 0], [0, 0, 0, 2.5e-4]], np.float32)

    # ! 定義測量噪聲協方差矩陣
    kf.measurementNoiseCov = np.array([[1e-1, 0], [0, 1e-1]], np.float32)

    # ! 定義狀態後驗誤差協方差矩陣
    kf.errorCovPost = np.array(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    '''
    已經有十組二維座標，使用python cv2的kalman filter 要如何預測未來十組二維座標
    如果是ROS Image消息中的地圖，要如何將預測的座標，避開特定區域？
    sensor_msgs/Image
    
    # ***********************************************************************
    dt = 1.0/30.0 # !每幀的時間間隔
    F = np.array([[1, dt, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, dt],
                [0, 0, 0, 1]], dtype=np.float32) # !狀態轉移矩陣
    H = np.array([[1, 0, 0, 0],
                [0, 0, 1, 0]], dtype=np.float32) # !觀測矩陣
    Q = np.eye(4, dtype=np.float32) * 0.1 # !系統噪聲協方差矩陣
    R = np.eye(2, dtype=np.float32) * 0.01 # !測量噪聲協方差矩陣
    kalman = cv2.KalmanFilter(state_dim, measure_dim, 0) # ! 狀態向量維度為4，觀測向量維度為2，控制變量為0
    kalman.transitionMatrix = F
    kalman.measurementMatrix = H
    kalman.processNoiseCov = Q
    kalman.measurementNoiseCov = R
    kalman.errorCovPost = np.eye(4, dtype=np.float32) # ! 初始誤差協方差矩陣
    '''

    # 初始化Kalman濾波器的狀態
    kf.statePost = np.array([[0], [0], [0], [0]], np.float32)
    is_print = True

    def init(self):
        x, y = self.max_contour[0][0]
        self.kf.statePost = np.array([[x], [y], [0], [0]], np.float32)

    def update(self, coordX, coordY):
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)

    def ObstacleDetector(self, map_msg):
        # 讀取地圖消息中的灰度圖像，並轉換為二值化的掩膜

        # 將 bgr8 地圖轉換為灰度圖像
        self.gray_img = cv2.cvtColor(map_msg, cv2.COLOR_BGR2GRAY)

        # 二值化圖像
        _, binary_img = cv2.threshold(
            self.gray_img, 30, 255, cv2.THRESH_BINARY)

        # 尋找輪廓
        self.contours, _ = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.max_contour = max(self.contours, key=cv2.contourArea)
        self.init()

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

    def predict_avoid_obstacle(self, coordX, coordY):
        # 更新 Kalman 滤波器
        measurement = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measurement)
        # 預測下一個狀態
        predicted_state = self.kf.predict()
        # 調整測量值以避開障礙物
        # 檢測預測的座標是否在輪廓內部

        if cv2.pointPolygonTest(self.max_contour, (int(predicted_state[0]), int(predicted_state[1])), False) >= 0:
            # 如果預測座標在輪廓內部，直接使用預測座標作為更新
            print("ok")
        else:
            # 如果預測座標在輪廓外部，找到最近的輪廓點作為更新
            print("fail")
            dist = cv2.pointPolygonTest(self.max_contour, (int(
                predicted_state[0]), int(predicted_state[1])), True)
            print("dist: ", dist)
            if dist != 0:
                distances = np.sqrt(
                    np.sum((self.max_contour - (int(predicted_state[0]), int(predicted_state[1])))**2, axis=2))
                min_index = np.argmin(distances)
                closest_pt = self.max_contour[min_index][0]
                '''
                為了找到最近的輪廓點，我們可以先計算所有輪廓點和預測座標之間的歐幾里得距離，然後找到最小距離對應的輪廓點即可。
                具體來說，我們可以使用numpy庫的sum()函數來計算每個輪廓點和預測座標之間的平方差，
                然後使用argmin()函數找到最小平方差對應的輪廓點的索引。最後，我們可以使用該索引來獲取最近的輪廓點。
                '''
                measurement = np.array(
                    [[closest_pt[0]], [closest_pt[1]]], np.float32)
                x, y = int(measurement[0]), int(measurement[1])
                return x, y
        x, y = int(predicted_state[0]), int(predicted_state[1])
        return x, y
