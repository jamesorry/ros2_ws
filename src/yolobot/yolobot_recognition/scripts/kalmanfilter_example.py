import numpy as np
import cv2

# 定義 Kalman Filter 的狀態變量數量和觀測變量數量
state_dim, measure_dim = 4, 2

# 創建一個 Kalman Filter 對象
kf = cv2.KalmanFilter(state_dim, measure_dim)

# 設定 Kalman Filter 的狀態轉移矩陣 A、觀測矩陣 H、控制矩陣 B、狀態噪聲協方差 Q、觀測噪聲協方差 R、狀態估計協方差 P、狀態預測值 x、觀測值 z
kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kf.processNoiseCov = np.eye(state_dim, dtype=np.float32) * 0.001
kf.measurementNoiseCov = np.eye(measure_dim, dtype=np.float32) * 1
kf.errorCovPost = np.eye(state_dim, dtype=np.float32)
kf.statePost = np.zeros((state_dim, 1), np.float32)

# 設定初始狀態的位置和速度
kf.statePost[0][0] = 10
kf.statePost[1][0] = 10
kf.statePost[2][0] = 1
kf.statePost[3][0] = 1

# 假設現在已經有十組二維座標，並存儲在一個二維數組 coords 中，每一行代表一個座標點的 x、y 坐標
coords = np.array([[10, 10], [20, 20], [30, 30], [40, 40], [50, 50], [60, 60], [70, 70], [80, 80], [90, 90], [100, 100]])

# 創建一個空的結果數組，用於存儲預測出的座標點
results = []
# 逐一處理每個座標點
for i in range(len(coords)):
    # 將座標點轉換為觀測值 z
    z = np.array([[coords[i][0]], [coords[i][1]]], np.float32)
    
    # 進行狀態預測
    x = kf.predict()
    
    # 進行狀態更新
    kf.correct(z)
    
    # 將預測值存入結果數組中
    results = (int(x[0]), int(x[1]))
    
# 查看預測結果
print(results)
