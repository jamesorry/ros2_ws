'''
如果您在ROS2中使用OpenCV bgr8地圖，並且需要預測未來十組座標並避開地圖中的障礙物，您可以使用以下步驟：

    使用ROS2的圖像處理庫來讀取和顯示OpenCV bgr8地圖。
    使用ROS2的機器人操作系統（ROS）中提供的座標系統和位移信息，將前十組座標轉換為ROS2中的全局座標系統。
    使用ROS2的地圖庫來載入地圖並標記障礙物。
    在全局座標系統中，使用路徑規劃算法（例如A*或Dijkstra）來計算避開障礙物的十組座標的路徑。
    使用卡爾曼濾波器來預測未來十組座標。
    將預測座標映射到地圖的像素坐標系統中，並檢查每個預測座標是否在障礙物上。
    如果預測座標在障礙物上，重新計算路徑，避開該障礙物。
    將預測座標和其對應的協方差矩陣返回。

以下是一個使用卡爾曼濾波器預測未來十組座標並避開地圖中障礙物的示例Python代碼：
# ! 這沒辦法用
'''
import numpy as np
import cv2
import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 定義卡爾曼濾波器的狀態維度和觀測維度
state_dim = 4  # (x, y, vx, vy)
obs_dim = 2  # (x, y)

# 定義卡爾曼濾波器的狀態轉移矩陣和觀測矩陣
dt = 1.0  # 時間步長
A = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])  # 狀態轉移矩陣
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])  # 觀測矩陣

# 定義卡爾曼濾波器的過程
class KalmanFilter:
    def __init__(self, x0, P0, Q, R):
        self.x = x0  # 狀態估計
        self.P = P0  # 狀態估計的協方差矩陣
        self.Q = Q  # 狀態轉移噪聲的協方差矩陣
        self.R = R  # 觀測噪聲的協方差矩陣

    def predict(self):
        self.x = A @ self.x  # 狀態預測
        self.P = A @ self.P @ A.T + self.Q  # 狀態協方差預測

    def update(self, z):
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.R)  # 卡爾曼增益
        self.x = self.x + K @ (z - H @ self.x)  # 狀態更新
        self.P = (np.eye(state_dim) - K @ H) @ self.P  # 狀態協方差更新

    # 定義路徑規劃算法（使用A*算法）
    def plan_path(start, goal, map):
        # 將起點和終點映射到地圖上的像素坐標
        start_px = map.world_to_map(start[0], start[1])
        goal_px = map.world_to_map(goal[0], goal[1])

        # 定義A*算法需要的數據結構
        open_set = {start_px: 0}
        came_from = {}
        g_score = {start_px: 0}
        f_score = {start_px: heuristic(start_px, goal_px)}

        # 開始A*算法
        while open_set:
            # 從open_set中選擇最佳節點
            current_px = min(open_set, key=lambda k: f_score[k])
            if current_px == goal_px:
                # 找到路徑
                path_px = [current_px]
                while current_px in came_from:
                    current_px = came_from[current_px]
                    path_px.append(current_px)
                path_px.reverse()
                return [map.map_to_world(px[0], px[1]) for px in path_px]

            # 將節點從open_set中移除
            del open_set[current_px]

            # 對節點的相鄰節點進行擴展
            for neighbor_px in map.get_neighbors(current_px):
                # 計算新路徑的代價
                tentative_g_score = g_score[current_px] + map.get_cost(current_px, neighbor_px)
                if neighbor_px not in g_score or tentative_g_score < g_score[neighbor_px]:
                    # 更新節點的代價
                    came_from[neighbor_px] = current_px
                    g_score[neighbor_px] = tentative_g_score
                    # 更新節點的估計代價
                    f_score[neighbor_px] = tentative_g_score + heuristic(neighbor_px, goal_px)
                    # 將節點添加到open_set中
                    if neighbor_px not in open_set:
                        open_set[neighbor_px] = f_score[neighbor_px]

        # 找不到路徑，返回空列表
        return []

    # 定義預測算法
    def predict_path(start, goal, map, initial_state, num_steps):
        # 規劃初始路徑
        path = plan_path(start, goal, map)
        if not path:
            # 找不到路徑，返回空列表
            return []

        # 將路徑轉換為像素坐標
        path_px = [map.world_to_map(pt[0], pt[1]) for pt in path]

        # 初始化卡爾曼濾波器
        kf = KalmanFilter(initial_state, P0, Q, R)

        # 遍歷每一個時間步
        for t in range(num_steps):
            # 如果已經到達終點，則停止預測
            if len(path_px) == 1:
                break

            # 預測下一個時間步的狀態
            kf.predict()

            # 更新觀測值
            z = np.array([path_px[1][0], path_px[1][1]])

            # 如果障礙物被檢測到，則重新計算路徑
            if map.is_obstacle(path_px[1]):
                new_path = plan_path(map.map_to_world(*path_px[0]), goal, map)
                if not new_path:
                    # 找不到路徑，返回空列表
                    return []
                path_px = [map.world_to_map(pt[0], pt[1]) for pt in new_path]
                z = np.array([path_px[1][0], path_px[1][1]])

            # 更新卡爾曼濾波器
            kf.update(z)

            # 將預測結果轉換為世界坐標
            predicted_px = kf.x[:2].astype(int)
            predicted_world = map.map_to_world(*predicted_px)

            # 將預測結果添加到路徑中
            path.insert(1, predicted_world)
            path_px.insert(1, predicted_px)

        # 返回預測的路徑
        return path
