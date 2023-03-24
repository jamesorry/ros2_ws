#!/usr/bin/env python3
from kalmanfilter import KalmanFilter
from nav_msgs.msg import *
from sensor_msgs.msg import *
from my_robot_interfaces.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import rclpy
from rclpy.node import Node
import math
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
# use oop
# Load Kalman filter to predict the trajectory
kf = KalmanFilter()
import time
from collections import deque

def quaternion_to_euler_angles(x, y, z, w):
    # 四元數轉歐拉角
    # 此網站才是正確轉換(https://blog.csdn.net/Fzc_Ztt/article/details/116668569)
    # (https://zh.m.wikipedia.org/zh-tw/%E5%BC%A7%E5%BA%A6)
    """w、x、y、z to euler angles"""
    try:
        angles = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))

        # print(f"r: {r}, p: {p}, y: {y}")

        angles['roll'] = r*180/math.pi  # (degree)
        angles['pitch'] = p*180/math.pi
        angles['yaw'] = y*180/math.pi

        return angles
    except:
        pass


def quaternion_from_euler(ai, aj, ak):
    # 歐拉角轉四元數
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q


class MyNode(Node):

    def __init__(self):
        super().__init__("get_position_node")
        self.counter_ = 0
        self.get_logger().info("start ros2 get_position_node")
        self.robot_1_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.robot_1_quaternion = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        self.map_resolution = 0.05
        self.robot_1_euler_angles = {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}
        self.Map_Image = None
        self.Init_Map_Image = None
        self.yolo_result_robot1 = {"class_id": [],
                                   "probability": [],
                                   "center_dist": [],
                                   "position": [],
                                   "degree": []
                                   }
        self.sub_robot1_position = self.create_subscription(
            Odometry, '/robot1/odom', self.sub_robot_1_position, 10)
        self.sub_robot1_position
        self.sub_bounding_boxes_robot1 = self.create_subscription(
            BoundingBoxesV2, '/robot1/yolov5/bounding_boxes', self.get_robot1_bounding_boxes, 10)
        self.sub_bounding_boxes_robot1
        self.sub_map_image = self.create_subscription(
            SendMapImage, '/SendMapImageIng', self.loadImage, 10)
        self.sub_map_image
        self.create_timer(0.5, self.timer_callback)
        self.now_distance = 0.0
        self.last_distance = 0.0
        self.__points = deque(maxlen=200)

        self.laser_ranges = None
        self.laser_subscription = self.create_subscription(
            LaserScan, '/robot1/scan', self.scan_callback, 10)
        self.laser_subscription
        
        self.coordinates = deque(maxlen=20)  # 创建一个队列，并指定最大长度为10

    def timer_callback(self):
        self.pose_to_pixel_map()

    def sub_robot_1_position(self, data: Odometry):
        self.robot_1_position['x'] = data.pose.pose.position.x
        self.robot_1_position['y'] = -1.0*data.pose.pose.position.y
        self.robot_1_position['z'] = data.pose.pose.position.z
        self.robot_1_quaternion['x'] = data.pose.pose.orientation.x
        self.robot_1_quaternion['y'] = data.pose.pose.orientation.y
        self.robot_1_quaternion['z'] = data.pose.pose.orientation.z
        self.robot_1_quaternion['w'] = data.pose.pose.orientation.w

        # self.get_logger().info(str(self.robot_1_position['x']))
        # self.get_logger().info(str(self.robot_1_position['y']))

        # ! 機器人偏移角度
        self.robot_1_euler_angles = quaternion_to_euler_angles(
            self.robot_1_quaternion['x'], self.robot_1_quaternion['y'],
            self.robot_1_quaternion['z'], self.robot_1_quaternion['w']
        )
        # self.get_logger().info(
        #     str(self.robot_1_euler_angles['yaw']))  # show angles
        self.robot_1_start_pos = (
            self.robot_1_position['x'], self.robot_1_position['y'])

    def pose_to_pixel_cell(self, position, center_pixel):
        # pose.x / resolution + center_point = cell_x
        return (position / self.map_resolution) + center_pixel

    def pixel_cell_to_pose(self, pixel_cell, center_pixel):
        return (pixel_cell - center_pixel) * self.map_resolution

    def get_another_point(self, point_x, point_y, angle, bevel):
        # print(f"origin angle: {angle}")
        if angle < 0.0:
            angle = 360.0 + angle
        # print(f"fix angle: {angle}")
        # point_x = 0.0
        # point_y = 0.0
        # angle = 138.0
        # bevel = 0.5
        radian = angle * math.pi / 180.0
        Margin = [0.0, 0.0]
        Margin[0] = (math.cos(radian) * bevel) + point_x
        Margin[1] = (math.sin(radian) * bevel) + point_y
        # print(f"radian: {radian}")
        # print(f"cos: {math.cos(radian)}, sin: {math.sin(radian)}")
        # print(f"point_x: {point_x}, point_y: {point_y}")
        # print(f"Margin[0]: {Margin[0]:.3f}, Margin[1]: {Margin[1]:.3f}")
        return Margin

    def get_robot1_bounding_boxes(self, data: BoundingBoxesV2):
        for score in data.bounding_boxes:
            # print("score: ",score)
            # print(f"class_id: {score.class_id}")
            # print(f"probability: {score.probability}")
            # print(f"center_dist: {score.center_dist}")
            if score.class_id in self.yolo_result_robot1["class_id"]:
                list_num = self.yolo_result_robot1["class_id"].index(
                    score.class_id)
                # print("last_distance: ", score.center_dist)
                # print("center_dist: ", score.center_dist)
                if score.center_dist - self.last_distance < 1.0:
                    self.yolo_result_robot1["probability"][list_num] = score.probability
                    self.yolo_result_robot1["center_dist"][list_num] = score.center_dist
                    self.yolo_result_robot1["degree"][list_num] = score.degree
                    # print("degree: ",score.degree)
                    position = self.get_another_point(float(self.robot_1_position['x']), -1.0*float(
                        self.robot_1_position['y']), float(self.robot_1_euler_angles['yaw'] - score.degree), score.center_dist)
                    self.yolo_result_robot1["position"][list_num] = position
                self.last_distance = score.center_dist
            else:
                if True:
                    self.yolo_result_robot1["class_id"].append(score.class_id)
                    self.yolo_result_robot1["probability"].append(
                        score.probability)
                    self.yolo_result_robot1["center_dist"].append(
                        score.center_dist)
                    self.yolo_result_robot1["degree"].append(
                        score.degree)
                    position = self.get_another_point(float(self.robot_1_position['x']), -1.0*float(
                        self.robot_1_position['y']), float(self.robot_1_euler_angles['yaw'] - score.degree), score.center_dist)
                    self.yolo_result_robot1["position"].append(position)
                    list_num = len(self.yolo_result_robot1["class_id"])
                    self.last_distance = score.degree
                    # self.insert_tablewidget_robot1(
                    #     self.yolo_result_robot1["class_id"][list_num - 1], self.yolo_result_robot1["probability"][list_num - 1], self.yolo_result_robot1["position"][list_num - 1], "robot1")
                    # self.insert_tablewidget_robot2(
                    #     self.yolo_result_robot1["class_id"][list_num - 1], self.yolo_result_robot1["probability"][list_num - 1], self.yolo_result_robot1["position"][list_num - 1], "robot1")

    def loadImage(self, img: SendMapImage):
        """ This function will load the user selected image
                and set it to label using the setPhoto function
        """
        # https://blog.csdn.net/sunyoop/article/details/79965673
        if(self.Map_Image is None):
            # self.get_logger().info("loadImage Ing.....")
            self.Map_Image = bridge.imgmsg_to_cv2(img.map_image, "bgr8")
            self.origin_height, self.origin_width, self.origin_channel = self.Map_Image.shape
            self.Init_Map_Image = self.Map_Image
            print("self.origin_height: ", self.origin_height)
            print("self.origin_width: ", self.origin_width)
            print("self.origin_channel: ", self.origin_channel)
            print("origin_position_x: ", img.origin_position_x)
            print("origin_position_y: ", img.origin_position_y)
            self.fix_map_center_pixel_x = -8.8/self.map_resolution
            self.fix_map_center_pixel_y = -5.24/self.map_resolution
            print("self.fix_map_center_pixel_x: ", self.fix_map_center_pixel_x)
            print("self.fix_map_center_pixel_y: ", self.fix_map_center_pixel_y)
            self.map_center_pixel_x = (self.origin_width -
                                       (self.origin_width + self.fix_map_center_pixel_x))
            self.map_center_pixel_y = (
                (self.origin_height + self.fix_map_center_pixel_y))

    def scan_callback(self, msg: LaserScan):
        self.laser_ranges = msg
        for i in range(len(self.laser_ranges.ranges)):
            degree = i * self.laser_ranges.angle_increment * 180 / 3.14159
            distance = self.laser_ranges.ranges[i]
            # if degree > 0.0 and degree < 180.0:
            #     print("Degree: {:0.2f}, Distance: {:0.2f}".format(
            #         degree, distance))

    def pose_to_pixel_map(self):
        start = time.time()
        if self.Init_Map_Image is not None:
            map = self.Init_Map_Image.copy()
            center_x = self.pose_to_pixel_cell(
                float(self.robot_1_position['x']), self.map_center_pixel_x)
            center_y = self.pose_to_pixel_cell(
                float(self.robot_1_position['y']), self.map_center_pixel_y)
            center_point = (int(center_x*1.0), int(center_y*(1.0)))

            another_center = self.get_another_point(float(self.robot_1_position['x']), -1.0*float(
                self.robot_1_position['y']), (float(self.robot_1_euler_angles['yaw'])-180.0), 0.5)
            # print("another_center: ", another_center)
            # print("self.map_center_pixel_x: ", self.map_center_pixel_x)
            # print("self.map_center_pixel_y: ", self.map_center_pixel_y)
            another_center_x = self.pose_to_pixel_cell(
                float(another_center[0]), self.map_center_pixel_x)
            another_center_y = self.pose_to_pixel_cell(
                -1.0*float(another_center[1]), self.map_center_pixel_y)
            another_center_point = (int(another_center_x*1.0),
                                    int(another_center_y*(1.0)))
            # ! 修改箭頭方向
            cv2.arrowedLine(map, another_center_point, center_point,
                            (0, 0, 255), thickness=3, line_type=cv2.LINE_4)
            # ! 印出像素地圖的中心點
            # cv2.drawMarker(map, (int(self.origin_width/2), int(self.origin_height/2)),
            #                (0, 0, 0), markerType=2, markerSize=3)
            # ! 印出地圖機器人初始位置中心點
            # cv2.drawMarker(map, (int(self.map_center_pixel_x), int(self.map_center_pixel_y)),
            #                (0, 100, 100), markerType=2, markerSize=3)

            # ! 繪製雷射掃描的角度與距離
            for i in range(len(self.laser_ranges.ranges)):
                degree = i * self.laser_ranges.angle_increment * 180 / 3.14159
                distance = self.laser_ranges.ranges[i]
                if not math.isinf(distance):
                    another_center = self.get_another_point(float(
                        self.robot_1_position['x']), -1.0*float(self.robot_1_position['y']), (float(degree+self.robot_1_euler_angles['yaw'])), distance)
                    another_center_x = self.pose_to_pixel_cell(
                        float(another_center[0]), self.map_center_pixel_x)
                    another_center_y = self.pose_to_pixel_cell(
                        -1.0*float(another_center[1]), self.map_center_pixel_y)
                    another_center_point = (int(another_center_x*1.0),
                                            int(another_center_y*(1.0)))
                    if (degree > 0.0 and degree < 90.0) or (degree > 270.0 and degree < 360.0):
                        cv2.circle(map, another_center_point,
                                   3, (10, 10, 255), -1)
                    else:
                        cv2.circle(map, another_center_point,
                                   3, (0, 255, 255), -1)

            for pos in self.yolo_result_robot1["position"]:
                # print("pos: ", pos)
                kf.init()
                another_center_x = self.pose_to_pixel_cell(
                    float(pos[0]), self.map_center_pixel_x)
                another_center_y = self.pose_to_pixel_cell(
                    -1.0*float(pos[1]), self.map_center_pixel_y)
                if not math.isinf(another_center_x) and not math.isinf(another_center_y) and not math.isnan(another_center_x) and not math.isnan(another_center_y):
                    another_center_point = (int(another_center_x*1.0),
                                            int(another_center_y*(1.0)))
                    # ! 印出目標物的座標
                    self.__points.append(another_center_point)
                    cv2.drawMarker(map, another_center_point,
                                   (0, 0, 199), markerType=2, markerSize=5)
                    # self.predicted = kf.predict(
                    #     another_center_x, another_center_y)
                    self.coordinates.append(another_center_point)  # 将包含x和y坐标的元组添加到队列的右侧
                    for pt in self.coordinates:
                        self.predicted = kf.predict(pt[0], pt[1])
                        print("pt: ", pt)
                    #!印出預測的下一個座標(圓形)
                    print("another_center_point: ", another_center_point)
                    print("self.predicted: ", self.predicted)
                    cv2.circle(map, self.predicted, 3, (255, 255, 0), 1)
                    predicted = self.predicted
                    #!印出未來5筆預測的座標(圓形)
                    for i in range(5):
                        predicted = kf.predict(predicted[0], predicted[1])
                        cv2.circle(map, predicted, 3, (200, 220, 100), 1)
                        # predicted = kf.only_predict()
                        # cv2.circle(map, predicted, 3, (200, 220, 100), 1)
                        print("predicted: ", predicted)
            #!印出目標物移動軌跡(線段)
            if len(self.__points) > 1:
                #! 只畫出線
                for i in range(len(self.__points) - 1):
                    cv2.line(
                        map, self.__points[i], self.__points[i+1], color=(255, 0, 0), thickness=1)
                #! 只畫出圓圈
                # for i in range(len(self.__points)):
                #     cv2.circle(
                #         map, (self.__points[i]), 3, color=(255, 0, 0), thickness=1)
                # print("self.__points: ", self.__points)

            # cv2.arrowedLine(输入图像，起始点(x,y)，结束点(x,y)，线段颜色，线段厚度，线段样式，位移因数，箭头因数)
            dsize = (int(3.0 * self.origin_width),
                     int(3.0 * self.origin_height))
            dst = cv2.resize(map, dsize)
            cv2.imshow('my_test_img_map', dst)
            key = cv2.waitKey(200)
            if key == 27:
                self.__points.clear()
                kf.init()
            loop_time = int((time.time()-start) * 1000)
            # print("loop time(ms): ", loop_time)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
