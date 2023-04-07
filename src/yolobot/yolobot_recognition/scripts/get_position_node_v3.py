#!/usr/bin/env python3
from enum import Enum, auto
from action_msgs.msg import GoalStatus
from my_robot_interfaces.action import ObjectTrack
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from collections import deque
import time
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


class FollowStatusMSG(Enum):
    NONE = ""
    CANCEL = 'Cancel'
    CANCEL_ERROR = 'Cancel Error'
    FOLLOWING = 'Following'
    MISSING = 'Missing'


class NavigationStatusMSG(Enum):
    NONE = ""
    CANCEL = 'Cancel'
    CANCEL_ERROR = 'Cancel Error'
    SUCCESS = 'Success'


class StateMachineStatus(Enum):
    Idle = auto()
    Target_Following = auto()
    Target_Missing = auto()
    Target_Missing_Proc = auto()
    Target_Missing_Nav = auto()
    Target_Missing_Nav_Check = auto()
    Target_Pos_Predict_Retry = auto()
    CANCEL_ERROR = auto()
    FOLLOWING = auto()
    MISSING = auto()


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
        self.create_timer(0.1, self.timer_callback)
        self.now_distance = 0.0
        self.last_distance = 0.0
        self.__points = deque(maxlen=200)

        self.laser_ranges = None
        self.laser_subscription = self.create_subscription(
            LaserScan, '/robot1/scan', self.scan_callback, 10)
        self.laser_subscription

        self.coordinates = deque(maxlen=20)  # 创建一个队列，并指定最大长度

        # ! 添加ActionClient
        self._action_client_follow = ActionClient(
            self, ObjectTrack, "/robot1/object_track")
        self.follow_target_name = "walker"
        self.follow_status_msg = FollowStatusMSG.NONE
        self.follow_elapsed_time = 0.0

        # ! 添加nav2
        self._action_client_robot1 = ActionClient(
            self, NavigateToPose, "/robot1/navigate_to_pose")
        self.future_predicted_maxlen = 10
        self.future_predicted = deque(
            maxlen=self.future_predicted_maxlen)  # 创建一个队列，并指定最大长度
        # 狀態機狀態初始化
        self.last_state_machine_status = None
        self.state_machine_status = StateMachineStatus.Idle
        self.NavStatusMsg = NavigationStatusMSG.NONE

        self.new_data_received = False
        self.predicted_start_time = time.time()
    # ! =========================================================================
    # *nav2 controller

    def goal_response_robot1_callback(self, future):
        self.robot1_goal_handle = future.result()
        if self.robot1_goal_handle.accepted == False:
            print("Goal Rejected")
            return None
        # //self.ui.label_target_status_robot1.setText("Goal Accepted")
        self._get_result_future = self.robot1_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_robot1_callback)

    def get_result_robot1_callback(self, future):
        result = future.result().result
        status = future.result().status
        print('robot1 result: ', result)
        print('robot1 status: ', status)

        # // x = self.__pixel_cell_to_pose(
        # //     self.mouse_target['robot1'][0], self.map_center_pixel_x)
        # // y = self.__pixel_cell_to_pose(
        # //     self.mouse_target['robot1'][1], self.map_center_pixel_y)
        # // theta = float(self.ui.lineEdit_robot1_target_theta.text())

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation robot1 succeeded! ')
            self.NavStatusMsg = NavigationStatusMSG.SUCCESS
            # //self.ui.label_target_status_robot1.setText("Goal Finish")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation robot1 canceled! ')
            # //self.ui.label_target_status_robot1.setText("Goal Canceled")
        else:
            self.get_logger().info(
                'Navigation robot1 failed with status: {0}'.format(status))

        # 設定誤差值，在以下誤差值以內則顯示Goal Finish
        # // if (self.robot_1_position['x']-0.5) <= x and (self.robot_1_position['x']+0.5) >= x:
        # //     if (self.robot_1_position['y']-0.5) <= y and (self.robot_1_position['y']+0.5) >= y:
        # //         if (self.robot_1_euler_angles['yaw']-4) <= theta and (self.robot_1_euler_angles['yaw']+4) >= theta:
        # //             self.ui.label_target_status_robot1.setText("Goal Finish")

    def cancel_robot1_goal(self):
        self.get_logger().info('robot1 canceling goal')
        future = self.robot1_goal_handle.cancel_goal_async()
        future.add_done_callback(self.goal_canceled_robot1_callback)

    def goal_canceled_robot1_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancelling of robot1 goal complete')
        else:
            self.get_logger().warning('Goal robot1 failed to cancel')

    def feedback_robot1_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def publish_robot_position_target(self, x, y, theta):
        # * 需要給xy座標以及選轉角度
        theta = theta - 0
        if theta > 360:
            theta = theta - 360
        elif theta < 0:
            theta = theta + 360
        print("=================================")
        print("x: ", float(x))
        print("y: ", float(y))
        print("theta: ", theta)
        print("=================================")
        result = self.handle_action_rpy_to_quaternion(
            float(x), float(y), theta)
        self._action_client_robot1.wait_for_server()
        self.NavStatusMsg = NavigationStatusMSG.NONE
        self._send_goal_future_robot1 = self._action_client_robot1.send_goal_async(
            result, feedback_callback=self.feedback_robot1_callback)
        self._send_goal_future_robot1.add_done_callback(
            self.goal_response_robot1_callback)

    def handle_action_rpy_to_quaternion(self, x, y, theta):
        t = NavigateToPose.Goal()
        t.pose.header.stamp = self.get_clock().now().to_msg()

        t.pose.pose.position.x = x
        t.pose.pose.position.y = -1.0*y
        t.pose.pose.position.z = 0.0
        # theta to radin
        radin = theta * math.pi / 180.0
        q = quaternion_from_euler(0, 0, radin)
        t.pose.pose.orientation.x = q[0]
        t.pose.pose.orientation.y = q[1]
        t.pose.pose.orientation.z = q[2]
        t.pose.pose.orientation.w = q[3]
        return t

    # ! =========================================================================
    # *follow controller
    def follow_controller_update(self, target_name: String):
        # ! 更新跟隨目標名稱
       # // choose_index = self.ui.tableWidget_robot1.currentRow()
        # //print("choose_index: ", choose_index)
        # //self.follow_target_name = self.ui.tableWidget_robot1.item(
        # //    choose_index, 0).text()
        self.follow_target_name = target_name
        print("target_name: ", self.follow_target_name)
        # //self.ui.label_follow_object_target.setText(self.follow_target_name)
        # //self.ui.pushButton_follow_start.setEnabled(True)

    def follow_controller_cancel(self):
        # ! 取消跟隨目標
        self.get_logger().info('Canceling follow goal')
        self.follow_status_msg = FollowStatusMSG.CANCEL
        self.follow_trigger = False
        # //self.ui.pushButton_follow_cancel.setEnabled(False)
        # //self.ui.pushButton_follow_start.setEnabled(False)
        # Cancel the goal
        try:
            future = self._follow_goal_handle.cancel_goal_async()
            future.add_done_callback(self.follow_cancel_done)
        except:
            pass

    def follow_cancel_done(self, future):
        # ! 取消完成
        cancel_response = future.result()
        print("cancel_response: ", cancel_response)
        if len(cancel_response.goals_canceling) > 0:
            self.follow_status_msg = FollowStatusMSG.CANCEL
            self.get_logger().info('Follow Goal successfully canceled')
            # self.ui.label_follow_status.setText("Cancel")
        else:
            self.follow_status_msg = FollowStatusMSG.CANCEL_ERROR
            self.get_logger().info('Follow Goal failed to cancel')
            # self.ui.label_follow_status.setText("Cancel error")

    def follow_controller_start_follow(self):
        # ! 觸發開始跟隨目標
        self.get_logger().info('Waiting for action server...')
        self._action_client_follow.wait_for_server()

        goal_msg = ObjectTrack.Goal()
        goal_msg.object_name = self.follow_target_name  # 此處填入須要跟隨的目標名稱（需為YOLO辨識結果中的）
        self.get_logger().info(
            'follow target name: {0}'.format(goal_msg.object_name))

        self.get_logger().info('Sending follow goal request...')
        self.follow_trigger = True
        self._follow_send_goal_future = self._action_client_follow.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_feedback_callback)

        self._follow_send_goal_future.add_done_callback(
            self.follow_goal_response_callback)

    def follow_feedback_callback(self, feedback_msg):
        # ! 獲取當前跟隨狀態
        feedback = feedback_msg.feedback
        if feedback.status == 1 and self.follow_trigger:
            self.follow_status_msg = FollowStatusMSG.FOLLOWING
        if feedback.status == 0 and self.follow_trigger:
            self.follow_status_msg = FollowStatusMSG.MISSING
        self.follow_elapsed_time = feedback.elapsed_time
        # self.get_logger().info(
        #     'Received feedback(elapsed_time): {0}'.format(feedback.elapsed_time))
        # self.get_logger().info(
        #     'Received feedback(status): {0}'.format(feedback.status))

    def follow_goal_response_callback(self, future):
        # ! server接收"開始"完成
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Follow Goal rejected :(')
            return
        self._follow_goal_handle = goal_handle
        # //self.ui.pushButton_follow_cancel.setEnabled(True)
        self.get_logger().info('Follow Goal accepted :)')
        self._follow_get_result_future = self._follow_goal_handle.get_result_async()
        self._follow_get_result_future.add_done_callback(
            self.follow_get_result_callback)

    def follow_get_result_callback(self, future):
        # ! server接收完成狀態
        result = future.result().result
        status = future.result().status
        print('follow result: ', result)
        print('follow status: ', status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Follow succeeded! ')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Follow canceled! ')
            self.follow_status_msg = FollowStatusMSG.CANCEL
            # //self.ui.label_follow_status.setText("Cancel")
        else:
            self.get_logger().info(
                'Follow failed with status: {0}'.format(status))
    # ! =========================================================================

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
        return ((position) / self.map_resolution) + center_pixel

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
        # ! 獲取運行時間
        # timestamp = data.header.stamp.sec + data.header.stamp.nanosec / 1e9
        # ms = data.header.stamp.nanosec / 1000000  # 獲取毫秒數
        # time_str = time.strftime('%H:%M:%S', time.gmtime(timestamp)) + f".{ms}"
        # print(f"Timestamp: {time_str}")

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
                self.new_data_received = True
            else:
                if True:
                    print("get into True================")
                    # 判斷到兩個不同的id，會出現兩個座標，所以會導致路線亂掉
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
            self.Map_Image = bridge.imgmsg_to_cv2(
                img.map_image, desired_encoding="bgr8")
            kf.ObstacleDetector(self.Map_Image)
            self.origin_height, self.origin_width, self.origin_channel = self.Map_Image.shape

            # 轉換圖像為灰階
            gray = cv2.cvtColor(self.Map_Image, cv2.COLOR_BGR2GRAY)

            # 二值化圖像
            ret, thresh = cv2.threshold(gray, 5, 255, 0)

            # 找到圖像中的輪廓
            contours, hierarchy = cv2.findContours(
                thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # 繪製所有輪廓
            cv2.drawContours(self.Map_Image, contours, -1, (0, 255, 0), 1)

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
            print("self.map_center_pixel_x: ", self.map_center_pixel_x)
            print("self.map_center_pixel_y: ", self.map_center_pixel_y)

    def scan_callback(self, msg: LaserScan):
        self.laser_ranges = msg
        ranges = msg.ranges
        front_distance = min(ranges[len(ranges)//2-10: len(ranges)//2+10])
        if front_distance < 0.5:
            # 機器人前方有障礙物，需要進行避障
            pass
        for i in range(len(self.laser_ranges.ranges)):
            degree = i * self.laser_ranges.angle_increment * 180 / 3.14159
            distance = self.laser_ranges.ranges[i]
            # if degree > 0.0 and degree < 180.0:
            #     print("Degree: {:0.2f}, Distance: {:0.2f}".format(
            #         degree, distance))

    def pose_to_pixel_map(self):
        self.start_loop_time = time.time()
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
            """ try:
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
            except:
                pass """

            for i, pos in enumerate(self.yolo_result_robot1["position"]):
                # print("pos: ", pos)
                if (self.yolo_result_robot1["class_id"][i] == self.follow_target_name):
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
                        if time.time() - self.predicted_start_time > 0.2:  # 壓縮預測速度(這樣看起來可能會比較好一點)
                            # 将包含x和y坐标的元组添加到队列的右侧
                            self.coordinates.append(
                                another_center_point)  # 共會儲存20組座標
                            for pt in self.coordinates:
                                self.predicted = kf.predict(pt[0], pt[1])
                                # //self.predicted = kf.predict_avoid_obstacle(pt[0], pt[1])

                            #!印出預測的下一個座標(圓形)
                            # print("another_center_point: ", another_center_point)
                            # print("self.predicted: ", self.predicted)
                            cv2.circle(map, self.predicted,
                                       3, (255, 255, 0), -1)
                            predicted = self.predicted
                            #!印出未來5筆預測的座標(圓形)
                            for i in range(self.future_predicted_maxlen):
                                predicted = kf.predict_avoid_obstacle(
                                    predicted[0], predicted[1])
                                self.future_predicted.append(predicted)
                                # //predicted = kf.predict(predicted[0], predicted[1])
                                cv2.circle(map, predicted, 3,
                                           (200, 220, 100), 1)
                                # print("predicted: ", predicted)
                            self.predicted_final_pos, self.predicted_final_degree = self.two_pos_get_degree(
                                self.future_predicted[0], self.future_predicted[self.future_predicted_maxlen-1])
                            # print("future_predicted: ",
                            #       self.future_predicted)
                            # print("predicted_final_pos: ",
                            #       self.predicted_final_pos)
                            # print("predicted_final_degree: ",
                            #       self.predicted_final_degree)
                            self.predicted_start_time = time.time()
                        else:
                            cv2.circle(map, self.predicted,
                                       3, (255, 255, 0), -1)
                            for predicted in self.future_predicted:
                                cv2.circle(map, predicted, 3,
                                           (200, 220, 100), 1)

            #!印出目標物移動軌跡(線段)
            if len(self.__points) > 1:
                pass
                #! 只畫出線
                # for i in range(len(self.__points) - 1):
                #     cv2.line(
                #         map, self.__points[i], self.__points[i+1], color=(255, 0, 0), thickness=1)
                #! 只畫出圓圈
                # for i in range(len(self.__points)):
                #     cv2.circle(
                #         map, (self.__points[i]), 3, color=(255, 0, 0), thickness=1)
                # print("self.__points: ", self.__points)

            # cv2.arrowedLine(输入图像，起始点(x,y)，结束点(x,y)，线段颜色，线段厚度，线段样式，位移因数，箭头因数)
            dsize = (int(2.0 * self.origin_width),
                     int(2.0 * self.origin_height))
            dst = cv2.resize(map, dsize)
            cv2.imshow('my_test_img_map', dst)
            key = cv2.waitKey(1)
            if key == 27:
                self.__points.clear()
                kf.init()
                self.print_debug_msg()
            elif key == ord('s'):
                self.follow_controller_update("walker")
                self.follow_controller_start_follow()
            elif key == ord('q'):
                self.follow_controller_cancel()
                self.state_machine_status = StateMachineStatus.Idle
            elif key == ord('z'):
                self.publish_robot_position_target(
                    self.predicted_final_pos[0], self.predicted_final_pos[1], self.predicted_final_degree)
            elif key == ord('c'):
                self.cancel_robot1_goal()
            # loop_time = int((time.time()-self.start_loop_time) * 1000)
            # print("loop time(ms): ", loop_time)
            self.state_machine_proc()
            self.new_data_received = False

    def print_debug_msg(self):
        print(self.yolo_result_robot1["position"])
        print("follow_status_msg:", self.follow_status_msg)
        print("follow_elapsed_time:", self.follow_elapsed_time)

    def state_machine_proc(self):
        if self.last_state_machine_status != self.state_machine_status:
            self.last_state_machine_status = self.state_machine_status
            print("state_machine_status: ", self.state_machine_status)

        if self.state_machine_status == StateMachineStatus.Idle:
            pass
            # if self.follow_status_msg == FollowStatusMSG.FOLLOWING:
            #     self.state_machine_status = StateMachineStatus.Target_Following

        elif self.state_machine_status == StateMachineStatus.Target_Following:
            # 目標正在跟隨中，但突然接收到Missing
            if self.follow_status_msg == FollowStatusMSG.MISSING:
                self.state_machine_status = StateMachineStatus.Target_Missing
                self.start_missing_time = self.follow_elapsed_time

        elif self.state_machine_status == StateMachineStatus.Target_Missing:
            # 目標已跟丟一定時間
            if self.follow_elapsed_time - self.start_missing_time > 2.5:
                print("target missing enough==================")
                self.saved_future_predicted = self.future_predicted
                self.state_machine_status = StateMachineStatus.Target_Missing_Proc
            # 跟丟後又找回來目標
            if self.follow_status_msg == FollowStatusMSG.FOLLOWING:
                self.state_machine_status = StateMachineStatus.Target_Following

        elif self.state_machine_status == StateMachineStatus.Target_Missing_Proc:
            # 取消跟隨
            self.follow_controller_cancel()
            # 開始移動至預測位置
            kf.init()
            for i in range(15):  # 使用之前的15組餵給KF
                predicted = kf.predict(
                    self.coordinates[i][0], self.coordinates[i][1])
            for i in range(self.future_predicted_maxlen):
                predicted = kf.predict_avoid_obstacle(
                    predicted[0], predicted[1])
                self.future_predicted.append(predicted)
            self.predicted_final_pos, self.predicted_final_degree = self.two_pos_get_degree(
                self.future_predicted[0], self.future_predicted[self.future_predicted_maxlen-1])

            self.publish_robot_position_target(
                self.predicted_final_pos[0], self.predicted_final_pos[1], self.predicted_final_degree)
            self.state_machine_status = StateMachineStatus.Target_Missing_Nav

        elif self.state_machine_status == StateMachineStatus.Target_Missing_Nav:
            # 如果在移動到預測位置的途中，又重新找到目標，則又繼續啟動跟隨
            if self.new_data_received == True:
                # self.follow_controller_cancel()
                self.follow_controller_update("walker")
                self.follow_controller_start_follow()  # 啟動跟隨
                self.cancel_robot1_goal()
                self.Target_Missing_Nav_time = time.time()
                self.state_machine_status = StateMachineStatus.Target_Missing_Nav_Check

            if self.NavStatusMsg == NavigationStatusMSG.SUCCESS:
                # 已經到nav定位，但還沒啟動跟隨
                self.state_machine_status = StateMachineStatus.Target_Pos_Predict_Retry

        elif self.state_machine_status == StateMachineStatus.Target_Missing_Nav_Check:
            # 已經啟動跟隨(取消nav)，但沒有在時間內找到目標
            print("---not find target---")
            if time.time() - self.Target_Missing_Nav_time > 2.0:
                print("---out time not find target---")
                if self.follow_status_msg == FollowStatusMSG.MISSING:
                    print("----------Start follow but not find target-----")
                    # 先用先前預測未來的座標，餵給kf預測然後再用predict_avoid_obstacle在預測未來的未來座標
                    # kf.init()
                    # for pt in self.saved_future_predicted:
                    #     predicted = kf.predict(pt[0], pt[1])
                    # for i in range(self.future_predicted_maxlen):
                    #     predicted = kf.predict_avoid_obstacle(
                    #         predicted[0], predicted[1])
                    #     self.future_predicted.append(predicted)
                    self.state_machine_status = StateMachineStatus.Target_Missing_Proc
                    self.start_missing_time = self.follow_elapsed_time
                self.Target_Missing_Nav_time = time.time()

            elif self.follow_status_msg == FollowStatusMSG.FOLLOWING:
                print("----------Start follow and find target-----")
                self.state_machine_status = StateMachineStatus.Idle

        elif self.state_machine_status == StateMachineStatus.Target_Pos_Predict_Retry:
            # 已經到nav定位，但還沒有找到目標，則需要再用未來預測的座標，再次進行預測
            print("----------Trough Nav but not find target-----")
            # 先用先前預測未來的座標，餵給kf預測然後再用predict_avoid_obstacle在預測未來的未來座標
            # kf.init()
            # for pt in self.saved_future_predicted:
            #     predicted = kf.predict(pt[0], pt[1])
            # for i in range(self.future_predicted_maxlen):
            #     predicted = kf.predict_avoid_obstacle(
            #         predicted[0], predicted[1])
            #     self.future_predicted.append(predicted)
            self.state_machine_status = StateMachineStatus.Target_Missing_Proc
            self.start_missing_time = self.follow_elapsed_time

    def two_pos_get_degree(self, pos1=(0.0, 0.0), pos2=(0.0, 0.0)):
        # * 二維座標求角度
        # 兩個座標
        x1, y1 = pos1[0], pos1[1]
        x2, y2 = pos2[0], pos2[1]

        x1 = self.pixel_cell_to_pose(x1, self.map_center_pixel_x)
        y1 = self.pixel_cell_to_pose(y1, self.map_center_pixel_y)

        x2 = self.pixel_cell_to_pose(x2, self.map_center_pixel_x)
        y2 = self.pixel_cell_to_pose(y2, self.map_center_pixel_y)

        # 計算位置向量的分量
        dx = x2 - x1
        dy = y2 - y1

        '''
        当点(x, y) 落入第一象限时，atan2(y, x)的范围是 0 ~ pi/2;
        当点(x, y) 落入第二象限时，atan2(y, x)的范围是 pi/2 ~ pi;
        当点(x, y) 落入第三象限时，atan2(y, x)的范围是 －pi～－pi/2;
        当点(x, y) 落入第四象限时，atan2(y, x)的范围是 -pi/2～0;
        '''
        # 計算角度(弧度)
        angle = math.atan2(dy, dx)
        # print("angle: ", angle)
        # 轉換度數表示
        # ! 因為是從pixel 轉換回來的 所以需要從乘上 (-1) 角度才會正確
        degree = (angle * 180 / math.pi) * (-1)
        # print("degree: ", degree)
        return (x2, y2), degree


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
