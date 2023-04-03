#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import threading
import math
from control_pid import PID
from rclpy.action import ActionServer
from my_robot_interfaces.action import ObjectTrack
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from my_robot_interfaces.msg import BoundingBoxes, BoundingBox, BoundingBoxesV2, CurvePIDarray, CurvePID
import sensor_msgs_py.point_cloud2 as point_cloud2
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
import argparse
from enum import IntEnum
import matplotlib.pyplot as plt
from collections import deque
from builtin_interfaces.msg import Time

ENABLE = True
DISABLE = False


class PID_Control_Status(IntEnum):
    MISSING = 0
    TRACKING = 1


class Robot_Linear_Status(IntEnum):
    Stop = 0
    Forward = 1
    Backward = 2


class Robot_Angular_Status(IntEnum):
    Stop = 0
    Right = 1
    Left = 2


class TrackActionServer(Node):
    def __init__(self):
        super().__init__('track_action_server')
        parser = argparse.ArgumentParser(description='manual to this script')
        parser.add_argument('-n', '--robot_name', type=str, default='robot1',
                            help='Name of the robot to spawn')
        parser.add_argument('-pid', '--PID_Control', type=bool, default=False,
                            help='是否啟用PID控制移動')
        parser.add_argument('-kp', '--PID_Kp', type=float, default=1.1,
                            help='Kp係數')
        parser.add_argument('-ki', '--PID_Ki', type=float, default=0.005,
                            help='Ki係數')
        parser.add_argument('-kd', '--PID_Kd', type=float, default=1.2,
                            help='Kd係數')
        self.args, self.unknown = parser.parse_known_args()
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        __action_server_name = "/" + self.args.robot_name + "/object_track"
        self._action_server = ActionServer(
            self,
            ObjectTrack,
            __action_server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        self.init_pid()
        self.target_object_name = ""
        self.class_id_list = ["", "", "", "", "", "", "", "", ""]
        __sub_bounding_boxes_name = "/" + self.args.robot_name + "/yolov5/bounding_boxes"
        self.sub_bounding_boxes = self.create_subscription(
            BoundingBoxesV2,
            __sub_bounding_boxes_name,
            self.get_robot_yolo_data,
            20
        )
        self.robot_linear_status = Robot_Linear_Status.Stop
        self.robot_angular_status = Robot_Angular_Status.Stop
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # 可以用來判斷機器人是否移動
        if self.last_cmd_vel is None:
            self.last_cmd_vel = msg
            return
        if msg.linear.x > 0:
            self.robot_linear_status = Robot_Linear_Status.Forward
            # print("Robot is moving forward")
        elif msg.linear.x < 0:
            self.robot_linear_status = Robot_Linear_Status.Backward
            # print("Robot is moving backward")
        else:
            self.robot_linear_status = Robot_Linear_Status.Stop
            # print("Robot is not moving")

        if msg.angular.z > 0:
            self.robot_angular_status_status = Robot_Angular_Status.Right
            # print("Robot is turning right")
        elif msg.angular.z < 0:
            self.robot_angular_status_status = Robot_Angular_Status.Left
            # print("Robot is turning left")
        else:
            self.robot_angular_status_status = Robot_Angular_Status.Stop
            # print("Robot is not turning")
        self.last_cmd_vel = msg

    def get_robot_yolo_data(self, data: BoundingBoxesV2):
        # ! 獲取運行時間
        # timestamp = data.header.stamp.sec + data.header.stamp.nanosec / 1e9
        # ms = data.header.stamp.nanosec / 1000000  # 獲取毫秒數
        # time_str = time.strftime('%H:%M:%S', time.gmtime(timestamp)) + f".{ms}"
        # print(f"Timestamp: {time_str}")
        self.boundbox_list_num = data.bounding_num
        # print("boundbox_list_num: ", self.boundbox_list_num)
        for i in range(0, self.boundbox_list_num):
            self.class_id_list[i] = (data.bounding_boxes[i].class_id)
            # print(self.class_id_list[i])
        for score in data.bounding_boxes:
            # print("score: ", score)
            # print(f"class_id: {(score.class_id)}")
            # print(f"probability: {score.probability}")
            # print(f"center_dist: {score.center_dist}")
            if str(score.class_id) == self.target_object_name:
                self.target_center_pixel = (score.x_center, score.y_center)
                self.target_center_distance = score.center_dist
                # print(f"target_center_pixel: {self.target_center_pixel}")
                # print(f"target_center_distance: {self.target_center_distance}")

    def init_pid(self):
        self.target_center_pixel = (0, 0)
        self.target_center_distance = 0.0
        self.pid_kp = [1.3, 1.5]
        self.pid_ki = [0.01, 0.01]
        self.pid_kd = [1.5, 1.5]
        # self.pid_kp = [self.args.PID_Kp, self.args.PID_Kp]
        # self.pid_ki = [self.args.PID_Ki, self.args.PID_Ki]
        # self.pid_kd = [self.args.PID_Kd, self.args.PID_Kd]
        self.pid_feedback = [0.0, 0.0]
        self.pid = PID(self.pid_kp, self.pid_ki, self.pid_kd)
        self.pid.last_time = self.get_clock().now()
        self.pid.SetPoint = [float(640/2), 1.2]  # 目標量[畫面中心點（X）, 物體距離(m)]
        self.pid.setSampleTime(0.01)
        __twist_topic_name = "/" + self.args.robot_name + "/cmd_vel"
        print("__twist_topic_name: ", __twist_topic_name)
        self._publisher_twist_robot1 = self.create_publisher(
            Twist,
            __twist_topic_name,
            10
        )
        self.control_PID_loop_timer_ = self.create_timer(
            0.01, self.control_PID_loop)
        self.control_PID_loop_run = DISABLE
        self.boundbox_list_num = 0
        self.PID_Control_Status = PID_Control_Status.MISSING

        __pid_curve_topic_name = "/pid_curve"
        self._publisher_pid_curve = self.create_publisher(
            CurvePIDarray,
            __pid_curve_topic_name,
            20
        )

    def x_linear_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def z_angular_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def _send_twist_robot1(self, z_angular, x_linear):
        if (self._publisher_twist_robot1 is None):
            return
        twist = Twist()

        if self.pid_kp[0] != 0:
            z_angular = self.z_angular_map(
                z_angular, -320.0 * self.pid_kp[0], 320.0 * self.pid_kp[0], -0.23, 0.23)  # 旋轉
        # print("z_angular: ", z_angular)

        if self.pid_kp[1] != 0:
            x_linear = self.x_linear_map(
                x_linear, -1.0 * self.pid_kp[1], 1.0 * self.pid_kp[1], 0.25, -0.35)  # 前進後退
        # print("x_linear: ", x_linear)
        # print("===============================")
        # 限制速度
        if x_linear > 0.9:
            x_linear = 0.0
        if x_linear < -1.5:
            x_linear = -0.0
        if math.isnan(x_linear) or math.isinf(x_linear):
            x_linear = 0.0
            self.pid.ITerm[1] = 0.0

        if z_angular > 0.5:
            z_angular = 0.0
        if z_angular < -0.5:
            z_angular = -0.0
        if math.isnan(z_angular) or math.isinf(z_angular):
            z_angular = 0.0
            self.pid.ITerm[0] = 0.0

        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_angular
        self._publisher_twist_robot1.publish(twist)

    def self_rotate(self, z_angular, x_linear):
        if (self._publisher_twist_robot1 is None):
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = z_angular
        self._publisher_twist_robot1.publish(twist)

    def init_twist_robot1(self):
        if (self._publisher_twist_robot1 is None):
            return
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self._publisher_twist_robot1.publish(twist)

    def publish_PID_Curve(self, process_variable, output, current_time):
        msg_array = CurvePIDarray()
        for i in range(2):
            msg = CurvePID()
            msg.time_stamp = current_time
            msg.line_feedback_value = process_variable[i]
            msg.line_output = output[i]
            msg_array.curvepid.append(msg)
        self._publisher_pid_curve.publish(msg_array)

    def control_PID_loop(self):
        if self.control_PID_loop_run is ENABLE:
            if self.boundbox_list_num > 0:
                now_time = self.get_clock().now()
                self.pid_feedback = [
                    float(self.target_center_pixel[0]), self.target_center_distance]
                self.pid.update(self.pid_feedback, now_time)
                output = self.pid.output
                self.publish_PID_Curve(
                    self.pid_feedback, output, now_time.to_msg())
                # print("pid_feedback: ", self.pid_feedback)
                # print("target_center_pixel: ", self.target_center_pixel)
                # print("output: ", output)
                self._send_twist_robot1(output[0], output[1])
                self.PID_Control_Status = PID_Control_Status.TRACKING
            else:
                self.pid.ITerm = [0.0, 0.0]
                if self.PID_Control_Status == PID_Control_Status.TRACKING:
                    self.init_twist_robot1()  # ! 不能一直卡在這裡，會無法控制車子移動(理論上應該只需要觸發一次)
                self.PID_Control_Status = PID_Control_Status.MISSING
                print("Target Missing!!!")
                # if self.target_center_pixel[0] <= 320:
                #     self.self_rotate(0.08, 0.0)
                # elif self.target_center_pixel[0] > 320:
                #     self.self_rotate(-0.08, 0.0)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """ 第1步 """
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        self.pid.ITerm = [0.0, 0.0]
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        self.control_PID_loop_run = DISABLE
        # cancel時機器人需停止移動
        self.pid.ITerm = [0.0, 0.0]
        self.init_twist_robot1()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """ 第2步 """
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        feedback_msg = ObjectTrack.Feedback()
        feedback_msg.status = self.PID_Control_Status
        feedback_msg.elapsed_time = 0.0

        # self.get_logger().info('goal: {0}'.format(
        #     goal_handle.request.object_name))
        self.target_object_name = str(goal_handle.request.object_name)
        self.get_logger().info('goal: {0}'.format(
            self.target_object_name))
        # Start executing the action
        self.pid.ITerm = [0.0, 0.0]
        t1 = time.time()
        while (goal_handle.request.object_name in self.class_id_list):
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return ObjectTrack.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                self.control_PID_loop_run = DISABLE
                return ObjectTrack.Result()
            # Update ObjectTrack Status
            self.control_PID_loop_run = ENABLE
            feedback_msg.status = self.PID_Control_Status
            feedback_msg.elapsed_time = round(time.time()-t1, 2)
            # Publish the feedback
            if self.control_PID_loop_run:
                # 約50ms發送一筆publish_feedback(需要確認發送速度是否有辦法達到此50ms)
                if time.time()-feedback_msg.elapsed_time > 0.1:
                    goal_handle.publish_feedback(feedback_msg)
            # time.sleep(1)

        goal_handle.succeed()

        # Populate result message
        result = ObjectTrack.Result()
        result.success = False

        self.get_logger().info(
            'Returning result(success): {0}'.format(result.success))

        return result

#! cancel 機器人不會停下來......(已修改)


def main(args=None):
    rclpy.init(args=args)

    action_server = TrackActionServer()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    # https://blog.csdn.net/liuerin/article/details/108749596
    # 多線程調用callback funcs ， cancel action
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
