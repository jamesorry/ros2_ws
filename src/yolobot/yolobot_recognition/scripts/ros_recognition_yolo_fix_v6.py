#!/usr/bin/env python3
import argparse
from control_pid import PID
from datetime import datetime
import math
import statistics
from utils.torch_utils import select_device, load_classifier, time_synchronized
from utils.plots import colors, plot_one_box
from utils.general import check_img_size, check_requirements, check_imshow, colorstr, non_max_suppression, \
    apply_classifier, scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective
from utils.datasets import LoadStreams, LoadImages
from models.experimental import attempt_load
import os
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

from my_robot_interfaces.msg import BoundingBoxes, BoundingBox
import sensor_msgs_py.point_cloud2 as point_cloud2
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
# 2023/1/24 修改

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())


bridge = CvBridge()


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.Need_Take_Picture = True
        # model.pt path(s) # 事先训练完成的权重文件，比如yolov5s.pt(官方事先訓練好的文件)
        # weights = 'yolov5s.pt'
        weights = 'best.pt'
        # inference size (pixels) # 预测时的放缩后图片大小(因为YOLO算法需要预先放缩图片), 两个值分别是height, width
        self.imgsz = 640
        self.conf_thres = 0.25  # confidence threshold # 置信度阈值, 高于此值的bounding_box才会被保留
        self.iou_thres = 0.45  # NMS IOU threshold # IOU阈值,高于此值的bounding_box才会被保留
        self.max_det = 1000  # maximum detections per image # 一张图片上检测的最大目标数量
        self.classes = None  # filter by class: --class 0, or --class 0 2 3 # 过滤指定类的预测结果
        # class-agnostic NMS # 如为True,则为class-agnostic. 否则为class-specific
        self.agnostic_nms = False
        self.augment = False  # augmented inference
        self.visualize = False  # visualize features
        # bounding box thickness (pixels) 绘制Bounding_box的线宽度
        self.line_thickness = 1
        self.hide_labels = False  # hide labels 隐藏标签
        self.hide_conf = False  # hide confidences 隐藏置信度
        self.half = False  # use FP16 half-precision inference 是否使用半精度推理（节约显存）
        self.stride = 32  # 推理时所用到的步长，默认为32， 大步长适合于大目标，小步长适合于小目标
        device_num = ''  # cuda device, i.e. 0 or 0,1,2,3 or cpu # 所使用的GPU编号，如果使用CPU就写cpu
        view_img = False  # show results
        save_crop = False  # save cropped prediction boxes
        nosave = False  # do not save images/videos
        update = False  # update all models
        name = 'exp'  # save results to project/name

        # Initialize
        set_logging()
        # 獲取設備返回值（cuda）
        self.device = select_device(device_num)
        self.half &= self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        # 加載float32模型，確保輸入圖片分辨率能整除32(如果不能整除則調整成可以整除並返回)
        self.model = attempt_load(
            weights, map_location=self.device)  # load FP32 model
        stride = int(self.model.stride.max())  # model stride 推理时所用到的步长
        imgsz = check_img_size(self.imgsz, s=stride)  # check image size
        # 将图片大小调整为步长的整数倍
        # 比如假如步长是10，imagesz是[100,101],则返回值是[100,100]

        # 獲取類別名字字符串列表
        self.names = self.model.module.names if hasattr(
            self.model, 'module') else self.model.names  # get class names

        # print("self.names: ", self.names)
        # ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

        if self.half:
            self.model.half()  # to FP16

        # Second-stage classifier
        # 設置第二類分辨，默認是不使用
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet50', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('resnet50.pt', map_location=self.device)[
                                        'model']).to(self.device).eval()

        # print("self.classify", self.classify)

        # Dataloader
        # Check if environment supports image displays # 检测cv2.imshow()方法是否可以执行，不能执行则抛出异常
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference # 该设置可以加速预测

        # Run inference
        # 進行一次向前推理，測試程序是否正常
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(
                next(self.model.parameters())))  # run once

        self.publish_bbox = self.create_publisher(
            BoundingBoxes, '/robot1/yolov5/bounding_boxes', 10)

        self.publish_image = self.create_publisher(
            Image, '/robot1/yolov5/image_raw', 10)

        self.subscription = self.create_subscription(
            Image,
            '/robot1/intel_realsense_r200_depth/image_raw',
            self.camera_callback,
            10)
        # self.subscription  # prevent unused variable warning

        # self.save_image_path = "/home/james/Downloads/"
        # ================================================================
        self.class_list = []
        # for point_cloud
        self._num_count = 0
        self._z_finial_disrance = 0.0
        self.x_list = []
        self.y_list = []
        self.z_list = []
        self.x_list_copy = []
        self.y_list_copy = []
        self.z_list_copy = []
        self.subscriber_ = self.create_subscription(
            PointCloud2, "/robot1/intel_realsense_r200_depth/points", self.callback_pointcloud_XY_Plot, 10)
        self.update_camera = False
        self.update_pointcloud = False
        # ================================================================
        self.target_center_pixel = (0, 0)
        self.target_center_distance = 0.0
        # self.target_center_distance = self.z_list_copy[self.imgsz * self.target_center_pixel[1] + self.target_center_pixel[0]]
        P = 1.1
        I = 0.005
        D = 1.2
        self.pid_p = [P, P]
        self.pid_i = [I, I]
        self.pid_d = [D, D]
        self.pid_feedback = [0.0, 0.0]
        self.pid = PID(self.pid_p, self.pid_i, self.pid_d)
        # 目標量[畫面中心點（X）, 物體距離(m)]
        self.pid.SetPoint = [float(self.imgsz/2), 1.0]
        self.pid.setSampleTime(0.01)
        self._publisher_twist_robot1 = self.create_publisher(
            Twist, "/robot1/cmd_vel", 10)
        if not self.Need_Take_Picture:
            self.control_PID_loop_timer_ = self.create_timer(
                0.01, self.control_PID_loop)
        # ================================================================
        """ self.Video_Path = Path().absolute() / 'videos'
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')          # 設定影片的格式為 MJPG
        self.video_count = self.get_files_num(self.Video_Path)
        self.video_nume = str(self.video_count) + '.mp4'
        self.videoWriter = cv2.VideoWriter(str(self.Video_Path / self.video_nume), fourcc, 5.0,
                                           (640,  480))  # 產生空的影片

        self.stop_video_capture = False
        if self.stop_video_capture:
            print("開始錄影")
        else:
            print("停止錄影") """
            
        self.start_time = time.time()
        # FPS update time in seconds
        self.display_time = 2
        self.fc = 0
        self.FPS = 0

        self.Image_Path = Path().absolute() / 'images'
        self.image_count = self.get_files_num(self.Image_Path)
        print("image_count nums: ", self.image_count)
        # ================================================================

    def x_linear_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def z_angular_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def _send_twist_robot1(self, z_angular, x_linear):
        if (self._publisher_twist_robot1 is None):
            return
        twist = Twist()

        if self.pid_p[0] != 0:
            z_angular = self.z_angular_map(
                z_angular, -320.0 * self.pid_p[0], 320.0 * self.pid_p[0], -0.18, 0.18)  # 旋轉
        print("z_angular: ", z_angular)

        if self.pid_p[1] != 0:
            x_linear = self.x_linear_map(
                x_linear, -1.0 * self.pid_p[1], 1.0 * self.pid_p[1], 0.18, -0.18)  # 前進後退
        print("x_linear: ", x_linear)
        print("===============================")
        # 限制速度
        if x_linear > 0.5:
            x_linear = 0.0
        if x_linear < -0.5:
            x_linear = -0.0
        if math.isnan(x_linear) or math.isinf(x_linear):
            x_linear = 0.0
            self.pid.ITerm[1] = 0.0

        if z_angular > 0.2:
            z_angular = 0.0
        if z_angular < -0.2:
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

    def control_PID_loop(self):
        if len(self.class_list):
            if self.z_list_copy:
                self.target_center_distance = self.z_list_copy[self.imgsz *
                                                               self.target_center_pixel[1] + self.target_center_pixel[0]]
            self.pid_feedback = [
                float(self.target_center_pixel[0]), self.target_center_distance]
            self.pid.update(self.pid_feedback)
            output = self.pid.output
            # print("pid_feedback: ", self.pid_feedback)
            print("target_center_pixel: ", self.target_center_pixel)
            print("output: ", output)

            self._send_twist_robot1(output[0], output[1])
        else:
            self.pid.ITerm = [0.0, 0.0]
            if self.target_center_pixel[0] <= 320:
                self.self_rotate(0.08, 0.0)
            elif self.target_center_pixel[0] > 320:
                self.self_rotate(-0.08, 0.0)

    def callback_pointcloud_XY_Plot(self, data):
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(
            data, field_names=("x", "y", "z"), skip_nans=True)
        self.x_list_copy.clear()
        self.y_list_copy.clear()
        self.x_list.clear()
        self.y_list.clear()
        self.z_list.clear()
        for p in gen:
            self.x_list.append(p[0])
            self.y_list.append(p[1])
            self.z_list.append(p[2])
        self.update_pointcloud = True
        self.x_list_copy = self.x_list.copy()
        self.y_list_copy = self.y_list.copy()
        self.z_list_copy = self.z_list.copy()

        # if self.update_camera is True:
        #     self.update_camera = False

    def yolovFive2bboxes_msgs(self, bboxes: list, scores: list, cls: list, img_header: Header):
        bboxes_msg = BoundingBoxes()
        bboxes_msg.header = img_header
        i = 0
        for score in scores:
            one_box = BoundingBox()
            one_box.xmin = int(bboxes[0][i])
            one_box.ymin = int(bboxes[1][i])
            one_box.xmax = int(bboxes[2][i])
            one_box.ymax = int(bboxes[3][i])
            one_box.x_center = abs(
                one_box.xmax - one_box.xmin) / 2 + one_box.xmin
            one_box.y_center = abs(
                one_box.ymax - one_box.ymin) / 2 + one_box.ymin
            one_box.probability = float(score)
            one_box.class_id = cls[i]
            one_box.center_dist = self._z_finial_disrance
            # print(f"xmin: {one_box.xmin}, ymin: {one_box.ymin}")
            # print(f"xmax: {one_box.xmax}, ymax: {one_box.ymax}")
            # print(f"x_center: {one_box.x_center}, y_center: {one_box.y_center}")
            if (one_box.x_center >= 250 and one_box.x_center <= 390):
                bboxes_msg.bounding_boxes.append(one_box)
                i = i+1
                # print(i)
        self.publish_bbox.publish(bboxes_msg)

    def camera_callback(self, data: Image):
        self.class_list = []
        confidence_list = []
        x_min_list = []
        y_min_list = []
        x_max_list = []
        y_max_list = []

        t0 = time.time()

        self.fc = self.fc + 1
        self.TIME = time.time() - self.start_time

        if (self.TIME) >= self.display_time:
            self.FPS = self.fc / (self.TIME)
            self.fc = 0
            self.start_time = time.time()
            fps_disp = "FPS: "+str(self.FPS)[:5]
            # print(fps_disp) #印出目前影像的fps

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        # img = bridge.imgmsg_to_cv2(data, "32FC1")

        # check for common shapes
        s = np.stack([letterbox(x, self.imgsz, stride=self.stride)[
                     0].shape for x in img], 0)  # shapes
        # rect inference if all shapes equal
        self.rect = np.unique(s, axis=0).shape[0] == 1
        if not self.rect:
            print('WARNING: Different stream shapes detected. For optimal performance supply similarly-shaped streams.')

        # Letterbox
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]

        # Stack
        img = np.stack(img, 0)

        # Convert
        # BGR to RGB, BHWC to BCHW
        img = img[..., ::-1].transpose((0, 3, 1, 2))
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)  # 将图片放到指定设备(如GPU)上识别
        # uint8 to fp16/32 # 把输入从整型转化为半精度/全精度浮点数。
        img = img.half() if self.half else img.float()
        img /= 255.0  # 0 - 255 to 0.0 - 1.0 #将图片归一化处理（这是图像表示方法的的规范，使用浮点数就要归一化）
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        save_dir = '/home/james/ros2_ws/src/yolobot/yolobot_recognition/scripts'
        # Inference
        t1 = time_synchronized()
        # visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
        # 如果为True则保留推理过程中的特征图，保存在runs文件夹中
        pred = self.model(img,
                          augment=self.augment,
                          visualize=increment_path(save_dir / 'features', mkdir=True) if self.visualize else False)[0]
        # 推理结果，pred保存的是所有的bound_box的信息，
        """ 
        pred[..., 0:4]为预测框坐标
        预测框坐标为xywh(中心点+宽长)格式
        pred[..., 4]为objectness置信度
        pred[..., 5:-1]为分类结果 
        """
        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres,
                                   self.classes, self.agnostic_nms, max_det=self.max_det)
        # 执行非极大值抑制，返回值为过滤后的预测框
        # conf_thres： 置信度阈值
        # iou_thres： iou阈值
        # classes: 需要过滤的类（数字列表）保留特定的类别
        # agnostic_nms： 标记class-agnostic或者使用class-specific方式。默认为class-agnostic 进行nms是否也去除不同类别之间的框
        # max_det: 检测框结果的最大数量
        """
        经过nms之后,预测框格式:xywh-->xyxy(左上角右下角)
        pred是一个列表list[torch.tensor],长度为batch_size
        每一个torch.tensor的shape为(num_boxes, 6),内容为box+conf+cls
        """
        t2 = time_synchronized()

        # Apply Classifier 預設不處理
        if self.classify:
            pred = apply_classifier(pred, self.modelc, img, img0)

        # Process detections
        # 對每張圖進行處理
        for i, det in enumerate(pred):  # detections per image
            # 設置印出訊息（圖片寬高）# 显示推理前裁剪后的图像尺寸
            s = f'{i}: '
            s += '%gx%g ' % img.shape[2:]  # print string

            if len(det):
                # Rescale boxes from img_size to im0 size
                # 調整預測框座標，基於resize+pad的圖片座標-->基於源size圖片的座標
                # 此時座標格式為xyxy
                # 将标注的bounding_box大小调整为和原图一致（因为训练时原图经过了放缩）
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], img0.shape).round()

                # Print results
                # 印出檢測的類別訊息
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    # add to string
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "
                if not self.Need_Take_Picture:
                    # 保存預測結果
                    for *xyxy, conf, cls in reversed(det):
                        if float(conf) >= 0.75:
                            c = int(cls)  # integer class
                            label = None if self.hide_labels else (
                                self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                            plot_one_box(xyxy, img0, label=label, color=colors(
                                c, True), line_thickness=self.line_thickness)

                            self.class_list.append(self.names[c])
                            confidence_list.append(conf)
                            # tensor to float
                            x_min_list.append(xyxy[0].item())
                            y_min_list.append(xyxy[1].item())
                            x_max_list.append(xyxy[2].item())
                            y_max_list.append(xyxy[3].item())
                            xmin = (xyxy[0].item())
                            ymin = (xyxy[1].item())
                            xmax = (xyxy[2].item())
                            ymax = (xyxy[3].item())
                            x_center = int(abs((xmax-xmin)/2) + xmin)
                            y_center = int(abs((ymax-ymin)/2) + ymin)
                            self.target_center_pixel = (x_center, y_center)
                            # print("x_center: ", x_center, "; y_center: ", y_center)
                            # print("integer class: ", c, "; lable: ", label)
                            cv2.circle(img0, (x_center, y_center),
                                       1, (0, 0, 255), 3)
                            pointcloud_num = self.imgsz * y_center + x_center
                            if self.update_pointcloud is True:
                                cv2.putText(img0, "(depth: {:.3f})".format(self.z_list_copy[pointcloud_num]), (int(
                                    x_center - 40), int(y_center + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        self.update_camera = True
        cv2.imshow("IMAGE", img0)
        if self.update_pointcloud is True:
            self.update_pointcloud = False
        if len(self.class_list) and not self.Need_Take_Picture:
            renamed_list = self.rename_duplicate(self.class_list)
            self.yolovFive2bboxes_msgs(bboxes=[x_min_list, y_min_list, x_max_list, y_max_list],
                                       scores=confidence_list, cls=renamed_list, img_header=data.header)
            self.publish_image.publish(bridge.cv2_to_imgmsg(img0, "bgr8"))
        """ if self.stop_video_capture:
            self.videoWriter.write(img0)       # 將取得的每一幀圖像寫入空的影片 """

        key = cv2.waitKey(1)
        if key == ord('s'):
            self.image_count = self.image_count + 1
            Img_File_Name = str(self.image_count) + '.jpg'
            cv2.imwrite(str(self.Image_Path / Img_File_Name), img0)
            print("SAVE PIC: ", str(self.Image_Path / Img_File_Name))
            
            """ self.videoWriter.release()
            print("結束錄影")
            print("SAVE VIDEO: ", self.Video_Path / self.video_nume)
        elif key == ord('s'):
            if self.stop_video_capture:
                print("停止錄影")
            else:
                print("開始錄影")
            self.stop_video_capture = not self.stop_video_capture """
        # https://cloud.tencent.com/developer/article/1456305 (PID控制算法)

    def get_files_num(self, dir_path):
        count = 0
        for path in Path(dir_path).iterdir():
            if path.is_file():
                count += 1
        return count

    def rename_duplicate(self, list):
        counts = {}
        for index, key in enumerate(list):
            if key in counts:
                counts[key] += 1
                list[index] = f'{key}_{counts[key]}'
            else:
                counts[key] = 0
        # print("Renamed list:", list)
        return list


if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
