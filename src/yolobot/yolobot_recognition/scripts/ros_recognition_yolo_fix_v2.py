#!/usr/bin/env python3
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
import matplotlib.pyplot as plt
import statistics
import math

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())


bridge = CvBridge()


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        weights = 'yolov5s.pt'  # model.pt path(s)
        self.imgsz = 640  # inference size (pixels)
        self.conf_thres = 0.25  # confidence threshold
        self.iou_thres = 0.45  # NMS IOU threshold
        self.max_det = 1000  # maximum detections per image
        self.classes = None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms = False  # class-agnostic NMS
        self.augment = False  # augmented inference
        self.visualize = False  # visualize features
        self.line_thickness = 1  # bounding box thickness (pixels)
        self.hide_labels = False  # hide labels
        self.hide_conf = False  # hide confidences
        self.half = False  # use FP16 half-precision inference
        self.stride = 32
        device_num = ''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
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
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.imgsz, s=stride)  # check image size
        # 獲取類別名字字符串列表
        self.names = self.model.module.names if hasattr(
            self.model, 'module') else self.model.names  # get class names
        if self.half:
            self.model.half()  # to FP16

        # Second-stage classifier
        # 設置第二類分辨，默認是不使用
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet50', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('resnet50.pt', map_location=self.device)[
                                        'model']).to(self.device).eval()

        # Dataloader
        view_img = check_imshow() # Check if environment supports image displays
        cudnn.benchmark = True  # set True to speed up constant image size inference

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

        # for point_cloud
        self._num_count = 0
        self._z_finial_disrance = 0.0
        self.x_list = []
        self.y_list = []
        self.x_list_fix = []
        self.y_list_fix = []
        self.z_list = []
        self.subscriber_ = self.create_subscription(
            PointCloud2, "/robot1/intel_realsense_r200_depth/points", self.callback_pointcloud_XY_Plot, 10)
        
    def callback_pointcloud_XY_Plot(self, data):
        assert isinstance(data, PointCloud2)
        gen = point_cloud2.read_points(
            data, field_names=("x", "y", "z"), skip_nans=True)
        for p in gen:
            [(p[0], p[1], p[2]), (p[0], p[1], p[2])]
            # type(p[0]) and type(p[1]) and type(p[2]) is float
            x = p[0]
            y = p[1]
            z = p[2]
            # 除去inf的數值
            if not math.isinf(x) and not math.isinf(y):
                if (x <= 0.01 and x >= -0.01) and (y <= 0.01 and y >= -0.01):
                    self.x_list_fix.append(x)
                    self.y_list_fix.append(y)
                    self.z_list.append(z)
        
        if len(self.z_list) != 0:
            self._z_finial_disrance = statistics.mean(self.z_list)
        # print('Z list mean value(m): ', self._z_finial_disrance) # 最後取平均值 得到z軸的距離
        # self._num_count = self._num_count + 1
        # print("writing.....", self._num_count)
        self.x_list_fix.clear()
        self.y_list_fix.clear()
        self.x_list.clear()
        self.y_list.clear()
        self.z_list.clear()
        
    def yolovFive2bboxes_msgs(self, bboxes: list, scores: list, cls: list, img_header: Header):
        bboxes_msg = BoundingBoxes()
        bboxes_msg.header = img_header
        # print(bboxes)
        # print(bbox[0][0])
        i = 0
        for score in scores:
            one_box = BoundingBox()
            one_box.xmin = int(bboxes[0][i])
            one_box.ymin = int(bboxes[1][i])
            one_box.xmax = int(bboxes[2][i])
            one_box.ymax = int(bboxes[3][i])
            one_box.x_center = abs(one_box.xmax - one_box.xmin) / 2 + one_box.xmin
            one_box.y_center = abs(one_box.ymax - one_box.ymin) / 2 + one_box.ymin
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
        class_list = []
        confidence_list = []
        x_min_list = []
        y_min_list = []
        x_max_list = []
        y_max_list = []

        t0 = time.time()
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        # img = bridge.imgmsg_to_cv2(data, "32FC1")

        # check for common shapes
        s = np.stack([letterbox(x, self.imgsz, stride=self.stride)[0].shape for x in img], 0)  # shapes
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

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        save_dir = '/home/james/ros2_ws/src/yolobot/yolobot_recognition/scripts'
        # Inference
        t1 = time_synchronized()
        pred = self.model(img,
                          augment=self.augment,
                          visualize=increment_path(save_dir / 'features', mkdir=True) if self.visualize else False)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres,
                                   self.classes, self.agnostic_nms, max_det=self.max_det)
        t2 = time_synchronized()

        # Apply Classifier
        if self.classify:
            pred = apply_classifier(pred, self.modelc, img, img0)

        # Process detections
        # 對每張圖進行處理
        for i, det in enumerate(pred):  # detections per image
            # 設置印出訊息（圖片寬高）
            s = f'{i}: '
            s += '%gx%g ' % img.shape[2:]  # print string

            if len(det):
                # Rescale boxes from img_size to im0 size
                # 調整預測框座標，基於resize+pad的圖片座標-->基於源size圖片的座標
                # 此時座標格式為xyxy
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], img0.shape).round()

                # Print results
                # 印出檢測的類別訊息
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    # add to string
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "

                # 保存預測結果
                for *xyxy, conf, cls in reversed(det):
                    if float(conf) >= 0.75:
                        c = int(cls)  # integer class
                        label = None if self.hide_labels else (
                            self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                        plot_one_box(xyxy, img0, label=label, color=colors(
                            c, True), line_thickness=self.line_thickness)

                        class_list.append(self.names[c])
                        confidence_list.append(conf)
                        # tensor to float
                        x_min_list.append(xyxy[0].item())
                        y_min_list.append(xyxy[1].item())
                        x_max_list.append(xyxy[2].item())
                        y_max_list.append(xyxy[3].item())
                        # print("integer class: ", c, "; lable: ", label)
                        # print("--------------------")

        cv2.imshow("IMAGE", img0)

        if len(class_list):
            renamed_list = self.rename_duplicate(class_list)
            self.yolovFive2bboxes_msgs(bboxes=[x_min_list, y_min_list, x_max_list, y_max_list],
                                       scores=confidence_list, cls=renamed_list, img_header=data.header)
            
        self.publish_image.publish(bridge.cv2_to_imgmsg(img0, "bgr8"))
        
        cv2.waitKey(1)

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
