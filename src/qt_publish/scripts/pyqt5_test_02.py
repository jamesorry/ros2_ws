#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from PIL import Image, ImageTk
from PyQt5 import QtWidgets, QtGui, QtCore
import cv2
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QDialog, QFileDialog, QGridLayout, QLabel, QPushButton


class ImageLabel(QLabel):
    scale = 1.0

    def showImage(self, img):
        height, width, channel = img.shape
        bytesPerline = 3 * width
        self.qImg = QImage(img.data, width, height,
                           bytesPerline, QImage.Format_RGB888).rgbSwapped()
        self.setPixmap(QPixmap.fromImage(self.qImg))

    def mousePressEvent(self, event):
        self.x = event.x()
        self.y = event.y()
        #print(str(self.x) + ' ' + str(self.y))

    def mouseMoveEvent(self, event):
        self.x = event.x()
        self.y = event.y()
        #print(str(self.x) + ' ' + str(self.y))

    def wheelEvent(self, event):
        numDegrees = event.angleDelta() / 8
        numSteps = numDegrees / 15
        # print(numSteps.y())
        height, width, _ = self.img.shape
        if numSteps.y() == -1:
            if (self.scale >= 0.1):
                self.scale -= 0.05
        else:
            if (self.scale <= 2.0):
                self.scale += 0.05
        # print(self.scale)
        height2 = int(height * self.scale)
        width2 = int(width * self.scale)
        img2 = cv2.resize(self.img, (width2, height2),
                          interpolation=cv2.INTER_AREA)
        self.showImage(img2)


class MyDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.resize(400, 300)
        self.label = ImageLabel()
        # self.label.setMouseTracking(True)
        self.btnOpen = QPushButton('Open Image', self)
        self.btnProcess = QPushButton('Blur Image', self)
        self.btnSave = QPushButton('Save Image', self)
        self.btnSave.setEnabled(False)

        layout = QGridLayout(self)
        layout.addWidget(self.label, 0, 0, 4, 4)
        layout.addWidget(self.btnOpen, 4, 0, 1, 1)
        layout.addWidget(self.btnProcess, 4, 1, 1, 1)
        layout.addWidget(self.btnSave, 4, 2, 1, 1)

        self.btnOpen.clicked.connect(self.openSlot)
        self.btnProcess.clicked.connect(self.processSlot)
        self.btnSave.clicked.connect(self.saveSlot)

    def openSlot(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, 'Open Image', 'Image', '*.png *.jpg *.bmp')
        if filename == '':
            return
        self.label.img = cv2.imread(filename, -1)
        if self.label.img.size == 1:
            return
        self.label.showImage(self.label.img)
        height, width, _ = self.label.img.shape
        self.label.setFixedSize(width, height)
        self.btnSave.setEnabled(True)

    def saveSlot(self):
        filename, _ = QFileDialog.getSaveFileName(
            self, 'Save Image', 'Image', '*.png *.jpg *.bmp')
        if filename == '':
            return
        cv2.imwrite(filename, self.label.img)

    def processSlot(self):
        self.label.img = cv2.blur(self.label.img, (7, 7))
        self.label.showImage(self.label.img)


if __name__ == '__main__':
    a = QApplication(sys.argv)
    dialog = MyDialog()
    dialog.show()
    sys.exit(a.exec_())
