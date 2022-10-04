# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'camera_window_01.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_CameraWindow(object):
    def setupUi(self, CameraWindow):
        CameraWindow.setObjectName("CameraWindow")
        CameraWindow.resize(711, 591)
        self.centralwidget = QtWidgets.QWidget(CameraWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.comboBox_robot_n = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_robot_n.setGeometry(QtCore.QRect(97, 15, 105, 25))
        self.comboBox_robot_n.setObjectName("comboBox_robot_n")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(21, 20, 63, 17))
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.label_camera_image = QtWidgets.QLabel(self.centralwidget)
        self.label_camera_image.setGeometry(QtCore.QRect(30, 61, 640, 480))
        self.label_camera_image.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_camera_image.setText("")
        self.label_camera_image.setObjectName("label_camera_image")
        CameraWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(CameraWindow)
        self.statusbar.setObjectName("statusbar")
        CameraWindow.setStatusBar(self.statusbar)

        self.retranslateUi(CameraWindow)
        QtCore.QMetaObject.connectSlotsByName(CameraWindow)

    def retranslateUi(self, CameraWindow):
        _translate = QtCore.QCoreApplication.translate
        CameraWindow.setWindowTitle(_translate("CameraWindow", "CameraWindow"))
        self.label.setText(_translate("CameraWindow", "Robot:"))
