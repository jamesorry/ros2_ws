# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Test_04.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1088, 775)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(616, 5, 210, 40))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.pushButton_start = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_start.setObjectName("pushButton_start")
        self.horizontalLayout_3.addWidget(self.pushButton_start)
        self.pushButton_stop = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_stop.setObjectName("pushButton_stop")
        self.horizontalLayout_3.addWidget(self.pushButton_stop)
        self.groupBox_robot_1 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_robot_1.setGeometry(QtCore.QRect(615, 64, 211, 185))
        self.groupBox_robot_1.setObjectName("groupBox_robot_1")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_robot_1)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(11, 27, 187, 151))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_4.addWidget(self.label_4)
        self.label_robot_1_x = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_robot_1_x.setObjectName("label_robot_1_x")
        self.horizontalLayout_4.addWidget(self.label_robot_1_x)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_5 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_5.addWidget(self.label_5)
        self.label_robot_1_y = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_robot_1_y.setObjectName("label_robot_1_y")
        self.horizontalLayout_5.addWidget(self.label_robot_1_y)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_6 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_6.addWidget(self.label_6)
        self.label_robot_1_z = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_robot_1_z.setObjectName("label_robot_1_z")
        self.horizontalLayout_6.addWidget(self.label_robot_1_z)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        self.groupBox_robot_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_robot_2.setGeometry(QtCore.QRect(854, 63, 209, 188))
        self.groupBox_robot_2.setObjectName("groupBox_robot_2")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_robot_2)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(10, 29, 184, 151))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_7 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_7.addWidget(self.label_7)
        self.label_robot_2_x = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_robot_2_x.setObjectName("label_robot_2_x")
        self.horizontalLayout_7.addWidget(self.label_robot_2_x)
        self.verticalLayout_2.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_8 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_8.addWidget(self.label_8)
        self.label_robot_2_y = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_robot_2_y.setObjectName("label_robot_2_y")
        self.horizontalLayout_8.addWidget(self.label_robot_2_y)
        self.verticalLayout_2.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_9 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_9.addWidget(self.label_9)
        self.label_robot_2_z = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_robot_2_z.setObjectName("label_robot_2_z")
        self.horizontalLayout_9.addWidget(self.label_robot_2_z)
        self.verticalLayout_2.addLayout(self.horizontalLayout_9)
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(10, 10, 582, 556))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.scrollArea = QtWidgets.QScrollArea(self.verticalLayoutWidget_3)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        
        # 依照此網站方法修改 (https://www.wongwonggoods.com/python/pyqt5-13/)
        # self.scrollAreaWidgetContents = QtWidgets.QWidget()
        # self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 637, 567))
        # self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.label_Image = QtWidgets.QLabel()
        self.label_Image.setGeometry(QtCore.QRect(0, 0, 0, 0))
        self.label_Image.setObjectName("label_Image")
        self.scrollArea.setWidget(self.label_Image)
        
        # self.scrollAreaWidgetContents = QtWidgets.QWidget()
        # self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 578, 552))
        # self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        # self.label_Image = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        # self.label_Image.setGeometry(QtCore.QRect(35, 90, 491, 361))
        # self.label_Image.setObjectName("label_Image")
        # self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout_3.addWidget(self.scrollArea)
        self.verticalLayoutWidget_6 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(9, 569, 584, 150))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_image_origin_shape = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_image_origin_shape.setObjectName("label_image_origin_shape")
        self.verticalLayout_4.addWidget(self.label_image_origin_shape)
        self.label_image_current_shape = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_image_current_shape.setObjectName("label_image_current_shape")
        self.verticalLayout_4.addWidget(self.label_image_current_shape)
        self.horizontalLayout_11.addLayout(self.verticalLayout_4)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_click_pos = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_click_pos.setObjectName("label_click_pos")
        self.verticalLayout_5.addWidget(self.label_click_pos)
        self.label_normal_pos = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_normal_pos.setObjectName("label_normal_pos")
        self.verticalLayout_5.addWidget(self.label_normal_pos)
        self.label_real_pos = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.label_real_pos.setObjectName("label_real_pos")
        self.verticalLayout_5.addWidget(self.label_real_pos)
        self.horizontalLayout_11.addLayout(self.verticalLayout_5)
        self.verticalLayout_6.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.pushButton_zoom_in = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.pushButton_zoom_in.setFont(font)
        self.pushButton_zoom_in.setObjectName("pushButton_zoom_in")
        self.horizontalLayout_10.addWidget(self.pushButton_zoom_in)
        self.Slider_ImageSizeArea = QtWidgets.QSlider(self.verticalLayoutWidget_6)
        self.Slider_ImageSizeArea.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_ImageSizeArea.setObjectName("Slider_ImageSizeArea")
        self.horizontalLayout_10.addWidget(self.Slider_ImageSizeArea)
        self.pushButton_zoom_out = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.pushButton_zoom_out.setFont(font)
        self.pushButton_zoom_out.setObjectName("pushButton_zoom_out")
        self.horizontalLayout_10.addWidget(self.pushButton_zoom_out)
        self.label_ratio = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(13)
        self.label_ratio.setFont(font)
        self.label_ratio.setObjectName("label_ratio")
        self.horizontalLayout_10.addWidget(self.label_ratio)
        self.verticalLayout_6.addLayout(self.horizontalLayout_10)
        self.verticalLayoutWidget_7 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_7.setGeometry(QtCore.QRect(845, 5, 210, 36))
        self.verticalLayoutWidget_7.setObjectName("verticalLayoutWidget_7")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_7)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget_7)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout.addWidget(self.label_3)
        self.label_Time = QtWidgets.QLabel(self.verticalLayoutWidget_7)
        self.label_Time.setObjectName("label_Time")
        self.horizontalLayout.addWidget(self.label_Time)
        self.verticalLayout_7.addLayout(self.horizontalLayout)
        self.groupBox_Target_robot1 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_Target_robot1.setGeometry(QtCore.QRect(611, 257, 228, 239))
        self.groupBox_Target_robot1.setCheckable(True)
        self.groupBox_Target_robot1.setChecked(False)
        self.groupBox_Target_robot1.setObjectName("groupBox_Target_robot1")
        self.horizontalLayoutWidget_6 = QtWidgets.QWidget(self.groupBox_Target_robot1)
        self.horizontalLayoutWidget_6.setGeometry(QtCore.QRect(8, 155, 204, 37))
        self.horizontalLayoutWidget_6.setObjectName("horizontalLayoutWidget_6")
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_6)
        self.horizontalLayout_15.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.pushButton_robot_1_clear = QtWidgets.QPushButton(self.horizontalLayoutWidget_6)
        self.pushButton_robot_1_clear.setObjectName("pushButton_robot_1_clear")
        self.horizontalLayout_15.addWidget(self.pushButton_robot_1_clear)
        self.pushButton_robot_1_send = QtWidgets.QPushButton(self.horizontalLayoutWidget_6)
        self.pushButton_robot_1_send.setObjectName("pushButton_robot_1_send")
        self.horizontalLayout_15.addWidget(self.pushButton_robot_1_send)
        self.pushButton_update = QtWidgets.QPushButton(self.horizontalLayoutWidget_6)
        self.pushButton_update.setObjectName("pushButton_update")
        self.horizontalLayout_15.addWidget(self.pushButton_update)
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.groupBox_Target_robot1)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(8, 28, 205, 129))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout()
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.label = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        self.label.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label.setObjectName("label")
        self.verticalLayout_9.addWidget(self.label)
        self.label_10 = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        self.label_10.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_10.setObjectName("label_10")
        self.verticalLayout_9.addWidget(self.label_10)
        self.label_11 = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        self.label_11.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_11.setObjectName("label_11")
        self.verticalLayout_9.addWidget(self.label_11)
        self.horizontalLayout_12.addLayout(self.verticalLayout_9)
        self.verticalLayout_10 = QtWidgets.QVBoxLayout()
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.lineEdit_robot1_target_x = QtWidgets.QLineEdit(self.horizontalLayoutWidget_3)
        self.lineEdit_robot1_target_x.setObjectName("lineEdit_robot1_target_x")
        self.verticalLayout_10.addWidget(self.lineEdit_robot1_target_x)
        self.lineEdit_robot1_target_y = QtWidgets.QLineEdit(self.horizontalLayoutWidget_3)
        self.lineEdit_robot1_target_y.setObjectName("lineEdit_robot1_target_y")
        self.verticalLayout_10.addWidget(self.lineEdit_robot1_target_y)
        self.lineEdit_robot1_target_theta = QtWidgets.QLineEdit(self.horizontalLayoutWidget_3)
        self.lineEdit_robot1_target_theta.setObjectName("lineEdit_robot1_target_theta")
        self.verticalLayout_10.addWidget(self.lineEdit_robot1_target_theta)
        self.horizontalLayout_12.addLayout(self.verticalLayout_10)
        self.widget = QtWidgets.QWidget(self.groupBox_Target_robot1)
        self.widget.setGeometry(QtCore.QRect(8, 197, 205, 34))
        self.widget.setObjectName("widget")
        self.horizontalLayout_16 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_16.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_16 = QtWidgets.QLabel(self.widget)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_16.addWidget(self.label_16)
        self.label_target_status_robot1 = QtWidgets.QLabel(self.widget)
        self.label_target_status_robot1.setObjectName("label_target_status_robot1")
        self.horizontalLayout_16.addWidget(self.label_target_status_robot1)
        self.groupBox_Target_robot2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_Target_robot2.setGeometry(QtCore.QRect(856, 257, 226, 237))
        self.groupBox_Target_robot2.setCheckable(True)
        self.groupBox_Target_robot2.setChecked(False)
        self.groupBox_Target_robot2.setObjectName("groupBox_Target_robot2")
        self.horizontalLayoutWidget_4 = QtWidgets.QWidget(self.groupBox_Target_robot2)
        self.horizontalLayoutWidget_4.setGeometry(QtCore.QRect(10, 27, 205, 129))
        self.horizontalLayoutWidget_4.setObjectName("horizontalLayoutWidget_4")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_4)
        self.horizontalLayout_13.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.verticalLayout_11 = QtWidgets.QVBoxLayout()
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.label_12 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        self.label_12.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_12.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_12.setObjectName("label_12")
        self.verticalLayout_11.addWidget(self.label_12)
        self.label_13 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        self.label_13.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_13.setObjectName("label_13")
        self.verticalLayout_11.addWidget(self.label_13)
        self.label_14 = QtWidgets.QLabel(self.horizontalLayoutWidget_4)
        self.label_14.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_14.setObjectName("label_14")
        self.verticalLayout_11.addWidget(self.label_14)
        self.horizontalLayout_13.addLayout(self.verticalLayout_11)
        self.verticalLayout_12 = QtWidgets.QVBoxLayout()
        self.verticalLayout_12.setObjectName("verticalLayout_12")
        self.lineEdit_robot2_target_x = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.lineEdit_robot2_target_x.setObjectName("lineEdit_robot2_target_x")
        self.verticalLayout_12.addWidget(self.lineEdit_robot2_target_x)
        self.lineEdit_robot2_target_y = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.lineEdit_robot2_target_y.setObjectName("lineEdit_robot2_target_y")
        self.verticalLayout_12.addWidget(self.lineEdit_robot2_target_y)
        self.lineEdit_robot2_target_theta = QtWidgets.QLineEdit(self.horizontalLayoutWidget_4)
        self.lineEdit_robot2_target_theta.setObjectName("lineEdit_robot2_target_theta")
        self.verticalLayout_12.addWidget(self.lineEdit_robot2_target_theta)
        self.horizontalLayout_13.addLayout(self.verticalLayout_12)
        self.horizontalLayoutWidget_5 = QtWidgets.QWidget(self.groupBox_Target_robot2)
        self.horizontalLayoutWidget_5.setGeometry(QtCore.QRect(10, 155, 204, 41))
        self.horizontalLayoutWidget_5.setObjectName("horizontalLayoutWidget_5")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_5)
        self.horizontalLayout_14.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.pushButton_robot_2_clear = QtWidgets.QPushButton(self.horizontalLayoutWidget_5)
        self.pushButton_robot_2_clear.setObjectName("pushButton_robot_2_clear")
        self.horizontalLayout_14.addWidget(self.pushButton_robot_2_clear)
        self.pushButton_robot_2_send = QtWidgets.QPushButton(self.horizontalLayoutWidget_5)
        self.pushButton_robot_2_send.setObjectName("pushButton_robot_2_send")
        self.horizontalLayout_14.addWidget(self.pushButton_robot_2_send)
        self.pushButton_update_2 = QtWidgets.QPushButton(self.horizontalLayoutWidget_5)
        self.pushButton_update_2.setObjectName("pushButton_update_2")
        self.horizontalLayout_14.addWidget(self.pushButton_update_2)
        self.layoutWidget = QtWidgets.QWidget(self.groupBox_Target_robot2)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 199, 205, 34))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout_17 = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_17.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_17.setObjectName("horizontalLayout_17")
        self.label_17 = QtWidgets.QLabel(self.layoutWidget)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_17.addWidget(self.label_17)
        self.label_target_status_robot2 = QtWidgets.QLabel(self.layoutWidget)
        self.label_target_status_robot2.setObjectName("label_target_status_robot2")
        self.horizontalLayout_17.addWidget(self.label_target_status_robot2)
        self.groupBox_controller_robot1 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_controller_robot1.setGeometry(QtCore.QRect(610, 501, 229, 249))
        self.groupBox_controller_robot1.setCheckable(True)
        self.groupBox_controller_robot1.setChecked(True)
        self.groupBox_controller_robot1.setObjectName("groupBox_controller_robot1")
        self.pushButton_reset_z_angular_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_reset_z_angular_robot1.setGeometry(QtCore.QRect(94, 197, 39, 25))
        self.pushButton_reset_z_angular_robot1.setObjectName("pushButton_reset_z_angular_robot1")
        self.Slider_z_angular_robot1 = QtWidgets.QSlider(self.groupBox_controller_robot1)
        self.Slider_z_angular_robot1.setGeometry(QtCore.QRect(22, 176, 185, 20))
        self.Slider_z_angular_robot1.setMouseTracking(False)
        self.Slider_z_angular_robot1.setTabletTracking(True)
        self.Slider_z_angular_robot1.setMaximum(1000)
        self.Slider_z_angular_robot1.setPageStep(10)
        self.Slider_z_angular_robot1.setProperty("value", 500)
        self.Slider_z_angular_robot1.setSliderPosition(500)
        self.Slider_z_angular_robot1.setTracking(True)
        self.Slider_z_angular_robot1.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_z_angular_robot1.setInvertedAppearance(True)
        self.Slider_z_angular_robot1.setInvertedControls(True)
        self.Slider_z_angular_robot1.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.Slider_z_angular_robot1.setTickInterval(5)
        self.Slider_z_angular_robot1.setObjectName("Slider_z_angular_robot1")
        self.pushButton_increase_z_angular_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_increase_z_angular_robot1.setGeometry(QtCore.QRect(173, 197, 53, 25))
        self.pushButton_increase_z_angular_robot1.setObjectName("pushButton_increase_z_angular_robot1")
        self.pushButton_increase_x_linear_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_increase_x_linear_robot1.setGeometry(QtCore.QRect(33, 21, 64, 25))
        self.pushButton_increase_x_linear_robot1.setObjectName("pushButton_increase_x_linear_robot1")
        self.pushButton_decrease_x_linear_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_decrease_x_linear_robot1.setGeometry(QtCore.QRect(33, 146, 64, 25))
        self.pushButton_decrease_x_linear_robot1.setObjectName("pushButton_decrease_x_linear_robot1")
        self.label_current_z_angular_robot1 = QtWidgets.QLabel(self.groupBox_controller_robot1)
        self.label_current_z_angular_robot1.setGeometry(QtCore.QRect(64, 225, 101, 20))
        self.label_current_z_angular_robot1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_current_z_angular_robot1.setObjectName("label_current_z_angular_robot1")
        self.Slider_x_linear_robot1 = QtWidgets.QSlider(self.groupBox_controller_robot1)
        self.Slider_x_linear_robot1.setGeometry(QtCore.QRect(98, 25, 30, 142))
        self.Slider_x_linear_robot1.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.Slider_x_linear_robot1.setAcceptDrops(False)
        self.Slider_x_linear_robot1.setToolTip("")
        self.Slider_x_linear_robot1.setStatusTip("")
        self.Slider_x_linear_robot1.setInputMethodHints(QtCore.Qt.ImhNone)
        self.Slider_x_linear_robot1.setMaximum(1000)
        self.Slider_x_linear_robot1.setPageStep(10)
        self.Slider_x_linear_robot1.setProperty("value", 500)
        self.Slider_x_linear_robot1.setSliderPosition(500)
        self.Slider_x_linear_robot1.setTracking(True)
        self.Slider_x_linear_robot1.setOrientation(QtCore.Qt.Vertical)
        self.Slider_x_linear_robot1.setInvertedAppearance(False)
        self.Slider_x_linear_robot1.setInvertedControls(False)
        self.Slider_x_linear_robot1.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.Slider_x_linear_robot1.setTickInterval(6)
        self.Slider_x_linear_robot1.setObjectName("Slider_x_linear_robot1")
        self.label_current_x_linear_robot1 = QtWidgets.QLabel(self.groupBox_controller_robot1)
        self.label_current_x_linear_robot1.setGeometry(QtCore.QRect(130, 87, 82, 17))
        self.label_current_x_linear_robot1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_current_x_linear_robot1.setObjectName("label_current_x_linear_robot1")
        self.pushButton_decrease_z_angular_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_decrease_z_angular_robot1.setGeometry(QtCore.QRect(3, 197, 50, 25))
        self.pushButton_decrease_z_angular_robot1.setObjectName("pushButton_decrease_z_angular_robot1")
        self.pushButton_reset_x_linear_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_reset_x_linear_robot1.setGeometry(QtCore.QRect(53, 87, 39, 25))
        self.pushButton_reset_x_linear_robot1.setObjectName("pushButton_reset_x_linear_robot1")
        self.groupBox_controller_robot2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_controller_robot2.setGeometry(QtCore.QRect(852, 501, 229, 249))
        self.groupBox_controller_robot2.setCheckable(True)
        self.groupBox_controller_robot2.setChecked(True)
        self.groupBox_controller_robot2.setObjectName("groupBox_controller_robot2")
        self.pushButton_reset_z_angular_robot2 = QtWidgets.QPushButton(self.groupBox_controller_robot2)
        self.pushButton_reset_z_angular_robot2.setGeometry(QtCore.QRect(94, 197, 39, 25))
        self.pushButton_reset_z_angular_robot2.setObjectName("pushButton_reset_z_angular_robot2")
        self.Slider_z_angular_robot2 = QtWidgets.QSlider(self.groupBox_controller_robot2)
        self.Slider_z_angular_robot2.setGeometry(QtCore.QRect(22, 176, 185, 20))
        self.Slider_z_angular_robot2.setMouseTracking(False)
        self.Slider_z_angular_robot2.setTabletTracking(True)
        self.Slider_z_angular_robot2.setMaximum(1000)
        self.Slider_z_angular_robot2.setProperty("value", 500)
        self.Slider_z_angular_robot2.setSliderPosition(500)
        self.Slider_z_angular_robot2.setTracking(True)
        self.Slider_z_angular_robot2.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_z_angular_robot2.setInvertedAppearance(True)
        self.Slider_z_angular_robot2.setInvertedControls(True)
        self.Slider_z_angular_robot2.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.Slider_z_angular_robot2.setTickInterval(5)
        self.Slider_z_angular_robot2.setObjectName("Slider_z_angular_robot2")
        self.pushButton_increase_z_angular_robot2 = QtWidgets.QPushButton(self.groupBox_controller_robot2)
        self.pushButton_increase_z_angular_robot2.setGeometry(QtCore.QRect(173, 197, 53, 25))
        self.pushButton_increase_z_angular_robot2.setObjectName("pushButton_increase_z_angular_robot2")
        self.pushButton_increase_x_linear_robot2 = QtWidgets.QPushButton(self.groupBox_controller_robot2)
        self.pushButton_increase_x_linear_robot2.setGeometry(QtCore.QRect(33, 21, 64, 25))
        self.pushButton_increase_x_linear_robot2.setObjectName("pushButton_increase_x_linear_robot2")
        self.pushButton_decrease_x_linear_robot2 = QtWidgets.QPushButton(self.groupBox_controller_robot2)
        self.pushButton_decrease_x_linear_robot2.setGeometry(QtCore.QRect(33, 146, 64, 25))
        self.pushButton_decrease_x_linear_robot2.setObjectName("pushButton_decrease_x_linear_robot2")
        self.label_current_z_angular_robot2 = QtWidgets.QLabel(self.groupBox_controller_robot2)
        self.label_current_z_angular_robot2.setGeometry(QtCore.QRect(62, 225, 99, 20))
        self.label_current_z_angular_robot2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_current_z_angular_robot2.setObjectName("label_current_z_angular_robot2")
        self.Slider_x_linear_robot2 = QtWidgets.QSlider(self.groupBox_controller_robot2)
        self.Slider_x_linear_robot2.setGeometry(QtCore.QRect(98, 25, 30, 142))
        self.Slider_x_linear_robot2.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.Slider_x_linear_robot2.setAcceptDrops(False)
        self.Slider_x_linear_robot2.setToolTip("")
        self.Slider_x_linear_robot2.setStatusTip("")
        self.Slider_x_linear_robot2.setInputMethodHints(QtCore.Qt.ImhNone)
        self.Slider_x_linear_robot2.setMaximum(1000)
        self.Slider_x_linear_robot2.setProperty("value", 500)
        self.Slider_x_linear_robot2.setSliderPosition(500)
        self.Slider_x_linear_robot2.setTracking(True)
        self.Slider_x_linear_robot2.setOrientation(QtCore.Qt.Vertical)
        self.Slider_x_linear_robot2.setInvertedAppearance(False)
        self.Slider_x_linear_robot2.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.Slider_x_linear_robot2.setTickInterval(6)
        self.Slider_x_linear_robot2.setObjectName("Slider_x_linear_robot2")
        self.label_current_x_linear_robot2 = QtWidgets.QLabel(self.groupBox_controller_robot2)
        self.label_current_x_linear_robot2.setGeometry(QtCore.QRect(130, 87, 82, 17))
        self.label_current_x_linear_robot2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_current_x_linear_robot2.setObjectName("label_current_x_linear_robot2")
        self.pushButton_decrease_z_angular_robot2 = QtWidgets.QPushButton(self.groupBox_controller_robot2)
        self.pushButton_decrease_z_angular_robot2.setGeometry(QtCore.QRect(3, 197, 50, 25))
        self.pushButton_decrease_z_angular_robot2.setObjectName("pushButton_decrease_z_angular_robot2")
        self.pushButton_reset_x_linear_robot2 = QtWidgets.QPushButton(self.groupBox_controller_robot2)
        self.pushButton_reset_x_linear_robot2.setGeometry(QtCore.QRect(53, 87, 39, 25))
        self.pushButton_reset_x_linear_robot2.setObjectName("pushButton_reset_x_linear_robot2")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_start.setText(_translate("MainWindow", "Start"))
        self.pushButton_stop.setText(_translate("MainWindow", "Stop"))
        self.groupBox_robot_1.setTitle(_translate("MainWindow", "Robot_1 Position"))
        self.label_4.setText(_translate("MainWindow", "X:"))
        self.label_robot_1_x.setText(_translate("MainWindow", "-"))
        self.label_5.setText(_translate("MainWindow", "Y:"))
        self.label_robot_1_y.setText(_translate("MainWindow", "-"))
        self.label_6.setText(_translate("MainWindow", "Z:"))
        self.label_robot_1_z.setText(_translate("MainWindow", "-"))
        self.groupBox_robot_2.setTitle(_translate("MainWindow", "Robot_2 Position"))
        self.label_7.setText(_translate("MainWindow", "X:"))
        self.label_robot_2_x.setText(_translate("MainWindow", "-"))
        self.label_8.setText(_translate("MainWindow", "Y:"))
        self.label_robot_2_y.setText(_translate("MainWindow", "-"))
        self.label_9.setText(_translate("MainWindow", "Z:"))
        self.label_robot_2_z.setText(_translate("MainWindow", "-"))
        self.label_Image.setText(_translate("MainWindow", "IMAGE"))
        self.label_image_origin_shape.setText(_translate("MainWindow", "TextLabel"))
        self.label_image_current_shape.setText(_translate("MainWindow", "TextLabel"))
        self.label_click_pos.setText(_translate("MainWindow", "TextLabel"))
        self.label_normal_pos.setText(_translate("MainWindow", "TextLabel"))
        self.label_real_pos.setText(_translate("MainWindow", "TextLabel"))
        self.pushButton_zoom_in.setText(_translate("MainWindow", "-"))
        self.pushButton_zoom_out.setText(_translate("MainWindow", "+"))
        self.label_ratio.setText(_translate("MainWindow", "                            "))
        self.label_3.setText(_translate("MainWindow", "Time:"))
        self.label_Time.setText(_translate("MainWindow", "-"))
        self.groupBox_Target_robot1.setTitle(_translate("MainWindow", "Robot_1 Target"))
        self.pushButton_robot_1_clear.setText(_translate("MainWindow", "clear"))
        self.pushButton_robot_1_send.setText(_translate("MainWindow", "send"))
        self.pushButton_update.setText(_translate("MainWindow", "Update"))
        self.label.setText(_translate("MainWindow", "X:"))
        self.label_10.setText(_translate("MainWindow", "Y:"))
        self.label_11.setText(_translate("MainWindow", "Theta:"))
        self.label_16.setText(_translate("MainWindow", "Status:"))
        self.label_target_status_robot1.setText(_translate("MainWindow", "-"))
        self.groupBox_Target_robot2.setTitle(_translate("MainWindow", "Robot_2 Target"))
        self.label_12.setText(_translate("MainWindow", "X:"))
        self.label_13.setText(_translate("MainWindow", "Y:"))
        self.label_14.setText(_translate("MainWindow", "Theta:"))
        self.pushButton_robot_2_clear.setText(_translate("MainWindow", "clear"))
        self.pushButton_robot_2_send.setText(_translate("MainWindow", "send"))
        self.pushButton_update_2.setText(_translate("MainWindow", "Update"))
        self.label_17.setText(_translate("MainWindow", "Status:"))
        self.label_target_status_robot2.setText(_translate("MainWindow", "-"))
        self.groupBox_controller_robot1.setTitle(_translate("MainWindow", "Robot_1 Controller"))
        self.pushButton_reset_z_angular_robot1.setText(_translate("MainWindow", "stop"))
        self.pushButton_increase_z_angular_robot1.setText(_translate("MainWindow", "Right + "))
        self.pushButton_increase_x_linear_robot1.setText(_translate("MainWindow", "Front +"))
        self.pushButton_decrease_x_linear_robot1.setText(_translate("MainWindow", "Back -"))
        self.label_current_z_angular_robot1.setText(_translate("MainWindow", "rad/s"))
        self.label_current_x_linear_robot1.setText(_translate("MainWindow", "m/s"))
        self.pushButton_decrease_z_angular_robot1.setText(_translate("MainWindow", "Left - "))
        self.pushButton_reset_x_linear_robot1.setText(_translate("MainWindow", "stop"))
        self.groupBox_controller_robot2.setTitle(_translate("MainWindow", "Robot_2 Controller"))
        self.pushButton_reset_z_angular_robot2.setText(_translate("MainWindow", "stop"))
        self.pushButton_increase_z_angular_robot2.setText(_translate("MainWindow", "Right + "))
        self.pushButton_increase_x_linear_robot2.setText(_translate("MainWindow", "Front +"))
        self.pushButton_decrease_x_linear_robot2.setText(_translate("MainWindow", "Back -"))
        self.label_current_z_angular_robot2.setText(_translate("MainWindow", "rad/s"))
        self.label_current_x_linear_robot2.setText(_translate("MainWindow", "m/s"))
        self.pushButton_decrease_z_angular_robot2.setText(_translate("MainWindow", "Left - "))
        self.pushButton_reset_x_linear_robot2.setText(_translate("MainWindow", "stop"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
