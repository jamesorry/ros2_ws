# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Test_11.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1108, 692)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(560, 0, 227, 40))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_2 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_3.addWidget(self.label_2)
        self.pushButton_start = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_start.setFont(font)
        self.pushButton_start.setObjectName("pushButton_start")
        self.horizontalLayout_3.addWidget(self.pushButton_start)
        self.pushButton_stop = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_stop.setFont(font)
        self.pushButton_stop.setObjectName("pushButton_stop")
        self.horizontalLayout_3.addWidget(self.pushButton_stop)
        self.groupBox_robot_1 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_robot_1.setGeometry(QtCore.QRect(557, 50, 263, 185))
        self.groupBox_robot_1.setObjectName("groupBox_robot_1")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_robot_1)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(23, 26, 221, 151))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_20 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_20.setObjectName("horizontalLayout_20")
        self.label_25 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_25.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_25.setObjectName("label_25")
        self.horizontalLayout_20.addWidget(self.label_25)
        self.label_system_status_robot1 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_system_status_robot1.setObjectName("label_system_status_robot1")
        self.horizontalLayout_20.addWidget(self.label_system_status_robot1)
        self.verticalLayout.addLayout(self.horizontalLayout_20)
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
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(0, 0, 551, 511))
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
        # self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 547, 507))
        # self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        # self.label_Image = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        # self.label_Image.setGeometry(QtCore.QRect(2, 2, 541, 542))
        # self.label_Image.setObjectName("label_Image")
        # self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        
        self.verticalLayout_3.addWidget(self.scrollArea)
        self.verticalLayoutWidget_6 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(0, 510, 552, 150))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.pushButton_save_recongnition = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_save_recongnition.setFont(font)
        self.pushButton_save_recongnition.setObjectName("pushButton_save_recongnition")
        self.horizontalLayout_11.addWidget(self.pushButton_save_recongnition)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_image_origin_shape = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_image_origin_shape.setFont(font)
        self.label_image_origin_shape.setObjectName("label_image_origin_shape")
        self.verticalLayout_4.addWidget(self.label_image_origin_shape)
        self.label_image_current_shape = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_image_current_shape.setFont(font)
        self.label_image_current_shape.setObjectName("label_image_current_shape")
        self.verticalLayout_4.addWidget(self.label_image_current_shape)
        self.horizontalLayout_11.addLayout(self.verticalLayout_4)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_click_pos = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_click_pos.setFont(font)
        self.label_click_pos.setObjectName("label_click_pos")
        self.verticalLayout_5.addWidget(self.label_click_pos)
        self.label_normal_pos = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_normal_pos.setFont(font)
        self.label_normal_pos.setObjectName("label_normal_pos")
        self.verticalLayout_5.addWidget(self.label_normal_pos)
        self.label_real_pos = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.label_real_pos.setFont(font)
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
        self.label_ratio.setText("")
        self.label_ratio.setObjectName("label_ratio")
        self.horizontalLayout_10.addWidget(self.label_ratio)
        self.verticalLayout_6.addLayout(self.horizontalLayout_10)
        self.groupBox_Target_robot1 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_Target_robot1.setGeometry(QtCore.QRect(555, 240, 269, 220))
        self.groupBox_Target_robot1.setCheckable(True)
        self.groupBox_Target_robot1.setChecked(False)
        self.groupBox_Target_robot1.setObjectName("groupBox_Target_robot1")
        self.layoutWidget = QtWidgets.QWidget(self.groupBox_Target_robot1)
        self.layoutWidget.setGeometry(QtCore.QRect(6, 41, 258, 174))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_19 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_19.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        self.verticalLayout_19.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_19.setSpacing(4)
        self.verticalLayout_19.setObjectName("verticalLayout_19")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout()
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label.setObjectName("label")
        self.verticalLayout_9.addWidget(self.label)
        self.label_10 = QtWidgets.QLabel(self.layoutWidget)
        self.label_10.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_10.setObjectName("label_10")
        self.verticalLayout_9.addWidget(self.label_10)
        self.label_11 = QtWidgets.QLabel(self.layoutWidget)
        self.label_11.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_11.setObjectName("label_11")
        self.verticalLayout_9.addWidget(self.label_11)
        self.horizontalLayout_12.addLayout(self.verticalLayout_9)
        self.verticalLayout_10 = QtWidgets.QVBoxLayout()
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.lineEdit_robot1_target_x = QtWidgets.QLineEdit(self.layoutWidget)
        self.lineEdit_robot1_target_x.setObjectName("lineEdit_robot1_target_x")
        self.verticalLayout_10.addWidget(self.lineEdit_robot1_target_x)
        self.lineEdit_robot1_target_y = QtWidgets.QLineEdit(self.layoutWidget)
        self.lineEdit_robot1_target_y.setObjectName("lineEdit_robot1_target_y")
        self.verticalLayout_10.addWidget(self.lineEdit_robot1_target_y)
        self.lineEdit_robot1_target_theta = QtWidgets.QLineEdit(self.layoutWidget)
        self.lineEdit_robot1_target_theta.setObjectName("lineEdit_robot1_target_theta")
        self.verticalLayout_10.addWidget(self.lineEdit_robot1_target_theta)
        self.horizontalLayout_12.addLayout(self.verticalLayout_10)
        self.verticalLayout_19.addLayout(self.horizontalLayout_12)
        self.verticalLayout_17 = QtWidgets.QVBoxLayout()
        self.verticalLayout_17.setObjectName("verticalLayout_17")
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.pushButton_robot_1_clear = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_robot_1_clear.setFont(font)
        self.pushButton_robot_1_clear.setObjectName("pushButton_robot_1_clear")
        self.horizontalLayout_15.addWidget(self.pushButton_robot_1_clear)
        self.pushButton_robot_1_send = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_robot_1_send.setFont(font)
        self.pushButton_robot_1_send.setObjectName("pushButton_robot_1_send")
        self.horizontalLayout_15.addWidget(self.pushButton_robot_1_send)
        self.pushButton_update_robot1 = QtWidgets.QPushButton(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_update_robot1.setFont(font)
        self.pushButton_update_robot1.setObjectName("pushButton_update_robot1")
        self.horizontalLayout_15.addWidget(self.pushButton_update_robot1)
        self.verticalLayout_17.addLayout(self.horizontalLayout_15)
        self.horizontalLayout_16 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_16 = QtWidgets.QLabel(self.layoutWidget)
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_16.addWidget(self.label_16)
        self.label_target_status_robot1 = QtWidgets.QLabel(self.layoutWidget)
        self.label_target_status_robot1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_target_status_robot1.setObjectName("label_target_status_robot1")
        self.horizontalLayout_16.addWidget(self.label_target_status_robot1)
        self.verticalLayout_17.addLayout(self.horizontalLayout_16)
        self.verticalLayout_19.addLayout(self.verticalLayout_17)
        self.groupBox_controller_robot1 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_controller_robot1.setGeometry(QtCore.QRect(1237, 2, 229, 311))
        self.groupBox_controller_robot1.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.groupBox_controller_robot1.setCheckable(True)
        self.groupBox_controller_robot1.setChecked(False)
        self.groupBox_controller_robot1.setObjectName("groupBox_controller_robot1")
        self.pushButton_reset_z_angular_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_reset_z_angular_robot1.setGeometry(QtCore.QRect(93, 221, 39, 25))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_reset_z_angular_robot1.setFont(font)
        self.pushButton_reset_z_angular_robot1.setObjectName("pushButton_reset_z_angular_robot1")
        self.Slider_z_angular_robot1 = QtWidgets.QSlider(self.groupBox_controller_robot1)
        self.Slider_z_angular_robot1.setGeometry(QtCore.QRect(21, 200, 185, 20))
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
        self.pushButton_decrease_z_angular_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_decrease_z_angular_robot1.setGeometry(QtCore.QRect(172, 221, 53, 25))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_decrease_z_angular_robot1.setFont(font)
        self.pushButton_decrease_z_angular_robot1.setObjectName("pushButton_decrease_z_angular_robot1")
        self.pushButton_increase_x_linear_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_increase_x_linear_robot1.setGeometry(QtCore.QRect(32, 45, 64, 25))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_increase_x_linear_robot1.setFont(font)
        self.pushButton_increase_x_linear_robot1.setObjectName("pushButton_increase_x_linear_robot1")
        self.pushButton_decrease_x_linear_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_decrease_x_linear_robot1.setGeometry(QtCore.QRect(32, 170, 64, 25))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_decrease_x_linear_robot1.setFont(font)
        self.pushButton_decrease_x_linear_robot1.setObjectName("pushButton_decrease_x_linear_robot1")
        self.label_current_z_angular_robot1 = QtWidgets.QLabel(self.groupBox_controller_robot1)
        self.label_current_z_angular_robot1.setGeometry(QtCore.QRect(63, 249, 101, 20))
        self.label_current_z_angular_robot1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_current_z_angular_robot1.setObjectName("label_current_z_angular_robot1")
        self.Slider_x_linear_robot1 = QtWidgets.QSlider(self.groupBox_controller_robot1)
        self.Slider_x_linear_robot1.setGeometry(QtCore.QRect(97, 49, 30, 142))
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
        self.label_current_x_linear_robot1.setGeometry(QtCore.QRect(129, 111, 82, 17))
        self.label_current_x_linear_robot1.setAlignment(QtCore.Qt.AlignCenter)
        self.label_current_x_linear_robot1.setObjectName("label_current_x_linear_robot1")
        self.pushButton_increase_z_angular_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_increase_z_angular_robot1.setGeometry(QtCore.QRect(2, 221, 50, 25))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_increase_z_angular_robot1.setFont(font)
        self.pushButton_increase_z_angular_robot1.setObjectName("pushButton_increase_z_angular_robot1")
        self.pushButton_reset_x_linear_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_reset_x_linear_robot1.setGeometry(QtCore.QRect(52, 111, 39, 25))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_reset_x_linear_robot1.setFont(font)
        self.pushButton_reset_x_linear_robot1.setObjectName("pushButton_reset_x_linear_robot1")
        self.pushButton_stop_moving_robot1 = QtWidgets.QPushButton(self.groupBox_controller_robot1)
        self.pushButton_stop_moving_robot1.setGeometry(QtCore.QRect(69, 274, 93, 32))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_stop_moving_robot1.setFont(font)
        self.pushButton_stop_moving_robot1.setObjectName("pushButton_stop_moving_robot1")
        self.layoutWidget1 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget1.setGeometry(QtCore.QRect(794, 0, 223, 39))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout.addWidget(self.label_3)
        self.label_Time = QtWidgets.QLabel(self.layoutWidget1)
        self.label_Time.setObjectName("label_Time")
        self.horizontalLayout.addWidget(self.label_Time)
        self.groupBox_fix_controller_robot1 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_fix_controller_robot1.setGeometry(QtCore.QRect(863, 48, 200, 228))
        self.groupBox_fix_controller_robot1.setCheckable(True)
        self.groupBox_fix_controller_robot1.setChecked(False)
        self.groupBox_fix_controller_robot1.setObjectName("groupBox_fix_controller_robot1")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_fix_controller_robot1)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(7, 42, 187, 60))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.label_18 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.verticalLayout_7.addWidget(self.label_18)
        self.label_19 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.verticalLayout_7.addWidget(self.label_19)
        self.horizontalLayout_2.addLayout(self.verticalLayout_7)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.lineEdit_x_liner_robot1 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_2)
        self.lineEdit_x_liner_robot1.setObjectName("lineEdit_x_liner_robot1")
        self.verticalLayout_8.addWidget(self.lineEdit_x_liner_robot1)
        self.lineEdit_z_angular_robot1 = QtWidgets.QLineEdit(self.horizontalLayoutWidget_2)
        self.lineEdit_z_angular_robot1.setObjectName("lineEdit_z_angular_robot1")
        self.verticalLayout_8.addWidget(self.lineEdit_z_angular_robot1)
        self.horizontalLayout_2.addLayout(self.verticalLayout_8)
        self.verticalLayout_13 = QtWidgets.QVBoxLayout()
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.label_15 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.verticalLayout_13.addWidget(self.label_15)
        self.label_20 = QtWidgets.QLabel(self.horizontalLayoutWidget_2)
        self.label_20.setAlignment(QtCore.Qt.AlignCenter)
        self.label_20.setObjectName("label_20")
        self.verticalLayout_13.addWidget(self.label_20)
        self.horizontalLayout_2.addLayout(self.verticalLayout_13)
        self.pushButton_fix_stop_moving_robot1 = QtWidgets.QPushButton(self.groupBox_fix_controller_robot1)
        self.pushButton_fix_stop_moving_robot1.setGeometry(QtCore.QRect(84, 152, 48, 25))
        self.pushButton_fix_stop_moving_robot1.setObjectName("pushButton_fix_stop_moving_robot1")
        self.pushButton_left_rotate_robot1 = QtWidgets.QPushButton(self.groupBox_fix_controller_robot1)
        self.pushButton_left_rotate_robot1.setGeometry(QtCore.QRect(47, 149, 33, 33))
        self.pushButton_left_rotate_robot1.setStyleSheet("background-image: url(:/icon/UI/icon/left_rotation.png);\n"
"border-image: url(:/icon/UI/icon/left_rotation.png);\n"
"image: url(:/icon/UI/icon/left_rotation.png);")
        self.pushButton_left_rotate_robot1.setText("")
        self.pushButton_left_rotate_robot1.setObjectName("pushButton_left_rotate_robot1")
        self.pushButton_move_forward_robot1 = QtWidgets.QPushButton(self.groupBox_fix_controller_robot1)
        self.pushButton_move_forward_robot1.setGeometry(QtCore.QRect(95, 103, 28, 40))
        self.pushButton_move_forward_robot1.setStyleSheet("background-image: url(:/icon/UI/icon/flecha_arriba.png);\n"
"border-image: url(:/icon/UI/icon/flecha_arriba.png);\n"
"image: url(:/icon/UI/icon/flecha_arriba.png);")
        self.pushButton_move_forward_robot1.setText("")
        self.pushButton_move_forward_robot1.setObjectName("pushButton_move_forward_robot1")
        self.pushButton_move_backward_robot1 = QtWidgets.QPushButton(self.groupBox_fix_controller_robot1)
        self.pushButton_move_backward_robot1.setGeometry(QtCore.QRect(95, 184, 28, 40))
        self.pushButton_move_backward_robot1.setStyleSheet("background-image: url(:/icon/UI/icon/flecha_abajo.png);\n"
"border-image: url(:/icon/UI/icon/flecha_abajo.png);\n"
"image: url(:/icon/UI/icon/flecha_abajo.png);")
        self.pushButton_move_backward_robot1.setText("")
        self.pushButton_move_backward_robot1.setObjectName("pushButton_move_backward_robot1")
        self.pushButton_right_rotate_robot1 = QtWidgets.QPushButton(self.groupBox_fix_controller_robot1)
        self.pushButton_right_rotate_robot1.setGeometry(QtCore.QRect(137, 148, 33, 33))
        self.pushButton_right_rotate_robot1.setStyleSheet("background-image: url(:/icon/UI/icon/rigth_rotation.png);\n"
"border-image: url(:/icon/UI/icon/rigth_rotation.png);\n"
"image: url(:/icon/UI/icon/rigth_rotation.png);")
        self.pushButton_right_rotate_robot1.setText("")
        self.pushButton_right_rotate_robot1.setObjectName("pushButton_right_rotate_robot1")
        self.pushButton_show_camera_window = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_show_camera_window.setGeometry(QtCore.QRect(563, 475, 118, 35))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_show_camera_window.setFont(font)
        self.pushButton_show_camera_window.setIconSize(QtCore.QSize(16, 16))
        self.pushButton_show_camera_window.setObjectName("pushButton_show_camera_window")
        self.tableWidget_robot1 = QtWidgets.QTableWidget(self.centralwidget)
        self.tableWidget_robot1.setGeometry(QtCore.QRect(564, 535, 471, 68))
        self.tableWidget_robot1.setObjectName("tableWidget_robot1")
        self.tableWidget_robot1.setColumnCount(0)
        self.tableWidget_robot1.setRowCount(0)
        self.label_27 = QtWidgets.QLabel(self.centralwidget)
        self.label_27.setGeometry(QtCore.QRect(565, 516, 90, 17))
        self.label_27.setObjectName("label_27")
        self.pushButton_start_yolov5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_start_yolov5.setGeometry(QtCore.QRect(1284, 362, 140, 48))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.pushButton_start_yolov5.setFont(font)
        self.pushButton_start_yolov5.setIconSize(QtCore.QSize(16, 16))
        self.pushButton_start_yolov5.setObjectName("pushButton_start_yolov5")
        self.groupBox_follow_controller = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_follow_controller.setGeometry(QtCore.QRect(830, 295, 268, 164))
        self.groupBox_follow_controller.setObjectName("groupBox_follow_controller")
        self.verticalLayoutWidget_4 = QtWidgets.QWidget(self.groupBox_follow_controller)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(6, 30, 256, 131))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.verticalLayout_21 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_21.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_21.setSpacing(6)
        self.verticalLayout_21.setObjectName("verticalLayout_21")
        self.horizontalLayout_24 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_24.setObjectName("horizontalLayout_24")
        self.label_31 = QtWidgets.QLabel(self.verticalLayoutWidget_4)
        self.label_31.setAlignment(QtCore.Qt.AlignCenter)
        self.label_31.setObjectName("label_31")
        self.horizontalLayout_24.addWidget(self.label_31)
        self.label_follow_object_target = QtWidgets.QLabel(self.verticalLayoutWidget_4)
        self.label_follow_object_target.setAlignment(QtCore.Qt.AlignCenter)
        self.label_follow_object_target.setObjectName("label_follow_object_target")
        self.horizontalLayout_24.addWidget(self.label_follow_object_target)
        self.verticalLayout_21.addLayout(self.horizontalLayout_24)
        self.horizontalLayout_23 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_23.setObjectName("horizontalLayout_23")
        self.label_30 = QtWidgets.QLabel(self.verticalLayoutWidget_4)
        self.label_30.setAlignment(QtCore.Qt.AlignCenter)
        self.label_30.setObjectName("label_30")
        self.horizontalLayout_23.addWidget(self.label_30)
        self.label_follow_status = QtWidgets.QLabel(self.verticalLayoutWidget_4)
        self.label_follow_status.setAlignment(QtCore.Qt.AlignCenter)
        self.label_follow_status.setObjectName("label_follow_status")
        self.horizontalLayout_23.addWidget(self.label_follow_status)
        self.verticalLayout_21.addLayout(self.horizontalLayout_23)
        self.horizontalLayout_19 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_19.setObjectName("horizontalLayout_19")
        self.pushButton_follow_cancel = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_follow_cancel.setFont(font)
        self.pushButton_follow_cancel.setObjectName("pushButton_follow_cancel")
        self.horizontalLayout_19.addWidget(self.pushButton_follow_cancel)
        self.pushButton_follow_start = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_follow_start.setFont(font)
        self.pushButton_follow_start.setObjectName("pushButton_follow_start")
        self.horizontalLayout_19.addWidget(self.pushButton_follow_start)
        self.pushButton_follow_update = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(9)
        self.pushButton_follow_update.setFont(font)
        self.pushButton_follow_update.setObjectName("pushButton_follow_update")
        self.horizontalLayout_19.addWidget(self.pushButton_follow_update)
        self.verticalLayout_21.addLayout(self.horizontalLayout_19)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "ROS-Foxy-UI"))
        self.label_2.setText(_translate("MainWindow", "System:"))
        self.pushButton_start.setText(_translate("MainWindow", "Start"))
        self.pushButton_stop.setText(_translate("MainWindow", "Stop"))
        self.groupBox_robot_1.setTitle(_translate("MainWindow", "Robot_1 Position"))
        self.label_25.setText(_translate("MainWindow", "Status:"))
        self.label_system_status_robot1.setText(_translate("MainWindow", "-"))
        self.label_4.setText(_translate("MainWindow", "X:"))
        self.label_robot_1_x.setText(_translate("MainWindow", "-"))
        self.label_5.setText(_translate("MainWindow", "Y:"))
        self.label_robot_1_y.setText(_translate("MainWindow", "-"))
        self.label_6.setText(_translate("MainWindow", "theta:"))
        self.label_robot_1_z.setText(_translate("MainWindow", "-"))
        self.label_Image.setText(_translate("MainWindow", "IMAGE"))
        self.pushButton_save_recongnition.setText(_translate("MainWindow", "Save recognition"))
        self.label_image_origin_shape.setText(_translate("MainWindow", "TextLabel"))
        self.label_image_current_shape.setText(_translate("MainWindow", "TextLabel"))
        self.label_click_pos.setText(_translate("MainWindow", "TextLabel"))
        self.label_normal_pos.setText(_translate("MainWindow", "TextLabel"))
        self.label_real_pos.setText(_translate("MainWindow", "TextLabel"))
        self.pushButton_zoom_in.setText(_translate("MainWindow", "-"))
        self.pushButton_zoom_out.setText(_translate("MainWindow", "+"))
        self.groupBox_Target_robot1.setTitle(_translate("MainWindow", "Robot_1 Target"))
        self.label.setText(_translate("MainWindow", "X:"))
        self.label_10.setText(_translate("MainWindow", "Y:"))
        self.label_11.setText(_translate("MainWindow", "Theta:"))
        self.pushButton_robot_1_clear.setText(_translate("MainWindow", "clear"))
        self.pushButton_robot_1_send.setText(_translate("MainWindow", "send"))
        self.pushButton_update_robot1.setText(_translate("MainWindow", "update"))
        self.label_16.setText(_translate("MainWindow", "Status:"))
        self.label_target_status_robot1.setText(_translate("MainWindow", "-"))
        self.groupBox_controller_robot1.setTitle(_translate("MainWindow", "Robot_1 Controller"))
        self.pushButton_reset_z_angular_robot1.setText(_translate("MainWindow", "stop"))
        self.pushButton_decrease_z_angular_robot1.setText(_translate("MainWindow", "Right -"))
        self.pushButton_increase_x_linear_robot1.setText(_translate("MainWindow", "Front +"))
        self.pushButton_decrease_x_linear_robot1.setText(_translate("MainWindow", "Back -"))
        self.label_current_z_angular_robot1.setText(_translate("MainWindow", "rad/s"))
        self.label_current_x_linear_robot1.setText(_translate("MainWindow", "m/s"))
        self.pushButton_increase_z_angular_robot1.setText(_translate("MainWindow", "Left +"))
        self.pushButton_reset_x_linear_robot1.setText(_translate("MainWindow", "stop"))
        self.pushButton_stop_moving_robot1.setText(_translate("MainWindow", "Stop Moving"))
        self.label_3.setText(_translate("MainWindow", "Time:"))
        self.label_Time.setText(_translate("MainWindow", "-"))
        self.groupBox_fix_controller_robot1.setTitle(_translate("MainWindow", "Robot_1 Controller"))
        self.label_18.setText(_translate("MainWindow", "liner:"))
        self.label_19.setText(_translate("MainWindow", "angular:"))
        self.label_15.setText(_translate("MainWindow", "m/s"))
        self.label_20.setText(_translate("MainWindow", "rad/s"))
        self.pushButton_fix_stop_moving_robot1.setText(_translate("MainWindow", "STOP"))
        self.pushButton_show_camera_window.setText(_translate("MainWindow", "RealTime Camera"))
        self.label_27.setText(_translate("MainWindow", "Robot1_msg"))
        self.pushButton_start_yolov5.setText(_translate("MainWindow", "Start YOLOV5"))
        self.groupBox_follow_controller.setTitle(_translate("MainWindow", "Follow Controller"))
        self.label_31.setText(_translate("MainWindow", "Object Target:"))
        self.label_follow_object_target.setText(_translate("MainWindow", "-"))
        self.label_30.setText(_translate("MainWindow", "Follow Status:"))
        self.label_follow_status.setText(_translate("MainWindow", "-"))
        self.pushButton_follow_cancel.setText(_translate("MainWindow", "Cancel"))
        self.pushButton_follow_start.setText(_translate("MainWindow", "Follow"))
        self.pushButton_follow_update.setText(_translate("MainWindow", "Update"))
import icon


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
