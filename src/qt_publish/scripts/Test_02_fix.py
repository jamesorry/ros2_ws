# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Test_02.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1101, 774)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(860, 550, 231, 51))
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
        self.groupBox_robot_1.setGeometry(QtCore.QRect(860, 100, 231, 211))
        self.groupBox_robot_1.setObjectName("groupBox_robot_1")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_robot_1)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 39, 211, 151))
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
        self.groupBox_robot_2.setGeometry(QtCore.QRect(860, 320, 231, 211))
        self.groupBox_robot_2.setObjectName("groupBox_robot_2")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_robot_2)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(10, 39, 211, 151))
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
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(860, 40, 231, 41))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout.addWidget(self.label_3)
        self.label_Time = QtWidgets.QLabel(self.layoutWidget)
        self.label_Time.setObjectName("label_Time")
        self.horizontalLayout.addWidget(self.label_Time)
        self.layoutWidget1 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget1.setGeometry(QtCore.QRect(860, 0, 231, 41))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.label_Date = QtWidgets.QLabel(self.layoutWidget1)
        self.label_Date.setObjectName("label_Date")
        self.horizontalLayout_2.addWidget(self.label_Date)
        self.pushButton_update = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_update.setGeometry(QtCore.QRect(860, 620, 101, 31))
        self.pushButton_update.setObjectName("pushButton_update")
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(10, 10, 641, 571))
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
        self.verticalLayout_3.addWidget(self.scrollArea)
        self.label_image_origin_shape = QtWidgets.QLabel(self.centralwidget)
        self.label_image_origin_shape.setGeometry(QtCore.QRect(570, 600, 271, 17))
        self.label_image_origin_shape.setObjectName("label_image_origin_shape")
        self.label_image_current_shape = QtWidgets.QLabel(self.centralwidget)
        self.label_image_current_shape.setGeometry(QtCore.QRect(570, 640, 271, 17))
        self.label_image_current_shape.setObjectName("label_image_current_shape")
        self.label_ratio = QtWidgets.QLabel(self.centralwidget)
        self.label_ratio.setGeometry(QtCore.QRect(480, 620, 67, 17))
        self.label_ratio.setObjectName("label_ratio")
        self.label_click_pos = QtWidgets.QLabel(self.centralwidget)
        self.label_click_pos.setGeometry(QtCore.QRect(570, 670, 271, 17))
        self.label_click_pos.setObjectName("label_click_pos")
        self.label_normal_pos = QtWidgets.QLabel(self.centralwidget)
        self.label_normal_pos.setGeometry(QtCore.QRect(570, 700, 271, 17))
        self.label_normal_pos.setObjectName("label_normal_pos")
        self.label_real_pos = QtWidgets.QLabel(self.centralwidget)
        self.label_real_pos.setGeometry(QtCore.QRect(570, 730, 271, 17))
        self.label_real_pos.setObjectName("label_real_pos")
        self.widget = QtWidgets.QWidget(self.centralwidget)
        self.widget.setGeometry(QtCore.QRect(10, 620, 461, 27))
        self.widget.setObjectName("widget")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.pushButton_zoom_in = QtWidgets.QPushButton(self.widget)
        self.pushButton_zoom_in.setObjectName("pushButton_zoom_in")
        self.horizontalLayout_10.addWidget(self.pushButton_zoom_in)
        self.Slider_ImageSizeArea = QtWidgets.QSlider(self.widget)
        self.Slider_ImageSizeArea.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_ImageSizeArea.setObjectName("Slider_ImageSizeArea")
        self.horizontalLayout_10.addWidget(self.Slider_ImageSizeArea)
        self.pushButton_zoom_out = QtWidgets.QPushButton(self.widget)
        self.pushButton_zoom_out.setObjectName("pushButton_zoom_out")
        self.horizontalLayout_10.addWidget(self.pushButton_zoom_out)
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
        self.groupBox_robot_1.setTitle(_translate("MainWindow", "Robot_1"))
        self.label_4.setText(_translate("MainWindow", "X:"))
        self.label_robot_1_x.setText(_translate("MainWindow", "-"))
        self.label_5.setText(_translate("MainWindow", "Y:"))
        self.label_robot_1_y.setText(_translate("MainWindow", "-"))
        self.label_6.setText(_translate("MainWindow", "Z:"))
        self.label_robot_1_z.setText(_translate("MainWindow", "-"))
        self.groupBox_robot_2.setTitle(_translate("MainWindow", "Robot_2"))
        self.label_7.setText(_translate("MainWindow", "X:"))
        self.label_robot_2_x.setText(_translate("MainWindow", "-"))
        self.label_8.setText(_translate("MainWindow", "Y:"))
        self.label_robot_2_y.setText(_translate("MainWindow", "-"))
        self.label_9.setText(_translate("MainWindow", "Z:"))
        self.label_robot_2_z.setText(_translate("MainWindow", "-"))
        self.label_3.setText(_translate("MainWindow", "Time:"))
        self.label_Time.setText(_translate("MainWindow", "-"))
        self.label_2.setText(_translate("MainWindow", "Date:"))
        self.label_Date.setText(_translate("MainWindow", "-"))
        self.pushButton_update.setText(_translate("MainWindow", "Update"))
        self.label_Image.setText(_translate("MainWindow", "TextLabel"))
        self.label_image_origin_shape.setText(_translate("MainWindow", "TextLabel"))
        self.label_image_current_shape.setText(_translate("MainWindow", "TextLabel"))
        self.label_ratio.setText(_translate("MainWindow", "Ratio"))
        self.label_click_pos.setText(_translate("MainWindow", "TextLabel"))
        self.label_normal_pos.setText(_translate("MainWindow", "TextLabel"))
        self.label_real_pos.setText(_translate("MainWindow", "TextLabel"))
        self.pushButton_zoom_in.setText(_translate("MainWindow", "Zoom In"))
        self.pushButton_zoom_out.setText(_translate("MainWindow", "Zoom Out"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
