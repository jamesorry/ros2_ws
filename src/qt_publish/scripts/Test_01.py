# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Test_01.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(691, 767)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.main = QtWidgets.QVBoxLayout()
        self.main.setObjectName("main")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.position_x = QtWidgets.QLabel(self.centralwidget)
        self.position_x.setObjectName("position_x")
        self.verticalLayout.addWidget(self.position_x)
        self.position_y = QtWidgets.QLabel(self.centralwidget)
        self.position_y.setObjectName("position_y")
        self.verticalLayout.addWidget(self.position_y)
        self.position_z = QtWidgets.QLabel(self.centralwidget)
        self.position_z.setObjectName("position_z")
        self.verticalLayout.addWidget(self.position_z)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.Button_clear = QtWidgets.QPushButton(self.centralwidget)
        self.Button_clear.setObjectName("Button_clear")
        self.horizontalLayout.addWidget(self.Button_clear)
        self.Button_read_position = QtWidgets.QPushButton(self.centralwidget)
        self.Button_read_position.setObjectName("Button_read_position")
        self.horizontalLayout.addWidget(self.Button_read_position)
        self.main.addLayout(self.horizontalLayout)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.main.addLayout(self.horizontalLayout_5)
        self.verticalLayout_2.addLayout(self.main)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "ROS2-PyQT5"))
        self.position_x.setText(_translate("MainWindow", "TextLabel"))
        self.position_y.setText(_translate("MainWindow", "TextLabel"))
        self.position_z.setText(_translate("MainWindow", "TextLabel"))
        self.Button_clear.setText(_translate("MainWindow", "Clear"))
        self.Button_read_position.setText(_translate("MainWindow", "Read"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
