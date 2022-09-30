#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
from PyQt5 import QtWidgets
from pyqt5_control_13 import MainWindow_controller

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow_controller()
    window.show()
    sys.exit(app.exec_())