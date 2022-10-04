#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
from PyQt5 import QtWidgets
from pyqt5_control_15 import MainWindow_controller
# from qt_material import list_themes
from qt_material import apply_stylesheet

if __name__ == '__main__':
    extra = {
        # 'font_size': '6px',
    }
    import sys
    app = QtWidgets.QApplication(sys.argv)
    apply_stylesheet(app, theme='dark_teal.xml', extra=extra)
    window = MainWindow_controller()
    window.show()
    sys.exit(app.exec_())
