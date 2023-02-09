#!/usr/bin/env python3
# -*- coding:UTF-8 -*-
from PyQt5 import QtWidgets
from pyqt5_control_20 import MainWindow_controller
# from qt_material import list_themes
from qt_material import apply_stylesheet
# https://github.com/UN-GCPDS/qt-material
if __name__ == '__main__':
    extra = {
        'font_size': '12px',
        'density_scale': '-3',
    }
    import sys
    app = QtWidgets.QApplication(sys.argv)
    # apply_stylesheet(app, theme='dark_teal.xml', extra=extra)
    # apply_stylesheet(app, theme='dark_amber.xml', extra=extra)
    # apply_stylesheet(app, theme='dark_red.xml', extra=extra)
    # apply_stylesheet(app, theme='light_teal.xml', extra=extra)
    
    window = MainWindow_controller()
    window.show()
    sys.exit(app.exec_())
