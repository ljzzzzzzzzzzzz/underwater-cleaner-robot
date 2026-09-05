# -*- coding: utf-8 -*-
"""
robot_contrl.py —— 运行期加载 motor_contrl.ui 的加载器（改编自模板 motor_contrl.py）
"""
import os

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide2")

from PySide2.QtCore import QObject, QMetaObject
from PySide2.QtUiTools import QUiLoader
from PySide2.QtWidgets import QWidget

from pyqtgraph import PlotWidget

import motor_contrl_rc  # noqa: F401  编译好的资源（图标等）


class _UiLoader(QUiLoader):
    def __init__(self, baseinstance=None):
        super(_UiLoader, self).__init__(baseinstance)
        self.baseinstance = baseinstance

    def createWidget(self, class_name, parent=None, name=""):
        if class_name == "PlotWidget":
            widget = PlotWidget(parent)
            widget.setObjectName(name)
            return widget
        if parent is None and self.baseinstance is not None:
            return self.baseinstance
        return super(_UiLoader, self).createWidget(class_name, parent, name)


class Ui_Robot(object):
    """把 motor_contrl.ui 中所有带 objectName 的控件暴露成 self.<name>"""

    def setupUi(self, Robot):
        ui_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motor_contrl.ui")
        loader = _UiLoader(Robot)
        loader.load(ui_path)

        for child in Robot.findChildren(QWidget):
            name = child.objectName()
            if name:
                setattr(self, name, child)

        for attr in [
            "rootLayout", "heroLayout", "heroTextLayout", "contentLayout",
            "leftColumn", "rightColumn", "topGrid", "verticalLayout_3",
            "horizontalLayout_3", "horizontalLayout_4", "horizontalLayout_6",
            "horizontalLayout", "cameraLayout", "cameraButtonLayout", "waveLayout",
            "controlGrid", "groupBox5Layout", "groupBox4Layout", "horizontalLayout_5",
            "motionGrid", "groupBox8Layout", "groupBox10Layout", "armSpeedColumn",
            "horizontalLayout_7", "groupBox9Layout", "groupBox7Layout",
            "verticalLayout_5", "groupBox2Layout", "groupBox3Layout", "frameLayout",
            "metricsLayout",
        ]:
            obj = Robot.findChild(QObject, attr)
            if obj is not None:
                setattr(self, attr, obj)

        # 模板 .ui 里未暴露但改造需要的布局
        for attr in ["pageMotionLayout", "pageVisionLayout", "pageDebugLayout",
                     "motionTopGrid", "motionBottomLayout", "navLayout",
                     "feedbackLayout", "sidePanelLayout", "mainAreaLayout"]:
            obj = Robot.findChild(QObject, attr)
            if obj is not None:
                setattr(self, attr, obj)

        QMetaObject.connectSlotsByName(Robot)
