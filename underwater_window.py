# -*- coding: utf-8 -*-
"""
underwater_window.py —— 智能水下清洁机器人控制系统 · 上位机主窗口（单主控连接版）

严格对齐郑莹莹《智能水下清洁机器人控制系统》软著材料：
  - 上位机只连接 91 主控：192.168.1.91:12345
  - 控制/视频/状态都走这条 TCP 连接
  - 指令为 7 字段 ASCII（10 90 1,-1 180,180 101 60 80，含灯光）
  - 89 从控（192.168.1.89:12346）为灯光从控：由 91 收到指令后自动转发，
    上位机不直接连 89
界面采用深蓝工业风三页布局：首页监控 / 作业控制台 / 指令调试。
"""
from __future__ import annotations

import os
import socket
import sys
import threading
import time
from datetime import datetime

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide2")

import cv2
import pyqtgraph as pg  # noqa: F401
from PySide2.QtCore import QObject, Qt, QTimer, Signal
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtWidgets import (
    QApplication,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

import robot_protocol as proto
from robot_contrl import Ui_Robot
from logger import Logger
from config_manager import ConfigManager
from video_receiver import FrameReceiver

APP_TITLE = "智能水下清洁机器人控制系统 V1.0"
SCHOOL_LINE = "广州航海学院 · 人工智能学院"

C_ACCENT = "#20a4f3"
C_GOOD = "#27ae60"
C_BAD = "#e74c3c"


# =========================================================================
# 主窗口
# =========================================================================
class MainWindow(QWidget):
    log_signal = Signal(str)

    PAGE_HINTS = {
        0: ("作业控制台", "推进器方向/PWM · 舵机角度 · 清洗刷 · LED 照明 · 模式(10运行/00待机)"),
        1: ("首页监控", "水下视频 · 速度波形 · 节点/运行监测"),
        2: ("指令调试与日志", "指令发送记录与系统日志（可手输任意 7 字段指令）"),
    }

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_Robot()
        self.ui.setupUi(self)

        # ------- 运行状态 -------
        self._sock = None
        self._send_lock = threading.Lock()
        self._connected = False
        self._receiver = None
        self._last_frame = None
        self._video_visible = False
        self._shot_dir = "data"
        self._wave_data = []
        self._closing = False
        self._reconnect_active = False
        self._reconnect_attempts = 0
        self._cmd_count = 0
        self._boot_time = time.time()
        self._last_frame_t = 0.0
        self._fps_est = 0.0
        self._last_stats_sec = 0
        self._auto_open_video = True    # 连接成功后收到第一帧自动显示
        self._video_auto_logged = False

        # 计划状态（整包 7 字段指令）
        self.plan = {
            "mode": proto.MODE_STOP, "angle": 90,
            "dir1": 0, "spd1": 0, "dir2": 0, "spd2": 0,
            "brush": "000", "brush_speed": 60.0, "light": 0,
        }

        self._syncing = False
        self.logger = Logger(level="INFO")
        self.config = ConfigManager()
        self._thruster_layout = str(
            self.config.get("robot91", "thruster_layout") or "normal").strip().lower()

        self.log_signal.connect(self._append_log)

        self._apply_branding()
        self._reorganize_monitor_page()
        self._setup_connection_panel()
        self._build_robot_console()
        self._build_debug_bar()
        self._configure_status_table()
        self._bind_signals()
        self._setup_navigation()
        self._init_plot()
        self._build_monitor_dashboard()

        self.ctrl_timer = QTimer(self)
        self.ctrl_timer.setSingleShot(True)
        self.ctrl_timer.setInterval(80)
        self.ctrl_timer.timeout.connect(lambda: self.send_plan(announce=False))

        self.reconnect_timer = QTimer(self)
        self.reconnect_timer.timeout.connect(self._check_reconnect)

        self.wave_timer = QTimer(self)
        self.wave_timer.setInterval(200)
        self.wave_timer.timeout.connect(self.update_wave)
        self.wave_timer.start()

        self._refresh_connection_state()
        self._refresh_status_panel()

    # =====================================================================
    # 品牌与文字
    # =====================================================================
    def _apply_branding(self):
        self.setWindowTitle(APP_TITLE)
        self.ui.heroTitle.setMinimumWidth(150)
        self.ui.heroHint.setMinimumWidth(340)
        try:
            # 校徽图片框：保留原 pixmap，缩小以让出空间
            self.ui.label_school_name.setFixedSize(118, 118)
            self.ui.subtitleLabel.setText(SCHOOL_LINE)
            self.ui.subtitleLabel.setStyleSheet(
                "color:#d6ecff; font-size:14px; font-weight:700; background:transparent;")
            self.ui.subtitleLabel.setWordWrap(False)
            self.ui.subtitleLabel.setAlignment(Qt.AlignCenter)
            self.ui.statusBadge.setText("未连接")
        except Exception:
            pass
        self.ui.groupBox.setTitle("主控连接（192.168.1.91:12345）")
        self.ui.groupBox_nav.setTitle("功能导航")
        for name in ("nav_motion", "nav_vision", "nav_debug"):
            btn = getattr(self.ui, name, None)
            if btn is not None:
                btn.setMinimumHeight(42)
        try:
            self.ui.verticalLayout_3.setSpacing(8)
        except Exception:
            pass
        self.ui.groupBox_5.setTitle("作业模式")
        self.ui.groupBox_4.setTitle("推进器速度")
        self.ui.groupBox_8.setTitle("运动方向控制")
        self.ui.groupBox_6.setTitle("水下视频监控（91 主控）")
        self.ui.groupBox_11.setTitle("推进器速度波形")
        self.ui.groupBox_feedback.setTitle("设备状态回显（最近指令）")
        self.ui.groupBox_2.setTitle("指令发送记录")
        self.ui.groupBox_3.setTitle("系统日志 / 回显")
        self.ui.camera_label.setText("水下视频画面")
        self.ui.camera_label_2.setText("运行监测")
        self.ui.camera_label_2.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.ui.camera_label_2.setWordWrap(True)
        self.ui.open_cam_btn.setText("打开视频显示")
        self.ui.open_cam_btn_3.setText("保存截图")
        # 隐藏模板“机械臂”等无关控件
        for name in ("groupBox_7", "groupBox_9", "groupBox_10",
                     "startlink", "stoplink", "ipadd", "port",
                     "label_2", "label_3"):
            w = getattr(self.ui, name, None)
            if w is not None:
                w.hide()

    # =====================================================================
    # 首页监控重组
    # =====================================================================
    def _reorganize_monitor_page(self):
        page_vision_layout = self.findChild(QObject, "pageVisionLayout")
        page_motion_layout = self.findChild(QObject, "pageMotionLayout")
        if page_vision_layout is None:
            return
        camera_box = self.ui.groupBox_6
        wave_box = self.ui.groupBox_11
        feedback_box = self.ui.groupBox_feedback
        for layout in (page_motion_layout, page_vision_layout):
            if layout is None:
                continue
            for w in (camera_box, wave_box, feedback_box):
                try:
                    layout.removeWidget(w)
                except Exception:
                    pass
        splitter = QSplitter(Qt.Vertical, self.ui.page_vision)
        splitter.setObjectName("monitor_splitter")
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(10)
        splitter.setOpaqueResize(False)
        splitter.setStyleSheet(
            "QSplitter::handle:vertical { background-color: rgba(32,164,243,110);"
            " margin: 4px 140px; border-radius: 3px; }")
        for w in (camera_box, wave_box, feedback_box):
            w.setMinimumHeight(150)
            splitter.addWidget(w)
        self.ui.camera_label.setMinimumHeight(110)
        self.ui.camera_label_2.setMinimumHeight(110)
        self.ui.wave_plot.setMinimumHeight(110)
        self.ui.motor_feedback_table.setMinimumHeight(90)
        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 4)
        splitter.setStretchFactor(2, 3)
        page_vision_layout.addWidget(splitter)
        splitter.setSizes([330, 200, 170])

    # =====================================================================
    # 首页监控大屏：顶部状态条 + 视频主视觉 + 设备状态灯行
    # =====================================================================
    def _build_monitor_dashboard(self):
        page_vision_layout = self.findChild(QObject, "pageVisionLayout")
        if page_vision_layout is None:
            return
        self._strip_labels = {}
        self._status_labels = {}

        # ---- 顶部状态条 ----
        strip = QFrame(self.ui.page_vision)
        strip.setObjectName("dashStrip")
        strip.setStyleSheet(_STRIP_STYLE)
        row = QHBoxLayout(strip)
        row.setContentsMargins(12, 6, 12, 6)
        row.setSpacing(16)

        def metric(text, key=None, width=150):
            lbl = QLabel(text, strip)
            lbl.setStyleSheet(_STRIP_LABEL)
            lbl.setMinimumWidth(width)
            row.addWidget(lbl)
            if key:
                self._strip_labels[key] = lbl
            return lbl

        metric("水下清洁机器人 · 自主清洁系统", "model", 250)
        metric("主控 --", "conn", 210)
        metric("时间 --:--:--", "time", 150)
        metric("运行 --", "run", 170)
        metric("帧率 --", "fps", 110)
        metric("指令 --", "cmd", 110)
        # 遥测占位（待接入下位机上报）
        for t in ("深度", "姿态", "温度"):
            metric("{} --（待接入）".format(t), t, 140)
        row.addStretch(1)
        page_vision_layout.insertWidget(0, strip)

        # ---- 设备状态灯行 ----
        status_row = QFrame(self.ui.page_vision)
        status_row.setObjectName("statusRow")
        status_row.setStyleSheet(_STRIP_LABEL)
        srow = QHBoxLayout(status_row)
        srow.setContentsMargins(12, 6, 12, 6)
        srow.setSpacing(12)
        for name, key in (("左推进", "left"), ("右推进", "right"), ("舵机", "servo"),
                          ("清洗刷", "brush"), ("照明", "light")):
            lbl = QLabel("● {}".format(name), status_row)
            lbl.setStyleSheet("color:#5b7a99; font-size:13px; font-weight:700;")
            srow.addWidget(lbl)
            self._status_labels[key] = lbl
        srow.addStretch(1)
        page_vision_layout.addWidget(status_row)

    # =====================================================================
    # 主控连接（单卡片）
    # =====================================================================
    def _setup_connection_panel(self):
        try:
            self.ui.groupBox.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        except Exception:
            pass

        card = QFrame(self.ui.groupBox)
        card.setObjectName("portCard")
        card.setStyleSheet(_CARD_STYLE)
        card.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        lay = QVBoxLayout(card)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(8)

        row1 = QHBoxLayout()
        row1.setSpacing(6)
        lab_name = QLabel("主控节点 91", card)
        lab_name.setFixedWidth(90)
        lab_name.setStyleSheet(
            "color:#9fd0ff; font-size:13px; font-weight:700; background:transparent;")
        self.ip_edit = QLineEdit(self.config.robot_ip)
        self.ip_edit.setPlaceholderText("192.168.1.91")
        self.port_edit = QLineEdit(str(self.config.robot_port))
        self.port_edit.setPlaceholderText(str(self.config.robot_port) or "12345")
        self.port_edit.setFixedWidth(70)
        row1.addWidget(lab_name)
        row1.addWidget(self.ip_edit, 1)
        row1.addWidget(self.port_edit)

        row2 = QHBoxLayout()
        row2.setSpacing(8)
        self.btn_connect = QPushButton("连 接")
        self.btn_disconnect = QPushButton("断 开")
        self.btn_disconnect.setEnabled(False)
        for b in (self.btn_connect, self.btn_disconnect):
            b.setMinimumHeight(22)
        self.btn_connect.clicked.connect(self.connect_robot)
        self.btn_disconnect.clicked.connect(self.disconnect_robot)
        row2.addWidget(self.btn_connect)
        row2.addWidget(self.btn_disconnect)

        note = QLabel("89 灯光从控由 91 自动转发，无需另行连接")
        note.setStyleSheet("color:#6f9dc4; font-size:11px; background:transparent;")
        note.setWordWrap(True)
        lay.addLayout(row1)
        lay.addLayout(row2)
        lay.addWidget(note)
        self.ui.verticalLayout_3.addWidget(card)

    # =====================================================================
    # 作业控制台
    # =====================================================================
    def _build_robot_console(self):
        page_motion_layout = self.findChild(QObject, "pageMotionLayout")
        if page_motion_layout is None:
            return
        self.ui.load_pro_config.setText("参数复位")
        self.ui.enable_motor.setText("启动运行（10）")
        self.ui.disable_motor.setText("停止待机（00）")
        self.ui.label_4.setText("PWM值")
        self.ui.var_speed.setText("150")
        self.ui.var_speed.setPlaceholderText("0~255")
        self.ui.setSpeed.setText("应用速度")
        self.ui.pause_m.setText("急 停")
        self.ui.continue_m.setText("推进停止")
        self.ui.label_4.setFixedWidth(84)
        self.ui.var_speed.setMaximumWidth(130)
        self.ui.setSpeed.setMaximumWidth(160)

        bottom = QHBoxLayout()
        bottom.setSpacing(14)
        bottom.addWidget(self._panel_servo())
        bottom.addWidget(self._panel_brush())
        bottom.addWidget(self._panel_light())
        page_motion_layout.addLayout(bottom)

    def _panel_servo(self):
        box = QGroupBox("舵机姿态")
        box.setObjectName("servoBox")
        box.setStyleSheet(_GROUP_STYLE)
        lay = QVBoxLayout(box)
        self.servo_slider = QSlider(Qt.Horizontal)
        self.servo_slider.setRange(0, 180)
        self.servo_slider.setValue(90)
        self.servo_slider.setTickInterval(15)
        self.servo_slider.setTickPosition(QSlider.TicksBelow)
        row = QHBoxLayout()
        lab_ang = QLabel("角度")
        lab_ang.setFixedWidth(64)
        row.addWidget(lab_ang)
        self.servo_spin = QSpinBox()
        self.servo_spin.setRange(0, 180)
        self.servo_spin.setValue(90)
        row.addWidget(self.servo_spin)
        btn = QPushButton("下发舵机")
        row.addWidget(btn)
        self.servo_slider.valueChanged.connect(self.servo_spin.setValue)
        self.servo_spin.valueChanged.connect(self.servo_slider.setValue)
        self.servo_spin.valueChanged.connect(self._on_servo_changed)
        btn.clicked.connect(self.send_plan)
        lay.addWidget(self.servo_slider)
        lay.addLayout(row)
        lay.insertStretch(0, 1)
        lay.addStretch(1)
        return box

    def _panel_brush(self):
        box = QGroupBox("清洗刷（3路）")
        box.setObjectName("brushBox")
        box.setStyleSheet(_GROUP_STYLE)
        lay = QVBoxLayout(box)
        row = QHBoxLayout()
        self.btn_brush1 = QPushButton("刷1")
        self.btn_brush2 = QPushButton("刷2")
        self.btn_brush3 = QPushButton("刷3")
        for b in (self.btn_brush1, self.btn_brush2, self.btn_brush3):
            b.setCheckable(True)
            row.addWidget(b)
        self.btn_brush1.toggled.connect(self._on_brush_changed)
        self.btn_brush2.toggled.connect(self._on_brush_changed)
        self.btn_brush3.toggled.connect(self._on_brush_changed)
        lay.addLayout(row)
        row2 = QHBoxLayout()
        lab_bs = QLabel("转速%")
        lab_bs.setFixedWidth(64)
        row2.addWidget(lab_bs)
        self.brush_speed_spin = QSpinBox()
        self.brush_speed_spin.setRange(0, 100)
        self.brush_speed_spin.setValue(60)
        row2.addWidget(self.brush_speed_spin)
        btn = QPushButton("下发刷子")
        btn.clicked.connect(self.send_plan)
        row2.addWidget(btn)
        self.brush_speed_spin.valueChanged.connect(self._on_brush_speed_changed)
        lay.addLayout(row2)
        lay.insertStretch(0, 1)
        lay.addStretch(1)
        return box

    def _panel_light(self):
        box = QGroupBox("LED 照明")
        box.setObjectName("lightBox")
        box.setStyleSheet(_GROUP_STYLE)
        lay = QVBoxLayout(box)
        self.light_slider = QSlider(Qt.Horizontal)
        self.light_slider.setRange(0, 100)
        self.light_slider.setValue(0)
        self.light_slider.setTickInterval(10)
        self.light_slider.setTickPosition(QSlider.TicksBelow)
        row = QHBoxLayout()
        lab_light = QLabel("亮度")
        lab_light.setFixedWidth(64)
        row.addWidget(lab_light)
        self.light_spin = QSpinBox()
        self.light_spin.setRange(0, 100)
        row.addWidget(self.light_spin)
        btn_off = QPushButton("关灯")
        btn_on = QPushButton("全亮")
        row.addWidget(btn_off)
        row.addWidget(btn_on)
        self.light_slider.valueChanged.connect(self.light_spin.setValue)
        self.light_spin.valueChanged.connect(self.light_slider.setValue)
        self.light_spin.valueChanged.connect(self._on_light_changed)
        btn_off.clicked.connect(lambda: self._set_light(0))
        btn_on.clicked.connect(lambda: self._set_light(100))
        lay.addWidget(self.light_slider)
        lay.addLayout(row)
        lay.insertStretch(0, 1)
        lay.addStretch(1)
        return box

    # =====================================================================
    # 指令调试栏
    # =====================================================================
    def _build_debug_bar(self):
        page_debug_layout = self.findChild(QObject, "pageDebugLayout")
        if page_debug_layout is None:
            return
        bar = QHBoxLayout()
        hint = QLabel("手动指令（10 90 1,-1 180,180 101 60 80；0000=断开）")
        hint.setStyleSheet("color:#8fc8ff;")
        hint.setWordWrap(False)
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("10 90 1,-1 180,180 101 60 80")
        btn_send = QPushButton("发送")
        btn_clear = QPushButton("清空日志")
        btn_send.clicked.connect(self._send_manual_cmd)
        btn_clear.clicked.connect(self._clear_logs)
        self.cmd_input.returnPressed.connect(self._send_manual_cmd)
        bar.addWidget(hint, 1)
        bar.addWidget(self.cmd_input, 3)
        bar.addWidget(btn_send)
        bar.addWidget(btn_clear)
        page_debug_layout.insertLayout(0, bar)

    # =====================================================================
    # 状态表
    # =====================================================================
    def _configure_status_table(self):
        table = self.ui.motor_feedback_table
        headers = ["作业模式", "舵机°", "左推进(方向/PWM)", "右推进(方向/PWM)",
                   "刷子状态", "刷速%", "灯光%", "状态"]
        table.setRowCount(1)
        table.setColumnCount(len(headers))
        table.setHorizontalHeaderLabels(headers)
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setEditTriggers(table.NoEditTriggers)
        table.setSelectionMode(table.NoSelection)

    # =====================================================================
    # 信号 / 导航
    # =====================================================================
    def _bind_signals(self):
        self.ui.enable_motor.clicked.connect(self.start_job)
        self.ui.disable_motor.clicked.connect(self.stop_job)
        self.ui.load_pro_config.clicked.connect(self.reset_plan)
        self.ui.setSpeed.clicked.connect(self.apply_speed_from_ui)
        self.ui.var_speed.returnPressed.connect(self.apply_speed_from_ui)
        self.ui.continue_m.clicked.connect(self.stop_thrusters)
        self.ui.pause_m.clicked.connect(self.emergency_stop)
        self.ui.goForward.clicked.connect(lambda: self._motion("forward"))
        self.ui.backForward.clicked.connect(lambda: self._motion("backward"))
        self.ui.btn_left.clicked.connect(lambda: self._motion("left"))
        self.ui.btn_right.clicked.connect(lambda: self._motion("right"))
        self.ui.open_cam_btn.clicked.connect(self.toggle_video)
        self.ui.open_cam_btn_3.clicked.connect(self.save_snapshot)

    def _setup_navigation(self):
        self.ui.nav_motion.setText("首页监控")
        self.ui.nav_vision.setText("作业控制台")
        self.ui.nav_debug.setText("指令调试")
        self.ui.nav_motion.clicked.connect(lambda: self.switch_page(1))
        self.ui.nav_vision.clicked.connect(lambda: self.switch_page(0))
        self.ui.nav_debug.clicked.connect(lambda: self.switch_page(2))
        self.switch_page(1)

    def _apply_hero(self, index):
        title, hint = self.PAGE_HINTS.get(index, self.PAGE_HINTS[0])
        self.ui.heroTitle.setText(title)
        self.ui.heroHint.setText(hint)
        self.ui.heroHint.setStyleSheet(
            "font-size:13px; color:#8fb7d9; background:transparent; border:none;")

    def _flash_hero(self, text, ok=True):
        color = C_GOOD if ok else C_BAD
        self.ui.heroHint.setText(text)
        self.ui.heroHint.setStyleSheet(
            "font-size:13px; font-weight:700; color:%s; background:transparent;" % color)
        idx = self.ui.stackedWidget.currentIndex()
        QTimer.singleShot(6000, lambda: self._apply_hero(idx))

    def switch_page(self, index):
        self.ui.stackedWidget.setCurrentIndex(index)
        self._apply_hero(index)

    # =====================================================================
    # 日志
    # =====================================================================
    def _append_log(self, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.ui.recv_text.appendPlainText("[%s] %s" % (stamp, text))
        self.ui.recv_text.verticalScrollBar().setValue(
            self.ui.recv_text.verticalScrollBar().maximum())

    def log(self, text):
        self.log_signal.emit(text)
        self.logger.info(text)

    def log_warn(self, text):
        self.log_signal.emit("⚠ " + text)
        self.logger.warning(text)

    def log_error(self, text):
        self.log_signal.emit("✖ " + text)
        self.logger.error(text)

    def append_sent(self, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.ui.send_text.appendPlainText("[%s] %s" % (stamp, text))
        self.ui.send_text.verticalScrollBar().setValue(
            self.ui.send_text.verticalScrollBar().maximum())

    def _clear_logs(self):
        self.ui.send_text.clear()
        self.ui.recv_text.clear()
        self.log("日志已清空")

    # =====================================================================
    # 连接管理
    # =====================================================================
    def _open_socket(self, ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.5)
        sock.connect((ip, int(port)))
        return sock

    def connect_robot(self, silent=False):
        ip = self.ip_edit.text().strip() or proto.DEFAULT_IP
        port = self.port_edit.text().strip() or str(proto.DEFAULT_PORT)
        if self._connected:
            return True
        if not silent:
            self.log("正在连接 主控节点 {}:{} ...".format(ip, port))
        try:
            sock = self._open_socket(ip, port)
        except Exception as exc:
            if not silent:
                self.log_error("连接失败：{}".format(exc))
                self._flash_hero("✖ 连接失败：{}".format(exc), ok=False)
                QMessageBox.warning(
                    self, "连接失败",
                    "无法连接 91 主控 {}:{}\n\n原因：{}\n\n请检查：\n"
                    "1) 地址/端口是否正确（真机默认 192.168.1.91:12345）\n"
                    "2) 树莓派91主控服务/模拟服务器是否已启动\n"
                    "3) 电脑与设备是否同网段（电脑建议 192.168.1.99）".format(ip, port, exc))
            self._refresh_connection_state()
            return False
        self._sock = sock
        self._connected = True
        self._stop_reconnect()
        if not silent:
            self.log("已连接 91 主控 {}:{}（视频通道就绪）".format(ip, port))
            self.log("指令格式：10 90 1,-1 180,180 101 60 80（MODE 角度 左右轮 刷 速 灯）")
            self._flash_hero("✔ 已连接 {}:{}".format(ip, port), ok=True)
        self.config.update_robot(ip, port)
        self._refresh_connection_state()
        # 连接成功并收到视频帧后自动打开显示（展会/现场演示更省事）
        self._auto_open_video = True
        self._video_auto_logged = False
        self._start_receiver()
        self.reset_plan(send=True)   # 连接后回安全状态
        return True

    def disconnect_robot(self):
        self._closing = True
        try:
            sock, self._sock = self._sock, None
            was = self._connected
            self._connected = False
            self._stop_receiver()
            if sock:
                try:
                    sock.shutdown(2)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
            if was:
                self.log("已断开 91 主控连接")
            self._refresh_connection_state()
        finally:
            self._closing = False
        self._stop_reconnect()

    # ---------------- 自动重连 ----------------
    def _start_reconnect(self):
        if self._closing or self._connected:
            return
        if not self.config.auto_reconnect:
            self.log_warn("连接已断开（自动重连未启用，请手动连接）")
            return
        if self._reconnect_active:
            return
        self._reconnect_active = True
        self._reconnect_attempts = 0
        interval = self.config.get("system", "reconnect_interval")
        self.reconnect_timer.start(int(interval or 3) * 1000)
        self.log("连接意外中断，开始自动重连（每 {}s 一次）".format(interval or 3))

    def _stop_reconnect(self):
        self._reconnect_active = False
        self._reconnect_attempts = 0
        self.reconnect_timer.stop()

    def _check_reconnect(self):
        if self._connected or self._closing:
            self._stop_reconnect()
            return
        max_attempts = self.config.get("system", "max_reconnect_attempts") or 10
        if self._reconnect_attempts >= max_attempts:
            self._stop_reconnect()
            self.log_warn("自动重连已达最大次数({})，已停止，请检查网络后手动连接".format(max_attempts))
            return
        self._reconnect_attempts += 1
        attempts = self._reconnect_attempts
        ok = self.connect_robot(silent=True)
        if ok:
            self.log("自动重连成功（第 {} 次）".format(attempts))
            self._stop_reconnect()
        elif attempts % 5 == 0:
            self.log_warn("自动重连已尝试 {} 次仍未成功，请检查主控节点是否在线".format(attempts))

    # ---------------- 视频接收 ----------------
    def _start_receiver(self):
        if self._receiver is not None or self._sock is None:
            return
        recv = FrameReceiver(self._sock)
        recv.frame_ready.connect(self.on_frame)
        recv.broken.connect(self._on_receiver_broken)
        recv.start()
        self._receiver = recv

    def _stop_receiver(self):
        if self._receiver is not None:
            recv = self._receiver
            self._receiver = None
            try:
                recv.stop()
                recv.wait(2000)
            except Exception:
                pass

    def _on_receiver_broken(self, reason):
        if self._closing or not self._connected:
            return
        self._receiver = None
        detail = reason if not str(reason).endswith("已退出") else "对端关闭或网络中断"
        self.log_warn("91 主控通道断开：{}".format(detail))
        self._connected = False
        self._refresh_connection_state()
        try:
            if self._sock:
                self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._start_reconnect()

    def _refresh_connection_state(self):
        self.btn_connect.setEnabled(not self._connected)
        self.btn_disconnect.setEnabled(self._connected)
        self._set_status_badge(self._connected)
        self.ui.statusBadge.setText("已连接" if self._connected else "未连接")

    def _set_status_badge(self, ok):
        try:
            color = C_GOOD if ok else C_BAD
            self.ui.statusBadge.setStyleSheet(
                "color:%s; font-size:15px; font-weight:800;" % color)
        except Exception:
            pass

    # =====================================================================
    # 指令
    # =====================================================================
    def _pwm_from_ui(self):
        try:
            return proto.clamp(int(float(self.ui.var_speed.text())), 0, 255)
        except (TypeError, ValueError):
            return 150

    def _build_line(self):
        p = self.plan
        return proto.build_command(
            p["mode"], p["angle"], p["dir1"], p["spd1"], p["dir2"], p["spd2"],
            p["brush"], p["brush_speed"], p["light"])

    def send_plan(self, announce=True):
        line = self._build_line()
        if not self._connected or self._sock is None:
            self.log_warn("未连接，指令未发送：{}".format(line))
            return False
        with self._send_lock:
            try:
                self._sock.sendall((line + "\n").encode("utf-8"))
            except OSError as exc:
                self.log_error("发送失败：{}".format(exc))
                self._on_receiver_broken("send failed")
                return False
        self.append_sent(line)
        self._cmd_count += 1
        if announce:
            self.log(proto.describe_command(line))
        self._refresh_status_panel()
        return True

    def _raw_send(self, line):
        if not self._connected or self._sock is None:
            self.log_warn("未连接，指令未发送")
            return False
        with self._send_lock:
            try:
                self._sock.sendall((line + "\n").encode("utf-8"))
            except OSError as exc:
                self.log_error("发送失败：{}".format(exc))
                return False
        self.append_sent(line)
        self._cmd_count += 1
        self._refresh_status_panel()
        return True

    def _send_manual_cmd(self):
        text = self.cmd_input.text().strip()
        if not text:
            return
        if text == proto.CMD_DISCONNECT:
            self.log("手动指令：断开连接(0000)")
            self.disconnect_robot()
            self.cmd_input.clear()
            return
        if proto.parse_command(text) is None:
            self.log_error("指令格式错误：{}".format(text))
            return
        self.cmd_input.clear()
        if not self._connected:
            self.log_warn("未连接，无法发送手动指令")
            return
        with self._send_lock:
            try:
                self._sock.sendall((text + "\n").encode("utf-8"))
            except OSError as exc:
                self.log_error("发送失败：{}".format(exc))
                return
        self.append_sent(text)
        self._cmd_count += 1
        data = proto.parse_command(text)
        if data and data["type"] == "command":
            for k in ("mode", "angle", "dir1", "spd1", "dir2", "spd2",
                      "brush_state", "brush_speed", "light"):
                if k in data:
                    self.plan[k] = data[k]
            self._sync_controls_from_plan()
            self._refresh_status_panel()
            self.log(proto.describe_command(text))

    # =====================================================================
    # 控制动作
    # =====================================================================
    def start_job(self):
        self.plan["mode"] = proto.MODE_START
        self.log("启动运行（10）：开始采集并运行")
        self.send_plan()

    def stop_job(self):
        self.plan["mode"] = proto.MODE_STOP
        for k in ("dir1", "spd1", "dir2", "spd2"):
            self.plan[k] = 0
        self.plan["brush"] = "000"
        self.plan["brush_speed"] = 0
        self._sync_controls_from_plan()
        self.log("停止待机（00）：停止采集，执行机构全部复位")
        self.send_plan()

    def reset_plan(self, send=True):
        self.plan.update({
            "mode": proto.MODE_STOP, "angle": 90,
            "dir1": 0, "spd1": 0, "dir2": 0, "spd2": 0,
            "brush": "000", "brush_speed": 0, "light": 0})
        self._sync_controls_from_plan()
        if send:
            self.log("参数复位并发送安全指令")
            self.send_plan()

    def emergency_stop(self):
        self.reset_plan(send=False)
        line = proto.emergency_stop_command()
        self.log("！！！ 急停触发（{}）".format(line))
        self._raw_send(line)

    def apply_speed_from_ui(self):
        spd = self._pwm_from_ui()
        if self.plan["dir1"] != 0 or self.plan["dir2"] != 0:
            self.plan["spd1"] = spd
            self.plan["spd2"] = spd
            self.log("推进速度更新为 PWM={}（按当前方向立即下发）".format(spd))
        else:
            self.log("推进 PWM 预设 = {}（等待方向操作）".format(spd))
        self.send_plan()

    def stop_thrusters(self):
        self.plan["dir1"] = 0
        self.plan["dir2"] = 0
        self.plan["spd1"] = 0
        self.plan["spd2"] = 0
        self.log("推进停止（方向置0，速度置0）")
        self.send_plan()

    def _motion_pair(self, action):
        mirror = self._thruster_layout == "mirror"
        if action == "forward":
            return (-1, 1) if mirror else (1, 1)
        if action == "backward":
            return (1, -1) if mirror else (-1, -1)
        if action == "left":
            return (-1, -1) if mirror else (-1, 1)
        if action == "right":
            return (1, 1) if mirror else (1, -1)
        return (0, 0)

    def _motion(self, action):
        spd = self._pwm_from_ui()
        dir1, dir2 = self._motion_pair(action)
        self.plan.update(dir1=dir1, dir2=dir2, spd1=spd, spd2=spd)
        names = {"forward": "前进", "backward": "后退", "left": "左转", "right": "右转"}
        layout = "镜像安装" if self._thruster_layout == "mirror" else "同向安装"
        self.log("运动：{}（{}，dir=({},{})，PWM={}）".format(
            names.get(action, action), layout, dir1, dir2, spd))
        self.send_plan()

    def _on_servo_changed(self, value):
        if self._syncing:
            return
        self.plan["angle"] = value
        self._throttle_send()

    def _on_brush_changed(self, _checked=False):
        if self._syncing:
            return
        self.plan["brush"] = "".join(
            "1" if b.isChecked() else "0"
            for b in (self.btn_brush1, self.btn_brush2, self.btn_brush3))
        self._throttle_send()

    def _on_brush_speed_changed(self, value):
        if self._syncing:
            return
        self.plan["brush_speed"] = float(value)
        self._throttle_send()

    def _on_light_changed(self, value):
        if self._syncing:
            return
        self.plan["light"] = value
        self._throttle_send()

    def _set_light(self, value):
        self.plan["light"] = value
        self.light_spin.setValue(value)
        self.log("灯光亮度设为 {}%（由 91 转发给 89 灯光从控）".format(value))
        self.send_plan()

    def _throttle_send(self):
        if not self.ctrl_timer.isActive():
            self.ctrl_timer.start()

    def _sync_controls_from_plan(self):
        self._syncing = True
        try:
            self.servo_slider.setValue(self.plan["angle"])
            self.servo_spin.setValue(self.plan["angle"])
            self.btn_brush1.setChecked(self.plan["brush"][0] == "1")
            self.btn_brush2.setChecked(self.plan["brush"][1] == "1")
            self.btn_brush3.setChecked(self.plan["brush"][2] == "1")
            self.brush_speed_spin.setValue(int(self.plan["brush_speed"]))
            self.light_slider.setValue(self.plan["light"])
            self.light_spin.setValue(self.plan["light"])
            self.ui.var_speed.setText(str(self.plan["spd1"] or 150))
        finally:
            self._syncing = False

    # =====================================================================
    # 视频
    # =====================================================================
    def on_frame(self, bgr):
        now = time.time()
        dt = now - self._last_frame_t
        self._last_frame_t = now
        if 0.001 < dt < 1.0:
            self._fps_est = 0.7 * self._fps_est + 0.3 * (1.0 / dt)
        self._last_frame = bgr
        if not self._video_visible:
            if self._auto_open_video and not self._video_auto_logged:
                # 收到第一帧后自动打开显示
                self._video_visible = True
                self._video_auto_logged = True
                self.ui.open_cam_btn.setText("关闭视频显示")
                self.log("已自动打开视频显示（收到 91 主控视频帧）")
            else:
                return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg).scaled(
            self.ui.camera_label.width(), self.ui.camera_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.ui.camera_label.setPixmap(pix)

    def toggle_video(self):
        if not self._connected:
            self.log_warn("请先连接 91 主控，再打开视频通道")
            return
        self._video_visible = not self._video_visible
        if self._video_visible:
            self._auto_open_video = False
            self._video_auto_logged = True
            self.ui.open_cam_btn.setText("关闭视频显示")
            self.log("视频显示已开启（帧流来自 91 主控，4字节长度+JPEG）")
            self.ui.camera_label.setText("等待视频帧...")
            if self._last_frame is not None:
                self.on_frame(self._last_frame)
        else:
            self._auto_open_video = False
            self.ui.open_cam_btn.setText("打开视频显示")
            self.ui.camera_label.clear()
            self.ui.camera_label.setText("水下视频画面")
            self.log("视频显示已关闭（后台仍在接收）")

    def save_snapshot(self):
        if self._last_frame is None:
            self.log_warn("当前没有可保存的视频帧")
            return
        os.makedirs(self._shot_dir, exist_ok=True)
        path = os.path.join(self._shot_dir,
                            datetime.now().strftime("snapshot_%Y%m%d_%H%M%S.png"))
        cv2.imwrite(path, self._last_frame)
        self.log("截图已保存：{}".format(path))

    # =====================================================================
    # 波形 / 状态
    # =====================================================================
    def _init_plot(self):
        self._curve = self.ui.wave_plot.plot(
            pen=pg.mkPen(C_ACCENT, width=2), name="PWM")
        self.ui.wave_plot.setLabel("left", "推进 PWM")
        self.ui.wave_plot.setLabel("bottom", "采样")
        self.ui.wave_plot.setYRange(-260, 260)
        self.ui.wave_plot.showGrid(x=True, y=True, alpha=0.25)

    def update_wave(self):
        p = self.plan
        value = float(p["spd1"]) * (1 if p["dir1"] >= 0 else -1)
        self._wave_data.append(value)
        if len(self._wave_data) > 100:
            self._wave_data.pop(0)
        self._curve.setData(self._wave_data)
        sec = int(time.time())
        if sec != self._last_stats_sec:
            self._last_stats_sec = sec
            self._refresh_status_panel()

    def _refresh_status_panel(self):
        p = self.plan
        mode = "运行(10)" if p["mode"] == proto.MODE_START else "待机(00)"
        d1 = {1: "正", -1: "反", 0: "停"}[p["dir1"]]
        d2 = {1: "正", -1: "反", 0: "停"}[p["dir2"]]
        row = [mode, str(p["angle"]),
               "%s/%s" % (d1, p["spd1"]), "%s/%s" % (d2, p["spd2"]),
               p["brush"], str(int(p["brush_speed"])), str(p["light"]),
               "已发送" if self._connected else "未连接"]
        table = self.ui.motor_feedback_table
        for col, text in enumerate(row):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignCenter)
            table.setItem(0, col, item)
        ip = self.ip_edit.text().strip() or "192.168.1.91"
        uptime = int(time.time() - self._boot_time)
        stat = (
            "节点与运行监测\n\n"
            "  主控节点：[{}] {}:{}（89 灯光从控由 91 转发）\n\n"
            "  视频帧率：约 {:.0f} fps\n"
            "  已发指令：{}\n"
            "  运行时长：{}分{}秒\n\n"
            "最近指令状态\n\n"
            "  模式：{}\n  舵机：{}°\n  左轮：{} PWM{}\n"
            "  右轮：{} PWM{}\n  刷子：{} @{}%\n  灯光：{}%（→89）"
        ).format(
            "在线" if self._connected else "离线", ip,
            self.port_edit.text().strip() or "12345",
            self._fps_est, self._cmd_count,
            uptime // 60, uptime % 60,
            mode, p["angle"], d1, p["spd1"], d2, p["spd2"],
            p["brush"], int(p["brush_speed"]), p["light"])
        self.ui.camera_label_2.setText(stat)
        self.ui.camera_label_2.setStyleSheet(
            "color:#dcefff; font-size:13px; padding:10px 14px;"
            "background-color: rgba(9,23,38,140); border-radius:10px;"
            "border:1px solid rgba(132,188,249,50);")
        self._refresh_dashboard_strip(uptime)

    def _separator(self, s):
        return "{}分{:02d}秒".format(s // 60, s % 60)

    def _refresh_dashboard_strip(self, uptime):
        try:
            lbl = self._strip_labels
            lbl["conn"].setText(
                "主控 [{}] {}:{}".format(
                    "在线" if self._connected else "离线",
                    self.ip_edit.text().strip() or "192.168.1.91",
                    self.port_edit.text().strip() or "12345"))
            lbl["time"].setText("时间 " + datetime.now().strftime("%H:%M:%S"))
            lbl["run"].setText("运行 " + self._separator(uptime))
            lbl["fps"].setText("帧率 {:.0f}".format(self._fps_est))
            lbl["cmd"].setText("指令 {}".format(self._cmd_count))
        except Exception:
            pass
        # 设备状态灯（基于当前真实指令状态）
        active_run = self._connected and self.plan["mode"] == proto.MODE_START

        def set_lamp(key, on, text, color=None):
            lbl = self._status_labels.get(key)
            if lbl is None:
                return
            lbl.setText("● {}".format(text))
            lbl.setStyleSheet(
                "color:%s; font-size:13px; font-weight:700;" % (color or (C_GOOD if on else "#5b7a99")))

        d1 = self.plan["dir1"]
        d2 = self.plan["dir2"]
        set_lamp("left", active_run and d1 != 0,
                 "左推进 {}".format({1: "正转" + str(self.plan["spd1"]),
                                     -1: "反转" + str(self.plan["spd1"]),
                                     0: "停止"}[d1]))
        set_lamp("right", active_run and d2 != 0,
                 "右推进 {}".format({1: "正转" + str(self.plan["spd2"]),
                                     -1: "反转" + str(self.plan["spd2"]),
                                     0: "停止"}[d2]))
        set_lamp("servo", active_run, "舵机 {}°".format(self.plan["angle"]))
        brush_on = active_run and self.plan["brush"] != "000"
        set_lamp("brush", brush_on, "清洗刷 {}".format(self.plan["brush"]))
        set_lamp("light", active_run and self.plan["light"] > 0,
                 "照明 {}%".format(self.plan["light"]))

    # =====================================================================
    # 收尾
    # =====================================================================
    def closeEvent(self, event):
        try:
            self.wave_timer.stop()
            self.ctrl_timer.stop()
            if self.plan["mode"] == proto.MODE_START:
                self.emergency_stop()
            self.disconnect_robot()
            self.logger.info("上位机退出")
            self.logger.close()
        except Exception:
            pass
        super(MainWindow, self).closeEvent(event)


_GROUP_STYLE = """
QGroupBox {
    color: #9fd0ff;
    font-size: 14px;
    font-weight: 700;
    border: 1px solid rgba(132,188,249,60);
    border-radius: 12px;
    background-color: rgba(10,26,42,170);
    margin-top: 12px;
    padding-top: 10px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
    color: #b9e0ff;
}
QSlider::groove:horizontal {
    height: 6px; background: rgba(132,188,249,70); border-radius: 3px;
}
QSlider::handle:horizontal {
    width: 16px; margin: -6px 0; border-radius: 8px;
    background: #20a4f3; border: 1px solid #59c2ff;
}
QSpinBox, QPushButton, QLineEdit {
    background-color: rgba(16,38,60,220);
    color: #eaf5ff;
    border: 1px solid rgba(132,188,249,80);
    border-radius: 8px;
    padding: 4px 8px;
}
QPushButton:hover { border-color: #20a4f3; }
QPushButton:checked { background-color: #20a4f3; color: #05131f; font-weight: 700; }
QPushButton:disabled { color: #567; border-color: rgba(132,188,249,30); }
"""

_CARD_STYLE = """
QFrame#portCard {
    background-color: rgba(13,31,49,196);
    border: 1px solid rgba(132,188,249,48);
    border-radius: 10px;
}
QFrame#portCard QLineEdit {
    background-color: rgba(16,38,60,220);
    color: #eaf5ff;
    border: 1px solid rgba(132,188,249,80);
    border-radius: 6px;
    padding: 2px 6px;
    min-height: 20px;
    font-size: 13px;
}
QFrame#portCard QLabel { color: #8fc8ff; font-size: 12px; background: transparent; }
QFrame#portCard QPushButton {
    background-color: rgba(21,50,77,210);
    color: #d9ecff;
    border: 1px solid rgba(132,188,249,80);
    border-radius: 6px;
    padding: 1px 6px;
    font-size: 13px;
    min-height: 20px;
    max-height: 26px;
}
QFrame#portCard QPushButton:hover { border-color: #20a4f3; }
QFrame#portCard QPushButton:disabled { color: #567; }
"""

_STRIP_STYLE = """
QFrame#dashStrip, QFrame#statusRow {
    background-color: rgba(10,26,42,180);
    border: 1px solid rgba(132,188,249,50);
    border-radius: 10px;
}
"""

_STRIP_LABEL = """
color:#cfeeff; font-size:13px; font-weight:700; background:transparent;
"""


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
