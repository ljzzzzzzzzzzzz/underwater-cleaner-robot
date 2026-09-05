# -*- coding: utf-8 -*-
"""
dashboard.py —— 单页大屏版主界面（按用户提供“水下机器人智能控制系统”界面规范实现）

布局：顶部标题栏 / 左侧导航+系统状态 / 主区面板
  A 实时画面(视频+OSD+深度条)   B 设备状态(俯视示意+数据列表)
  C 运动控制(虚拟摇杆/按键)     D 任务信息   E 导航路径图
  F 传感器数据面板              G 推进器输出柱图 + 告警信息流

数据说明：连接/指令/视频为真实（主控 192.168.1.91:12345）；
深度/航向/姿态/舱内环境等遥测目前无下位机上报源，界面统一显示「模拟」
并在右下告警区记录来源，接入真值只需替换 SimTelemetry。
"""
from __future__ import annotations

import math
import os
import random
import socket
import struct
import sys
import threading
import time
from datetime import datetime

os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide2")

import numpy as np
import pyqtgraph as pg
import cv2
from PySide2.QtCore import QSize, Qt, QRectF, QTimer, Signal
from PySide2.QtGui import (QColor, QFont, QImage, QLinearGradient, QPainter,
                           QPen, QPixmap, QRadialGradient)
from PySide2.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSlider,
    QSpinBox,
    QSizePolicy,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

import robot_protocol as proto
from logger import Logger
from config_manager import ConfigManager
from underwater_window import FrameReceiver  # 复用 4字节+JPEG 帧接收线程

# ---------------- 调色规范 ----------------
BG0 = "#0B132B"
BG1 = "#14213D"
PANEL = "#101B33"          # 面板底色
PANEL_EDGE = "#2A3A60"
TXT = "#FFFFFF"
TXT_SUB = "#8FA9C9"
GREEN = "#3DDC97"
CYAN = "#31C4F3"
YELLOW = "#FFD166"
RED = "#FF5A5F"
BLUE = "#4D96FF"

SYS = {  # 模拟遥测（接入真值后替换 SimTelemetry.tick）
    "depth_m": 25.6, "yaw_deg": 128.7, "pitch_deg": -3.2, "roll_deg": 1.6,
    "water_temp": 12.4, "voltage": 48.5, "battery": 78.0, "cabin_temp": 23.6,
    "humidity": 45.0, "storage_gb": 256.0, "dvl": 0.36, "mag_ut": 128.4,
    "turbidity_ntu": 0.42, "motor_temp": 32.6,
}
SIM_NOTE = "（模拟/待接入真值）"
_ZERO = {k: 0.0 for k in SYS}
# 展会默认“归零/中性”；需跳动演示时打开“动态演示数值”开关
_ZERO["storage_gb"] = 0.0
_ZERO["battery"] = 0.0


class SimTelemetry(object):
    """展示用模拟遥测：缓慢随机游走。接入下位机后删除本类并改从协议解析。"""

    def __init__(self):
        self.v = dict(SYS)
        self._targets = {k: v for k, v in SYS.items()}

    def tick(self):
        rnd = random.random
        for k in ("depth_m", "water_temp", "cabin_temp", "humidity",
                  "dvl", "mag_ut", "turbidity_ntu", "motor_temp"):
            t = self._targets[k]
            t += (rnd() - 0.5) * (1 if k in ("humidity",) else 0.8)
            self._targets[k] = t
            cur = self.v[k]
            self.v[k] = cur + (t - cur) * 0.15
        # 小幅漂移
        self.v["depth_m"] += (self._targets["depth_m"] - self.v["depth_m"]) * 0.1
        self.v["yaw_deg"] = (self.v["yaw_deg"] + (rnd() - 0.5) * 1.2) % 360.0
        self.v["pitch_deg"] = max(-90, min(90, self.v["pitch_deg"] + (rnd() - 0.5) * 0.6))
        self.v["roll_deg"] = max(-90, min(90, self.v["roll_deg"] + (rnd() - 0.5) * 0.6))
        # 电源电压/电量：按真实情况轻微随机（电压小幅浮动、电量缓慢下降）
        self.v["voltage"] = 48.5 + (rnd() - 0.5) * 0.3
        self.v["battery"] = max(30.0, min(100.0, self.v["battery"] - rnd() * 0.05))
        # 舱内温度/湿度小范围波动（更贴近实际）
        self.v["cabin_temp"] = max(18.0, min(32.0, self.v["cabin_temp"] + (rnd() - 0.5) * 0.3))
        self.v["humidity"] = max(30.0, min(80.0, self.v["humidity"] + (rnd() - 0.5) * 1.2))


class DepthGauge(QWidget):
    """自绘深度仪表：刻度(0~50m)+从下往上的填充+数值随深度对齐到刻度位置"""

    def __init__(self, parent=None):
        super(DepthGauge, self).__init__(parent)
        self._depth = 0.0
        self.setMinimumHeight(260)
        self.setFixedWidth(92)
        self.setStyleSheet("background:transparent;")

    def set_depth(self, d):
        self._depth = max(0.0, min(50.0, d))
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        # 轨道区（右侧 40px）
        bar_x = w - 40
        bar_w = 22
        top, bottom = 8, h - 20
        # 轨道
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#0a1428"))
        p.drawRoundedRect(QRectF(bar_x, top, bar_w, bottom - top), 5, 5)
        p.setBrush(QColor("#1b3d63"))
        p.setPen(QPen(QColor("#49c4f3"), 1))
        p.drawRoundedRect(QRectF(bar_x, top, bar_w, bottom - top), 5, 5)
        # 填充（从下往上，比例=深度/50）
        frac = self._depth / 50.0
        fill_h = (bottom - top) * frac
        if fill_h > 0:
            grad = QLinearGradient(bar_x, bottom, bar_x, bottom - fill_h)
            grad.setColorAt(0, QColor("#31c4f3"))
            grad.setColorAt(1, QColor("#9fe7ff"))
            p.setBrush(grad)
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(QRectF(bar_x, bottom - fill_h, bar_w, fill_h), 5, 5)
        # 刻度线 + 数值标签（0/10/20/30/40/50）
        p.setPen(QColor("#5b7a99"))
        for m in (0, 10, 20, 30, 40, 50):
            yy = bottom - (bottom - top) * (m / 50.0)
            p.drawLine(bar_x - 8, int(yy), bar_x - 2, int(yy))
            p.drawText(QRectF(0, yy - 7, 20, 14), Qt.AlignRight | Qt.AlignVCenter, str(m))
        # 当前深度值：统一贴到填充最上沿（0 时即刻度底部），值画在轨道右侧，避免与刻度重叠
        marker_y = max(top + 4, bottom - fill_h)
        p.setPen(QPen(QColor("#ffd166"), 2))
        p.drawLine(bar_x - 10, int(marker_y), bar_x + bar_w + 2, int(marker_y))
        p.setPen(QColor("#dcefff"))
        p.setFont(QFont("Consolas", 12, QFont.Bold))
        p.drawText(QRectF(bar_x + bar_w + 8, marker_y - 9, 34, 18),
                   Qt.AlignLeft | Qt.AlignVCenter, "%.1f" % self._depth)
        p.end()


def _panel(title, accent=CYAN):
    """通用面板：Fluent 风卡片 —— 圆角 + 柔和渐变 + 标题左侧accent条 + 细分隔线"""
    box = QFrame()
    box.setObjectName("dashPanel")
    box.setStyleSheet(
        "QFrame#dashPanel{"
        "background:qlineargradient(x1:0,y1:0,x2:0,y2:1,"
        "stop:0 rgba(28,42,78,242),stop:1 rgba(14,23,46,242));"
        "border:1px solid rgba(78,104,158,150);"
        "border-radius:14px;}")
    lay = QVBoxLayout(box)
    lay.setContentsMargins(14, 10, 14, 12)
    lay.setSpacing(6)

    # 标题行：彩色竖条 + 标题（Fluent 分区头）
    hrow = QHBoxLayout()
    hrow.setSpacing(8)
    bar = QFrame()
    bar.setFixedSize(4, 14)
    bar.setStyleSheet("background:%s; border:none; border-radius:2px;" % accent)
    head = QLabel(title)
    head.setStyleSheet(
        "color:%s; font-size:14px; font-weight:800; border:none; background:transparent;"
        "letter-spacing:1px;" % accent)
    hrow.addWidget(bar)
    hrow.addWidget(head, 1)
    lay.addLayout(hrow)

    # 细分隔线：标题与内容之间
    div = QFrame()
    div.setFixedHeight(1)
    div.setStyleSheet("background:rgba(90,116,168,70); border:none;")
    lay.addWidget(div)

    body = QVBoxLayout()
    body.setSpacing(6)
    lay.addLayout(body, 1)
    return box, body


def _mono(widget, on=True):
    """动态数值统一显式等宽字体（Consolas Bold + 字距），防止刷新抖动"""
    if on:
        f = QFont("Consolas", 11)
        f.setBold(True)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 1.0)
        widget.setFont(f)
    widget.setProperty("mono", "true" if on else "false")
    widget.style().unpolish(widget)
    widget.style().polish(widget)


ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")


def load_icon(name):
    """从 assets/icons 加载 SVG；文件缺失/插件缺失返回 None（调用方回落文字）"""
    from PySide2.QtGui import QIcon
    from PySide2.QtWidgets import QApplication as _App
    path = os.path.join(ASSET_DIR, "icons", name + ".svg")
    if not os.path.exists(path):
        return None
    icon = QIcon(path)
    # 探测能否真的渲染出像素（无 QtSvg 插件时 QIcon 无效）
    if _App.instance() and icon.pixmap(24, 24).isNull():
        return None
    return icon


def _kv(label, value="--", color=TXT_SUB, unit="", mono=False, icon=None):
    from PySide2.QtGui import QPixmap
    row = QHBoxLayout()
    row.setSpacing(8)
    if icon:
        ico = load_icon(icon)
        lbl_ico = QLabel()
        if ico is not None:
            lbl_ico.setPixmap(ico.pixmap(20, 20))
        row.addWidget(lbl_ico)
    lab = QLabel(label)
    lab.setStyleSheet("color:%s; font-size:12px; letter-spacing:0.5px;" % TXT_SUB)
    val = QLabel("%s%s" % (value, unit))
    val.setStyleSheet("color:%s; font-size:14px; font-weight:800;" % color)
    if mono:
        _mono(val)
    row.addWidget(lab)
    row.addStretch(1)
    row.addWidget(val)
    return row, val


class StatusLight(QWidget):
    """发光圆形状态灯（QPainter 绘制：外圈光晕 + 内芯）"""

    def __init__(self, color=GREEN, size=16, parent=None):
        super(StatusLight, self).__init__(parent)
        self._color = QColor(color)
        self.setFixedSize(size, size)
        self.setAttribute(Qt.WA_TranslucentBackground)

    def set_color(self, color):
        self._color = QColor(color)
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        center = w / 2.0
        r = w / 2.0 - 1
        # 光晕（多层半透明）
        glow = QRadialGradient(center, center, r * 2.2)
        c = self._color
        glow.setColorAt(0, QColor(c.red(), c.green(), c.blue(), 200))
        glow.setColorAt(0.35, QColor(c.red(), c.green(), c.blue(), 90))
        glow.setColorAt(1, QColor(c.red(), c.green(), c.blue(), 0))
        p.setBrush(glow)
        p.setPen(Qt.NoPen)
        p.drawEllipse(QRectF(center - r * 2.2, center - r * 2.2, r * 4.4, r * 4.4))
        # 芯
        grad = QRadialGradient(center - r * 0.35, center - r * 0.35, r)
        grad.setColorAt(0, QColor(255, 255, 255, 255))
        grad.setColorAt(0.3, c.lighter(140))
        grad.setColorAt(1, c.darker(120))
        p.setBrush(grad)
        edge = QColor(c.darker(180))
        edge.setAlpha(200)
        p.setPen(edge)
        p.drawEllipse(QRectF(center - r, center - r, r * 2, r * 2))
        p.end()


class LogView(QTextBrowser):
    """彩色分级日志：图标 + 时间戳 + 分级配色，自动滚底"""

    LEVELS = {
        "ok": (GREEN, "✅"), "info": (CYAN, "ℹ️"),
        "warn": (YELLOW, "⚠️"), "err": (RED, "❌"),
    }

    def __init__(self, parent=None):
        super(LogView, self).__init__(parent)
        self.setObjectName("alarm")
        self.setOpenExternalLinks(False)

    def append_log(self, level, message):
        color, icon = self.LEVELS.get(level, (TXT_SUB, "•"))
        stamp = datetime.now().strftime("%H:%M:%S")
        html = ("<span style='color:#6f8cb3'>[%s]</span> "
                "<span style='color:%s; font-weight:600'>%s %s</span><br/>"
                % (stamp, color, icon, message))
        self.append(html)
        bar = self.verticalScrollBar()
        bar.setValue(bar.maximum())


class Dashboard(QWidget):
    log_line = Signal(str)         # (level|text) 用于告警信息流
    STATUS_LEVELS = {"ok": GREEN, "info": CYAN, "warn": YELLOW, "err": RED}

    def __init__(self):
        super(Dashboard, self).__init__()
        self.setObjectName("root")
        self.setStyleSheet(_ROOT_QSS)
        self._sock = None
        self._send_lock = threading.Lock()
        self._connected = False
        self._receiver = None
        self._last_frame = None
        self._video_on = False
        self._shot_dir = "data"
        self._plan = {"mode": proto.MODE_STOP, "angle": 90,
                      "dir1": 0, "spd1": 0, "dir2": 0, "spd2": 0,
                      "brush": "000", "brush_speed": 60, "light": 0}
        self._cmd_count = 0
        self._boot = time.time()
        self._fps = 0.0
        self._last_frame_t = 0.0

        self.config = ConfigManager()
        self.logger = Logger(prefix="dash")
        self._thruster_layout = str(self.config.get("robot91", "thruster_layout") or "normal")
        self.tel = SimTelemetry()
        self._tick = 0
        # ---- 模块数据（设置/数据记录/日志/曲线）----
        self._recording = True
        self._lively = False    # 展会默认归零；开“动态演示数值”后显示模拟跳动值
        self._cmd_rows = []     # (time, 指令行, 描述)
        self._tele_rows = []    # (time, 深度, 航向, 俯仰, 横滚, 水温, 电压)
        self._log_rows = []     # (time, level, text)
        self._curve_buf = {k: [] for k in ("depth", "yaw", "pitch", "roll", "temp")}

        self.log_line.connect(self._push_log)
        self._build()
        self.clock_timer = QTimer(self)
        self.clock_timer.timeout.connect(self._tick_ui)
        self.clock_timer.start(1000)
        self._tick_ui()
        self._flash("系统就绪 · 请连接 主控", "ok")

    # =====================================================================
    # 界面搭建
    # =====================================================================
    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._header())
        body = QHBoxLayout()
        body.setSpacing(0)
        body.addWidget(self._sidebar())
        body.addWidget(self._main_area(), 1)
        root.addLayout(body, 1)

    # ----- 顶部标题栏 -----
    def _header(self):
        bar = QWidget()
        bar.setObjectName("hdr")
        bar.setFixedHeight(58)
        bar.setStyleSheet(
            "QWidget#hdr{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 %s,stop:1 %s);"
            "border-bottom:1px solid #27406B;}" % (BG0, BG1))
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(16, 0, 16, 0)

        left = QHBoxLayout()
        logo = QLabel("◉")
        logo.setStyleSheet("color:%s; font-size:24px;" % CYAN)
        brand = QLabel("ROV-SEAEXPLORER")
        brand.setStyleSheet("color:%s; font-size:15px; font-weight:800;" % TXT)
        self._online_dot = StatusLight("#5b6b80", 12)
        self._online_txt = QLabel("离线")
        self._online_txt.setStyleSheet("color:%s; font-size:12px;" % TXT_SUB)
        left.addWidget(logo)
        left.addWidget(brand)
        left.addSpacing(14)
        left.addWidget(self._online_dot)
        left.addWidget(self._online_txt)
        lay.addLayout(left)

        title = QLabel("❮ 水下机器人智能控制系统 ❯")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "color:%s; font-size:19px; font-weight:800; letter-spacing:2px;"
            "border:none; background:transparent;" % TXT)
        lay.addStretch(1)
        lay.addWidget(title)
        lay.addStretch(1)

        right = QHBoxLayout()
        self._time_lbl = QLabel("--:--:--")
        self._time_lbl.setStyleSheet("color:%s; font-size:13px;" % TXT_SUB)
        bell = QPushButton("🔔")
        gear = QPushButton("⚙")
        power = QPushButton("⏻")
        for b in (bell, gear, power):
            b.setFixedSize(32, 32)
            b.setObjectName("iconBtn")
            b.setCursor(Qt.PointingHandCursor)
        bell.clicked.connect(lambda: self._flash("通知：无新消息", "info"))
        gear.clicked.connect(lambda: self._flash("设置：演示占位（待接入配置页）", "info"))
        power.clicked.connect(QApplication.instance().quit if QApplication.instance() else
                              lambda: None)
        right.addWidget(self._time_lbl)
        right.addWidget(bell)
        right.addWidget(gear)
        right.addWidget(power)
        right.addSpacing(10)

        # 右上角：校徽 + 校名（校徽取自工程资源，取不到用蓝点兜底）
        school = QHBoxLayout()
        school.setSpacing(6)
        self._logo = QLabel()
        badge_paths = [":/runstatus/E:/CANpy/11.jpg",
                       os.path.join(ASSET_DIR, "logo.png")]
        pm = None
        for src in badge_paths:
            cand = QPixmap(src)
            if not cand.isNull():
                pm = cand
                break
        if pm is not None:
            self._logo.setPixmap(pm.scaled(30, 30, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self._logo.setText("◉")
            self._logo.setStyleSheet("color:%s; font-size:22px;" % CYAN)
        school.addWidget(self._logo)
        name = QLabel("广州航海学院")
        name.setStyleSheet(
            "color:%s; font-size:13px; font-weight:800; background:transparent;"
            "border:none;" % TXT)
        school.addWidget(name)
        right.addLayout(school)
        lay.addLayout(right)
        return bar

    # ----- 左侧栏 -----
    def _sidebar(self):
        side = QWidget()
        side.setFixedWidth(238)
        side.setStyleSheet(
            "background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 %s,stop:1 %s);"
            "border-right:1px solid #27406B;" % (BG0, BG0))
        lay = QVBoxLayout(side)
        lay.setContentsMargins(10, 12, 10, 12)
        lay.setSpacing(8)

        # 连接卡
        conn_box, conn_body = _panel("主控连接 · 主端口", CYAN)
        iprow = QHBoxLayout()
        self._ip = QLineEdit(self.config.robot_ip)
        self._port = QLineEdit(str(self.config.robot_port))
        self._port.setFixedWidth(62)
        iprow.addWidget(self._ip, 1)
        iprow.addWidget(self._port)
        btns = QHBoxLayout()
        self._btn_conn = QPushButton("连 接")
        self._btn_disc = QPushButton("断 开")
        self._btn_disc.setEnabled(False)
        self._btn_conn.clicked.connect(self.connect_91)
        self._btn_disc.clicked.connect(self.disconnect_91)
        for b in (self._btn_conn, self._btn_disc):
            b.setObjectName("solidBtn")
        btns.addWidget(self._btn_conn)
        btns.addWidget(self._btn_disc)
        conn_body.addLayout(iprow)
        conn_body.addLayout(btns)
        lay.addWidget(conn_box)

        # 导航
        nav_lbl = QLabel("功能导航")
        nav_lbl.setStyleSheet("color:%s; font-size:12px; font-weight:700;" % TXT_SUB)
        lay.addWidget(nav_lbl)
        self._nav_items = []
        nav_def = [("首页概览", 0), ("实时监控", 1), ("任务规划", 2), ("自主控制", 3),
                   ("数据管理", 5), ("系统设置", 4), ("日志信息", 6), ("声纳探测", 7)]
        for name, page in nav_def:
            b = QPushButton(name)
            b.setCheckable(True)
            b.setObjectName("navBtn")
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, n=name, p=page: self._nav(n, p))
            lay.addWidget(b)
            self._nav_items.append(b)
        self._nav_items[0].setChecked(True)
        lay.addStretch(1)

        # 系统状态面板
        sys_box, sys_body = _panel("系统状态", GREEN)
        rows = [
            ("电源电压", "voltage", "%.1f V", TXT, "volt"),
            ("剩余电量", "battery", "%.0f%%", GREEN, "battery"),
            ("舱内温度", "cabin_temp", "%.1f °C", TXT, "temp"),
            ("舱内湿度", "humidity", "%.0f%%", TXT, "humidity"),
            ("存储空间", "storage_gb", "%.0f GB", TXT, "storage"),
        ]
        self._sys_rows = []
        for label, key, fmt, col, icon in rows:
            r, v = _kv(label, fmt % (_ZERO[key]), col, mono=True, icon=icon)
            sys_body.addLayout(r)
            self._sys_rows.append((v, key, fmt, col))
            if label == "剩余电量":
                pbar = QProgressBar()
                pbar.setRange(0, 100)
                pbar.setValue(0)
                pbar.setFixedHeight(10)
                pbar.setObjectName("batBar")
                pbar.setTextVisible(False)   # 隐藏条内文字，避免遮挡（电量值由右侧标签显示）
                self._bat_bar = pbar
                sys_body.addWidget(pbar)
        shield = QLabel("🛡 一切正常")
        shield.setStyleSheet("color:%s; font-size:14px; font-weight:800;" % GREEN)
        sys_body.addWidget(shield)
        lay.addWidget(sys_box)
        return side

    # ----- 主区域（多页结构）-----
    def _main_area(self):
        host = QWidget()
        lay = QVBoxLayout(host)
        lay.setContentsMargins(0, 0, 0, 0)
        self._stack = QStackedWidget()
        lay.addWidget(self._stack)
        self._stack.addWidget(self._overview_page())
        self._stack.addWidget(self._monitor_page())
        self._stack.addWidget(self._mission_page())
        self._stack.addWidget(self._autonomous_page())
        self._stack.addWidget(self._settings_page())
        self._stack.addWidget(self._data_page())
        self._stack.addWidget(self._logs_page())
        self._stack.addWidget(self._sonar_page())
        self._stack.setCurrentIndex(0)
        return host

    def _toolbar(self):
        """QGC 式紧凑状态条：顶部一条细栏，状态以“药丸”紧凑排布，不占大片空间"""
        bar = QWidget()
        bar.setObjectName("toolbar")
        bar.setStyleSheet(
            "QWidget#toolbar{background:rgba(20,33,61,235);"
            "border:1px solid #2A3A60;border-radius:10px;}")
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 6, 12, 6)
        lay.setSpacing(12)

        def vsep():
            s = QFrame()
            s.setFixedSize(1, 16)
            s.setStyleSheet("background:rgba(90,116,168,90);border:none;")
            return s

        def pill(label, color=TXT):
            h = QHBoxLayout()
            h.setSpacing(5)
            lb = QLabel(label)
            lb.setStyleSheet("color:%s;font-size:11px;" % TXT_SUB)
            val = QLabel("--")
            val.setStyleSheet("color:%s;font-size:13px;font-weight:800;" % color)
            _mono(val)
            h.addWidget(lb)
            h.addWidget(val)
            return h, val

        # 连接
        self._tool_dot = StatusLight("#5b6b80", 11)
        self._tool_conn = QLabel("未连接")
        self._tool_conn.setStyleSheet("color:%s;font-size:12px;font-weight:700;" % TXT_SUB)
        con = QHBoxLayout()
        con.setSpacing(5)
        con.addWidget(self._tool_dot)
        con.addWidget(self._tool_conn)
        lay.addLayout(con)

        h, self._tool_mode = pill("模式", BLUE)
        self._tool_mode.setText("手动")
        lay.addWidget(vsep())
        lay.addLayout(h)
        h, self._tool_depth = pill("深度", CYAN)
        lay.addWidget(vsep())
        lay.addLayout(h)
        h, self._tool_volt = pill("电压", GREEN)
        lay.addWidget(vsep())
        lay.addLayout(h)
        h, self._tool_bat = pill("电量", GREEN)
        lay.addWidget(vsep())
        lay.addLayout(h)

        lay.addStretch(1)
        if getattr(self, "_lively", False):
            tm = QLabel("演示模式")
        else:
            tm = QLabel("展会演示模式")
        tm.setStyleSheet("color:%s;font-size:11px;" % TXT_SUB)
        lay.addWidget(tm)
        return bar

    def _overview_page(self):
        """QGC 式首页：顶部紧凑状态条 + 左侧视频主画面(撑满) + 右侧/底部紧凑卡片(贴内容)"""
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(12, 8, 12, 8)
        outer.setSpacing(8)
        outer.addWidget(self._toolbar())

        self._video_box, self._video_body = _panel("实时画面 · CAM 01", CYAN)
        self._equip_box, self._equip_body = _panel("设备状态", GREEN)
        self._motion_box, self._motion_body = _panel("运动控制 · 手动模式", BLUE)
        self._task_box, self._task_body = _panel("任务信息", CYAN)
        self._sensor_box, self._sensor_body = _panel("传感器数据", GREEN)
        self._alarm_box, self._alarm_body = _panel("推进器输出与告警", YELLOW)

        # 主区：左视频(主画面, 撑满) + 右紧凑卡片列(贴内容, 顶部对齐)
        main = QHBoxLayout()
        main.setSpacing(8)
        main.addWidget(self._video_box, 5)
        right = QVBoxLayout()
        right.setSpacing(8)
        for b in (self._equip_box, self._motion_box):
            b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)   # 高度贴内容, 不再撑满
        right.addWidget(self._equip_box)
        right.addWidget(self._motion_box)
        right.addStretch(1)          # 卡片聚顶, 下方留白（QGC 仪表盘风）
        main.addLayout(right, 4)
        outer.addLayout(main, 1)

        # 底部紧凑横带：任务信息 / 传感器数据 / 推进器输出与告警
        bottom = QHBoxLayout()
        bottom.setSpacing(8)
        for b in (self._task_box, self._sensor_box, self._alarm_box):
            b.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        self._task_box.setMinimumHeight(150)
        self._sensor_box.setMinimumHeight(150)
        self._alarm_box.setMinimumHeight(150)
        bottom.addWidget(self._task_box, 1)
        bottom.addWidget(self._sensor_box, 1)
        bottom.addWidget(self._alarm_box, 2)
        outer.addLayout(bottom)

        self._build_video()
        self._build_equipment()
        self._build_motion()
        self._build_task()
        self._build_sensor()
        self._build_thruster_alarm()
        return page

    # =====================================================================
    # 模块页：设备状态 / 实时监控 / 任务规划 / 自主控制
    # =====================================================================
    def _monitor_page(self):
        page = QWidget()
        grid = QGridLayout(page)
        grid.setContentsMargins(12, 8, 12, 8)
        grid.setSpacing(12)
        box, body = _panel("实时监控 · 视频与关键数据", CYAN)
        self._monitor_video = QLabel("实时视频（连接 91 后显示）")
        self._monitor_video.setObjectName("video")
        self._monitor_video.setAlignment(Qt.AlignCenter)
        self._monitor_video.setMinimumSize(640, 360)
        body.addWidget(self._monitor_video, 1)

        osd = QHBoxLayout()
        osd.setSpacing(20)
        self._mon_osd = {}
        for name, key, unit in (("深度", "depth_m", " m"), ("航向", "yaw_deg", "°"),
                                ("俯仰", "pitch_deg", "°"), ("横滚", "roll_deg", "°"),
                                ("水温", "water_temp", "°C")):
            grp = QVBoxLayout()
            lab = QLabel(name)
            lab.setStyleSheet("color:%s; font-size:12px;" % TXT_SUB)
            val = QLabel("--")
            val.setStyleSheet("color:%s; font-size:20px; font-weight:800;" % TXT)
            _mono(val)
            unit_l = QLabel(unit)
            unit_l.setStyleSheet("color:%s; font-size:12px;" % CYAN)
            grp.addWidget(lab)
            grp.addWidget(val)
            grp.addWidget(unit_l)
            osd.addLayout(grp)
            self._mon_osd[key] = val
        osd.addStretch(1)
        body.addLayout(osd)
        grid.addWidget(box, 0, 0)
        grid.setRowStretch(0, 1)
        return page

    def _mission_page(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(12)
        box, body = _panel("任务规划 · 航点列表", CYAN)
        self._wp_table = QTableWidget(0, 4)
        self._wp_table.setHorizontalHeaderLabels(["航点", "X(m)", "Y(m)", "动作"])
        self._wp_table.setEditTriggers(QTableWidget.AllEditTriggers)   # 可编辑，双击单元格输入
        body.addWidget(self._wp_table, 1)
        for i, (x, y) in enumerate([(0, 0), (4, 3), (9, 2), (14, 6), (20, 5), (26, 9)], 1):
            r = self._wp_table.rowCount()
            self._wp_table.insertRow(r)
            for c, val in enumerate([str(i), str(x), str(y), "获取图像"]):
                self._wp_table.setItem(r, c, QTableWidgetItem(val))
        brow = QHBoxLayout()
        btn_add = QPushButton("添加航点")
        btn_del = QPushButton("删除选中航点")
        btn_run = QPushButton("下发规划")
        for b in (btn_add, btn_del, btn_run):
            b.setObjectName("solidBtn")
        btn_add.clicked.connect(self._add_waypoint)
        btn_del.clicked.connect(self._del_waypoint)
        btn_run.clicked.connect(self._submit_mission)
        brow.addWidget(btn_add)
        brow.addWidget(btn_del)
        brow.addWidget(btn_run)
        body.addLayout(brow)
        lay.addWidget(box, 1)

        pbox, pbody = _panel("路径预览", CYAN)
        self._wp_plot = pg.PlotWidget()
        self._wp_plot.setBackground(QColor(BG0))
        self._wp_plot.showGrid(x=True, y=True, alpha=0.15)
        self._wp_plot.hideButtons()
        self._wp_plot.setLabel("left", "Y(m)", color=TXT_SUB)
        self._wp_plot.setLabel("bottom", "X(m)", color=TXT_SUB)
        self._wp_curve = self._wp_plot.plot(pen=pg.mkPen(CYAN, width=2))
        self._wp_points = None
        pbody.addWidget(self._wp_plot)
        lay.addWidget(pbox, 1)
        self._wp_table.itemChanged.connect(self._refresh_path_preview)
        self._refresh_path_preview()
        return page

    def _refresh_path_preview(self):
        if not hasattr(self, "_wp_curve"):
            return
        xs, ys = [], []
        for r in range(self._wp_table.rowCount()):
            try:
                x = float(self._wp_table.item(r, 1).text())
                y = float(self._wp_table.item(r, 2).text())
            except (AttributeError, TypeError, ValueError):
                continue
            xs.append(x)
            ys.append(y)
        if not xs:
            self._wp_curve.setData([], [])
            return
        self._wp_curve.setData(xs, ys)
        self._wp_plot.enableAutoRange()

    def _add_waypoint(self):
        r = self._wp_table.rowCount()
        self._wp_table.insertRow(r)
        for c, val in enumerate([str(r + 1), "0", "0", "获取图像"]):
            self._wp_table.setItem(r, c, QTableWidgetItem(val))
        self._refresh_path_preview()
        self._flash("已添加航点 %d（可双击编辑 X/Y/动作）" % (r + 1), "info")

    def _del_waypoint(self):
        rows = sorted(set(i.row() for i in self._wp_table.selectedIndexes()), reverse=True)
        if not rows:
            self._flash("请先选中要删除的航点", "warn")
            return
        for r in rows:
            self._wp_table.removeRow(r)
        self._refresh_path_preview()
        self._flash("已删除 %d 个航点" % len(rows), "info")

    def _submit_mission(self):
        rows = self._wp_table.rowCount()
        self._flash("任务规划：共 %d 个航点（演示——待接入任务系统后下发）" % rows, "warn")

    def _autonomous_page(self):
        page = QWidget()
        grid = QGridLayout(page)
        grid.setContentsMargins(12, 8, 12, 8)
        grid.setSpacing(12)
        box, body = _panel("自主控制 · 演示配置", BLUE)
        r = QHBoxLayout()
        r.addWidget(QLabel("控制模式"))
        self._auto_mode = QComboBox()
        self._auto_mode.addItems(["手动控制", "自主任务", "姿态保持"])
        self._auto_mode.setCurrentIndex(0)
        self._auto_mode.setMaximumWidth(300)
        r.addWidget(self._auto_mode)
        r.addStretch(1)
        body.addLayout(r)
        r2 = QHBoxLayout()
        r2.addWidget(QLabel("最大速度(PWM)"))
        self._auto_speed = QSpinBox()
        self._auto_speed.setRange(0, 255)
        self._auto_speed.setValue(150)
        self._auto_speed.setMaximumWidth(120)
        r2.addWidget(self._auto_speed)
        r2.addStretch(1)
        body.addLayout(r2)
        self._auto_en = QPushButton("启用自主控制")
        self._auto_en.setObjectName("solidBtn")
        self._auto_en.clicked.connect(self._toggle_auto)
        body.addWidget(self._auto_en)
        body.addStretch(1)
        grid.addWidget(box, 0, 0)
        grid.setColumnStretch(0, 2)

        # 摄像头画面（与首页/实时监控同步更新）
        cam_box, cam_body = _panel("摄像头", CYAN)
        self._auto_video = QLabel("实时视频（连接主控后显示）")
        self._auto_video.setObjectName("video")
        self._auto_video.setAlignment(Qt.AlignCenter)
        self._auto_video.setMinimumSize(480, 260)
        cam_body.addWidget(self._auto_video, 1)
        grid.addWidget(cam_box, 0, 1)
        grid.setColumnStretch(1, 3)

        # 机器数据（下方 · 演示归零，字号加大、两列紧凑排布）
        data_box, data_body = _panel("机器数据", GREEN)
        mrows = QGridLayout()
        mrows.setHorizontalSpacing(22)
        mrows.setVerticalSpacing(4)
        self._auto_data = []
        items = (
            ("电源电压", "voltage", "%.1f V", TXT, "volt"),
            ("剩余电量", "battery", "%.0f%%", GREEN, "battery"),
            ("舱内温度", "cabin_temp", "%.1f °C", TXT, "temp"),
            ("舱内湿度", "humidity", "%.0f%%", TXT, "humidity"),
            ("存储空间", "storage_gb", "%.0f GB", TXT, "storage"),
            ("电机温度", "motor_temp", "%.1f °C", TXT, "motor"),
            ("推进器状态", None, None, GREEN, "motor"),
            ("连接状态", None, None, TXT_SUB, "link"),
        )
        for idx, (label, key, fmt, col, icon) in enumerate(items):
            if fmt is None:
                value = "正常" if label == "推进器状态" else "离线"
            else:
                value = fmt % _ZERO[key]
            lbl = QLabel(label)
            lbl.setStyleSheet("color:%s; font-size:13px; font-weight:700;" % TXT_SUB)
            # 标签前加小图标
            grp = QWidget()
            gh = QHBoxLayout(grp)
            gh.setContentsMargins(0, 0, 0, 0)
            gh.setSpacing(6)
            ico = load_icon(icon)
            if ico is not None:
                ico_lbl = QLabel()
                ico_lbl.setPixmap(ico.pixmap(20, 20))
                gh.addWidget(ico_lbl)
            gh.addWidget(lbl)
            gh.addStretch(1)
            val = QLabel(value)
            val.setStyleSheet("color:%s; font-size:17px; font-weight:800;" % col)
            if fmt is not None:
                _mono(val)
            row, colc = idx // 2, (idx % 2) * 2
            mrows.addWidget(grp, row, colc)
            mrows.addWidget(val, row, colc + 1)
            self._auto_data.append((val, key if fmt is not None else label,
                                    fmt if fmt is not None else None, col))
        data_body.addLayout(mrows)
        data_body.addStretch(1)
        grid.addWidget(data_box, 1, 0, 1, 2)
        grid.setRowStretch(0, 4)
        grid.setRowStretch(1, 3)
        return page

    def _update_auto_data(self):
        if not hasattr(self, "_auto_data"):
            return
        v = _ZERO   # 机器数据保持归零（仅左侧“系统状态”受动态演示开关影响）
        for val, key, fmt, _col in self._auto_data:
            if fmt is None:
                continue
            val.setText(fmt % v[key])

    def _toggle_auto(self):
        mode = self._auto_mode.currentText()
        if mode == "手动控制":
            self._flash("自主控制：已切回手动控制", "info")
        else:
            self._flash("自主控制：%s（演示——待下位机/任务数据接入）" % mode, "warn")

    # =====================================================================
    # 声纳探测（演示波形 / 待接入真声纳）
    # =====================================================================
    def _sonar_page(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(12)
        box, body = _panel("声纳探测 · 演示 / 待接入", CYAN)
        top = QHBoxLayout()
        grp1 = QVBoxLayout()
        lab1 = QLabel("探测距离")
        lab1.setStyleSheet("color:%s; font-size:12px;" % TXT_SUB)
        self._sonar_dist = QLabel("-- m")
        self._sonar_dist.setStyleSheet("color:%s; font-size:28px; font-weight:800;" % TXT)
        _mono(self._sonar_dist)
        grp1.addWidget(lab1)
        grp1.addWidget(self._sonar_dist)
        grp2 = QVBoxLayout()
        lab2 = QLabel("目标状态")
        lab2.setStyleSheet("color:%s; font-size:12px;" % TXT_SUB)
        self._sonar_target = QLabel("无目标")
        self._sonar_target.setStyleSheet(
            "color:%s; font-size:20px; font-weight:800;" % TXT_SUB)
        grp2.addWidget(lab2)
        grp2.addWidget(self._sonar_target)
        top.addLayout(grp1)
        top.addSpacing(40)
        top.addLayout(grp2)
        top.addStretch(1)
        body.addLayout(top)

        # 保留曲线图框与坐标轴（数据归零，不跑随机）
        self._sonar_plot = pg.PlotWidget()
        self._sonar_plot.setBackground(QColor(BG0))
        self._sonar_plot.showGrid(x=True, y=True, alpha=0.15)
        self._sonar_plot.hideButtons()
        self._sonar_plot.setLabel("left", "回波强度", color=TXT_SUB)
        self._sonar_plot.setLabel("bottom", "距离 (m)", color=TXT_SUB)
        self._sonar_plot.setYRange(0, 1.05)
        self._sonar_plot.setXRange(0, 50)
        self._sonar_zero = self._sonar_plot.plot(pen=pg.mkPen("#31c4f3", width=2))
        self._sonar_zero.setData([0, 50], [0, 0])
        body.addWidget(self._sonar_plot, 1)

        lay.addWidget(box)
        self._update_sonar()
        return page

    def _update_sonar(self):
        if not hasattr(self, "_sonar_target"):
            return
        # 展会默认归零：不跑随机数
        self._sonar_dist.setText("0.0 m")
        self._sonar_target.setText("无目标")
        self._sonar_target.setStyleSheet(
            "color:%s; font-size:20px; font-weight:800;" % TXT_SUB)
        if hasattr(self, "_sonar_zero"):
            self._sonar_zero.setData([0, 50], [0, 0])

    # =====================================================================
    # 模块页：系统设置 / 数据管理 / 日志信息 / 实时曲线
    # =====================================================================
    def _settings_page(self):
        page = QWidget()
        grid = QGridLayout(page)
        grid.setContentsMargins(12, 8, 12, 8)
        grid.setSpacing(12)

        box, body = _panel("系统设置 · 主控连接", CYAN)
        r = QHBoxLayout()
        r.addWidget(QLabel("主端口地址"))
        self._set_ip = QLineEdit(self.config.robot_ip)
        self._set_port = QLineEdit(str(self.config.robot_port))
        self._set_port.setFixedWidth(80)
        r.addWidget(self._set_ip, 1)
        r.addWidget(self._set_port)
        body.addLayout(r)

        self._set_recon = QCheckBox("连接意外断开时自动重连")
        self._set_recon.setChecked(bool(self.config.auto_reconnect))
        body.addWidget(self._set_recon)

        r2 = QHBoxLayout()
        r2.addWidget(QLabel("重连间隔(s)"))
        self._set_recon_int = QSpinBox()
        self._set_recon_int.setRange(1, 30)
        self._set_recon_int.setValue(int(self.config.get("system", "reconnect_interval") or 3))
        r2.addWidget(self._set_recon_int)
        r2.addWidget(QLabel("最大次数"))
        self._set_recon_max = QSpinBox()
        self._set_recon_max.setRange(1, 100)
        self._set_recon_max.setValue(int(self.config.get("system", "max_reconnect_attempts") or 10))
        r2.addWidget(self._set_recon_max)
        body.addLayout(r2)

        r3 = QHBoxLayout()
        r3.addWidget(QLabel("推进器安装"))
        self._set_layout = QComboBox()
        self._set_layout.addItem("同向安装（前进=双正转）", "normal")
        self._set_layout.addItem("镜像安装（前进=左正+右反）", "mirror")
        idx = self._set_layout.findData(self._thruster_layout)
        self._set_layout.setCurrentIndex(idx if idx >= 0 else 0)
        r3.addWidget(self._set_layout)
        body.addLayout(r3)

        self._chk_lively = QCheckBox("动态演示数值（默认归零）")
        self._chk_lively.setChecked(self._lively)
        self._chk_lively.toggled.connect(self._on_lively)
        body.addWidget(self._chk_lively)

        btn_save = QPushButton("保存设置")
        btn_reset = QPushButton("恢复默认")
        for b in (btn_save, btn_reset):
            b.setObjectName("solidBtn")
        btn_save.clicked.connect(self._save_settings)
        btn_reset.clicked.connect(self._reset_settings)
        brow = QHBoxLayout()
        brow.addWidget(btn_save)
        brow.addWidget(btn_reset)
        body.addLayout(brow)
        grid.addWidget(box, 0, 0, Qt.AlignTop)
        about, about_body = _panel("系统信息", CYAN)
        for line in ("软件：智能水下清洁机器人控制系统 V1.0",
                     "协议：TCP/IP ASCII 7字段（软著）",
                     "主端口：192.168.1.91:12345 · 灯光从控(由主控转发)",
                     "数据：真实=连接/视频/指令；遥测=模拟(待接入)"):
            about_body.addWidget(QLabel(line))
        about_body.addStretch(1)
        grid.addWidget(about, 0, 1, Qt.AlignTop)
        grid.setColumnStretch(0, 3)
        grid.setColumnStretch(1, 2)
        # 面板贴顶、只占内容高度，下方留白
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        about.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        grid.setRowStretch(0, 1)
        return page

    def _save_settings(self):
        ip = self._set_ip.text().strip() or "192.168.1.91"
        port = self._set_port.text().strip() or "12345"
        self.config.update_robot(ip, port)
        self.config.set("system", "reconnect_interval", self._set_recon_int.value())
        self.config.set("system", "max_reconnect_attempts", self._set_recon_max.value())
        self.config.set("system", "auto_reconnect", self._set_recon.isChecked())
        self.config.set("robot91", "thruster_layout", self._set_layout.currentData())
        layout = self._set_layout.currentData()
        self._thruster_layout = layout
        # 同步左侧连接卡地址
        self._ip.setText(ip)
        self._port.setText(port)
        self._flash("设置已保存：%s:%s | 推进器=%s" % (ip, port, layout), "ok")

    def _reset_settings(self):
        self.config.update_robot("192.168.1.91", 12345)
        self.config.set("system", "reconnect_interval", 3)
        self.config.set("system", "max_reconnect_attempts", 10)
        self.config.set("system", "auto_reconnect", True)
        self.config.set("robot91", "thruster_layout", "normal")
        self._set_ip.setText("192.168.1.91")
        self._set_port.setText("12345")
        self._set_recon.setChecked(True)
        self._set_recon_int.setValue(3)
        self._set_recon_max.setValue(10)
        self._set_layout.setCurrentIndex(self._set_layout.findData("normal"))
        self._thruster_layout = "normal"
        self._ip.setText("192.168.1.91")
        self._port.setText("12345")
        self._flash("已恢复默认设置", "ok")

    # ---------------- 数据管理 ----------------
    def _data_page(self):
        page = QWidget()
        grid = QGridLayout(page)
        grid.setContentsMargins(12, 8, 12, 8)
        grid.setSpacing(12)
        box, body = _panel("数据管理 · 指令/遥测记录", CYAN)
        top = QHBoxLayout()
        self._btn_rec = QPushButton("停止记录" if self._recording else "开始记录")
        self._btn_rec.setCheckable(True)
        self._btn_rec.setChecked(self._recording)
        self._btn_rec.setObjectName("solidBtn")
        self._btn_rec.toggled.connect(lambda on: self._set_recording(on))
        btn_export = QPushButton("导出CSV")
        btn_export.setObjectName("solidBtn")
        btn_export.clicked.connect(self._export_record)
        self._stat_rec = QLabel("")
        self._stat_rec.setStyleSheet("color:%s;" % TXT_SUB)
        top.addWidget(self._btn_rec)
        top.addWidget(btn_export)
        top.addStretch(1)
        top.addWidget(self._stat_rec)
        body.addLayout(top)
        self._data_table = QTableWidget(0, 4)
        self._data_table.setHorizontalHeaderLabels(["时间", "类型", "内容", "描述"])
        self._data_table.horizontalHeader().setStretchLastSection(True)
        self._data_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._data_table.setAlternatingRowColors(True)
        body.addWidget(self._data_table, 1)
        grid.addWidget(box, 0, 0)
        self._refresh_data_stats()
        return page

    def _set_recording(self, on):
        self._recording = on
        self._btn_rec.setText("停止记录" if on else "开始记录")
        self._flash("数据记录 %s" % ("开启" if on else "暂停"), "info")

    def _on_lively(self, on):
        self._lively = on
        self._tick_ui()
        self._flash("动态演示数值：%s" % ("动态演示" if on else "归零中性"), "info")

    def _append_data_row(self, kind, content, desc):
        self._data_table.insertRow(0)
        stamp = datetime.now().strftime("%H:%M:%S")
        for c, v in enumerate([stamp, kind, content, desc]):
            self._data_table.setItem(0, c, QTableWidgetItem(str(v)))
        if self._data_table.rowCount() > 300:
            self._data_table.removeRow(self._data_table.rowCount() - 1)
        self._refresh_data_stats()

    def _refresh_data_stats(self):
        if hasattr(self, "_stat_rec"):
            up = int(time.time() - self._boot)
            self._stat_rec.setText(
                "指令 %d · 遥测 %d · 时长 %02d:%02d:%02d"
                % (len(self._cmd_rows), len(self._tele_rows), up // 3600, up // 60 % 60, up % 60))

    def _export_record(self):
        os.makedirs(self._shot_dir, exist_ok=True)
        path = os.path.join(self._shot_dir, datetime.now().strftime("record_%Y%m%d_%H%M%S.csv"))
        with open(path, "w", encoding="utf-8") as f:
            f.write("time,kind,content,desc\n")
            for row in self._cmd_rows:
                f.write("%s,cmd,%s,%s\n" % row)
            for row in self._tele_rows:
                f.write("%s,tele,%s,%s\n" % (row[0], ",".join(map(str, row[1:])), "模拟遥测"))
        self._flash("已导出记录：%s" % path, "ok")

    # ---------------- 日志信息 ----------------
    def _logs_page(self):
        page = QWidget()
        grid = QGridLayout(page)
        grid.setContentsMargins(12, 8, 12, 8)
        grid.setSpacing(12)
        box, body = _panel("日志信息 · 分级筛选", YELLOW)
        top = QHBoxLayout()
        self._f_ok = QCheckBox("正常")
        self._f_warn = QCheckBox("告警")
        self._f_err = QCheckBox("严重")
        for c in (self._f_ok, self._f_warn, self._f_err):
            c.setChecked(True)
            c.toggled.connect(self._refresh_logs_table)
        btn_export = QPushButton("导出日志")
        btn_export.setObjectName("solidBtn")
        btn_export.clicked.connect(self._export_logs)
        top.addWidget(self._f_ok)
        top.addWidget(self._f_warn)
        top.addWidget(self._f_err)
        top.addStretch(1)
        top.addWidget(btn_export)
        body.addLayout(top)
        self._logs_table = QTableWidget(0, 3)
        self._logs_table.setHorizontalHeaderLabels(["时间", "等级", "内容"])
        self._logs_table.horizontalHeader().setStretchLastSection(True)
        self._logs_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._logs_table.setAlternatingRowColors(True)
        body.addWidget(self._logs_table, 1)
        grid.addWidget(box, 0, 0)
        return page

    def _refresh_logs_table(self):
        show_ok = getattr(self, "_f_ok", None) and self._f_ok.isChecked()
        show_warn = getattr(self, "_f_warn", None) and self._f_warn.isChecked()
        show_err = getattr(self, "_f_err", None) and self._f_err.isChecked()
        table = getattr(self, "_logs_table", None)
        if table is None:
            return
        table.setRowCount(0)
        for stamp, level, text in reversed(self._log_rows[-400:]):
            keep = (level in ("ok", "info") and show_ok) or (level == "warn" and show_warn) \
                or (level == "err" and show_err)
            if not keep:
                continue
            r = table.rowCount()
            table.insertRow(r)
            table.setItem(r, 0, QTableWidgetItem(stamp))
            table.setItem(r, 1, QTableWidgetItem(level))
            table.setItem(r, 2, QTableWidgetItem(text))

    def _export_logs(self):
        os.makedirs(self._shot_dir, exist_ok=True)
        path = os.path.join(self._shot_dir, datetime.now().strftime("logs_%Y%m%d_%H%M%S.txt"))
        with open(path, "w", encoding="utf-8") as f:
            for stamp, level, text in self._log_rows:
                f.write("[%s] [%s] %s\n" % (stamp, level, text))
        self._flash("已导出日志：%s" % path, "ok")

    # ---------------- 实时曲线 ----------------
    def _build_video(self):
        body = self._video_body
        # 主视觉区只用于实时视频，不放实拍图（实拍图仅用于“设备状态”里的 ROV 视图）
        self._photo = None
        # 顶部 OSD 行（加了一层半透明黑遮罩，避免未来亮色视频把文字淹没）
        osd = QHBoxLayout()
        osd.setSpacing(18)
        self._osd = {}
        for name, key, unit in (("深度", "depth_m", " m"), ("航向", "yaw_deg", "°"),
                                ("俯仰", "pitch_deg", "°"), ("横滚", "roll_deg", "°"),
                                ("水温", "water_temp", "°C")):
            row = QHBoxLayout()
            lab = QLabel(name)
            lab.setStyleSheet("color:%s; font-size:12px;" % TXT_SUB)
            val = QLabel("--")
            val.setStyleSheet("color:%s; font-size:18px; font-weight:800;" % TXT)
            _mono(val)
            unit_l = QLabel(unit)
            unit_l.setStyleSheet("color:%s; font-size:12px;" % CYAN)
            row.addWidget(lab)
            row.addWidget(val)
            row.addWidget(unit_l)
            osd.addLayout(row)
            self._osd[key] = val
        osd.addStretch(1)
        osd_frame = QFrame()
        osd_frame.setStyleSheet(
            "QFrame{background:rgba(0,0,0,110);border:none;border-radius:8px;}")
        osd_frame.setLayout(osd)
        body.addWidget(osd_frame)

        vrow = QHBoxLayout()
        self._video_label = QLabel("未连接视频（连接 主控后自动显示）")
        self._video_label.setObjectName("video")
        self._video_label.setAlignment(Qt.AlignCenter)
        self._video_label.setMinimumSize(560, 300)
        self._video_label.setScaledContents(False)
        self._video_label.setMinimumHeight(300)
        vrow.addWidget(self._video_label, 1)

        # 深度指示：自绘仪表（刻度+填充+数值严格对齐）
        depth_col = QVBoxLayout()
        depth_lbl = QLabel("深度指示 (m)")
        depth_lbl.setStyleSheet("color:%s; font-size:12px; font-weight:700;" % TXT_SUB)
        depth_lbl.setAlignment(Qt.AlignLeft)
        self._depth_gauge = DepthGauge()
        depth_col.addWidget(depth_lbl)
        depth_col.addWidget(self._depth_gauge, 1)
        depth_col.addStretch(1)
        vrow.addLayout(depth_col)
        body.addLayout(vrow, 1)

        # 底栏
        bar = QHBoxLayout()
        cam = QLabel("● CAM 01")
        cam.setStyleSheet("color:%s; font-size:12px; font-weight:700;" % GREEN)
        bar.addWidget(cam)
        bar.addStretch(1)
        self._btn_video = QPushButton("打开视频")
        self._btn_photo = QPushButton("📷 截图")
        self._btn_light = QPushButton("☀ 灯光0")
        for b in (self._btn_video, self._btn_photo, self._btn_light):
            b.setObjectName("ghostBtn")
        self._btn_video.clicked.connect(self.toggle_video)
        self._btn_photo.clicked.connect(self.save_shot)
        self._btn_light.clicked.connect(self.light_toggle)
        self._btn_light.setCheckable(True)
        bar.addWidget(self._btn_video)
        bar.addWidget(self._btn_photo)
        bar.addWidget(self._btn_light)
        body.addLayout(bar)
        self._video_idle()

    def _video_idle(self):
        """未显示实时视频时：优先展示 ROV 实拍图，否则文字占位"""
        pm = getattr(self, "_photo", None)
        if pm is not None:
            w = self._video_label.width() or 560
            h = self._video_label.height() or 300
            self._video_label.setPixmap(pm.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            self._video_label.setToolTip("ROV 实拍图（连接后显示实时视频）")
        else:
            self._video_label.clear()
            self._video_label.setText("未连接视频（连接 主控后自动显示）")
            self._video_label.setToolTip("")

    def _build_equipment(self):
        body = self._equip_body
        # ROV 实拍图（干净展示，不放状态灯示意）
        photo = QLabel()
        photo.setObjectName("rovPhoto")
        photo.setAlignment(Qt.AlignCenter)
        path = os.path.join(ASSET_DIR, "rov_photo.png")
        if os.path.exists(path):
            pm = QPixmap(path)
            if not pm.isNull():
                photo.setPixmap(pm.scaledToHeight(160, Qt.SmoothTransformation))
        else:
            photo.setText("ROV 示意图（待放置俯视图）")
        photo.setStyleSheet(
            "background:#0a1426; border:1px solid #2a4a7a; border-radius:10px;")
        body.addWidget(photo, 1)

        # 2 通道推进器状态指示（软著：左/右）—— 健康圆点 + 实时输出%
        thr = QHBoxLayout()
        thr.setSpacing(16)
        title_t = QLabel("推进器")
        title_t.setStyleSheet("color:%s; font-size:12px; font-weight:700;" % TXT_SUB)
        thr.addWidget(title_t)
        self._thr_ind = []
        for tag, name in (("T1", "左"), ("T2", "右")):
            b = QHBoxLayout()
            b.setSpacing(5)
            dot = StatusLight(GREEN, 12)
            tlab = QLabel("%s·%s" % (tag, name))
            tlab.setStyleSheet("color:%s; font-size:12px; font-weight:700;" % TXT)
            val = QLabel("0%")
            val.setStyleSheet("color:%s; font-size:13px; font-weight:800;" % CYAN)
            _mono(val)
            b.addWidget(dot)
            b.addWidget(tlab)
            b.addWidget(val)
            thr.addLayout(b)
            self._thr_ind.append((dot, val))
        thr.addStretch(1)
        body.addLayout(thr)

        rows = [
            ("连接状态", "已连接", GREEN), ("工作模式", "手动模式", BLUE),
            ("推进器状态", "正常", GREEN), ("漏水检测", "正常", GREEN),
            ("姿态传感器", "正常", GREEN), ("电机温度", "32.6 °C", TXT),
        ]
        self._status_rows = []
        for label, value, color in rows:
            rr, val = _kv(label, value, color)
            body.addLayout(rr)
            self._status_rows.append((label, val))
        body.addStretch(1)

    def _build_motion(self):
        body = self._motion_body
        tabs = QHBoxLayout()
        tab_manual = QPushButton("手动控制")
        tab_manual.setObjectName("tabOn")
        tab_hold = QPushButton("姿态保持")
        tab_hold.setObjectName("tabOff")
        tab_hold.clicked.connect(lambda: self._flash("姿态保持：演示占位（待接入闭环控制）", "info"))
        tabs.addWidget(tab_manual)
        tabs.addWidget(tab_hold)
        tabs.addStretch(1)
        body.addLayout(tabs)

        grid = QGridLayout()
        grid.setSpacing(6)

        def mvbtn(text, act=None, disabled=False, tooltip="", icon=None):
            b = QPushButton(text)
            b.setObjectName("moveBtn")
            b.setCursor(Qt.PointingHandCursor)
            if icon:
                ic = load_icon(icon)
                if ic is not None:
                    b.setIcon(ic)
                    b.setIconSize(QSize(26, 26))
                    b.setText("")
                    b.setToolTip(text)
            if act:
                b.clicked.connect(act)
            if disabled:
                b.setEnabled(False)
                b.setToolTip(tooltip or "本机为双推进布局（软著），无此自由度")
            return b

        # 左列：垂向（本机无 → 占位，图标演示）
        col = QVBoxLayout()
        col.addWidget(mvbtn("上升", disabled=True, icon="up"))
        self._pct_up = QLabel("0%")
        self._pct_up.setAlignment(Qt.AlignCenter)
        self._pct_up.setStyleSheet("color:%s; font-size:16px; font-weight:800;" % CYAN)
        col.addWidget(self._pct_up)
        col.addWidget(mvbtn("下降", disabled=True, icon="down"))
        col.addStretch(1)
        grid.addLayout(col, 0, 0, 3, 1)

        # 中列：十字
        mid = QGridLayout()
        mid.addWidget(mvbtn("前进", lambda: self._move("forward"), icon="up"), 0, 1)
        mid.addWidget(mvbtn("左转", lambda: self._move("left"), icon="left"), 1, 0)
        self._pct_center = QLabel("0%")
        self._pct_center.setAlignment(Qt.AlignCenter)
        self._pct_center.setStyleSheet("color:%s; font-size:16px; font-weight:800;" % CYAN)
        mid.addWidget(self._pct_center, 1, 1)
        mid.addWidget(mvbtn("右转", lambda: self._move("right"), icon="right"), 1, 2)
        mid.addWidget(mvbtn("后退", lambda: self._move("backward"), icon="down"), 2, 1)
        mid.setHorizontalSpacing(4)
        mid.setVerticalSpacing(4)
        grid.addLayout(mid, 0, 1, 3, 1)

        # 右列：侧向（本机无 → 占位）
        colr = QVBoxLayout()
        colr.addWidget(mvbtn("左移", disabled=True, icon="left"))
        self._pct_side = QLabel("0%")
        self._pct_side.setAlignment(Qt.AlignCenter)
        self._pct_side.setStyleSheet("color:%s; font-size:16px; font-weight:800;" % CYAN)
        colr.addWidget(self._pct_side)
        colr.addWidget(mvbtn("右移", disabled=True, icon="right"))
        colr.addStretch(1)
        grid.addLayout(colr, 0, 2, 3, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 2)
        grid.setColumnStretch(2, 1)
        body.addLayout(grid)

        ctrl = QHBoxLayout()
        self._btn_start = QPushButton("▶ 启动运行(10)")
        self._btn_stop = QPushButton("⏹ 停止待机(00)")
        self._btn_e = QPushButton("⛔ 急停")
        for b in (self._btn_start, self._btn_stop, self._btn_e):
            b.setObjectName("solidBtn")
        self._btn_start.clicked.connect(self.start_run)
        self._btn_stop.clicked.connect(self.stop_idle)
        self._btn_e.clicked.connect(self.emergency)
        ctrl.addWidget(self._btn_start)
        ctrl.addWidget(self._btn_stop)
        ctrl.addWidget(self._btn_e)
        body.addLayout(ctrl)

        # 灯光 & 刷子 & 转速 快捷
        q = QHBoxLayout()
        q.addWidget(QLabel("灯光"))
        self._light = QSlider(Qt.Horizontal)
        self._light.setRange(0, 100)
        self._light.valueChanged.connect(self._on_light)
        q.addWidget(self._light, 1)
        q.addWidget(QLabel("刷速"))
        self._bs = QSpinBox()
        self._bs.setRange(0, 100)
        self._bs.setValue(60)
        self._bs.valueChanged.connect(self._on_bs)
        q.addWidget(self._bs)
        self._b1 = QCheckBox("刷1")
        self._b2 = QCheckBox("刷2")
        self._b3 = QCheckBox("刷3")
        for b in (self._b1, self._b2, self._b3):
            b.setStyleSheet("color:%s;" % TXT)
            b.toggled.connect(self._on_brush)
        q.addWidget(self._b1)
        q.addWidget(self._b2)
        q.addWidget(self._b3)
        body.addLayout(q)

    def _build_task(self):
        body = self._task_body
        vals = [
            ("任务名称", "无"), ("任务编号", "无"),
            ("任务时长", "00:00:00"), ("已完成航点", "0 / 0"),
        ]
        for k, v in vals:
            r, _ = _kv(k, v, TXT)
            body.addLayout(r)
        body.addStretch(1)

    def _build_sensor(self):
        body = self._sensor_body
        rows = [
            ("深度计", "depth_m", "%.1f m", "depth"),
            ("DVL 速度", "dvl", "%.2f m/s", "speed"),
            ("姿态 R/P/Y", None, None, "attitude"),
            ("磁力计", "mag_ut", "%.1f μT", "compass"),
            ("水温", "water_temp", "%.1f °C", "temp"),
            ("浊度", "turbidity_ntu", "%.2f NTU", "water"),
        ]
        self._sensor_vals = []
        for label, key, fmt, icon in rows:
            if fmt is None:
                val_txt = "%.1f/%.1f/%.0f" % (_ZERO["roll_deg"], _ZERO["pitch_deg"], _ZERO["yaw_deg"])
                f = lambda v: "%.1f/%.1f/%.0f" % (v["roll_deg"], v["pitch_deg"], v["yaw_deg"])
                r, val = _kv(label, val_txt, TXT, mono=True, icon=icon)
                self._sensor_vals.append((label, val, "att", f))
            else:
                r, val = _kv(label, fmt % _ZERO[key], TXT, mono=True, icon=icon)
                self._sensor_vals.append((label, val, key, (lambda k=key, f=fmt: (lambda v: f % v[k]))()))
            body.addLayout(r)
        body.addStretch(1)

    def _build_thruster_alarm(self):
        body = self._alarm_body
        # 推进器输出（软著 2 通道：左/右）—— 实时 PWM 柱状图
        bars = QHBoxLayout()
        bars.setSpacing(16)
        for tag, name, color, bar_attr, pwm_attr in (
                ("T1", "左", GREEN, "_bar_l", "_pwm_l"),
                ("T2", "右", CYAN, "_bar_r", "_pwm_r")):
            lay = QVBoxLayout()
            lay.setSpacing(4)
            hrow = QHBoxLayout()
            hrow.setSpacing(6)
            hd = QLabel(tag)
            hd.setStyleSheet("color:%s; font-size:12px; font-weight:800;" % color)
            nm = QLabel(name)
            nm.setStyleSheet("color:%s; font-size:11px;" % TXT_SUB)
            hrow.addWidget(hd)
            hrow.addWidget(nm)
            hrow.addStretch(1)
            pwmL = QLabel("PWM")
            pwmL.setStyleSheet("color:%s; font-size:10px;" % TXT_SUB)
            pwm = QLabel("0")
            pwm.setStyleSheet("color:%s; font-size:13px; font-weight:800;" % color)
            _mono(pwm)
            hrow.addWidget(pwmL)
            hrow.addWidget(pwm)
            setattr(self, pwm_attr, pwm)
            bar = QProgressBar()
            bar.setRange(0, 255)
            bar.setValue(0)
            bar.setFixedHeight(14)
            bar.setObjectName("thrBar")
            bar.setFormat("")
            bar.setStyleSheet("QProgressBar::chunk{background:%s;}" % color)
            setattr(self, bar_attr, bar)
            lay.addLayout(hrow)
            lay.addWidget(bar)
            bars.addLayout(lay, 1)
        body.addLayout(bars)

        self._log = LogView()
        body.addWidget(self._log, 1)
        clear = QPushButton("清除告警")
        clear.setObjectName("solidBtn")
        clear.clicked.connect(lambda: self._log.clear())
        body.addWidget(clear, 0, Qt.AlignRight)

    # =====================================================================
    # 连接 / 视频
    # =====================================================================
    def _raw_send(self, line):
        if not self._connected or self._sock is None:
            self._flash("未连接，指令未发送：%s" % line, "warn")
            return False
        with self._send_lock:
            try:
                self._sock.sendall((line + "\n").encode("utf-8"))
            except OSError as e:
                self._flash("发送失败：%s" % e, "err")
                return False
        self._cmd_count += 1
        return True

    def _send_plan(self, announce=True):
        p = self._plan
        line = proto.build_command(
            p["mode"], p["angle"], p["dir1"], p["spd1"], p["dir2"], p["spd2"],
            p["brush"], p["brush_speed"], p["light"])
        if self._raw_send(line):
            desc = proto.describe_command(line)
            self._cmd_rows.append((datetime.now().strftime("%H:%M:%S"), line, desc))
            if hasattr(self, "_data_table"):
                self._append_data_row("指令", line, desc)
            if announce:
                self._flash(desc, "ok")
            self._sync_bars()
        return True

    def connect_91(self):
        ip = self._ip.text().strip() or proto.DEFAULT_IP
        port = self._port.text().strip() or str(proto.DEFAULT_PORT)
        if self._connected:
            return
        self._flash("正在连接 %s:%s ..." % (ip, port), "info")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.5)
            sock.connect((ip, int(port)))
        except Exception as e:
            self._flash("连接失败：%s" % e, "err")
            QMessageBox.warning(self, "连接失败",
                                "无法连接 主控 %s:%s\n\n原因：%s\n\n"
                                "请检查设备/模拟服务器与网络（默认 192.168.1.91:12345）"
                                % (ip, port, e))
            return
        self._sock = sock
        self._connected = True
        self.config.update_robot(ip, port)
        self._set_online(True)
        self._flash("已连接 主控 %s:%s（视频通道就绪）" % (ip, port), "ok")
        self._btn_conn.setEnabled(False)
        self._btn_disc.setEnabled(True)
        recv = FrameReceiver(sock)
        recv.frame_ready.connect(self._on_frame)
        recv.broken.connect(self._on_broken)
        recv.start()
        self._receiver = recv
        # 回安全状态（并同步快捷控件显示）
        self._plan.update(mode=proto.MODE_STOP, dir1=0, spd1=0, dir2=0, spd2=0,
                          brush="000", brush_speed=0, light=0)
        self._sync_quick_ui()
        self._send_plan(announce=False)

    def disconnect_91(self):
        recv, self._receiver = self._receiver, None
        if recv:
            try:
                recv.stop()
                recv.wait(1500)
            except Exception:
                pass
        sock, self._sock = self._sock, None
        if sock:
            try:
                sock.close()
            except OSError:
                pass
        was = self._connected
        self._connected = False
        if was:
            self._flash("已断开 主控连接", "info")
        self._set_online(False)
        self._btn_conn.setEnabled(True)
        self._btn_disc.setEnabled(False)

    def _on_broken(self, reason):
        if self._connected:
            detail = reason if not str(reason).endswith("已退出") else "对端关闭或网络中断"
            self._flash("主控通道断开：%s" % detail, "err")
            self._connected = False
            self._set_online(False)
            self._btn_conn.setEnabled(True)
            self._btn_disc.setEnabled(False)
            try:
                if self._sock:
                    self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _on_frame(self, bgr):
        t = time.time()
        dt = t - self._last_frame_t
        self._last_frame_t = t
        if 0.001 < dt < 1.0:
            self._fps = 0.7 * self._fps + 0.3 * (1.0 / dt)
        self._last_frame = bgr
        if not self._video_on:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        pix = QPixmap.fromImage(img).scaled(
            self._video_label.width(), self._video_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._video_label.setPixmap(pix)
        # 同步到“实时监控”页大图
        if hasattr(self, "_monitor_video") and self._monitor_video.width() > 0:
            pix2 = QPixmap.fromImage(img).scaled(
                self._monitor_video.width(), self._monitor_video.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self._monitor_video.setPixmap(pix2)
        if hasattr(self, "_auto_video") and self._auto_video.width() > 0:
            pix3 = QPixmap.fromImage(img).scaled(
                self._auto_video.width(), self._auto_video.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self._auto_video.setPixmap(pix3)

    def toggle_video(self):
        if not self._connected:
            self._flash("请先连接 主控", "warn")
            return
        self._video_on = not self._video_on
        if self._video_on:
            self._btn_video.setText("关闭视频")
            self._flash("视频显示已开启（主控帧流）", "ok")
            self._video_label.setText("等待视频帧...")
            if self._last_frame is not None:
                self._on_frame(self._last_frame)
        else:
            self._btn_video.setText("打开视频")
            self._video_label.clear()
            self._video_idle()

    def save_shot(self):
        if self._last_frame is None:
            self._flash("当前没有视频帧", "warn")
            return
        os.makedirs(self._shot_dir, exist_ok=True)
        p = os.path.join(self._shot_dir,
                         datetime.now().strftime("shot_%Y%m%d_%H%M%S.png"))
        cv2.imwrite(p, self._last_frame)
        self._flash("截图已保存：%s" % p, "ok")

    # =====================================================================
    # 运动映射（7字段 → 主控）
    # =====================================================================
    def _ensure_running(self):
        if self._plan["mode"] != proto.MODE_START:
            self._plan["mode"] = proto.MODE_START
            self._flash("自动进入运行模式(10)", "info")

    def _move(self, action):
        if not self._connected:
            self._flash("未连接主控", "warn")
            return
        self._ensure_running()
        spd = 150
        mirror = getattr(self, "_thruster_layout", "normal") == "mirror"
        if action == "forward":
            d1, d2 = (-1, 1) if mirror else (1, 1)
        elif action == "backward":
            d1, d2 = (1, -1) if mirror else (-1, -1)
        elif action == "left":
            d1, d2 = (-1, -1) if mirror else (-1, 1)
        else:
            d1, d2 = (1, 1) if mirror else (1, -1)
        self._plan.update(dir1=d1, spd1=spd, dir2=d2, spd2=spd)
        self._send_plan()

    def start_run(self):
        self._plan["mode"] = proto.MODE_START
        self._flash("启动运行(10)：开始采集并运行", "ok")
        self._send_plan()

    def stop_idle(self):
        self._plan.update(mode=proto.MODE_STOP, dir1=0, spd1=0, dir2=0, spd2=0,
                          brush="000", brush_speed=0, light=0)
        self._sync_quick_ui()
        self._flash("停止待机(00)：全部复位", "ok")
        self._send_plan()

    def emergency(self):
        self._plan.update(mode=proto.MODE_STOP, dir1=0, spd1=0, dir2=0, spd2=0,
                          brush="000", brush_speed=0, light=0)
        line = proto.emergency_stop_command()
        self._flash("！！！ 急停（%s）" % line, "err")
        self._raw_send(line)
        self._sync_quick_ui()

    def _on_light(self, v):
        self._plan["light"] = v
        self._btn_light.setText("☀ 灯光%d" % v)
        self._send_plan(announce=False)

    def light_toggle(self):
        cur = self._light.value()
        self._light.setValue(0 if cur > 0 else 80)

    def _on_bs(self, v):
        self._plan["brush_speed"] = v
        self._send_plan(announce=False)

    def _on_brush(self, _=None):
        self._plan["brush"] = "".join(
            "1" if b.isChecked() else "0" for b in (self._b1, self._b2, self._b3))
        self._send_plan(announce=False)

    def _sync_quick_ui(self):
        p = self._plan
        self._light.setValue(p["light"])
        self._bs.setValue(int(p["brush_speed"]))
        self._b1.setChecked(p["brush"][0] == "1")
        self._b2.setChecked(p["brush"][1] == "1")
        self._b3.setChecked(p["brush"][2] == "1")

    def _sync_bars(self):
        p = self._plan
        mag1 = p["spd1"] if p["dir1"] != 0 else 0
        mag2 = p["spd2"] if p["dir2"] != 0 else 0
        self._bar_l.setValue(int(mag1))
        self._bar_r.setValue(int(mag2))
        if hasattr(self, "_pwm_l"):
            self._pwm_l.setText(str(int(mag1)))
        if hasattr(self, "_pwm_r"):
            self._pwm_r.setText(str(int(mag2)))
        if getattr(self, "_thr_ind", None):
            for (dot, val), m in zip(self._thr_ind, (mag1, mag2)):
                val.setText("%d%%" % int(100 * m / 255.0))
                dot.set_color(GREEN if m >= 0 else RED)
        if hasattr(self, "_dev_bar_l"):
            self._dev_bar_l.setValue(int(mag1))
            self._dev_bar_r.setValue(int(mag2))
        self._pct_center.setText("%d%%" % int(100 * max(mag1, mag2) / 255.0))

    # =====================================================================
    # 定时刷新
    # =====================================================================
    def _set_online(self, ok):
        self._online_dot.set_color(GREEN if ok else "#5b6b80")
        self._online_txt.setText("在线" if ok else "离线")
        self._online_txt.setStyleSheet(
            "color:%s; font-size:12px; font-weight:700;" % (GREEN if ok else TXT_SUB))
        if hasattr(self, "_tool_dot"):
            self._tool_dot.set_color(GREEN if ok else "#5b6b80")
            self._tool_conn.setText("已连接" if ok else "未连接")
            self._tool_conn.setStyleSheet(
                "color:%s; font-size:12px; font-weight:700;" % (GREEN if ok else TXT_SUB))

    def _tick_ui(self):
        now = datetime.now()
        self._time_lbl.setText(now.strftime("%H:%M:%S"))
        self._tick += 1
        if self._tick % 5 == 0:
            self.tel.tick()
        # 传感器/OSD/机器数据等保持归零；仅“系统状态”在开“动态演示数值”时变化
        v = _ZERO
        v_sys = self.tel.v if self._lively else _ZERO
        for key, lbl in self._osd.items():
            lbl.setText("%.1f" % v[key])
        if hasattr(self, "_depth_gauge"):
            self._depth_gauge.set_depth(v["depth_m"])
        if hasattr(self, "_mon_osd"):
            for key2, lbl2 in self._mon_osd.items():
                lbl2.setText("%.1f" % v[key2])
        for _label, val, _key, f in self._sensor_vals:
            val.setText(f(v))
        for val, key, fmt, _col in self._sys_rows:
            val.setText(fmt % v_sys[key])
        self._bat_bar.setValue(int(v_sys["battery"]))
        # 顶部紧凑状态条
        if hasattr(self, "_tool_depth"):
            self._tool_depth.setText("%.1f m" % v["depth_m"])
        if hasattr(self, "_tool_volt"):
            self._tool_volt.setText("%.1f V" % v_sys["voltage"])
        if hasattr(self, "_tool_bat"):
            self._tool_bat.setText("%.0f%%" % v_sys["battery"])
        if self._status_rows:
            self._status_rows[5][1].setText("%.1f °C" % v["motor_temp"])
        # 遥测记录（缺省归零）
        if self._recording:
            self._tele_rows.append((now.strftime("%H:%M:%S"),
                                    round(v["depth_m"], 1), round(v["yaw_deg"], 1),
                                    round(v["pitch_deg"], 1), round(v["roll_deg"], 1),
                                    round(v["water_temp"], 1), round(v["voltage"], 1)))
            if len(self._tele_rows) > 5000:
                self._tele_rows.pop(0)
        self._refresh_data_stats()
        self._update_sonar()
        self._update_auto_data()

    def _flash(self, text, level="ok"):
        if self._log is not None:
            self._log.append_log(level, text)
        self._log_rows.append((datetime.now().strftime("%H:%M:%S"), level, text))
        if hasattr(self, "_logs_table"):
            self._refresh_logs_table()
        self.logger.info(text)

    def _push_log(self, text):
        pass  # signal 保留

    def _nav(self, name, page):
        for b in self._nav_items:
            b.setChecked(b.text() == name)
        self._flash("切换到：%s" % name, "info")
        if page is None:
            self._flash("%s：演示占位（待接入该模块数据）" % name, "info")
            return
        if hasattr(self, "_stack"):
            self._stack.setCurrentIndex(page)
        if name == "日志信息":
            self._refresh_logs_table()
        elif name == "数据管理":
            self._refresh_data_stats()

    # =====================================================================
    def closeEvent(self, event):
        try:
            self.clock_timer.stop()
            self.disconnect_91()
            self.logger.close()
        except Exception:
            pass
        super(Dashboard, self).closeEvent(event)


_ROOT_QSS = """
QWidget#root { background:qlineargradient(x1:0,y1:0,x2:1,y2:1,
    stop:0 %(bg0)s, stop:0.55 %(bg1)s, stop:1 %(bg0)s); }
QLabel { color:%(sub)s; background:transparent; }
QLabel[mono="true"] { font-family:"Consolas","Cascadia Mono","Microsoft YaHei UI"; }
QLabel#video { background:#050b16; border:1px solid #27406b; border-radius:8px;
    color:#7c93b5; font-size:15px; }

/* ---- 按钮：暗底亮字，hover 发光，pressed 位移 ---- */
QPushButton {
    background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #17325c,stop:1 #10233f);
    color:#d7ecff; border:1px solid #2f5186; border-radius:7px;
    padding:5px 10px; font-size:12px;
}
QPushButton:hover { border-color:%(cyan)s; background:#1c3f70;
    color:#ffffff; }
QPushButton:pressed { padding-top:6px; padding-left:11px;
    background:#0c1f38; }
QPushButton:disabled { color:#4c5c72; border-color:#22334f; }

QPushButton#solidBtn { background:#1b3d63; color:#eaf7ff;
    border:1px solid %(cyan)s; font-weight:700; min-height:26px; }
QPushButton#solidBtn:hover { background:%(cyan)s; color:#04121f;
    border-color:#8fe4ff; }
QPushButton#iconBtn { border:none; background:transparent; font-size:18px;
    color:%(sub)s; border-radius:8px; }
QPushButton#iconBtn:hover { background:rgba(49,196,243,40); color:#fff; }

QPushButton#navBtn { text-align:left; padding:8px 14px; border-radius:8px;
    border:none; color:%(sub)s; font-size:13px; }
QPushButton#navBtn:hover { background:rgba(49,196,243,30); color:#eaf7ff; }
QPushButton#navBtn:checked { background:rgba(49,196,243,44); color:#ffffff; font-weight:800;
    border-left:4px solid #9fe7ff; }

QPushButton#moveBtn { min-width:58px; min-height:36px; background:#123052;
    border:1px solid #2b6da3; color:#cfe7ff; border-radius:10px; font-weight:800; }
QPushButton#moveBtn:hover { border-color:#9fe7ff; background:#1c4a7d;
    color:#fff; }
QPushButton#moveBtn:pressed { background:%(cyan)s; color:#04121f; }
QPushButton#moveBtn:disabled { background:#101d33; color:#4a5d78;
    border-color:#1c2c47; }

QPushButton#tabOn { border-bottom:2px solid %(cyan)s; color:#fff; background:transparent; }
QPushButton#tabOff { border:none; background:transparent; color:%(sub)s; }
QPushButton#ghostBtn { background:rgba(15,30,55,200); border:1px solid #33577f;
    color:%(sub)s; border-radius:6px; padding:4px 10px; }
QPushButton#ghostBtn:hover { border-color:%(cyan)s; color:#fff; }
QPushButton#ghostBtn:checked { background:rgba(49,196,243,80); color:#fff; }

/* ---- 输入 / 进度条 / 滑动条 / 复选框 / 日志 ---- */
QLineEdit { background:#0c1830; color:#fff; border:1px solid #2a4a7a;
    border-radius:6px; padding:4px 8px; }
QLineEdit:focus { border-color:%(cyan)s; }

QProgressBar { background:#0b1730; border:1px solid #23436e; border-radius:4px;
    text-align:center; }
QProgressBar::chunk { border-radius:3px; }
QProgressBar#batBar::chunk {
    background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #1f9a5f,stop:1 %(green)s); }
QProgressBar#depthBar::chunk { border-radius:2px; }
QProgressBar#depthBar { border:1px solid rgba(49,196,243,140); border-radius:5px;
    background:rgba(6,14,28,220); }
QProgressBar#depthBar::chunk { border-radius:3px; }

QCheckBox { color:#d7ecff; spacing:6px; }
QCheckBox::indicator { width:15px; height:15px; border-radius:4px;
    border:1px solid #2f5186; background:#0c1830; }
QCheckBox::indicator:checked { background:%(cyan)s; border-color:%(cyan)s; }

QTextBrowser#alarm { background:rgba(8,16,32,200);
    border:1px solid #22395e; border-radius:8px;
    color:#cfe4ff; font-size:12px; padding:4px; }

QSlider::groove:horizontal { height:6px; background:#14233f; border-radius:3px; }
QSlider::handle:horizontal { width:16px; margin:-6px 0; border-radius:8px;
    background:%(cyan)s; border:1px solid #bdeaff; }
QSlider::sub-page:horizontal { background:rgba(49,196,243,120); border-radius:3px; }
""" % {"bg0": BG0, "bg1": BG1, "cyan": CYAN, "green": GREEN, "sub": TXT_SUB}


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    # 全局无衬线字体；数字走等宽（见 mono 属性 QSS）
    app.setFont(QFont("Microsoft YaHei UI", 10))
    w = Dashboard()
    w.setWindowTitle("智能水下清洁机器人控制系统 V1.0 · 监控大屏")
    w.resize(1600, 940)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
