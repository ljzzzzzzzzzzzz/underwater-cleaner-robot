# -*- coding: utf-8 -*-
"""
video_receiver.py —— 视频帧接收线程（91 主控协议）

从 TCP 连接上持续读取「4 字节大端长度 + JPEG 帧」数据流，解出图像后通过
frame_ready 信号发出；连接断开/出错经 broken 信号通知。
为 dashboard.py 与 underwater_window.py 共用的公共组件（已从 underwater_window.py 拆出）。
"""
from __future__ import annotations

import socket
import struct
import threading

import cv2
import numpy as np
from PySide2.QtCore import QThread, Signal


class FrameReceiver(QThread):
    frame_ready = Signal(object)
    broken = Signal(str)

    def __init__(self, sock, parent=None):
        super(FrameReceiver, self).__init__(parent)
        self._sock = sock
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        sock = self._sock
        try:
            sock.settimeout(1.0)
            while not self._stop.is_set():
                hdr = self._recv_exact(sock, 4)
                if hdr is None:
                    break
                (n,) = struct.unpack("!I", hdr)
                if n <= 0 or n > 16 * 1024 * 1024:
                    continue
                data = self._recv_exact(sock, n)
                if data is None:
                    break
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    self.frame_ready.emit(img)
        except Exception as exc:  # noqa: BLE001
            self.broken.emit(str(exc))
        finally:
            self.broken.emit("视频接收线程已退出")

    @staticmethod
    def _recv_exact(sock, size):
        buf = b""
        while len(buf) < size:
            try:
                chunk = sock.recv(size - len(buf))
            except socket.timeout:
                continue
            except OSError:
                return None
            if not chunk:
                return None
            buf += chunk
        return buf
