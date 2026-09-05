# -*- coding: utf-8 -*-
"""
mock_pi91_server.py —— 树莓派91主控节点 本地模拟服务器（无真机调试用）

行为对齐真实 SMP_91.py 对上位机客户端暴露的接口：
  1) TCP 监听端口（默认 12345）
  2) 同一连接上持续下发 JPEG 视频帧：4 字节大端长度 + 帧数据
     （与 9.12_swj91_lunshuaxiangduo.py 客户端的收帧格式一致）
  3) 随时可收到 ASCII 指令行（robot_protocol 7 字段格式 / 0000 断开）
  4) 指令只作执行模拟，状态被画进视频帧 + 打印到控制台
     （真机协议中视频通道上不会混入文本应答，因此模拟也不回文本）

用法:
    python mock_pi91_server.py [--host 0.0.0.0] [--port 12345]
                               [--fps 15] [--quality 70]
"""
from __future__ import annotations

import argparse
import json
import select
import socket
import struct
import threading
import time
from datetime import datetime

try:
    import cv2
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit("需要 opencv/numpy：请先 pip install opencv-python-headless numpy") from exc

import robot_protocol as proto

FRAME_W, FRAME_H = 640, 480


class MockRobotState(object):
    """模拟的机器人状态（相当于真机反馈的状态寄存器）"""

    FIELDS = (
        "mode", "angle", "dir1", "spd1", "dir2", "spd2",
        "brush", "brush_speed", "light", "online_at", "cmd_count",
    )

    def __init__(self):
        self._lock = threading.Lock()
        self._t0 = time.time()
        self.reset()

    def reset(self):
        with self._lock:
            self.mode = proto.MODE_STOP
            self.angle = 90
            self.dir1 = 0
            self.spd1 = 0
            self.dir2 = 0
            self.spd2 = 0
            self.brush = "000"
            self.brush_speed = 0.0
            self.light = 0
            self.cmd_count = 0

    def apply(self, data):
        with self._lock:
            self.mode = data["mode"]
            self.angle = data["angle"]
            self.dir1 = data["dir1"]
            self.spd1 = data["spd1"]
            self.dir2 = data["dir2"]
            self.spd2 = data["spd2"]
            self.brush = data["brush_state"]
            self.brush_speed = data["brush_speed"]
            self.light = data["light"]
            self.cmd_count += 1

    def snapshot(self):
        with self._lock:
            return {
                "mode": self.mode,
                "angle": self.angle,
                "dir1": self.dir1,
                "spd1": self.spd1,
                "dir2": self.dir2,
                "spd2": self.spd2,
                "brush": self.brush,
                "brush_speed": self.brush_speed,
                "light": self.light,
                "uptime": time.time() - self._t0,
                "cmd_count": self.cmd_count,
                "running": self.mode == proto.MODE_START,
            }


def _make_frame(state: MockRobotState, seq: int, t: float) -> np.ndarray:
    """合成一帧“水下”画面：深蓝渐变 + 气泡 + 状态面板，模拟 91 摄像头。"""
    st = state.snapshot()

    base = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
    # 深水纵向渐变（上亮下暗）
    for y in range(FRAME_H):
        v = 70 - int(38 * y / FRAME_H)
        base[y, :, 0] = v + 40
        base[y, :, 1] = v
        base[y, :, 2] = v + 5

    rng = np.random.RandomState(int(t * 1000) % 65536)
    # 悬浮气泡
    for _ in range(14):
        bx = rng.randint(0, FRAME_W)
        by = rng.randint(0, FRAME_H)
        br = rng.randint(2, 9)
        cv2.circle(base, (bx, by), br, (200, 220, 235), 1, cv2.LINE_AA)
    # 光照效果（灯光亮度变化）
    if st["light"] > 0:
        lx, ly = FRAME_W // 2, FRAME_H // 3
        overlay = base.copy()
        cv2.circle(overlay, (lx, ly), 120 + int(st["light"] * 2),
                   (160, 200, 235), -1, cv2.LINE_AA)
        base = cv2.addWeighted(overlay, 0.25 * st["light"] / 100.0, base, 0.75, 0)

    # 时间戳 & 模拟"ROBOT-CAM-91"
    cv2.putText(base, "MOCK CAM 192.168.1.91:12345  (simulator)",
                (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 240, 255), 1, cv2.LINE_AA)
    cv2.putText(base, "{}  frame#{}".format(
        datetime.now().strftime("%Y-%m-%d %H:%M:%S"), seq),
        (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 240, 255), 1, cv2.LINE_AA)

    # 状态面板（画在画面下半部，模拟状态回传可视化）
    panel_y = FRAME_H - 130
    cv2.rectangle(base, (6, panel_y - 8), (FRAME_W - 6, FRAME_H - 4),
                  (18, 40, 60), -1)
    cv2.rectangle(base, (6, panel_y - 8), (FRAME_W - 6, FRAME_H - 4),
                  (90, 160, 210), 1)

    mode_txt = "RUNNING(10)" if st["running"] else "STANDBY(00)"
    cv2.putText(base, mode_txt, (16, panel_y + 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (90, 255, 160) if st["running"] else (200, 200, 200), 1, cv2.LINE_AA)
    line1 = "servo={}deg  L:{}{}  R:{}{}  cmd#{}".format(
        st["angle"],
        "+" if st["dir1"] > 0 else ("-" if st["dir1"] < 0 else "="),
        st["spd1"],
        "+" if st["dir2"] > 0 else ("-" if st["dir2"] < 0 else "="),
        st["spd2"],
        st["cmd_count"],
    )
    cv2.putText(base, line1, (16, panel_y + 42), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (235, 235, 245), 1, cv2.LINE_AA)
    line2 = "brush={} @{}%  light={}%  uptime={:.0f}s".format(
        st["brush"], int(st["brush_speed"]), st["light"], st["uptime"],
    )
    cv2.putText(base, line2, (16, panel_y + 66), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (235, 235, 245), 1, cv2.LINE_AA)
    cv2.putText(base, "safety: watchdog 5s | ranges servo0-180 pwm0-255",
                (16, panel_y + 90), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 190, 220), 1, cv2.LINE_AA)
    return base


class MockPi91Server(object):
    def __init__(self, host="127.0.0.1", port=12345, fps=15, quality=70):
        self.host = host
        self.port = port
        self.fps = max(1, min(30, int(fps)))
        self.quality = max(20, min(95, int(quality)))
        self.state = MockRobotState()
        self.running = False
        self._srv = None
        self._clients = []

    # ---------- 对外 ----------
    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(5)
        self.running = True
        threading.Thread(target=self._accept_loop, daemon=True).start()
        print("[mock91] 监听 {}:{}  (fps={}, quality={})".format(
            self.host, self.port, self.fps, self.quality))
        print("[mock91] 模拟状态已就绪。发送格式见 robot_protocol.py")

    def stop(self):
        self.running = False
        try:
            if self._srv:
                self._srv.close()
        except OSError:
            pass
        for sock in list(self._clients):
            try:
                sock.close()
            except OSError:
                pass
        self._clients.clear()

    # ---------- 内部 ----------
    def _accept_loop(self):
        while self.running:
            try:
                sock, addr = self._srv.accept()
            except OSError:
                break
            self._clients.append(sock)
            print("[mock91] 上位机已连接: {}".format(addr))
            threading.Thread(target=self._client_loop, args=(sock, addr), daemon=True).start()

    def _client_loop(self, sock, addr):
        sock.settimeout(0.05)
        buf = b""
        seq = 0
        next_tick = time.time()
        interval = 1.0 / self.fps
        try:
            while self.running:
                # 1) 收指令（一个 recv 块 = 一条指令；也兼容多行批量）
                try:
                    while True:
                        chunk = sock.recv(4096)
                        if not chunk:
                            raise ConnectionError("closed")
                        buf += chunk
                        while b"\n" in buf:
                            line, _, buf = buf.partition(b"\n")
                            line = line.strip()
                            if line:
                                self._on_command(line.decode("utf-8", "replace"))
                        if buf.strip():
                            # 没有换行的整块数据：按真实服务器“每次 recv 一条指令”处理
                            self._on_command(buf.decode("utf-8", "replace").strip())
                            buf = b""
                except socket.timeout:
                    pass
                # 2) 按 fps 节奏发一帧视频
                now = time.time()
                if now >= next_tick:
                    frame = _make_frame(self.state, seq, now)
                    ok, jpg = cv2.imencode(".jpg", frame,
                                           [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
                    if ok:
                        payload = jpg.tobytes()
                        sock.sendall(struct.pack("!I", len(payload)) + payload)
                        seq += 1
                    next_tick = now + interval
                time.sleep(0.01)
        except (ConnectionError, OSError) as exc:
            if self.running:
                print("[mock91] 客户端 {} 断开: {}".format(addr, exc))
        finally:
            try:
                sock.close()
            except OSError:
                pass
            if sock in self._clients:
                self._clients.remove(sock)

    def _on_command(self, line):
        try:
            data = proto.parse_command(line)
        except Exception as exc:
            print("[mock91] 指令解析异常 {}: {}".format(exc, line))
            return
        if data is None:
            print("[mock91] 忽略无法解析的指令: {!r}".format(line))
            return
        if data["type"] == "disconnect":
            print("[mock91] 收到断开指令(0000)")
            return
        self.state.apply(data)
        print("[mock91] {} -> {}".format(
            datetime.now().strftime("%H:%M:%S"), proto.describe_command(line)))


def main(argv=None):
    ap = argparse.ArgumentParser(description="树莓派91主控节点本地模拟服务器")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=proto.DEFAULT_PORT)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--quality", type=int, default=70)
    args = ap.parse_args(argv)

    server = MockPi91Server(args.host, args.port, args.fps, args.quality)
    server.start()
    print("[mock91] Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[mock91] 正在退出...")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
