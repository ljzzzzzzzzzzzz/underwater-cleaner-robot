#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
p91_cam_server.py —— 树莓派 91 主控·摄像头推送服务（上位机直连）

协议（与上位机 FrameReceiver 完全一致）：
  * 监听 0.0.0.0:12345
  * 同一连接上持续下发 JPEG 帧：4 字节大端长度 + 帧数据
  * 可收到 ASCII 指令行（7 字段 / 0000 断开）；本服务只转发画面，
    指令仅作日志记录，不回复文本（真机视频通道也不混文本应答）

用法： python3 p91_cam_server.py [--port 12345] [--fps 15] [--quality 70]
"""
from __future__ import annotations

import argparse
import socket
import struct
import threading
import time

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit("需要 opencv：sudo apt install python3-opencv 或 pip install opencv-python") from exc

FRAME_W, FRAME_H = 640, 480


class CamServer(object):
    def __init__(self, host="0.0.0.0", port=12345, fps=15, quality=70, w=FRAME_W, h=FRAME_H):
        self.host = host
        self.port = port
        self.fps = max(1, min(30, int(fps)))
        self.quality = max(20, min(95, int(quality)))
        self.w, self.h = int(w), int(h)
        self.running = False
        self._srv = None
        self._clients = []
        self._cam = None
        self._cam_lock = threading.Lock()

    # ---------- 对外 ----------
    def start(self):
        self._open_camera()
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(5)
        self.running = True
        threading.Thread(target=self._accept_loop, daemon=True).start()
        print("[cam] 监听 {}:{}  (fps={}, quality={}, size={}x{})".format(
            self.host, self.port, self.fps, self.quality, self.w, self.h), flush=True)
        print("[cam] 摄像头推送已就绪，等待上位机连接...", flush=True)

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
        if self._cam is not None:
            self._cam.release()

    # ---------- 摄像头 ----------
    def _open_camera(self):
        self._cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        if not self._cam.isOpened():
            raise RuntimeError("摄像头打开失败（/dev/video0）")

    def _capture(self):
        with self._cam_lock:
            ok, frame = self._cam.read()
        if not ok or frame is None:
            return None
        success, jpg = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        return jpg.tobytes() if success else None

    # ---------- 内部 ----------
    def _accept_loop(self):
        while self.running:
            try:
                sock, addr = self._srv.accept()
            except OSError:
                break
            self._clients.append(sock)
            print("[cam] 上位机已连接: {}".format(addr), flush=True)
            threading.Thread(target=self._client_loop, args=(sock, addr), daemon=True).start()

    def _client_loop(self, sock, addr):
        sock.settimeout(0.05)
        buf = b""
        next_tick = time.time()
        interval = 1.0 / self.fps
        try:
            while self.running:
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
                            self._on_command(buf.decode("utf-8", "replace").strip())
                            buf = b""
                except socket.timeout:
                    pass
                now = time.time()
                if now >= next_tick:
                    payload = self._capture()
                    if payload is not None:
                        sock.sendall(struct.pack("!I", len(payload)) + payload)
                    next_tick = now + interval
                time.sleep(0.005)
        except (ConnectionError, OSError):
            pass
        finally:
            try:
                sock.close()
            except OSError:
                pass
            if sock in self._clients:
                self._clients.remove(sock)

    def _on_command(self, line):
        # 只记指令，不回文本（视频通道不混应答）
        print("[cam] 指令: {}".format(line), flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description="树莓派 91 主控·摄像头推送服务")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=12345)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--quality", type=int, default=70)
    args = ap.parse_args(argv)

    server = CamServer(args.host, args.port, args.fps, args.quality)
    try:
        server.start()
    except RuntimeError as exc:
        print("[cam] {}".format(exc), flush=True)
        return 1
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[cam] 正在退出...", flush=True)
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
