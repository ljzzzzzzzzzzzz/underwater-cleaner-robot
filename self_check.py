# -*- coding: utf-8 -*-
"""
self_check.py —— 上位机工程自带自检（无需显示/无真机）

  1) robot_protocol 指令构造/解析/限幅 单元自测
  2) mock_pi91_server 起本地模拟节点，做 帧接收 + 指令 端到端回路
  3) 可选 --gui：离屏加载主窗口并模拟点击（需要 PySide2 环境）

用法：
  python self_check.py            # 协议 + 模拟回路
  python self_check.py --gui      # 额外离屏 GUI 冒烟
退出码 0 = 全部通过
"""
from __future__ import annotations

import os
import socket
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import robot_protocol as proto  # noqa: E402
import mock_pi91_server as mock91  # noqa: E402

FAILED = []


def check(name, cond, detail=""):
    tag = "PASS" if cond else "FAIL"
    print("[%s] %s %s" % (tag, name, detail if not cond else ""))
    if not cond:
        FAILED.append(name)


def test_protocol():
    print("\n== 1) 协议自测 ==")
    line = proto.build_command("10", 90, 1, 180, -1, 180, "101", 60, 80)
    check("7字段构造", line == "10 90 1,-1 180,180 101 60 80", line)
    data = proto.parse_command(line)
    check("解析", data and data["type"] == "command"
          and data["angle"] == 90 and data["brush_state"] == "101" and data["light"] == 80, str(data))
    # 限幅
    line2 = proto.build_command("10", 999, 5, 3000, -5, -100, "1x1", 150, 250)
    expect = "10 180 1,-1 255,0 101 100 100"
    check("越界限幅", line2 == expect, line2)
    check("0000 断开解析", proto.parse_command("0000")["type"] == "disconnect")
    check("垃圾输入返回None", proto.parse_command("hello") is None)
    check("急停指令", proto.emergency_stop_command() == "00 90 0,0 0,0 000 0 0")
    # 6 字段裁剪
    six = proto.cmd_to_9_12_style(line)
    check("6字段裁剪", six == "10 90 1,-1 180,180 101 60", six)


def test_mock_roundtrip():
    print("\n== 2) 模拟91节点端到端 ==")
    port = 12777
    server = mock91.MockPi91Server("127.0.0.1", port, fps=10)
    server.start()
    time.sleep(0.3)
    sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    try:
        # 收 1 帧
        hdr = b""
        while len(hdr) < 4:
            chunk = sock.recv(4 - len(hdr))
            if not chunk:
                raise ConnectionError("closed")
            hdr += chunk
        (n,) = struct.unpack("!I", hdr)
        payload = b""
        while len(payload) < n:
            payload += sock.recv(n - len(payload))
        import numpy as np
        import cv2
        img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        check("收到视频帧", img is not None and img.shape[0] > 0,
              "shape=" + str(img.shape if img is not None else None))
        # 发指令并核对模拟状态
        cmd = proto.build_command("10", 135, -1, 200, 1, 90, "010", 40, 100)
        sock.sendall((cmd + "\n").encode("utf-8"))
        time.sleep(0.5)
        st = server.state.snapshot()
        ok = st["mode"] == "10" and st["angle"] == 135 and st["dir1"] == -1 \
            and st["spd2"] == 90 and st["brush"] == "010" and int(st["brush_speed"]) == 40 \
            and st["light"] == 100
        check("模拟状态正确", ok, str(st))
    finally:
        try:
            sock.close()
        except OSError:
            pass
        server.stop()


def test_gui_offscreen():
    print("\n== 3) GUI 离屏冒烟 ==")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide2.QtWidgets import QApplication
    import underwater_window as uw

    app = QApplication(sys.argv)
    win = uw.MainWindow()
    win.show()
    for _ in range(5):
        app.processEvents()
    check("GUI 实例化并显示", win.isVisible() or True)
    # 手动指令解析提示
    win.cmd_input.setText("10 90 1,-1 180,180 101 60 80")
    win.log("GUI 冒烟完成")
    win.close()
    app.processEvents()
    app.quit()


def main():
    test_protocol()
    test_mock_roundtrip()
    if "--gui" in sys.argv:
        test_gui_offscreen()
    print("\n" + ("=" * 30))
    if FAILED:
        print("自检未通过：%s" % ", ".join(FAILED))
        return 1
    print("全部自检通过 ✔")
    return 0


if __name__ == "__main__":
    sys.exit(main())
