# -*- coding: utf-8 -*-
"""
robot_protocol.py —— 智能水下清洁机器人控制系统 · 指令协议层

与树莓派91主控节点通信采用固定格式 ASCII 指令行（说明书附录A）：
    MODE ANGLE d1,d2 s1,s2 BRUSH_STATE BRUSH_SPEED LIGHT
例如：
    10 90 1,-1 180,180 101 60 80    启动采集 + 舵机90° + 左轮正转180 + 右轮反转180
                                     + 刷子1&3开启@60% + 灯光80%
    00 0 0,0 0,0 000 0 0            停止采集并全部停机
    0000                             断开连接

参数范围（与说明书中“参数限幅保护”一致）：
    MODE        10=启动(采集+运行)  00=停止(待机)
    ANGLE       0 ~ 180          （舵机）
    dir1,dir2   -1 / 0 / 1        （推进器方向：-1反转 0停止 1正转）
    s1,s2       0 ~ 255           （推进器 PWM）
    BRUSH_STATE 3 位 01 字符串      （三路清洗刷继电器）
    BRUSH_SPEED 0 ~ 100           （刷子转速百分比）
    LIGHT       0 ~ 100           （灯光亮度百分比）
"""
from __future__ import annotations

import re
from typing import Dict, List, Optional, Tuple

DEFAULT_IP = "192.168.1.91"
DEFAULT_PORT = 12345

MODE_START = "10"   # 开始采集并运行
MODE_STOP = "00"    # 停止采集并待机
CMD_DISCONNECT = "0000"

# ---------------- 取值范围 ----------------
ANGLE_MIN, ANGLE_MAX = 0, 180
SPEED_MIN, SPEED_MAX = 0, 255          # 推进器 PWM
PERCENT_MIN, PERCENT_MAX = 0, 100      # 刷子转速 / 灯光亮度
DIRS = (-1, 0, 1)

_COMMAND_RE = re.compile(
    r"^(?P<mode>10|00)\s+"
    r"(?P<angle>-?\d+)\s+"
    r"(?P<dir1>-?\d)\s*,\s*(?P<dir2>-?\d)\s+"
    r"(?P<spd1>-?\d+)\s*,\s*(?P<spd2>-?\d+)\s+"
    r"(?P<brush>[01]{3})\s+"
    r"(?P<brush_speed>-?\d+(?:\.\d+)?)\s+"
    r"(?P<light>-?\d+)\s*$"
)


def clamp(value, low, high):
    return max(low, min(high, int(round(value))))


def clamp_dir(value):
    if value < 0:
        return -1
    if value > 0:
        return 1
    return 0


def build_command(
    mode: str = MODE_STOP,
    angle: int = 90,
    dir1: int = 0,
    spd1: int = 0,
    dir2: int = 0,
    spd2: int = 0,
    brush_state: str = "000",
    brush_speed: float = 0.0,
    light: int = 0,
) -> str:
    """
    按 7 字段格式构造一条完整指令，含参数校验 / 限幅。
    返回可直接 send 的 ASCII 字符串。
    """
    mode = mode if mode in (MODE_START, MODE_STOP) else MODE_STOP
    angle = clamp(angle, ANGLE_MIN, ANGLE_MAX)
    dir1 = clamp_dir(dir1)
    dir2 = clamp_dir(dir2)
    spd1 = clamp(spd1, SPEED_MIN, SPEED_MAX)
    spd2 = clamp(spd2, SPEED_MIN, SPEED_MAX)
    # 刷子状态：三位 0/1，非法字符一律按 0
    bs_digits = []
    for ch in str(brush_state):
        if ch in "01" and len(bs_digits) < 3:
            bs_digits.append(ch)
        elif len(bs_digits) < 3:
            bs_digits.append("0")
    brush_state = "".join(bs_digits).ljust(3, "0")
    brush_speed = max(PERCENT_MIN, min(PERCENT_MAX, float(brush_speed)))
    light = clamp(light, PERCENT_MIN, PERCENT_MAX)

    return "{} {} {},{:d} {},{:d} {} {} {}".format(
        mode, angle, dir1, dir2, spd1, spd2, brush_state,
        int(brush_speed), light,
    )


def parse_command(line: str) -> Optional[Dict]:
    """解析收到的指令行；无法解析返回 None。"""
    if line is None:
        return None
    text = line.strip()
    if not text:
        return None
    if text == CMD_DISCONNECT:
        return {"type": "disconnect", "raw": text}
    m = _COMMAND_RE.match(text)
    if not m:
        return None
    parts = m.groupdict()
    return {
        "type": "command",
        "mode": parts["mode"],
        "angle": int(parts["angle"]),
        "dir1": int(parts["dir1"]),
        "dir2": int(parts["dir2"]),
        "spd1": int(parts["spd1"]),
        "spd2": int(parts["spd2"]),
        "brush_state": parts["brush"],
        "brush_speed": float(parts["brush_speed"]),
        "light": int(parts["light"]),
        "raw": text,
    }


def emergency_stop_command() -> str:
    """全系统安全停机指令：00 0 0,0 0,0 000 0 0"""
    return build_command(MODE_STOP, 90, 0, 0, 0, 0, "000", 0, 0)


def stop_only_command() -> str:
    return emergency_stop_command()


def describe_command(line: str) -> str:
    """给人看的指令描述，用于日志。"""
    data = parse_command(line)
    if not data:
        return line
    if data["type"] == "disconnect":
        return "断开连接(0000)"
    mode = "启动采集/运行" if data["mode"] == MODE_START else "停止/待机"
    return (
        "{} | 舵机={}° | 左轮 {}({}) | 右轮 {}({}) | 刷子={} @{}% | 灯={}%".format(
            mode,
            data["angle"],
            "正转" if data["dir1"] > 0 else ("反转" if data["dir1"] < 0 else "停止"),
            data["spd1"],
            "正转" if data["dir2"] > 0 else ("反转" if data["dir2"] < 0 else "停止"),
            data["spd2"],
            data["brush_state"],
            int(data["brush_speed"]),
            data["light"],
        )
    )


def cmd_to_9_12_style(line: str) -> str:
    """
    若真机只支持 6 字段（不含灯），去掉最后的灯字段。
    （保留此函数以便 6/7 字段切换调试）
    """
    data = parse_command(line)
    if not data or data["type"] != "command":
        return line
    return "{} {} {},{:d} {},{:d} {} {}".format(
        data["mode"], data["angle"], data["dir1"], data["dir2"],
        data["spd1"], data["spd2"], data["brush_state"],
        int(data["brush_speed"]),
    )


if __name__ == "__main__":
    # 自测
    samples = [
        "10 90 1,-1 180,180 101 60 80",
        "00 0 0,0 0,0 000 0 0",
        "10 200 1,1 300,300 111 150 150",  # 超范围 -> 应被限幅
        "0000",
        "garbage",
    ]
    for s in samples:
        print("IN :", s)
        print("parse:", parse_command(s))
    print("built:", build_command("10", 200, 1, 300, -1, 0, "101", 150, 150))
    print("emergency:", emergency_stop_command())
