# -*- coding: utf-8 -*-
"""
智能水下清洁机器人控制系统 V1.0 —— 上位机入口（大屏版主界面）
运行：python main.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dashboard  # noqa: E402

if __name__ == "__main__":
    dashboard.main()
