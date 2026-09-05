# -*- coding: utf-8 -*-

import os
import time
import threading
from datetime import datetime


class Logger:
    def __init__(self, log_dir="logs", level="INFO", prefix="robot_host"):
        self._lock = threading.Lock()
        self._levels = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40}
        self._level = self._levels.get(level.upper(), 20)

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(log_dir, "%s_%s.log" % (prefix, stamp))
        self._can_log_path = os.path.join(log_dir, "%s_packet_%s.log" % (prefix, stamp))

        self._fp = open(self._log_path, "a", encoding="utf-8", buffering=1)
        self._can_fp = open(self._can_log_path, "a", encoding="utf-8", buffering=1)

        self.info(f"日志系统启动, 级别={level}, 文件={self._log_path}")

    def _timestamp(self):
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def _write(self, level, msg):
        line = f"[{self._timestamp()}] [{level}] {msg}"
        with self._lock:
            self._fp.write(line + "\n")
            self._fp.flush()

    def debug(self, msg):
        if self._level <= 10:
            self._write("DEBUG", msg)

    def info(self, msg):
        if self._level <= 20:
            self._write("INFO", msg)

    def warning(self, msg):
        if self._level <= 30:
            self._write("WARNING", msg)

    def error(self, msg):
        if self._level <= 40:
            self._write("ERROR", msg)

    def can_send(self, raw):
        line = f"[{self._timestamp()}] SEND > {raw}"
        with self._lock:
            self._can_fp.write(line + "\n")
            self._can_fp.flush()

    def can_recv(self, raw):
        line = f"[{self._timestamp()}] RECV < {raw}"
        with self._lock:
            self._can_fp.write(line + "\n")
            self._can_fp.flush()

    def close(self):
        self.info("日志系统关闭")
        with self._lock:
            self._fp.close()
            self._can_fp.close()
