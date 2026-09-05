# -*- coding: utf-8 -*-

import json
import os


DEFAULT_CONFIG = {
    "robot91": {
        "ip": "192.168.1.91",
        "port": 12345,
        "thruster_layout": "normal",
    },
    "system": {
        "log_dir": "logs",
        "data_dir": "data",
        "auto_reconnect": True,
        "reconnect_interval": 3,
        "max_reconnect_attempts": 10,
        "feedback_poll_interval": 300,
    },
}


class ConfigManager:
    def __init__(self, config_path="config.json"):
        self._config_path = config_path
        self._data = {}
        self.load()

    def load(self):
        if os.path.exists(self._config_path):
            try:
                with open(self._config_path, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
            except (json.JSONDecodeError, IOError):
                loaded = {}

            self._data = self._deep_merge(DEFAULT_CONFIG.copy(), loaded)
        else:
            self._data = DEFAULT_CONFIG.copy()
            self.save()

    def save(self):
        try:
            with open(self._config_path, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=2, ensure_ascii=False)
        except IOError:
            pass

    def _deep_merge(self, base, override):
        result = base.copy()
        for k, v in override.items():
            if k in result and isinstance(result[k], dict) and isinstance(v, dict):
                result[k] = self._deep_merge(result[k], v)
            else:
                result[k] = v
        return result

    def get(self, *keys):
        node = self._data
        for k in keys:
            if isinstance(node, dict):
                node = node.get(k, None)
            else:
                return None
        return node

    def set(self, *keys_and_value):
        if len(keys_and_value) < 2:
            return
        value = keys_and_value[-1]
        keys = keys_and_value[:-1]

        node = self._data
        for k in keys[:-1]:
            if k not in node:
                node[k] = {}
            node = node[k]

        node[keys[-1]] = value
        self.save()

    @property
    def robot_ip(self):          # 91 主控
        return self.get("robot91", "ip") or "192.168.1.91"

    @property
    def robot_port(self):        # 91 主控
        return self.get("robot91", "port") or 12345

    @property
    def auto_reconnect(self):
        return self.get("system", "auto_reconnect")

    def update_robot(self, ip, port):          # 91 主控
        self.set("robot91", "ip", ip)
        self.set("robot91", "port", int(port))

    def to_dict(self):
        return self._data.copy()
