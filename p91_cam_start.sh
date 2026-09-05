#!/bin/bash
# p91_cam_start.sh —— 在树莓派上后台启动 91 摄像头推送服务（可脱离 SSH 会话存活）
cd /home/zhangjun || exit 1
pkill -f p91_cam_server.py 2>/dev/null
sleep 0.3
nohup setsid /usr/bin/python3 /home/zhangjun/p91_cam_server.py --port 12345 \
    </dev/null >/home/zhangjun/p91_cam.log 2>&1 &
disown
echo "started pid=$!"
sleep 1
echo "--- log ---"
cat /home/zhangjun/p91_cam.log 2>/dev/null
