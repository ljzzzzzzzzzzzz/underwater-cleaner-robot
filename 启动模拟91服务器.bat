@echo off
chcp 65001 >nul
cd /d "%~dp0"

set VENV=.venv
if exist "..\.venv39\Scripts\python.exe" set VENV=..\.venv39
if not exist "%VENV%\Scripts\python.exe" (
  echo 未找到虚拟环境，请先运行 安装依赖.bat
  pause
  exit /b 1
)

echo 启动 树莓派91主控节点 本地模拟服务器 (127.0.0.1:12345) ...
echo 关闭窗口即停止。Ctrl+C 也可退出。
"%VENV%\Scripts\python.exe" -u mock_pi91_server.py --host 127.0.0.1 --port 12345
pause
