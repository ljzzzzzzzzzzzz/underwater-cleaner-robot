@echo off
chcp 936 >nul
cd /d "%~dp0"

set VENV=.venv
if exist "..\.venv39\Scripts\python.exe" set VENV=..\.venv39
if not exist "%VENV%\Scripts\python.exe" (
  echo [错误] 未找到虚拟环境，请先运行 安装依赖.bat
  pause
  exit /b 1
)

echo 启动 智能水下清洁机器人控制系统 上位机 ...
"%VENV%\Scripts\python.exe" main.py
pause
