@echo off
chcp 936 >nul
REM ============================================================
REM  智能水下清洁机器人控制系统 - 一键安装运行环境
REM  在本项目目录下创建 .venv（优先用 Python 3.9），安装依赖
REM ============================================================
cd /d "%~dp0"

set PY=py -3.9
%PY% -c "import sys" 2>nul || set PY=python

echo [1/3] 创建虚拟环境 .venv ...
%PY% -m venv .venv
if errorlevel 1 goto :err

echo [2/3] 安装依赖（先清华镜像，失败自动换官方源）...
".venv\Scripts\python.exe" -m pip install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple >nul 2>&1
".venv\Scripts\python.exe" -m pip install PySide2==5.15.2.1 pyqtgraph "opencv-python-headless==4.10.0.84" "numpy<2" -i https://pypi.tuna.tsinghua.edu.cn/simple
if errorlevel 1 (
  echo 清华镜像失败，改用官方源重试 ...
  ".venv\Scripts\python.exe" -m pip install PySide2==5.15.2.1 pyqtgraph "opencv-python-headless==4.10.0.84" "numpy<2"
  if errorlevel 1 goto :err
)

echo [3/3] 校验安装 ...
".venv\Scripts\python.exe" -c "import PySide2, pyqtgraph, cv2, numpy; print('OK  PySide2', PySide2.__version__, '| cv2', cv2.__version__)"
if errorlevel 1 goto :err

echo.
echo 环境就绪！可运行： 启动模拟91服务器.bat 与 启动上位机.bat
pause
exit /b 0

:err
echo 安装失败，请检查网络后重试。
pause
exit /b 1
