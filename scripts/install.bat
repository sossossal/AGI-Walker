@echo off
REM AGI-Walker 一键安装脚本 (Windows)

echo =============================
echo AGI-Walker 安装脚本
echo =============================
echo.

REM 检查 Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ❌ Python 未安装
    echo 请先安装 Python 3.8+
    pause
    exit /b 1
)

echo ✅ Python 已安装
python --version

REM 创建虚拟环境
echo.
echo 📦 创建虚拟环境...
python -m venv venv
call venv\Scripts\activate.bat

REM 升级 pip
echo.
echo ⬆️  升级 pip...
python -m pip install --upgrade pip

REM 安装依赖
echo.
echo 📥 安装依赖...
python -m pip install -e .

REM 可选: MuJoCo
set /p MUJOCO="是否安装 MuJoCo 物理引擎? (y/n): "
if /i "%MUJOCO%"=="y" (
    echo   - MuJoCo
    pip install mujoco
)

REM 可选: PyQt6
set /p PYQT="是否安装 PyQt6 (任务编辑器 GUI)? (y/n): "
if /i "%PYQT%"=="y" (
    echo   - PyQt6
    pip install PyQt6
)

REM 运行测试
echo.
echo 🧪 运行测试...
python tests\test_integration.py

REM 完成
echo.
echo =============================
echo ✅ 安装完成!
echo =============================
echo.
echo 下一步:
echo   1. 激活虚拟环境: venv\Scripts\activate.bat
echo   2. 运行演示: python examples\zenoh_ros2_demo.py
echo   3. 查看文档: type README.md
echo.
echo Happy coding! 🎉
pause
