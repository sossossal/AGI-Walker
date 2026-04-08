@echo off
REM AGI-Walker Web-Godot 快速开始脚本 (Windows)
REM 一键启动完整开发环境

setlocal enabledelayedexpansion

REM 设置颜色 (需要 Windows 10+)
set "GREEN=[92m"
set "YELLOW=[93m"
set "RED=[91m"
set "BLUE=[94m"
set "RESET=[0m"

REM 主菜单
:menu
cls
echo.
echo ============================================================
echo  AGI-Walker Web-Godot 快速开始
echo ============================================================
echo.
echo 请选择操作:
echo.
echo   1) 完全启动 (启动 Web 服务器 + Godot)
echo   2) 仅启动 Web 服务器
echo   3) 仅启动 Godot
echo   4) 运行集成测试
echo   5) 检查环境
echo   6) 清理临时文件
echo   0) 退出
echo.
set /p choice="请输入选项 [0-6]: "

if "%choice%"=="1" goto complete_start
if "%choice%"=="2" goto start_server
if "%choice%"=="3" goto start_godot
if "%choice%"=="4" goto run_tests
if "%choice%"=="5" goto check_env
if "%choice%"=="6" goto cleanup
if "%choice%"=="0" goto end
goto invalid

:invalid
echo.
echo 无效选项，请重试
echo.
pause
goto menu

REM 检查环境
:check_env
cls
echo.
echo ============================================================
echo  环境检查
echo ============================================================
echo.

echo 检查 Python...
python --version >nul 2>&1
if !errorlevel! equ 0 (
    for /f "tokens=*" %%i in ('python --version') do echo ✓ %%i
) else (
    echo ✗ Python 未安装
    goto check_env_end
)

echo.
echo 检查虚拟环境...
if exist test_env (
    echo ✓ 虚拟环境存在
) else (
    echo ✗ 虚拟环境不存在
    echo   请运行: python -m venv test_env
)

echo.
echo 检查关键文件...
set files_ok=1

if not exist "godot_project\scripts\web_godot_client.gd" (
    echo ✗ 文件缺失: godot_project\scripts\web_godot_client.gd
    set files_ok=0
) else (
    echo ✓ godot_project\scripts\web_godot_client.gd
)

if not exist "web_panel\static\js\web-godot-client.js" (
    echo ✗ 文件缺失: web_panel\static\js\web-godot-client.js
    set files_ok=0
) else (
    echo ✓ web_panel\static\js\web-godot-client.js
)

if not exist "web_panel\static\godot-control.html" (
    echo ✗ 文件缺失: web_panel\static\godot-control.html
    set files_ok=0
) else (
    echo ✓ web_panel\static\godot-control.html
)

if not exist "tests\test_web_godot_integration.py" (
    echo ✗ 文件缺失: tests\test_web_godot_integration.py
    set files_ok=0
) else (
    echo ✓ tests\test_web_godot_integration.py
)

echo.
echo 检查 Godot...
where godot >nul 2>&1
if !errorlevel! equ 0 (
    echo ✓ Godot 已安装
) else (
    echo ✗ Godot 未找到 (可选)
)

:check_env_end
echo.
if %files_ok% equ 1 (
    echo 环境检查通过！
) else (
    echo 某些文件缺失，请检查
)
echo.
pause
goto menu

REM 初始化环境
:init_env
echo.
echo 初始化 Python 虚拟环境...
if not exist test_env (
    echo 创建虚拟环境...
    python -m venv test_env
)

echo 激活虚拟环境...
call test_env\Scripts\activate.bat

echo 安装依赖...
python -m pip install -q --upgrade pip
python -m pip install -q -e ".[dev]"

echo ✓ 环境初始化完成
exit /b 0

REM 配置 Web workflow 环境变量文件
:configure_web_panel_env
set "AGI_WALKER_WEB_ENV_FILE="
if exist "deployment\web_panel.env" (
    set "AGI_WALKER_WEB_ENV_FILE=deployment\web_panel.env"
) else if exist "deployment\web_panel.env.example" (
    set "AGI_WALKER_WEB_ENV_FILE=deployment\web_panel.env.example"
)

if defined AGI_WALKER_WEB_ENV_FILE (
    echo ✓ Web workflow 配置: !AGI_WALKER_WEB_ENV_FILE!
) else (
    echo ! 未找到 Web workflow 配置文件，使用代码默认值
)
exit /b 0

REM 启动 Web 服务器
:start_server
call :init_env
if !errorlevel! neq 0 goto menu

cls
echo.
echo ============================================================
echo  启动 Web 服务器
echo ============================================================
echo.

echo 激活虚拟环境...
call test_env\Scripts\activate.bat
call :configure_web_panel_env

echo.
echo 启动 FastAPI 服务器...
echo.
echo 服务器地址: http://localhost:8000
echo 控制面板: http://localhost:8000/static/godot-control.html
echo API 文档: http://localhost:8000/docs
echo.
echo 按 Ctrl+C 停止服务器
echo.

python -m uvicorn web_panel.server:app --reload --host 0.0.0.0 --port 8000

goto menu

REM 启动 Godot
:start_godot
cls
echo.
echo ============================================================
echo  启动 Godot
echo ============================================================
echo.

where godot >nul 2>&1
if !errorlevel! neq 0 (
    echo ✗ Godot 未找到
    echo.
    echo 安装说明:
    echo   1. 从 https://godotengine.org/ 下载 Godot 4.x
    echo   2. 安装到系统路径 (PATH)
    echo   3. 重新运行此脚本
    echo.
    pause
    goto menu
)

echo 打开 Godot 编辑器...
start godot godot_project

echo ✓ Godot 已启动
echo.
echo 提示: 在编辑器中打开场景并按 F5 运行
echo.
pause
goto menu

REM 完全启动
:complete_start
call :init_env
if !errorlevel! neq 0 goto menu

cls
echo.
echo ============================================================
echo  完全启动
echo ============================================================
echo.

echo 激活虚拟环境...
call test_env\Scripts\activate.bat
call :configure_web_panel_env

echo.
echo 启动 Web 服务器 (后台)...
REM 使用 start /b 在后台启动
start /B python -m uvicorn web_panel.server:app --host 0.0.0.0 --port 8000

echo ✓ Web 服务器已启动
echo   访问: http://localhost:8000

REM 等待服务器启动
timeout /t 3 /nobreak

echo.
echo 启动 Godot...
where godot >nul 2>&1
if !errorlevel! equ 0 (
    start godot godot_project
    echo ✓ Godot 已启动
) else (
    echo ✗ Godot 未找到
)

echo.
echo ============================================================
echo 系统已启动!
echo ============================================================
echo.
echo 在浏览器中打开:
echo   http://localhost:8000/static/godot-control.html
echo.
echo 在编辑器中:
echo   1. 打开 godot_project
echo   2. 打开场景
echo   3. 按 F5 运行
echo.
pause

REM 尝试打开浏览器
start http://localhost:8000/static/godot-control.html

goto menu

REM 运行测试
:run_tests
call :init_env
if !errorlevel! neq 0 goto menu

cls
echo.
echo ============================================================
echo  运行集成测试
echo ============================================================
echo.

call test_env\Scripts\activate.bat

echo 运行 pytest...
echo.

pytest tests\test_web_godot_integration.py -v

if !errorlevel! equ 0 (
    echo.
    echo ✓ 所有测试通过！
) else (
    echo.
    echo ✗ 某些测试失败
)

echo.
pause
goto menu

REM 清理临时文件
:cleanup
cls
echo.
echo ============================================================
echo  清理临时文件
echo ============================================================
echo.

echo 删除 Python 缓存...
for /d /r . %%d in (__pycache__) do (
    if exist "%%d" (
        rmdir /s /q "%%d"
    )
)
del /s /q *.pyc >nul 2>&1

echo ✓ 缓存已清理

echo.
echo 删除测试临时文件...
rmdir /s /q htmlcov >nul 2>&1
rmdir /s /q .pytest_cache >nul 2>&1
del /q .coverage >nul 2>&1

echo ✓ 测试临时文件已清理

echo.
echo 清理完成
echo.
pause
goto menu

REM 退出
:end
echo.
echo 再见！
echo.
endlocal
exit /b 0
