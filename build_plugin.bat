@echo off
REM AGI-Walker C++ 插件编译脚本 (Windows)
REM 使用 MinGW-w64

echo ========================================
echo AGI-Walker C++ 插件编译
echo ========================================

REM 检查工具
where scons >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [错误] 未找到 scons，请先安装:
    echo   pip install scons
    exit /b 1
)

REM 检查目录
if not exist "gdextension_src" (
    echo [错误] 未找到 gdextension_src 目录
    echo 请在项目根目录运行此脚本
    exit /b 1
)

cd gdextension_src

echo.
echo [1/3] 编译 godot-cpp (Debug)...
cd godot-cpp
scons platform=windows target=template_debug -j4
if %ERRORLEVEL% NEQ 0 (
    echo [错误] godot-cpp 编译失败
    exit /b 1
)

echo.
echo [2/3] 编译 godot-cpp (Release)...
scons platform=windows target=template_release -j4
if %ERRORLEVEL% NEQ 0 (
    echo [错误] godot-cpp 编译失败
    exit /b 1
)

cd ..

echo.
echo [3/3] 编译项目插件...
scons platform=windows -j4
if %ERRORLEVEL% NEQ 0 (
    echo [错误] 插件编译失败
    exit /b 1
)

echo.
echo ========================================
echo 编译完成!
echo ========================================
echo.
echo 输出文件位于: gdextension_src\bin\
dir bin\*.dll
echo.
echo 下一步:
echo   1. 打开 Godot 编辑器
echo   2. 加载项目: godot_project\project.godot
echo   3. 启用插件: 项目设置 - 插件
echo.

pause
