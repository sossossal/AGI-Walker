@echo off
REM AGI-Walker 代码覆盖率优化 - Windows 快速命令

setlocal enabledelayedexpansion

if "%1"=="" (
    call :show_help
    exit /b 0
)

if "%1"=="coverage-overall" (
    python -m pytest tests/ --cov=agi_walker --cov-report=term -q -m "not integration and not hardware"
    exit /b !ERRORLEVEL!
)

if "%1"=="coverage-modules" (
    python -m pytest tests/ --cov=agi_walker --cov-report=term-missing -q -m "not integration and not hardware"
    exit /b !ERRORLEVEL!
)

if "%1"=="coverage-html" (
    python -m pytest tests/ --cov=agi_walker --cov-report=html -q -m "not integration and not hardware"
    start htmlcov\index.html
    exit /b !ERRORLEVEL!
)

if "%1"=="run-optimization" (
    python -m pytest tests\test_skills_loader_enhanced.py tests\test_cli.py tests\test_error_handling.py -v --tb=short
    exit /b !ERRORLEVEL!
)

if "%1"=="run-quick" (
    python -m pytest tests\test_skills_loader_enhanced.py tests\test_cli.py tests\test_error_handling.py -q
    exit /b !ERRORLEVEL!
)

if "%1"=="run-full" (
    python -m pytest tests\ -v -m "not integration and not hardware"
    exit /b !ERRORLEVEL!
)

if "%1"=="show-missing" (
    python -m pytest tests\ --cov=agi_walker --cov-report=term-missing:skip-covered -q
    exit /b !ERRORLEVEL!
)

if "%1"=="show-slow" (
    python -m pytest tests\ --durations=10 -q
    exit /b !ERRORLEVEL!
)

if "%1"=="verify-all" (
    echo 🔍 运行所有测试...
    python -m pytest tests\ -q
    if !ERRORLEVEL! NEQ 0 exit /b 1
    
    echo 📊 检查覆盖率...
    python -m pytest tests\ --cov=agi_walker --cov-report=term -q
    exit /b !ERRORLEVEL!
)

if "%1"=="verify-incremental" (
    python -m pytest tests\test_skills_loader_enhanced.py tests\test_cli.py tests\test_error_handling.py -q
    if !ERRORLEVEL! EQU 0 (
        echo ✅ 所有新增测试通过
    )
    exit /b !ERRORLEVEL!
)

if "%1"=="generate-full" (
    echo === 测试执行 ===
    python -m pytest tests\ --cov=agi_walker --cov-report=term --cov-report=html --cov-report=json -v -q
    
    echo.
    echo === 报告位置 ===
    echo HTML 报告: htmlcov\index.html
    echo JSON 报告: .coverage
    exit /b !ERRORLEVEL!
)

if "%1"=="generate-ci" (
    python -m pytest tests\ --cov=agi_walker --cov-report=xml:coverage.xml --cov-report=json:coverage.json --junitxml=junit.xml -q
    exit /b !ERRORLEVEL!
)

if "%1"=="clean" (
    if exist .coverage del .coverage
    if exist htmlcov rmdir /s /q htmlcov
    if exist coverage.xml del coverage.xml
    if exist coverage.json del coverage.json
    echo ✅ 清理完成
    exit /b 0
)

if "%1"=="help" (
    call :show_help
    exit /b 0
)

echo 未知命令: %1
call :show_help
exit /b 1

:show_help
echo.
echo AGI-Walker 代码覆盖率优化命令参考 (Windows)
echo.
echo 📊 覆盖率查询:
echo   coverage-utils coverage-overall    - 总体覆盖率
echo   coverage-utils coverage-modules    - 模块级详情
echo   coverage-utils coverage-html       - 生成HTML报告
echo.
echo 🧪 测试运行:
echo   coverage-utils run-optimization    - 只运行新增优化测试
echo   coverage-utils run-quick           - 快速测试
echo   coverage-utils run-full            - 完整测试套件
echo.
echo 📈 分析:
echo   coverage-utils show-missing        - 显示未覆盖的行
echo   coverage-utils show-slow           - 最慢的10个测试
echo.
echo 📋 报告:
echo   coverage-utils generate-full       - 完整报告
echo   coverage-utils generate-ci         - CI友好的报告
echo.
echo 🎯 快捷:
echo   coverage-utils verify-all          - 验证所有测试
echo   coverage-utils verify-incremental  - 验证新增测试
echo   coverage-utils clean               - 清理缓存文件
echo.
echo 使用示例:
echo   python coverage-utils coverage-overall
echo   python coverage-utils run-quick
echo   python coverage-utils coverage-html
echo.
