#!/usr/bin/env bash
# AGI-Walker 代码覆盖率优化 - 快速命令参考

## 📊 覆盖率查询命令

# 查看整体覆盖率
coverage_overall() {
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=term \
        -q -m "not integration and not hardware"
}

# 查看模块级覆盖率
coverage_modules() {
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=term-missing \
        -q -m "not integration and not hardware"
}

# 生成 HTML 覆盖率报告
coverage_html() {
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=html \
        -q -m "not integration and not hardware"
    
    # 打开报告
    if [[ "$OSTYPE" == "darwin"* ]]; then
        open htmlcov/index.html
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        xdg-open htmlcov/index.html
    elif [[ "$OSTYPE" == "msys" ]]; then
        start htmlcov/index.html
    fi
}

# 只看最近增加的覆盖率
coverage_delta() {
    python -m pytest tests/ --cov=agi_walker --cov-report=term -q
}

## 🧪 测试运行命令

# 运行所有新增优化测试
run_optimization_tests() {
    python -m pytest \
        tests/test_skills_loader_enhanced.py \
        tests/test_cli.py \
        tests/test_error_handling.py \
        -v --tb=short
}

# 快速测试（仅新增）
run_quick_tests() {
    python -m pytest \
        tests/test_skills_loader_enhanced.py \
        tests/test_cli.py \
        tests/test_error_handling.py \
        -q
}

# 完整测试套件
run_full_tests() {
    python -m pytest tests/ \
        -v \
        -m "not integration and not hardware"
}

# 测试特定模块
run_module_tests() {
    local module=$1
    python -m pytest tests/ -k "$module" -v
}

# 显示最慢的 10 个测试
show_slow_tests() {
    python -m pytest tests/ --durations=10 -q
}

## 📈 分析命令

# 显示未覆盖的行
show_missing_lines() {
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=term-missing:skip-covered \
        -q
}

# 生成覆盖率JSON报告
generate_json_report() {
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=json \
        -q
}

# 生成XML报告（用于CI/CD）
generate_xml_report() {
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=xml \
        -q
}

## 🔍 调试命令

# 运行特定测试并显示详细输出
debug_test() {
    local test_name=$1
    python -m pytest "tests/${test_name}.py" -vvs --tb=long
}

# 运行测试并显示print输出
run_with_output() {
    python -m pytest tests/ -s -v
}

# 运行测试并进入pdb（debugger）
run_with_pdb() {
    local test_name=$1
    python -m pytest "tests/${test_name}.py" --pdb -s
}

## 📋 报告命令

# 生成完整的测试和覆盖率报告
generate_full_report() {
    echo "=== 测试执行 ==="
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=term \
        --cov-report=html \
        --cov-report=json \
        -v -q
    
    echo ""
    echo "=== 报告位置 ==="
    echo "HTML 报告: htmlcov/index.html"
    echo "JSON 报告: .coverage"
    echo "终端输出: 上方显示"
}

# 生成CI友好的报告
generate_ci_report() {
    python -m pytest tests/ \
        --cov=agi_walker \
        --cov-report=xml:coverage.xml \
        --cov-report=json:coverage.json \
        --junitxml=junit.xml \
        -q
}

## 🎯 快捷命令

# 检查并通过所有测试
verify_all() {
    echo "🔍 运行所有测试..."
    python -m pytest tests/ -q || return 1
    
    echo "📊 检查覆盖率..."
    python -m pytest tests/ --cov=agi_walker --cov-report=term -q
}

# 增量验证（仅新增测试）
verify_incremental() {
    python -m pytest \
        tests/test_skills_loader_enhanced.py \
        tests/test_cli.py \
        tests/test_error_handling.py \
        -q && echo "✅ 所有新增测试通过"
}

# 清理并重新生成覆盖率报告
clean_and_report() {
    rm -rf .coverage htmlcov/ coverage.xml
    coverage_html
}

## 📊 快速统计

# 统计测试数量
count_tests() {
    echo "总测试数:"
    python -m pytest --collect-only -q | tail -1
}

# 显示模块代码行数
count_lines() {
    local module=${1:-.}
    find "$module" -name "*.py" | xargs wc -l | tail -1
}

## 帮助

help_commands() {
    cat << 'EOF'
AGI-Walker 代码覆盖率优化命令参考

📊 覆盖率查询:
  coverage_overall     - 总体覆盖率
  coverage_modules     - 模块级详情
  coverage_html        - 生成HTML报告

🧪 测试运行:
  run_optimization_tests - 只运行新增优化测试
  run_quick_tests        - 快速测试
  run_full_tests         - 完整测试套件

📈 分析:
  show_missing_lines     - 显示未覆盖的行
  show_slow_tests        - 最慢的10个测试

🔍 调试:
  debug_test [name]      - 调试特定测试
  run_with_output        - 显示print输出
  run_with_pdb [name]    - 进入调试器

📋 报告:
  generate_full_report   - 完整报告
  generate_ci_report     - CI友好的报告

🎯 快捷:
  verify_all             - 验证所有测试
  verify_incremental     - 验证新增测试
  clean_and_report       - 清理并重新生成报告

使用示例:
  $ coverage_overall
  $ run_optimization_tests
  $ coverage_html
  $ debug_test test_cli
EOF
}

# 如果提供了参数，执行对应命令
if [ $# -eq 0 ]; then
    help_commands
else
    "$@"
fi
