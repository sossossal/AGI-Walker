#!/bin/bash
# AGI-Walker Web-Godot 快速开始脚本
# 一键启动完整开发环境

set -e  # 任何命令失败都退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC} $1"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
}

print_step() {
    echo -e "${YELLOW}➜${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# 主菜单
show_menu() {
    print_header "AGI-Walker Web-Godot 快速开始"
    echo ""
    echo "请选择操作:"
    echo ""
    echo "  1) 完全启动 (启动 Web 服务器 + Godot)"
    echo "  2) 仅启动 Web 服务器"
    echo "  3) 仅启动 Godot"
    echo "  4) 运行集成测试"
    echo "  5) 检查环境"
    echo "  6) 清理临时文件"
    echo "  7) Docker 部署"
    echo "  0) 退出"
    echo ""
    read -p "请输入选项 [0-7]: " choice
}

# 检查环境
check_environment() {
    print_header "环境检查"
    
    local all_ok=true
    
    # 检查 Python
    if command -v python &> /dev/null; then
        python_version=$(python --version 2>&1)
        print_success "Python: $python_version"
    else
        print_error "Python 未安装"
        all_ok=false
    fi
    
    # 检查虚拟环境
    if [ -d "test_env" ]; then
        print_success "虚拟环境存在"
    else
        print_error "虚拟环境不存在，请运行: python -m venv test_env"
        all_ok=false
    fi
    
    # 检查关键文件
    local files=(
        "godot_project/scripts/web_godot_client.gd"
        "godot_project/scripts/web_godot_integration.gd"
        "web_panel/static/js/web-godot-client.js"
        "web_panel/static/godot-control.html"
        "tests/test_web_godot_integration.py"
        "requirements.txt"
    )
    
    for file in "${files[@]}"; do
        if [ -f "$file" ]; then
            print_success "文件存在: $file"
        else
            print_error "文件缺失: $file"
            all_ok=false
        fi
    done
    
    # 检查 Godot
    if command -v godot &> /dev/null; then
        godot_version=$(godot --version 2>&1 || echo "unknown")
        print_success "Godot: $godot_version"
    else
        print_error "Godot 未找到 (可选)"
    fi
    
    # 检查 Docker
    if command -v docker &> /dev/null; then
        print_success "Docker 已安装"
    else
        print_error "Docker 未安装 (可选)"
    fi
    
    echo ""
    if [ "$all_ok" = true ]; then
        print_success "所有检查通过！"
        return 0
    else
        print_error "某些检查失败，请解决后重试"
        return 1
    fi
}

# 初始化环境
init_environment() {
    print_step "初始化 Python 虚拟环境..."
    
    if [ ! -d "test_env" ]; then
        python -m venv test_env
        print_success "虚拟环境已创建"
    fi
    
    # 激活虚拟环境
    if [ -f "test_env/bin/activate" ]; then
        source test_env/bin/activate
    elif [ -f "test_env/Scripts/activate" ]; then
        source test_env/Scripts/activate
    fi
    
    print_step "安装依赖..."
    pip install -q --upgrade pip
    pip install -q -r requirements.txt
    pip install -q -r requirements-dev.txt
    print_success "依赖安装完成"
}

configure_web_panel_env() {
    local env_file=""

    if [ -f "deployment/web_panel.env" ]; then
        env_file="deployment/web_panel.env"
    elif [ -f "deployment/web_panel.env.example" ]; then
        env_file="deployment/web_panel.env.example"
    fi

    if [ -n "$env_file" ]; then
        export AGI_WALKER_WEB_ENV_FILE="$env_file"
        print_success "Web workflow 配置: $AGI_WALKER_WEB_ENV_FILE"
    else
        unset AGI_WALKER_WEB_ENV_FILE
        print_step "未找到 Web workflow 配置文件，使用代码默认值"
    fi
}

# 启动 Web 服务器
start_web_server() {
    print_header "启动 Web 服务器"
    
    # 激活虚拟环境
    if [ -f "test_env/bin/activate" ]; then
        source test_env/bin/activate
    elif [ -f "test_env/Scripts/activate" ]; then
        source test_env/Scripts/activate
    fi
    configure_web_panel_env
    
    print_step "启动 FastAPI 服务器..."
    echo ""
    echo "服务器地址: http://localhost:8000"
    echo "控制面板: http://localhost:8000/static/godot-control.html"
    echo "API 文档: http://localhost:8000/docs"
    echo ""
    echo "按 Ctrl+C 停止服务器"
    echo ""
    
    python -m uvicorn web_panel.server:app --reload --host 0.0.0.0 --port 8000
}

# 启动 Godot
start_godot() {
    print_header "启动 Godot"
    
    if command -v godot &> /dev/null; then
        print_step "打开 Godot 编辑器..."
        godot godot_project &
        print_success "Godot 已启动（PID: $!）"
        print_step "提示: 在编辑器中打开场景并按 F5 运行"
    else
        print_error "Godot 未找到，请确保已安装"
        echo ""
        echo "安装说明:"
        echo "  1. 从 https://godotengine.org/ 下载 Godot 4.x"
        echo "  2. 将 godot 可执行文件添加到 PATH"
        echo "  3. 重新运行此脚本"
    fi
}

# 启动完整环境
start_complete() {
    print_header "完全启动"
    print_step "初始化环境..."
    init_environment
    
    print_step "启动 Web 服务器（后台）..."
    if [ -f "test_env/bin/activate" ]; then
        source test_env/bin/activate
    elif [ -f "test_env/Scripts/activate" ]; then
        source test_env/Scripts/activate
    fi
    configure_web_panel_env
    
    python -m uvicorn web_panel.server:app --host 0.0.0.0 --port 8000 > logs/server.log 2>&1 &
    SERVER_PID=$!
    print_success "Web 服务器已启动 (PID: $SERVER_PID)"
    
    # 等待服务器启动
    sleep 2
    
    # 检查服务器是否响应
    if curl -s http://localhost:8000 > /dev/null; then
        print_success "Web 服务器已就绪"
        echo ""
        echo "在浏览器中打开:"
        echo "  http://localhost:8000/static/godot-control.html"
        echo ""
    else
        print_error "Web 服务器未响应"
    fi
    
    print_step "启动 Godot..."
    if command -v godot &> /dev/null; then
        godot godot_project &
        GODOT_PID=$!
        print_success "Godot 已启动 (PID: $GODOT_PID)"
    else
        print_error "Godot 未找到"
    fi
    
    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo "系统已启动!"
    echo ""
    echo "运行中的服务:"
    echo "  • Web 服务器 (PID: $SERVER_PID) - http://localhost:8000"
    echo "  • Godot        (PID: $GODOT_PID)  - 编辑器"
    echo ""
    echo "按 Enter 继续..."
    read
    
    print_step "打开浏览器..."
    if command -v xdg-open &> /dev/null; then
        xdg-open "http://localhost:8000/static/godot-control.html"
    elif command -v open &> /dev/null; then
        open "http://localhost:8000/static/godot-control.html"
    elif command -v start &> /dev/null; then
        start "http://localhost:8000/static/godot-control.html"
    fi
}

# 运行集成测试
run_tests() {
    print_header "运行集成测试"
    
    print_step "初始化环境..."
    init_environment
    
    if [ -f "test_env/bin/activate" ]; then
        source test_env/bin/activate
    elif [ -f "test_env/Scripts/activate" ]; then
        source test_env/Scripts/activate
    fi
    
    print_step "运行 pytest..."
    echo ""
    
    if pytest tests/test_web_godot_integration.py -v; then
        print_success "所有测试通过！"
    else
        print_error "某些测试失败"
        exit 1
    fi
}

# 清理临时文件
cleanup() {
    print_header "清理临时文件"
    
    print_step "删除 Python 缓存..."
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    find . -type f -name "*.pyc" -delete 2>/dev/null || true
    print_success "缓存已清理"
    
    print_step "删除测试临时文件..."
    rm -rf htmlcov .pytest_cache .coverage 2>/dev/null || true
    print_success "测试临时文件已清理"
    
    print_step "删除日志..."
    rm -rf logs/*.log 2>/dev/null || true
    print_success "日志已清理"
    
    print_success "清理完成"
}

# Docker 部署
docker_deploy() {
    print_header "Docker 部署"
    
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装"
        echo "请从 https://docker.com 下载并安装 Docker"
        exit 1
    fi
    
    print_step "构建 Docker 镜像..."
    docker build -t agi-walker-server .
    print_success "镜像已构建: agi-walker-server"
    
    print_step "启动容器..."
    docker run -p 8000:8000 --name agi-walker-server agi-walker-server
    
    echo ""
    echo "容器已启动，访问: http://localhost:8000"
}

# 主循环
while true; do
    show_menu
    
    case $choice in
        1)
            start_complete
            ;;
        2)
            init_environment
            start_web_server
            ;;
        3)
            start_godot
            ;;
        4)
            run_tests
            ;;
        5)
            check_environment
            ;;
        6)
            cleanup
            ;;
        7)
            docker_deploy
            ;;
        0)
            print_success "再见！"
            exit 0
            ;;
        *)
            print_error "无效选项"
            ;;
    esac
    
    echo ""
    read -p "按 Enter 继续..."
done
