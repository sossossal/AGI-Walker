#!/bin/bash
# AGI-Walker 一键安装脚本 (Linux/macOS)

set -e

echo "🚀 AGI-Walker 安装脚本"
echo "======================="

# 检测操作系统
OS="$(uname -s)"
case "${OS}" in
    Linux*)     PLATFORM=Linux;;
    Darwin*)    PLATFORM=Mac;;
    *)          PLATFORM="UNKNOWN:${OS}"
esac

echo "检测到平台: $PLATFORM"

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 未安装"
    echo "请先安装 Python 3.8+"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "✅ Python 版本: $PYTHON_VERSION"

# 创建虚拟环境
echo ""
echo "📦 创建虚拟环境..."
python3 -m venv venv
source venv/bin/activate

# 升级 pip
echo ""
echo "⬆️  升级 pip..."
pip install --upgrade pip

# 安装依赖
echo ""
echo "📥 安装依赖..."

# 基础依赖
echo "  - 基础依赖"
pip install -r requirements.txt

# 可选: ROS 2 (仅 Linux)
if [ "$PLATFORM" = "Linux" ]; then
    read -p "是否安装 ROS 2 支持? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "  - ROS 2 依赖"
        sudo apt update
        sudo apt install -y ros-jazzy-desktop ros-jazzy-rclpy
    fi
fi

# 可选: MuJoCo
read -p "是否安装 MuJoCo 物理引擎? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "  - MuJoCo"
    pip install mujoco
fi

# 可选: PyQt6 (GUI)
read -p "是否安装 PyQt6 (任务编辑器 GUI)? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "  - PyQt6"
    pip install PyQt6
fi

# 运行测试
echo ""
echo "🧪 运行测试..."
python tests/test_integration.py

# 完成
echo ""
echo "="*60
echo "✅ 安装完成!"
echo "="*60
echo ""
echo "下一步:"
echo "  1. 激活虚拟环境: source venv/bin/activate"
echo "  2. 运行演示: python examples/zenoh_ros2_demo.py"
echo "  3. 查看文档: cat README.md"
echo ""
echo "Happy coding! 🎉"
