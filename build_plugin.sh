#!/bin/bash
# AGI-Walker C++ 插件编译脚本 (Linux/macOS)

echo "========================================"
echo "AGI-Walker C++ 插件编译"
echo "========================================"

# 检查工具
if ! command -v scons &> /dev/null; then
    echo "[错误] 未找到 scons，请先安装:"
    echo "  sudo apt-get install scons  # Linux"
    echo "  brew install scons          # macOS"
    exit 1
fi

# 检查目录
if [ ! -d "gdextension_src" ]; then
    echo "[错误] 未找到 gdextension_src 目录"
    echo "请在项目根目录运行此脚本"
    exit 1
fi

# 检测平台
PLATFORM="linux"
if [[ "$OSTYPE" == "darwin"* ]]; then
    PLATFORM="macos"
fi

cd gdextension_src

echo ""
echo "[1/3] 编译 godot-cpp (Debug)..."
cd godot-cpp
scons platform=$PLATFORM target=template_debug -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)
if [ $? -ne 0 ]; then
    echo "[错误] godot-cpp 编译失败"
    exit 1
fi

echo ""
echo "[2/3] 编译 godot-cpp (Release)..."
scons platform=$PLATFORM target=template_release -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)
if [ $? -ne 0 ]; then
    echo "[错误] godot-cpp 编译失败"
    exit 1
fi

cd ..

echo ""
echo "[3/3] 编译项目插件..."
scons platform=$PLATFORM -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)
if [ $? -ne 0 ]; then
    echo "[错误] 插件编译失败"
    exit 1
fi

echo ""
echo "========================================"
echo "编译完成!"
echo "========================================"
echo ""
echo "输出文件位于: gdextension_src/bin/"
ls -lh bin/
echo ""
echo "下一步:"
echo "  1. 打开 Godot 编辑器"
echo "  2. 加载项目: godot_project/project.godot"
echo "  3. 启用插件: 项目设置 → 插件"
echo ""
