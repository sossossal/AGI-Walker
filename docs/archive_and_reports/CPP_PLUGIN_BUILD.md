# C++ 插件编译快速指�?

本指南帮助您编译 AGI-Walker �?GDExtension C++ 插件，提升物理模拟性能�?**10�?*�?

---

## 📋 前置要求

### Windows

```powershell
# 安装 MinGW-w64（推荐使�?MSYS2�?
# 访问: https://www.msys2.org/

# �?MSYS2 中安装工�?
pacman -S mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-python-scons

# 添加�?PATH
# C:\msys64\mingw64\bin
```

### Linux

```bash
sudo apt-get update
sudo apt-get install build-essential scons pkg-config libx11-dev libxcursor-dev \
    libxinerama-dev libgl1-mesa-dev libglu-dev libasound2-dev libpulse-dev \
    libudev-dev libxi-dev libxrandr-dev
```

### macOS

```bash
brew install scons
xcode-select --install
```

---

## 🔨 编译步骤

### 步骤 1: 初始化子模块

```bash
cd d:\新建文件夹\AGI-Walker

# 如果还未克隆子模�?
git submodule update --init --recursive
```

### 步骤 2: 编译 godot-cpp

```bash
cd gdextension_src/godot-cpp

# Windows (MinGW)
scons platform=windows target=template_debug
scons platform=windows target=template_release

# Linux
scons platform=linux target=template_debug
scons platform=linux target=template_release

# macOS
scons platform=macos target=template_debug
scons platform=macos target=template_release
```

**编译时间**: �?10-20 分钟（首次）

### 步骤 3: 编译项目插件

```bash
cd ../  # 回到 gdextension_src

# Windows
scons platform=windows

# Linux
scons platform=linux

# macOS
scons platform=macos
```

**输出文件**:
- Windows: `bin/libgdexample.windows.template_debug.x86_64.dll`
- Linux: `bin/libgdexample.linux.template_debug.x86_64.so`
- macOS: `bin/libgdexample.macos.template_debug.universal.dylib`

---

## �?验证安装

### �?Godot 中测�?

1. **打开 Godot 项目**
   ```bash
   # 使用 Godot 4.2+ 打开
   godot --path d:\新建文件夹\AGI-Walker\godot_project
   ```

2. **检查插�?*
   - 项目设置 �?插件
   - 确认 "Robot Simulation Toolkit" 显示并启�?

3. **运行测试场景**
   - 打开 `scenes/test_physics.tscn`
   - �?F5 运行
   - 观察 FPS（应该显著提升）

---

## 🐛 常见问题

### 问题 1: `scons: command not found`

**解决**:
```bash
# Windows
pip install scons

# Linux/macOS
sudo apt-get install scons  # Linux
brew install scons          # macOS
```

### 问题 2: 找不�?Python

**解决**:
```bash
# 确保 Python 3.6+ 已安�?
python --version

# Windows: 添加 Python �?PATH
```

### 问题 3: 编译错误 - 找不到头文件

**解决**:
```bash
# 确保子模块已正确初始�?
cd gdextension_src/godot-cpp
git submodule update --init --recursive
```

### 问题 4: Godot 无法加载插件

**检�?*:
1. 插件文件是否�?`gdextension_src/bin/` 目录
2. 文件扩展名是否匹配操作系�?
3. Godot 版本是否 4.2+

---

## 📊 性能对比

编译前后性能对比（预期）:

| 指标 | Python/GDScript | C++ 插件 | 提升 |
|------|----------------|----------|------|
| 物理步进 | ~30 FPS | ~300 FPS | **10x** |
| 电机模拟 | ~50 FPS | ~500 FPS | **10x** |
| 内存占用 | 500 MB | 200 MB | **2.5x** |

---

## 🎯 下一�?

编译成功后：
1. 运行完整训练（性能更快�?
2. 测试复杂场景（更多并行环境）
3. 调整物理参数（更精确�?

---

## 📚 参考资�?

- [GDExtension 官方文档](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/index.html)
- [Godot-cpp GitHub](https://github.com/godotengine/godot-cpp)
- [详细构建指南](BUILD_GUIDE.md)

---

**需要帮助？**
- Discord: discord.gg/agi-walker
- GitHub Issues: github.com/agi-walker/agi-walker-sim/issues
