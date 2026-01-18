# GDExtension 编译指南

## 📋 前置要求

### Windows
- Visual Studio 2019 或更高版本（包含 C++ 桌面开发工具）
- CMake 3.20+
- Git

### Linux (Ubuntu/Debian)
```bash
sudo apt-get install build-essential cmake git
```

### macOS
```bash
brew install cmake
# 需要安装 Xcode Command Line Tools
xcode-select --install
```

---

## 🚀 快速编译（Windows）

### 方法 1: 使用提供的脚本

```powershell
# 在 gdextension_src 目录下
cd d:\新建文件夹\AGI-Walker\gdextension_src
.\build.ps1
```

### 方法 2: 手动编译

```powershell
# 1. 确保 godot-cpp 已克隆
git submodule update --init --recursive

# 2. 创建构建目录
New-Item -ItemType Directory -Force -Path build
cd build

# 3. 生成 Visual Studio 项目
cmake .. -G "Visual Studio 17 2022" -A x64

# 4. 编译（Release 模式）
cmake --build . --config Release

# 或者编译 Debug 模式
cmake --build . --config Debug
```

---

## 🔧 编译选项

### 选择构建类型

**Release 模式**（推荐用于最终使用）:
```powershell
cmake --build build --config Release
```

**Debug 模式**（推荐用于开发调试）:
```powershell
cmake --build build --config Debug
```

### 清理并重新编译

```powershell
Remove-Item -Recurse -Force build
New-Item -ItemType Directory build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

---

## 📦 输出位置

编译成功后，DLL 文件会自动复制到：

```
godot_project/addons/robot_sim_toolkit/bin/
└── robotparts.windows.x86_64.dll
```

Godot 会自动加载这个文件（如果插件已启用）。

---

## ✅ 验证编译

### 1. 检查文件是否生成

```powershell
Test-Path "d:\新建文件夹\AGI-Walker\godot_project\addons\robot_sim_toolkit\bin\robotparts.windows.x86_64.dll"
```

### 2. 在 Godot 中测试

1. 打开 Godot 项目
2. 进入 `项目` -> `项目设置` -> `插件`
3. 启用 "Robot Simulation Toolkit"
4. 查看控制台输出：

```
✅ Robot Simulation Toolkit GDExtension loaded
```

### 3. 创建测试脚本

```gdscript
extends Node3D

func _ready():
    var motor = EnhancedMotorJoint.new()
    motor.set_motor_specs(1.4, 5.236, 3.5e-6)
    print("✅ EnhancedMotorJoint 创建成功!")
    print(motor.get_diagnostic_info())
```

---

## 🐛 常见问题

### 问题 1: "CMake not found"

**解决**:
```powershell
# 安装 CMake
winget install Kitware.CMake

# 或者从官网下载: https://cmake.org/download/
```

### 问题 2: "godot-cpp not found"

**原因**: godot-cpp 子模块未初始化

**解决**:
```powershell
cd d:\新建文件夹\AGI-Walker\gdextension_src
git submodule update --init --recursive
```

### 问题 3: "LINK : fatal error LNK1104: cannot open file 'godot-cpp\bin\...'"

**原因**: godot-cpp 未编译

**解决**: godot-cpp 会由 CMake 自动编译，确保网络畅通（下载依赖）

### 问题 4: 编译成功但 Godot 无法加载

**检查清单**:
1. 确认 `.gdextension` 文件路径正确
2. 确认 DLL 文件在 `bin/` 目录下
3. 检查 Godot 版本是否为 4.2+
4. 查看 Godot 控制台的错误信息

### 问题 5: 修改代码后未生效

**解决**: 需要重新编译
```powershell
cd build
cmake --build . --config Release
# 然后重启 Godot
```

---

## 📊 编译性能

| 配置 | 首次编译时间 | 增量编译时间 |
|------|--------------|--------------|
| godot-cpp (首次) | ~10-15 分钟 | - |
| RobotSimToolkit | ~30-60 秒 | ~5-10 秒 |
| **总计（首次）** | **~15 分钟** | - |

**提示**: 首次编译会很慢（编译整个 godot-cpp），后续只需编译修改的文件。

---

## 🔄 开发工作流

### 推荐的迭代流程：

1. **修改 C++ 代码** → 编辑 `.cpp/.h` 文件
2. **编译** → `cmake --build build --config Debug`
3. **重启 Godot** → 关闭并重新打开项目
4. **测试** → 运行测试场景
5. **查看日志** → 检查控制台输出
6. **重复** → 回到步骤 1

### 加速技巧：

- **使用 Debug 模式开发**（编译更快）
- **只修改实现文件**（`.cpp`），避免修改头文件（`.h`）
- **使用增量编译**（CMake 会自动处理）

---

## 📚 参考资源

- [Godot GDExtension 官方文档](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/index.html)
- [godot-cpp GitHub](https://github.com/godotengine/godot-cpp)
- [CMake 文档](https://cmake.org/documentation/)

---

**版本**: 1.0  
**最后更新**: 2026-01-14
