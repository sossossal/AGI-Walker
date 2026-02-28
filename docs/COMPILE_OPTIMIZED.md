# C++ 插件优化编译流程

## 构建系统分析

**发现**: 项目使用 **CMake** 构建系统（不是 scons）

**优势**:
- ✅ 更好的 IDE 集成
- ✅ 跨平台支持
- ✅ 自动依赖管理
- ✅ Visual Studio 原生支持

## 编译步骤优化

### 阶段 1: 准备（一次性）
```powershell
# 1. 初始化 godot-cpp 子模块
cd d:\新建文件夹\AGI-Walker\gdextension_src
git submodule update --init --recursive

# 预计时间: 2-3 分钟（下载 godot-cpp）
```

### 阶段 2: 配置（一次性）
```powershell
# 2. 创建构建目录
New-Item -ItemType Directory -Force -Path build
cd build

# 3. 生成 Visual Studio 项目
cmake .. -G "Visual Studio 17 2022" -A x64

# 预计时间: 30 秒
```

### 阶段 3: 编译
```powershell
# 4a. Release 模式（推荐，用于性能测试）
cmake --build . --config Release -- /m

# 4b. 或 Debug 模式（开发调试用）
cmake --build . --config Debug -- /m

# 预计时间:
# - 首次: 15-20 分钟（编译 godot-cpp）
# - 后续: 1-2 分钟（仅编译修改部分）
```

## 性能优化参数

### 并行编译
```powershell
# /m 标志 = 使用所有CPU核心
cmake --build . --config Release -- /m
```

### 编译时间预估

| 阶段 | 首次 | 增量 |
|------|------|------|
| godot-cpp | 10-15 分钟 | - |
| 项目代码 | 1-2 分钟 | 10-30 秒 |
| **总计** | **15-20 分钟** | **< 1 分钟** |

## 验证编译结果

```powershell
# 检查 DLL 文件
Test-Path "..\godot_project\addons\robot_sim_toolkit\bin\robotparts.windows.x86_64.dll"

# 应该返回: True
```

## 常见错误处理

### 错误 1: CMake not found
```powershell
# 安装 CMake
winget install Kitware.CMake
# 或从 https://cmake.org/download/ 下载
```

### 错误 2: Visual Studio not found
```powershell
# 选项 1: 使用 MinGW（见 MINGW_BUILD_GUIDE.md）
# 选项 2: 安装 Visual Studio 2022 Community（免费）
# https://visualstudio.microsoft.com/downloads/
```

### 错误 3: godot-cpp not found
```powershell
# 重新初始化子模块
git submodule update --init --recursive --force
```

## 下一步

编译成功后：
1. 在 Godot 编辑器中打开项目
2. 启用插件（项目设置 → 插件）
3. 运行测试场景验证性能提升
