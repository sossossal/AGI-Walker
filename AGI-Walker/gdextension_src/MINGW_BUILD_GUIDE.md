# 免费编译器配置指南 - MSYS2/MinGW-w64

## 📋 选择的方案

**编译器**: MinGW-w64 (via MSYS2)
**优点**:
- ✅ 完全免费开源
- ✅ 轻量级（~500 MB vs Visual Studio 的 GB）
- ✅ 快速安装
- ✅ 兼容 CMake
- ✅ 与 Godot 官方推荐一致

## 🚀 安装步骤

### 1. 安装 MSYS2

```powershell
winget install MSYS2.MSYS2
```

安装位置：`C:\msys64\`

### 2. 安装 MinGW-w64 工具链

安装完成后，打开 "MSYS2 MINGW64" 终端，运行：

```bash
# 更新包数据库
pacman -Syu

# 安装 MinGW-w64 GCC 工具链
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja

# 确认安装
gcc --version
cmake --version
```

### 3. 配置环境变量

将 MinGW-w64 添加到 Windows PATH：

**PowerShell (管理员)**:
```powershell
$env:Path += ";C:\msys64\mingw64\bin"
[Environment]::SetEnvironmentVariable("Path", $env:Path, [EnvironmentVariableTarget]::User)
```

**或手动添加**:
1. 搜索"环境变量"
2. 编辑用户变量 PATH
3. 添加 `C:\msys64\mingw64\bin`

### 4. 验证安装

```powershell
# 重启 PowerShell，然后运行
gcc --version
g++ --version
cmake --version
```

应该看到类似输出：
```
gcc (Rev3, Built by MSYS2 project) 13.2.0
```

## 🔨 编译 GDExtension

### 方法 1: 使用 Ninja（推荐）

```powershell
cd d:\新建文件夹\AGI-Walker\gdextension_src
mkdir build
cd build

# 生成 Ninja 构建文件
cmake .. -G "Ninja" -DCMAKE_BUILD_TYPE=Release

# 编译
ninja
```

### 方法 2: 使用 MinGW Makefiles

```powershell
cd d:\新建文件夹\AGI-Walker\gdextension_src
mkdir build
cd build

# 生成 Makefile
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release

# 编译
mingw32-make
```

### 自动编译脚本

创建 `build_mingw.ps1`:

```powershell
# 自动编译脚本（MinGW版）
param([string]$BuildType = "Release")

Write-Host "=== GDExtension Build (MinGW-w64) ===" -ForegroundColor Cyan

# 检查 gcc
if (!(Get-Command gcc -ErrorAction SilentlyContinue)) {
    Write-Host "ERROR: GCC not found! Please install MSYS2 MinGW-w64" -ForegroundColor Red
    exit 1
}

# 创建构建目录
if (!(Test-Path "build")) {
    New-Item -ItemType Directory -Path "build" | Out-Null
}

Push-Location build

try {
    # 使用 Ninja（更快）
    cmake .. -G "Ninja" -DCMAKE_BUILD_TYPE=$BuildType
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: CMake generation failed!" -ForegroundColor Red
        Pop-Location
        exit 1
    }
    
    # 编译
    ninja
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Build successful!" -ForegroundColor Green
    } else {
        Write-Host "❌ Build failed!" -ForegroundColor Red
    }
    
} finally {
    Pop-Location
}
```

## 📊 性能对比

| 特性 | MSVC (Visual Studio) | MinGW-w64（GCC） |
|------|----------------------|------------------|
| 安装大小 | ~6 GB | ~500 MB |
| 安装时间 | 30-60 分钟 | 5-10 分钟 |
| 编译速度 | 中等 | 快 |
| 调试工具 | 优秀（VS Debugger） | 良好（GDB） |
| Godot 兼容性 | ✅ 完美 | ✅ 完美 |
| 许可证 | 免费（个人使用） | GPL（完全自由） |

## 🐛 常见问题

### 1. "gcc: command not found"

**原因**: PATH 未配置

**解决**:
```powershell
$env:Path += ";C:\msys64\mingw64\bin"
```

### 2. "Ninja not found"

**解决**: 使用 MinGW Makefiles
```powershell
cmake .. -G "MinGW Makefiles"
mingw32-make
```

### 3. 编译错误：找不到 Windows.h

**原因**: 使用了 MSYS2 的常规终端而不是 MINGW64

**解决**: 确保在 MINGW64 终端中运行或直接在 PowerShell 中使用

## ✅ 完整流程示例

```powershell
# 1. 安装 MSYS2
winget install MSYS2.MSYS2

# 2. 打开 MSYS2 MINGW64 终端
pacman -Syu
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja

# 3. 配置 PATH（PowerShell）
$env:Path += ";C:\msys64\mingw64\bin"

# 4. 编译
cd d:\新建文件夹\AGI-Walker\gdextension_src
mkdir build
cd build
cmake .. -G "Ninja" -DCMAKE_BUILD_TYPE=Release
ninja

# 5. 验证输出
ls ..\godot_project\addons\robot_sim_toolkit\bin\
```

## 🎯 总结

使用 MinGW-w64 的优势：
- 🚀 **快速**: 安装和编译都更快
- 💾 **轻量**: 只需要 500 MB
- 🆓 **自由**: 完全开源，无限制
- ✅ **兼容**: Godot 官方推荐之一

**推荐给**: 想要快速开始、硬盘空间有限、或偏好命令行工具的开发者

---

**版本**: 1.0
**最后更新**: 2026-01-14
