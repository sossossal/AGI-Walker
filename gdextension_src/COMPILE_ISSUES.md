# GDExtension 编译问题诊断与解决方案

## 问题根源

CMake 和 MSBuild 在 Windows 上**无法正确处理路径中的中文字符**,包括:
- 源代码路径: `D:\新建文件夹\AGI-Walker\gdextension_src`
- 用户主目录: `C:\Users\荣耀\AppData\Local\Temp\...`

即使使用 Junction 链接将源码映射到 `%TEMP%`,MSBuild 仍会因用户目录中的"荣耀"字符失败。

## 错误信息

```
error MSB3191: 无法创建目录"C:\Users\��ҫ\AppData\..."
```

字符"荣耀"被误解析为乱码 `��ҫ`。

## 解决方案

### 方案 1: 使用英文用户名的临时账户(推荐)

1. 创建新的 Windows 用户账户(用户名必须为纯英文,如 `builder`)
2. 将项目复制到该账户的纯英文路径,如 `C:\Projects\AGI-Walker`
3. 在新账户下运行编译:
   ```powershell
   cd C:\Projects\AGI-Walker\gdextension_src
   .\build.ps1
   ```

### 方案 2: 修改系统环境变量(高级)

将 `TEMP` 和 `TMP` 环境变量指向纯英文路径:

```powershell
# 以管理员身份运行
[System.Environment]::SetEnvironmentVariable("TEMP", "C:\BuildTemp", "User")
[System.Environment]::SetEnvironmentVariable("TMP", "C:\BuildTemp", "User")
New-Item -ItemType Directory -Force -Path "C:\BuildTemp"
```

重启 PowerShell 后再运行 `build_workaround.ps1`。

### 方案 3: 使用 Docker(最稳定)

```dockerfile
FROM mcr.microsoft.com/windows/servercore:ltsc2022
RUN choco install cmake visualstudio2022buildtools -y
COPY gdextension_src C:/build/gdextension_src
WORKDIR C:/build/gdextension_src
RUN cmake -S . -B build -G "Visual Studio 17 2022" -A x64
RUN cmake --build build --config Release
```

### 方案 4: 使用 WSL2 + MinGW(跨平台)

如果安装了 WSL2,可以在 Linux 环境下使用 MinGW 交叉编译:

```bash
sudo apt install mingw-w64 cmake
cd /mnt/d/新建文件夹/AGI-Walker/gdextension_src
cmake -S . -B build -G "Unix Makefiles" \
  -DCMAKE_TOOLCHAIN_FILE=mingw-w64-toolchain.cmake
cmake --build build
```

## 当前状态

- ✅ Python 硬件驱动已完成并通过测试
- ✅ SysID 数据采集和参数拟合脚本已就绪
- ❌ C++ GDExtension 编译因路径编码问题受阻

## 临时替代方案

如果暂时无法编译 GDExtension,可以:
1. 使用纯 GDScript 实现电机模型(性能较低但功能完整)
2. 仅使用 Python 驱动进行真实硬件控制(绕过 Godot)
3. 等待项目迁移到纯英文路径后再编译

## 参考资料

- [CMake Issue #16139: Unicode paths on Windows](https://gitlab.kitware.com/cmake/cmake/-/issues/16139)
- [MSBuild Unicode Support](https://github.com/dotnet/msbuild/issues/3468)
