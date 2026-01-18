# 简化的编译脚本 - MinGW版本
$ErrorActionPreference = "Continue"

Write-Host "=== GDExtension Build (MinGW-w64) ===" -ForegroundColor Cyan

# 添加 MinGW 和 CMake 到 PATH
$env:Path = "C:\msys64\mingw64\bin;C:\Program Files\CMake\bin;" + $env:Path

# 验证工具
Write-Host "`n[1] 验证工具..." -ForegroundColor Yellow
try {
    $gccVersion = & gcc --version 2>&1 | Select-Object -First 1
    Write-Host "  GCC: $gccVersion" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: GCC not found!" -ForegroundColor Red
    exit 1
}

try {
    $cmakeVersion = & cmake --version 2>&1 | Select-Object -First 1
    Write-Host "  CMake: $cmakeVersion" -ForegroundColor Green
} catch {
    Write-Host "  ERROR: CMake not found!" -ForegroundColor Red
    exit 1
}

# 清理并创建构建目录
Write-Host "`n[2] 准备构建目录..." -ForegroundColor Yellow
if (Test-Path "build") {
    Remove-Item -Recurse -Force "build"
}
New-Item -ItemType Directory -Path "build" | Out-Null

Push-Location build

try {
    # 生成 Makefiles
    Write-Host "`n[3] 生成 CMake 项目..." -ForegroundColor Yellow
    cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_VERBOSE_MAKEFILE=ON
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "`nERROR: CMake generation failed!" -ForegroundColor Red
        Pop-Location
        exit 1
    }
    
    # 编译
    Write-Host "`n[4] 开始编译..." -ForegroundColor Yellow
    mingw32-make
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "`n✅ Build successful!" -ForegroundColor Green
        
        # 检查输出
        $dllPath = "..\godot_project\addons\robot_sim_toolkit\bin\robotparts.windows.x86_64.dll"
        if (Test-Path $dllPath) {
            $fileInfo = Get-Item $dllPath
            Write-Host "`nOutput file:" -ForegroundColor Cyan
            Write-Host "  Path: $dllPath" -ForegroundColor White
            Write-Host "  Size: $([math]::Round($fileInfo.Length / 1KB, 2)) KB" -ForegroundColor White
        }
    } else {
        Write-Host "`n❌ Build failed!" -ForegroundColor Red
    }
    
} finally {
    Pop-Location
}
