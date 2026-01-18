# GDExtension Build Script
param([string]$BuildType = "Release")

Write-Host "=== GDExtension Build Script ===" -ForegroundColor Cyan
Write-Host "Build Type: $BuildType" -ForegroundColor Yellow

# Check godot-cpp
if (!(Test-Path "godot-cpp")) {
    Write-Host "Initializing godot-cpp submodule..." -ForegroundColor Yellow
    git submodule update --init --recursive
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Failed to initialize godot-cpp!" -ForegroundColor Red
        exit 1
    }
}

# Create build directory
if (!(Test-Path "build")) {
    Write-Host "Creating build directory..." -ForegroundColor Green
    New-Item -ItemType Directory -Path "build" | Out-Null
}

# Enter build directory
Push-Location build

try {
    # Generate CMake project
    Write-Host "Generating CMake project..." -ForegroundColor Green
    cmake .. -G "Visual Studio 17 2022" -A x64
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: CMake generation failed!" -ForegroundColor Red
        Pop-Location
        exit 1
    }
    
    # Build
    Write-Host "Building ($BuildType)..." -ForegroundColor Green
    cmake --build . --config $BuildType
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Build failed!" -ForegroundColor Red
        Pop-Location
        exit 1
    }
    
    Write-Host "Build successful!" -ForegroundColor Green
    
    # Check output
    $dllPath = "..\godot_project\addons\robot_sim_toolkit\bin\robotparts.windows.x86_64.dll"
    if (Test-Path $dllPath) {
        $fileInfo = Get-Item $dllPath
        Write-Host "Output file info:" -ForegroundColor Cyan
        Write-Host "  Path: $dllPath" -ForegroundColor White
        Write-Host "  Size: $([math]::Round($fileInfo.Length / 1KB, 2)) KB" -ForegroundColor White
        Write-Host "  Modified: $($fileInfo.LastWriteTime)" -ForegroundColor White
    }
    else {
        Write-Host "WARNING: DLL file not found!" -ForegroundColor Yellow
    }
    
}
finally {
    Pop-Location
}

Write-Host ""
Write-Host "=== Build Complete ===" -ForegroundColor Cyan
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "  1. Open Godot project" -ForegroundColor White
Write-Host "  2. Enable plugin: Project -> Project Settings -> Plugins -> Robot Simulation Toolkit" -ForegroundColor White
Write-Host "  3. Check console output for success message" -ForegroundColor White
