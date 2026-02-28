param([string]$BuildType = "Release")

$ErrorActionPreference = "Stop"
$SourceDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $SourceDir
$TempBuildRoot = "$env:TEMP\agi_walker_build"
$TempSrc = "$TempBuildRoot\gdextension_src"
$OutputDll = "$ProjectRoot\godot_project\addons\robot_sim_toolkit\bin\robotparts.windows.x86_64.dll"

Write-Host "=== GDExtension Build (Path-Fix) ===" -ForegroundColor Cyan
Write-Host "Source:     $SourceDir"
Write-Host "Temp Build: $TempBuildRoot"

# 1. Cleanup
if (Test-Path $TempBuildRoot) {
    if (Test-Path $TempSrc) { cmd /c "rmdir `"$TempSrc`"" 2>$null }
    Remove-Item -Recurse -Force $TempBuildRoot -ErrorAction SilentlyContinue
}

# 2. Junction
New-Item -ItemType Directory -Force -Path $TempBuildRoot | Out-Null
cmd /c "mklink /J `"$TempSrc`" `"$SourceDir`""
if (-not (Test-Path "$TempSrc\CMakeLists.txt")) {
    Write-Host "ERROR: Junction failed!" -ForegroundColor Red
    exit 1
}

# 3. CMake Configure
$BuildDir = "$TempBuildRoot\build"
Write-Host "CMake configuring..." -ForegroundColor Green
cmake -S $TempSrc -B $BuildDir -G "Visual Studio 17 2022" -A x64
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: CMake config failed ($LASTEXITCODE)" -ForegroundColor Red
    exit 1
}

# 4. Build
Write-Host "Building $BuildType..." -ForegroundColor Green
cmake --build $BuildDir --config $BuildType
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Build failed!" -ForegroundColor Red
    exit 1
}

# 5. Copy DLL
$BuiltDll = Get-ChildItem -Path $BuildDir -Recurse -Filter "robotparts*.dll" | Select-Object -First 1
if ($null -ne $BuiltDll) {
    $OutputDir = Split-Path -Parent $OutputDll
    New-Item -ItemType Directory -Force -Path $OutputDir -ErrorAction SilentlyContinue | Out-Null
    Copy-Item $BuiltDll.FullName $OutputDll -Force
    Write-Host "OK: $OutputDll ($([math]::Round((Get-Item $OutputDll).Length/1KB,1)) KB)" -ForegroundColor Green
}
else {
    Write-Host "WARNING: DLL not found in build output" -ForegroundColor Yellow
}

# 6. Cleanup junction
cmd /c "rmdir `"$TempSrc`"" 2>$null
Write-Host "=== Done ===" -ForegroundColor Cyan
