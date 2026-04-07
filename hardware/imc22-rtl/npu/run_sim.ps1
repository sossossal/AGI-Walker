# IMC-22 NPU 仿真脚本
Write-Host "IMC-22 NPU Simulation" -ForegroundColor Cyan

# 编译
iverilog -o mac.vvp mac_int8.v mac_tb.v
if ($LASTEXITCODE -eq 0) {
    Write-Host "✓ Compiled" -ForegroundColor Green
    
    # 运行
    vvp mac.vvp
    
    if (Test-Path "mac.vcd") {
        Write-Host "✓ Waveform: mac.vcd" -ForegroundColor Green
    }
}
else {
    Write-Host "✗ Compilation failed" -ForegroundColor Red
}
