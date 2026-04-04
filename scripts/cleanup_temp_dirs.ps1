param (
    [string[]]$Paths = @(
        ".pytest_cache",
        ".pytest_tmp",
        "pytest-cache-files-7kqopaqn",
        "pytest-cache-files-o1j28pq4",
        ".output",
        ".ruff_cache",
        "test_env/tmp",
        "test_env/godot_agent_factory_*",
        "test_env/godot_agent_adapter_*",
        "test_env/pytest_tmp",
        "exports",
        "__pycache__",
        "*.pyc",
        "*.log"
    ),
    [switch]$DryRun
)

function Remove-Path {
    param ($path)
    $expanded = Get-Item -LiteralPath $path -ErrorAction SilentlyContinue
    if (-not $expanded) {
        return
    }

    try {
        if ($DryRun) {
            Write-Host "Would remove: $($expanded.FullName)"
        } else {
            Write-Host "Removing: $($expanded.FullName)"
            Remove-Item -LiteralPath $expanded.FullName -Recurse -Force -ErrorAction Stop
        }
    } catch {
        Write-Warning "Failed to remove ${path}: $_"
    }
}

foreach ($path in $Paths) {
    Remove-Path $path
}

Write-Host "Cleanup finished. Run `git status` afterwards to verify."
