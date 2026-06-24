# One-click ButlerBot demo — dashboard (if needed) + Webots twin + demo UI flag.
# Usage: .\scripts\launch_demo.ps1

param(
    [string]$DashboardUrl = "http://127.0.0.1:5000"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot

Write-Host "ButlerBot Demo Mode" -ForegroundColor Cyan

try {
    Invoke-WebRequest -Uri "$DashboardUrl/api/demo/activate" -Method POST -UseBasicParsing -TimeoutSec 5 | Out-Null
    Write-Host "Dashboard demo mode ON" -ForegroundColor Green
} catch {
    Write-Host "Dashboard not up — starting docker compose..." -ForegroundColor Yellow
    Push-Location $ProjectRoot
    docker compose up --build -d
    Pop-Location
    Start-Sleep -Seconds 4
    try {
        Invoke-WebRequest -Uri "$DashboardUrl/api/demo/activate" -Method POST -UseBasicParsing -TimeoutSec 8 | Out-Null
        Write-Host "Dashboard demo mode ON" -ForegroundColor Green
    } catch {
        Write-Host "Could not reach dashboard at $DashboardUrl" -ForegroundColor Red
    }
}

& (Join-Path $PSScriptRoot "launch_webots_twin.ps1") -DashboardUrl $DashboardUrl

Write-Host ""
Write-Host "Open $DashboardUrl — twin panel is highlighted in demo mode." -ForegroundColor Cyan