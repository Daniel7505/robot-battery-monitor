# Launch ButlerBot Webots digital twin alongside the dashboard.
# Usage: .\scripts\launch_webots_twin.ps1 [-DashboardUrl "http://127.0.0.1:5000"]

param(
    [string]$DashboardUrl = "http://127.0.0.1:5000"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$WebotsDir = Join-Path $ProjectRoot "webots"
$WorldFile = Join-Path $WebotsDir "worlds\butlerbot.wbt"

Write-Host "ButlerBot Webots Digital Twin" -ForegroundColor Cyan
Write-Host "Dashboard: $DashboardUrl"
Write-Host "World:     $WorldFile"
Write-Host ""

# Check dashboard is reachable
try {
    $resp = Invoke-WebRequest -Uri "$DashboardUrl/api/twin/schema" -UseBasicParsing -TimeoutSec 5
    if ($resp.StatusCode -eq 200) {
        Write-Host "Dashboard twin API OK" -ForegroundColor Green
    }
} catch {
    Write-Host "WARNING: Dashboard not reachable at $DashboardUrl" -ForegroundColor Yellow
    Write-Host "Start it first: .\scripts\start.ps1"
    Write-Host ""
}

# Find Webots executable
$WebotsExe = $null
if ($env:WEBOTS_HOME) {
    foreach ($rel in @("msys64\mingw64\bin\webots.exe", "msys64\webots.exe", "webots.exe")) {
        $candidate = Join-Path $env:WEBOTS_HOME $rel
        if (Test-Path $candidate) { $WebotsExe = $candidate; break }
    }
}
if (-not $WebotsExe) {
    $paths = @(
        "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
        "${env:LOCALAPPDATA}\Programs\Webots\msys64\mingw64\bin\webots.exe",
        "C:\Program Files\Webots\msys64\webots.exe",
        "C:\Program Files\Cyberbotics\Webots\msys64\webots.exe"
    )
    foreach ($p in $paths) {
        if (Test-Path $p) { $WebotsExe = $p; break }
    }
}
if (-not $WebotsExe) {
    $cmd = Get-Command webots -ErrorAction SilentlyContinue
    if ($cmd) { $WebotsExe = $cmd.Source }
}

if (-not $WebotsExe) {
    Write-Host "ERROR: Webots not found. Install from https://cyberbotics.com/download" -ForegroundColor Red
    Write-Host "Or set WEBOTS_HOME to your Webots install folder."
    exit 1
}

Write-Host "Webots: $WebotsExe" -ForegroundColor Green

# Avoid duplicate instances (port conflicts / auto-close from stale processes)
$existing = Get-Process -Name "webots" -ErrorAction SilentlyContinue
if ($existing) {
    Write-Host "Closing $($existing.Count) existing Webots process(es)..." -ForegroundColor Yellow
    $existing | Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 2
}

$env:TWIN_DASHBOARD_URL = $DashboardUrl
$env:WEBOTS_PROJECT_HOME = $WebotsDir

Set-Location $WebotsDir
Write-Host "Launching ButlerBot world (realtime) - close Webots window to exit." -ForegroundColor Cyan
& $WebotsExe --mode=realtime --stdout --stderr 'worlds\butlerbot.wbt'