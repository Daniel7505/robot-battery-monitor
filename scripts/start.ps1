# =============================================================================
# start.ps1 — bring up the Robot Battery Monitor Docker stack (Windows)
# =============================================================================
# Core profile (default): Postgres + PMS dashboard on DASHBOARD_PORT (default 5000)
# Full profile:           + ROS2 sim container (docker compose --profile full)
#
# Usage:
#   .\scripts\start.ps1
#   .\scripts\start.ps1 -Profile full
#
# After success, open http://127.0.0.1:<DASHBOARD_PORT>. For Webots twin, also run
# launch_webots_twin.ps1 once the dashboard is healthy.
# =============================================================================
param(
    [ValidateSet("core", "full")]
    [string]$Profile = "core"
)

$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

# First-run convenience: seed env from example so compose has DB credentials
if (-not (Test-Path ".env") -and (Test-Path ".env.example")) {
    Write-Host "No .env found — copying .env.example"
    Copy-Item ".env.example" ".env"
}

if ($Profile -eq "full") {
    Write-Host "Starting full stack (dashboard + Postgres + ROS2 sim)..."
    docker compose --profile full up --build -d
} else {
    Write-Host "Starting core stack (dashboard + Postgres)..."
    docker compose up --build -d
}

# Health wait: entrypoint may still be creating tables / starting Flask.
# Same port compose publishes (shell env, else .env, else 5000).
$port = "5000"
if ($env:DASHBOARD_PORT) {
    $port = $env:DASHBOARD_PORT
} elseif (Test-Path ".env") {
    foreach ($line in Get-Content ".env") {
        if ($line -match '^\s*DASHBOARD_PORT\s*=\s*(\d+)\s*$') { $port = $Matches[1]; break }
    }
}
$dashUrl = "http://127.0.0.1:$port/"
Write-Host ""
Write-Host "Waiting for dashboard on $dashUrl ..."
$ready = $false
for ($i = 0; $i -lt 30; $i++) {
    try {
        $resp = Invoke-WebRequest -Uri $dashUrl -UseBasicParsing -TimeoutSec 3
        if ($resp.StatusCode -eq 200) {
            $ready = $true
            break
        }
    } catch {
        Start-Sleep -Seconds 2
    }
}

if ($ready) {
    Write-Host ""
    Write-Host "Dashboard ready: $dashUrl"
    docker compose ps
} else {
    Write-Host "Dashboard did not respond in time — check: docker compose logs dashboard"
    docker compose ps
    exit 1
}