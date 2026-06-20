# Start the Robot Battery Monitor Docker stack (Windows)
param(
    [ValidateSet("core", "full")]
    [string]$Profile = "core"
)

$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

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

Write-Host ""
Write-Host "Waiting for dashboard..."
$ready = $false
for ($i = 0; $i -lt 30; $i++) {
    try {
        $resp = Invoke-WebRequest -Uri "http://127.0.0.1:5000/" -UseBasicParsing -TimeoutSec 3
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
    Write-Host "Dashboard ready: http://127.0.0.1:5000"
    docker compose ps
} else {
    Write-Host "Dashboard did not respond in time — check: docker compose logs dashboard"
    docker compose ps
    exit 1
}