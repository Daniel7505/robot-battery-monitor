# stop.ps1 — tear down the Docker stack (core + full profile services)
# Usage: .\scripts\stop.ps1
# Uses --profile full so the optional ros2-sim container is included if present.
Set-Location (Split-Path -Parent $PSScriptRoot)
docker compose --profile full down
Write-Host "Stack stopped."