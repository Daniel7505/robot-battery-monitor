Set-Location (Split-Path -Parent $PSScriptRoot)
docker compose --profile full down
Write-Host "Stack stopped."