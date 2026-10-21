# =============================================================================
# overnight_optimize.ps1 — start the unattended energy optimizer (Windows)
# =============================================================================
# Leave this running overnight. It needs the Docker dashboard and Webots.
# Each iteration launches one warehouse lap through launch_webots_twin.ps1
# (so the selected BoM mass is applied), then keeps or rolls back one
# allowlisted config change. In the morning read optimizer\runs\morning.md.
#
# The loop restores config\config.yaml when it finishes. It does not commit.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1 -MaxIterations 4 -MaxHours 6
#   powershell -ExecutionPolicy Bypass -File scripts\overnight_optimize.ps1 -DryRun
# =============================================================================

param(
    [int]$MaxIterations = 10,
    [double]$MaxHours = 8,
    [int]$LapTimeoutMin = 40,
    [ValidateSet("rules", "llm")]
    [string]$Picker = "rules",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

if (-not $DryRun) {
    $up = $false
    try {
        $resp = Invoke-WebRequest -Uri "http://127.0.0.1:5000/" -UseBasicParsing -TimeoutSec 5
        if ($resp.StatusCode -eq 200) { $up = $true }
    } catch {
        $up = $false
    }
    if (-not $up) {
        Write-Host "Dashboard is not up. Starting it with scripts\start.ps1"
        & (Join-Path $PSScriptRoot "start.ps1")
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
}

$AgentJournal = Join-Path $Root "docs\AGENT_JOURNAL.md"
$pyArgs = @(
    "-m", "optimizer",
    "--max-iterations", "$MaxIterations",
    "--max-hours", "$MaxHours",
    "--lap-timeout-min", "$LapTimeoutMin",
    "--picker", $Picker,
    "--agent-journal", $AgentJournal
)
if ($DryRun) { $pyArgs += "--dry-run" }

Write-Host "Overnight optimizer: iterations=$MaxIterations hours=$MaxHours picker=$Picker"
python @pyArgs
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$report = Join-Path $Root "optimizer\runs\morning.md"
if (Test-Path $report) {
    Write-Host ""
    Write-Host "Morning report: $report"
}
