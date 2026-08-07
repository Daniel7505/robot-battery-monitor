# =============================================================================
# ButlerBot stop-quality suite (API teleop + pose + control_diag)
# =============================================================================
# Usage (dashboard + Webots twin already running):
#   .\scripts\stop_suite.ps1
#   .\scripts\stop_suite.ps1 -DriveSeconds 3.5
#
# Drive durations are intentionally longer than a "kneejerk" so humans can
# see motion before Stop. Between sims: HARD PARK + intermission pause.
# Prints control_diag (hub rates, yaw, abs_active, locks) after each park.
# =============================================================================
param(
    [double]$DriveSeconds = 3.0,
    [double]$TurnSeconds = 2.0,
    [double]$LongDriveSeconds = 4.5,
    [double]$IntermissionSeconds = 5.0,
    [string]$DashboardUrl = "http://127.0.0.1:5000"
)

$ErrorActionPreference = "Stop"

function Sleep-Ms([int]$ms) { [System.Threading.Thread]::Sleep($ms) }

function Get-Twin {
    $uri = "$DashboardUrl/api/twin/state"
    return (Invoke-WebRequest -Uri $uri -UseBasicParsing -TimeoutSec 5).Content | ConvertFrom-Json
}

function Send-Cmd($body) {
    $json = $body | ConvertTo-Json -Depth 6 -Compress
    Invoke-WebRequest -Uri "$DashboardUrl/api/twin/command" -Method POST `
        -Body $json -ContentType "application/json" -UseBasicParsing -TimeoutSec 5 | Out-Null
}

function Show-Diag([string]$label) {
    try {
        $j = Get-Twin
        $d = $j.control_diag
        if (-not $d) {
            $d = $j.external_feed.sensors.control_diag
        }
        if (-not $d) {
            Write-Host "  [diag $label] (no control_diag yet — is Webots publishing?)"
            return
        }
        Write-Host ("  [diag {0}] hubs L={1} R={2} diff={3} yaw={4} gps={5} abs={6} spinMode={7} locks={8} epoch={9} phase={10}" -f `
            $label,
            $d.hub_left_rad_s, $d.hub_right_rad_s, $d.hub_diff_rad_s,
            $d.yaw_rate_rad_s, $d.gps_speed_m_s, $d.abs_active, $d.abs_spin_mode,
            $d.locks_engaged, $d.stop_epoch_seen, $d.phase)
    } catch {
        Write-Host "  [diag $label] error: $_"
    }
}

function Hard-Park([string]$why) {
    Write-Host ""
    Write-Host "  >>> HARD PARK ($why)"
    for ($k = 0; $k -lt 3; $k++) {
        Send-Cmd @{ drive_stop = $true }
        Sleep-Ms 350
    }
    $deadline = [datetime]::UtcNow.AddSeconds(14)
    $still = $false
    while ([datetime]::UtcNow -lt $deadline) {
        $a = Get-Twin
        $pa = $a.external_feed.pose
        Sleep-Ms 700
        $b = Get-Twin
        $pb = $b.external_feed.pose
        $dx = [math]::Abs([double]$pb.x_m - [double]$pa.x_m)
        $dy = [math]::Abs([double]$pb.y_m - [double]$pa.y_m)
        $dh = [math]::Abs([double]$pb.heading_rad - [double]$pa.heading_rad)
        if ($dh -gt [math]::PI) { $dh = 2 * [math]::PI - $dh }
        $trans = [math]::Sqrt($dx * $dx + $dy * $dy)
        $spd = 0.0
        if ($b.bridge.power_feed.locomotion) {
            $spd = [double]$b.bridge.power_feed.locomotion.speed_m_s
        }
        Write-Host ("      check dTrans={0:N4}m dH={1:N2}deg spd={2}" -f $trans, ($dh * 180 / [math]::PI), $spd)
        Show-Diag "park"
        if ($trans -lt 0.04 -and $dh -lt 0.12) {
            $still = $true
            break
        }
        Send-Cmd @{ drive_stop = $true }
    }
    if ($still) { Write-Host "      PARK SETTLED" } else { Write-Host "      PARK TIMEOUT" }
    Sleep-Ms 2000
}

function Measure-Still([string]$label, [int]$windowMs = 8000) {
    $rows = @()
    $n = [int][math]::Ceiling($windowMs / 500)
    for ($i = 0; $i -lt $n; $i++) {
        $j = Get-Twin
        $p = $j.external_feed.pose
        $rows += [pscustomobject]@{
            x = [double]$p.x_m
            y = [double]$p.y_m
            h = [double]$p.heading_rad
        }
        Sleep-Ms 500
    }
    $a = $rows[0]
    $b = $rows[$rows.Count - 1]
    $trans = [math]::Sqrt(($b.x - $a.x) * ($b.x - $a.x) + ($b.y - $a.y) * ($b.y - $a.y))
    $dh = [math]::Abs($b.h - $a.h)
    if ($dh -gt [math]::PI) { $dh = 2 * [math]::PI - $dh }
    $pass = ($trans -lt 0.05) -and ($dh -lt 0.12)
    Write-Host ""
    Write-Host (">>> MEASURE {0}  dTrans={1:N4}m  dH={2:N2}deg  => {3}" -f `
        $label, $trans, ($dh * 180 / [math]::PI), $(if ($pass) { "PASS" } else { "FAIL" }))
    Show-Diag $label
    return [pscustomobject]@{ label = $label; pass = $pass; trans = $trans; dh = $dh }
}

Write-Host "============================================"
Write-Host " ButlerBot stop suite (longer drives + diag)"
Write-Host " Dashboard: $DashboardUrl"
Write-Host "============================================"

$j = Get-Twin
if (-not $j.bridge.external_active -or $j.bridge.active_source -ne "webots") {
    Write-Host "FAIL: Webots twin not linked. Start dashboard + Webots first."
    exit 1
}
Write-Host ("PRECHECK linked bat={0}% source={1}" -f $j.robot.main_battery_pct, $j.bridge.active_source)
Show-Diag "precheck"

$results = @()
Hard-Park "baseline"

# --- SIM 1 ---
Write-Host ""
Write-Host "==== SIM 1/3  FORWARD then STOP  (drive ${DriveSeconds}s) ===="
Send-Cmd @{ drive = @{ left = 5.5; right = 5.5; duration_s = $DriveSeconds } }
Sleep-Ms ([int]($DriveSeconds * 1000 * 0.85))
Write-Host "  >> STOP"
Send-Cmd @{ drive_stop = $true }
Hard-Park "after S1"
$results += Measure-Still "S1-ForwardStop"

Write-Host ""
Write-Host "==== INTERMISSION 1->2  (${IntermissionSeconds}s, robot should stay parked) ===="
Hard-Park "inter 1-2"
Sleep-Ms ([int]($IntermissionSeconds * 1000))

# --- SIM 2 ---
Write-Host ""
Write-Host "==== SIM 2/3  FORWARD, TURN, STOP ===="
Send-Cmd @{ drive = @{ left = 5.5; right = 5.5; duration_s = [math]::Max(1.5, $DriveSeconds * 0.55) } }
Sleep-Ms ([int]([math]::Max(1.5, $DriveSeconds * 0.55) * 1000 * 0.85))
Write-Host "  >> TURN"
Send-Cmd @{ drive = @{ left = -2.6; right = 2.6; duration_s = $TurnSeconds } }
Sleep-Ms ([int]($TurnSeconds * 1000 * 0.85))
Write-Host "  >> STOP"
Send-Cmd @{ drive_stop = $true }
Hard-Park "after S2"
$results += Measure-Still "S2-FwdTurnStop"

Write-Host ""
Write-Host "==== INTERMISSION 2->3  (${IntermissionSeconds}s) ===="
Hard-Park "inter 2-3"
Sleep-Ms ([int]($IntermissionSeconds * 1000))

# --- SIM 3 ---
Write-Host ""
Write-Host "==== SIM 3/3  LONG FORWARD then STOP  (drive ${LongDriveSeconds}s) ===="
Send-Cmd @{ drive = @{ left = 5.5; right = 5.5; duration_s = $LongDriveSeconds } }
Sleep-Ms ([int]($LongDriveSeconds * 1000 * 0.85))
Write-Host "  >> STOP"
Send-Cmd @{ drive_stop = $true }
Hard-Park "after S3"
$results += Measure-Still "S3-LongForwardStop"

Write-Host ""
Write-Host "============================================"
Write-Host " SUMMARY"
$passN = 0
foreach ($r in $results) {
    if ($r.pass) { $passN++ }
    $mark = if ($r.pass) { "PASS" } else { "FAIL" }
    Write-Host ("  {0,-22} {1}  dTrans={2:N4}m  dH={3:N2}deg" -f `
        $r.label, $mark, $r.trans, ($r.dh * 180 / [math]::PI))
}
Write-Host ("  TOTAL {0}/{1}" -f $passN, $results.Count)
Write-Host "============================================"
if ($passN -eq $results.Count) { exit 0 } else { exit 2 }
