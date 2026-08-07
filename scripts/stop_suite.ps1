# =============================================================================
# ButlerBot stop-quality suite (API teleop + pose + control_diag)
# =============================================================================
# Usage (dashboard + Webots twin already running):
#   .\scripts\stop_suite.ps1
#   .\scripts\stop_suite.ps1 -DriveSeconds 3.5 -LongDriveSeconds 5
#
# Longer drives so motion is visible before Stop. Between sims: HARD PARK.
# Prints control_diag (hub rates, yaw, abs, locks) after each park.
# =============================================================================
param(
    [double]$DriveSeconds = 3.5,
    [double]$TurnSeconds = 2.2,
    [double]$LongDriveSeconds = 5.0,
    [double]$IntermissionSeconds = 4.0,
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
        if (-not $d -and $j.external_feed -and $j.external_feed.sensors) {
            $d = $j.external_feed.sensors.control_diag
        }
        if (-not $d) {
            Write-Host "  [diag $label] no control_diag yet (is Webots publishing?)"
            return
        }
        $legs = ($j.channels | Where-Object { $_.id -eq "Legs" }).draw_w
        Write-Host ("  [diag {0}] hubs L={1} R={2} diff={3} yaw={4} gps={5} abs={6} locks={7} legs={8}W phase={9}" -f `
            $label,
            $d.hub_left_rad_s, $d.hub_right_rad_s, $d.hub_diff_rad_s,
            $d.yaw_rate_rad_s, $d.gps_speed_m_s, $d.abs_active,
            $d.locks_engaged, $legs, $d.phase)
    } catch {
        Write-Host "  [diag $label] error: $_"
    }
}

function Hard-Park([string]$why) {
    Write-Host ""
    Write-Host "  --- HARD PARK ($why) ---"
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
        $legs = ($b.channels | Where-Object { $_.id -eq "Legs" }).draw_w
        Write-Host ("      check dTrans={0:N4}m dH={1:N2}deg spd={2} legs={3}W" -f $trans, ($dh * 180 / [math]::PI), $spd, $legs)
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
        $legs = ($j.channels | Where-Object { $_.id -eq "Legs" }).draw_w
        $spd = 0.0
        if ($j.bridge.power_feed.locomotion) {
            $spd = [double]$j.bridge.power_feed.locomotion.speed_m_s
        }
        $rows += [pscustomobject]@{
            x = [double]$p.x_m
            y = [double]$p.y_m
            h = [double]$p.heading_rad
            legs = [double]$legs
            speed = $spd
        }
        Sleep-Ms 500
    }
    $a = $rows[0]
    $b = $rows[$rows.Count - 1]
    $trans = [math]::Sqrt(($b.x - $a.x) * ($b.x - $a.x) + ($b.y - $a.y) * ($b.y - $a.y))
    $dh = [math]::Abs($b.h - $a.h)
    if ($dh -gt [math]::PI) { $dh = 2 * [math]::PI - $dh }
    $avgLegs = ($rows | Measure-Object -Property legs -Average).Average
    $posePass = ($trans -lt 0.05) -and ($dh -lt 0.12)
    $idlePass = $avgLegs -lt 8.0
    $pass = $posePass -and $idlePass
    Write-Host ""
    Write-Host ("MEASURE {0}  dTrans={1:N4}m  dH={2:N2}deg  avgLegs={3:N1}W  endSpd={4}  pose={5} idle={6} => {7}" -f `
        $label, $trans, ($dh * 180 / [math]::PI), $avgLegs, $b.speed,
        $(if ($posePass) { "OK" } else { "FAIL" }),
        $(if ($idlePass) { "OK" } else { "FAIL" }),
        $(if ($pass) { "PASS" } else { "FAIL" }))
    Show-Diag $label
    return [pscustomobject]@{
        label = $label; pass = $pass; posePass = $posePass; idlePass = $idlePass
        trans = $trans; dh = $dh; avgLegs = $avgLegs
    }
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

Write-Host ""
Write-Host "==== SIM 1/3  FORWARD then STOP  (drive ${DriveSeconds}s) ===="
Send-Cmd @{ drive = @{ left = 5.5; right = 5.5; duration_s = $DriveSeconds } }
Sleep-Ms ([int]($DriveSeconds * 1000 * 0.88))
Write-Host "  STOP now"
Send-Cmd @{ drive_stop = $true }
Hard-Park "after S1"
$results += Measure-Still "S1-ForwardStop"

Write-Host ""
Write-Host "==== INTERMISSION 1->2  (${IntermissionSeconds}s, should stay parked) ===="
Hard-Park "inter 1-2"
Sleep-Ms ([int]($IntermissionSeconds * 1000))

Write-Host ""
Write-Host "==== SIM 2/3  FORWARD, TURN, STOP ===="
$fwd2 = [math]::Max(1.8, $DriveSeconds * 0.55)
Send-Cmd @{ drive = @{ left = 5.5; right = 5.5; duration_s = $fwd2 } }
Sleep-Ms ([int]($fwd2 * 1000 * 0.88))
Write-Host "  TURN now"
Send-Cmd @{ drive = @{ left = -2.6; right = 2.6; duration_s = $TurnSeconds } }
Sleep-Ms ([int]($TurnSeconds * 1000 * 0.88))
Write-Host "  STOP now"
Send-Cmd @{ drive_stop = $true }
Hard-Park "after S2"
$results += Measure-Still "S2-FwdTurnStop"

Write-Host ""
Write-Host "==== INTERMISSION 2->3  (${IntermissionSeconds}s) ===="
Hard-Park "inter 2-3"
Sleep-Ms ([int]($IntermissionSeconds * 1000))

Write-Host ""
Write-Host "==== SIM 3/3  LONG FORWARD then STOP  (drive ${LongDriveSeconds}s) ===="
Send-Cmd @{ drive = @{ left = 5.5; right = 5.5; duration_s = $LongDriveSeconds } }
Sleep-Ms ([int]($LongDriveSeconds * 1000 * 0.88))
Write-Host "  STOP now"
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
    Write-Host ("  {0,-22} {1}  dTrans={2:N4}m  dH={3:N2}deg  avgLegs={4:N1}W  pose={5} idle={6}" -f `
        $r.label, $mark, $r.trans, ($r.dh * 180 / [math]::PI), $r.avgLegs,
        $(if ($r.posePass) { "OK" } else { "FAIL" }),
        $(if ($r.idlePass) { "OK" } else { "FAIL" }))
}
Write-Host ("  TOTAL {0}/{1}" -f $passN, $results.Count)
Write-Host "============================================"
if ($passN -eq $results.Count) { exit 0 } else { exit 2 }
