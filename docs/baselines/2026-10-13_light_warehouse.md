# Light warehouse lap, 2026-10-13

Reference for the model-versus-measured comparison. This file records
Dan's run #8. It is not a coefficient in the energy model.

## Run

- Commit: `faf98a6`
- World: warehouse
- Build: `light`
- Laps: 1
- Where: Dan's PC

## MISSION_SUMMARY

One summary line:

| Field | Value |
| --- | --- |
| duration_s | 901.7 |
| dist_m | 200.91 |
| turns | 13 |
| soc | 100.00 → 96.84 |
| wh_model | 6.464 |
| wh_model_idle | 2.495 |
| wh_model_drive | 3.969 |
| wh_model_roll | 0.441 |
| wh_model_visc | 3.521 |
| wh_model_turn | 0 |
| wh_model_accel | 0.007 |
| wh_measured_drive_shaft | 1.182 |
| wh_measured_drive_elec | 5.107 |
| eta_avg | 0.231 |
| wh_measured | 7.577 |
| wh_measured_soc | 7.584 |
| err_pct | 28.7 |

`wh_measured` and `wh_measured_soc` agree. `err_pct` is
`(wh_measured_drive_elec − wh_model_drive) / wh_model_drive × 100`.

## Segments on that run

Straights:

- about 11–25% err at 0.17–0.23 m/s
- about 25–39% err at 0.27–0.36 m/s

Turns:

- seg 48: 107.6 deg, 15 accel events, err 110%
- seg 47: 31 deg, err 46%

## What the next lap is checking

The mission log now bills `wh_model_turn` from the axle track (0.34 m),
the drawn wheel radius (0.08 m), and the world's drive-wheel contact
(`coulombFriction` 8 as a cap, `forceDependentSlip` 0.01). Pack watts
use the same Pololu 4693 speed-and-torque line as the measured
electrical column. Driver efficiency 0.92 is applied once. μ = 8 is
not used as a skid coefficient. Straight-line error is not fitted.

Prediction, not a measurement, if a turn segment is about 3.048 m at
about 0.22 m/s:

- seg 48 adds about 0.037 Wh and err falls from about 110% to about 30%
- seg 47 adds about 0.010 Wh and err falls from about 46% to about 26%
- straight segments stay in the 11–39% band above
- the lap `err_pct` moves from 28.7% toward the mid-20s if the other
  yaw is also in short corners, and stays nearer 28.7% if that yaw is
  spread out

The rerun is the check. `wh_model_turn` on the summary was 0 on this
reference and should be greater than 0. `wh_measured` and
`wh_measured_soc` should still agree.

## Run #9, after the turn term

Dan's light warehouse lap on `255e769` (1 lap). This does not replace
the run #8 numbers above.

| Field | Value |
| --- | --- |
| duration_s | 896.2 |
| dist_m | 200.91 |
| turns | 13 |
| soc | 100 → 96.86 |
| wh_model | 6.563 |
| wh_model_idle | 2.479 |
| wh_model_drive | 4.084 |
| wh_model_roll | 0.441 |
| wh_model_visc | 3.540 |
| wh_model_turn | 0.096 |
| wh_model_accel | 0.007 |
| wh_measured_drive_shaft | 1.175 |
| wh_measured_drive_elec | 5.075 |
| eta_avg | 0.232 |
| wh_measured | 7.529 |
| wh_measured_soc | 7.536 |
| err_pct (MISSION_SUMMARY, drive only) | 24.3 |
| err_pct (MISSION_ENERGY, drive + idle) | 14.7 |

Turn segments of about 103° were about 20% (about 110% on run #8).
Straights stayed about 10–25% low near 0.2 m/s and 30–40% low near
0.29 m/s. Webots showed 0.70× real time. The picture hitched about
0.5 s every other 0.5 s.

`MISSION_SUMMARY` err_pct divides by drive only.
`(7.529 − 6.563) / 6.563` is the 14.7% on `MISSION_ENERGY`, which
includes idle. Those two lines are not a mismatch.

## Straight gap, checked and not fitted

No straight-line coefficient was added.

- `rollingFriction` is unset on `drive_wheel` vs `default`, so ODE
  applies none (default `0 0 0`).
- `softCFM` 1e-6 and `softERP` 0.7 are contact softness, not a drag.
- Caster bottoms are about 4 mm above the tire, so caster damping
  0.8 is not floor drag.
- A constant extra torque makes the percent error fall as speed rises.
  Extra viscous damping stays nearly flat from 0.20 to 0.29 m/s. The
  logged gap grows with speed.
- A 0.2 rad/s left/right split changes electrical power by under 1%,
  and a larger split makes the percent fall with speed. The split is
  logged on `SPEED_TRACE` / `SIM_RTF`. It is not billed.

The next lap should still show straights in that 10–40% band.
`wh_model_turn` should stay near 0.096 Wh on a similar lap.

## Run #11, HUD paint off, wall clock

Dan's light warehouse lap on `fc55ab7` (1 lap, HUD paint off, dashboard
restarted before the lap). This does not replace run #8 or run #9.
Run #10 was the same commit without a dashboard restart, so it never
printed `MISSION_SUMMARY`. Its segments summed to about 4.19 Wh
electrical against 4.74 Wh model (−11.5%), and segment 65 carried
about 1500 s parked.

| Field | Value |
| --- | --- |
| duration_s | 764.8 |
| dist_m | 205.19 |
| turns | 13 |
| soc | 100 → 96.99 |
| wh_model | 6.930 |
| wh_model_idle | 2.116 |
| wh_model_drive | 4.814 |
| wh_model_roll | 0.448 |
| wh_model_visc | 4.256 |
| wh_model_turn | 0.103 |
| wh_model_accel | 0.007 |
| wh_measured_drive_shaft | 0.992 |
| wh_measured_drive_elec | 4.287 |
| eta_avg | 0.231 |
| wh_measured | 6.381 |
| wh_measured_soc | 7.224 |
| err_pct (MISSION_SUMMARY, drive only) | −11.0 |
| err_pct (MISSION_ENERGY, drive + idle) | −7.9 |

`wh_measured_soc` is `240 × (100 − 96.99) / 100 = 7.224`. The 0.843 Wh
gap is about 101 s at ~30 W. Posts were about every 3 s. The log
dropped any timestamp gap over 15 s. The controller drained every
step's wall gap, including gaps under 15 s while a POST failed or the
dashboard was down. An aborted 29.9 m mission at 22:31 is not inside
this summary (`dist_m` is one lap and `soc_start` is 100).

Drive watts were about 20 W on both run #9 (5.075 Wh / 896 s) and
run #11 (4.287 Wh / 765 s). HUD paint off shortened the wall clock.
The watt-hours fell with it. That is not a lower sim torque and not
a new coefficient.

The next lap bills the controller's simulation-step counters
(`energy_wh_*`). `wh_measured` and `wh_measured_soc` should agree.
`duration_s` is sim seconds. `TELEMETRY_GAPS` reports the post
interval, seconds over 15 s, and the watt-hours in that hole
(`cumulative=1` means they were kept). `err_pct` will not match
−11.0: the old number scaled with wall time. `STEER_SPLIT` is the
left/right variance for a later HUD-on comparison. It is not billed.
