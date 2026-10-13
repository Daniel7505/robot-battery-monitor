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
