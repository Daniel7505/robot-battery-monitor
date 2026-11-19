# Hand drive, 2026-11-19

Stock spawn, depth seed 1, `RBM_DRIVER=1`, `RBM_MOVIE=1`, headless. Aims are in the robot's odometry frame. Truth is `gt_pose_log.csv` only. The green line on a backup-camera image is the path `WaypointDriver` would follow (`_path_on_own_map`). Orange would be a straight aim through unknown space. None of these aims were that.

Pose error is the odometry estimate placed on the spawn (6.10, 4.70, yaw 1.571) and compared with the pose log.

## First run, before `7c24c91`

| stop | command | status | estimate (x, y, yaw) | truth (x, y, yaw) | pos err |
|---|---|---|---|---|---|
| 0 | — | ready | (0, 0, 0) | (6.10, 4.70, 1.571) | 0.000 |
| 1 | look | arrived | (−0.02, 0.01, −0.08) | (6.10, 4.69, 1.490) | 0.005 |
| 2 | go_to 0.4 2.7 | stuck, no motion for 2 s | (0.25, 1.77, 1.57) | (4.34, 4.94, 3.14) | 0.010 |

The map after the look had a gap at about y=2.4, x=0.2 to 0.6, and A* reached (0.4, 2.7) without crossing an occupied cell. The shell drove to (0.25, 1.77) at 0.28 m/s and the command became (0, 0). `choose_motion` had returned cruise. `motion_blocked` clipped the jamb with the turn rectangle. Nose was still 1.37 m. `up_z` 1.000. No wall contact.

## Rerun, `7c24c91`

| stop | command | status | estimate (x, y, yaw) | truth (x, y, yaw) | pos err |
|---|---|---|---|---|---|
| 0 | — | ready | (0, 0, 0) | (6.10, 4.70, 1.571) | 0.000 |
| 1 | look | arrived | (−0.02, 0.01, −0.08) | (6.10, 4.69, 1.48) | 0.006 |
| 2 | go_to 0.4 2.7 | stopped_short, 0.49 m | (0.24, 2.23, 1.12) | (3.85, 4.94, 2.72) | 0.017 |
| 3 | go_to 0.6 2.15 | stuck, no motion for 2 s | (0.24, 2.22, 1.06) | (3.87, 4.92, 2.64) | 0.015 |

The rerun passed the old stuck pose. Truth x reached 3.787, still on the living side of the shared wall (west face 3.759). The shell did not enter the bedroom, the bath, the laundry, or the kitchen. Max pose error on this run was 0.026 m at t=14.9 s, during the look. Max yaw error 0.006 rad. Final 0.015 m. No loop closure. One `floor still, translation frozen` at t=28.3 s. The error did not jump. Lowest `up_z` is 1.000. No wall-contact row.

At stop 3 the command swapped every 16 ms between creep `(1.5, 1.5)` and straight reverse `(-1.25, -1.25)`. The encoders rocked and the shell stayed. A side gap under 0.10 m taxes the forward command, and the rectangle vetoes the yaw-away reverse the jamb-press test requires, so the two straight commands take turns. That tax was left in place.

Overlays and the front camera are in `stops/`. `run.mp4` is the pose-logger frames at 2 fps from sim t=8 s. The doorway mouth is about 5 s in (sim t=28 s).

## After the back-off clock, `6562ac6`

Same three commands, then a stop. The creep/reverse swap is gone. On `go_to 0.4 2.7` the back-off `(-0.65, -2.35)` held from t=28.832 s to t=30.840 s, then the command was a spin.

| stop | command | status | estimate (x, y, yaw) | truth (x, y, yaw) | pos err |
|---|---|---|---|---|---|
| 1 | look | arrived | (−0.02, 0.01, −0.08) | (6.10, 4.69, 1.49) | 0.005 |
| 2 | go_to 0.4 2.7 | stopped_short, 0.40 m | (0.35, 2.30, 1.05) | (3.81, 5.03, 2.62) | 0.023 |
| 3 | go_to 0.6 2.15 | stopped_short, 0.31 m | (0.41, 2.40, −0.54) | (3.58, 4.99, 1.02) | 0.169 |

Stop 3 is the new issue. At t=37.1 s the log is `pinned, depth floor and visual still, wheel translation frozen`. From t=37 s to t=48 s the truth path is 0.74 m and the estimate stays near (0.42, 2.38). Error goes 0.021 → 0.171. Yaw error stays under 0.01 rad. `up_z` 1.000. No loop closure. Truth x 3.58 is through the shared wall (west face 3.759), so the shell entered the bedroom while the estimate did not follow. The drive stopped there. The pin is the existing still-scene gate in `pin.py`. It was not changed.
