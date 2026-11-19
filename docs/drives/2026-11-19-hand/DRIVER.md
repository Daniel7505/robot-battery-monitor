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

## After the raw-floor cut, `a23b846`

`RAW_FLOOR_M` is 0.045 m. Still windows sum 0.024–0.034 m of raw floor. A real slow reverse sums 0.056–0.064 m. The cut is the midpoint, so each cloud keeps 0.011 m. Same three commands.

| stop | command | status | estimate (x, y, yaw) | truth (x, y, yaw) | pos err |
|---|---|---|---|---|---|
| 2 | go_to 0.4 2.7 | stopped_short, 0.41 m | (0.35, 2.30, 1.05) | (3.93, 4.95) at t=32 | 0.009 |
| 3 | go_to 0.6 2.15 | stopped_short, 0.29 m | (0.42, 2.38, −0.36) | (3.67, 5.02, 1.26) | 0.117 |

No `pinned` line. Max pose error 0.123 m at t=43.2 s. Truth x reached 3.663, through the shared wall. At explorer t=37.2 s the log is `floor blank, wheel translation held std=0.1`. That hold rewound x,y to its anchor once the wheels had claimed 0.02 m on a floor image with std under 2. The drive stopped on that jump. The blank-floor hold was not changed.

## Blank floor, `73b14c6`

A floor image with std under 2 is not a vote. If this frame's forward picture changed — a tracked translation, or a picture that moved and did not fit (`moved`, `few_tracks`, `no_fit`, `too_large`) — the step stays. A still picture, or no picture, still rewinds. Fusion off still rewinds, which is the flat-floor test and the climb. No new threshold. The cuts are the ones `_images_still` already uses.

A tracked translation alone did not cover the doorway: the picture moved and the corners did not fit, and the hold still fired. `73b14c6` is the yield for that picture.

## A new waypoint, `97229ea`

`imu_stuck` ended a slide and the next `go_to` stopped in the same tick. The IMU samples and the camera clock were the previous command's. Arming a command clears them. The 2.5 s checks are unchanged. No new threshold.

## Lap

Stock spawn, living room. `RBM_MOVIE=1`. Short steps. Truth from the pose log only.

| place | truth (x, y) | pos err | note |
|---|---|---|---|
| start, living | (6.096, 4.688) | 0.005 | look arrived |
| living, toward the door | (4.954, 4.914) | 0.018 | `go_to 0.2 1.2` arrived |
| living, at the door | (4.067, 4.909) | 0.028 | `go_to 0.25 2.1` arrived |
| bedroom door | (3.745, 4.943) | 0.019 | `imu_stuck`. Shared-wall face is x=3.759 |
| same door, next aim | (3.745, 4.943) | 0.033 | `imu_stuck` again. Wheels 0.39 m, floor 0.15 m |

The bedroom interior, the bathroom, the hallway, and the return were not reached. The lap did not finish, so there is no lap movie.

One earlier crossing, before the revert below, did enter the bedroom: truth (3.12, 5.18), pose error 0.091. From t=41.2 s to t=42.0 s the nose was blind, the wheels claimed 0.14 m, and the pose took 0.03 m. No blank-floor line in that window. Putting a blind step back when the floor said it matched then walked a later still shell from 0.053 m to 0.171 m while truth x stayed 3.74. That change is `cd59b13`, reverted by `4cb96aa`. The wedge rule stays.

## Door center

Truth (3.745, 4.943) is the south jamb of door G, not the opening. The opening is world y 4.72–5.54, center (3.71, 5.13). In the odometry frame that center is (0.43, 2.39), and the opening runs x 0.02–0.84. The lap goals were x 0.25–0.28. A* on the robot's map followed that line. The same map had a free path at x 0.45–0.55. The waypoint was the mistake. The map and the planner were not.

The next run aimed at that center.

| place | truth (x, y) | pos err |
|---|---|---|
| start, living | (6.096, 4.688) | 0.005 |
| door center, living side | (5.052, 5.094) | 0.027 |
| door center, at the face | (4.134, 5.151) | 0.039 |
| bedroom, just inside | (3.388, 5.182) | 0.061 |
| bedroom | (2.880, 5.213) | 0.063 |
| bedroom, toward the walkway | (2.464, 4.964) | 0.248 |

The last step stopped 0.30 m short. From t=50 s to t=55 s the truth path was 0.49 m and the estimate moved 0.32 m. `floor still, translation frozen` at t=50.8 s. Error 0.14 → 0.25 m. The drive stopped. The bathroom, the hallway, and the return were not driven. No lap movie. `up_z` 1.000.
