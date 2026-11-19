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

The last step stopped 0.30 m short. From t=50 s to t=55 s the truth path was 0.49 m and the estimate moved 0.32 m. `floor still, translation frozen` at t=50.8 s. Error 0.14 → 0.25 m. That line is not the pin and not the blank-floor hold. From t=49.3 s to t=50.3 s the wheels and the truth both moved about 0.07 m and the pose stayed. `e050d1d` does not rewind when the floor scene already matches the wheels, including a straight line. A scene that did not follow still rewinds.

## After that release

Door G on the center line, odometry x 0.43.

| place | truth (x, y) | pos err |
|---|---|---|
| start, living | (6.096, 4.688) | 0.005 |
| door G, living side | (5.050, 5.078) | 0.022 |
| door G, at the face | (4.173, 5.134) | 0.032 |
| bedroom | (3.328, 5.143) | 0.073 |
| bedroom, toward the walkway | (2.567, 5.125) | 0.070 |
| walkway mouth | (1.009, 4.858) | 0.078 |

The walkway wall is world y 4.57. The shell stayed on the bedroom side. The south aim was unknown on the map, a straight line, and it stalled: `no motion for 2 s`. The bathroom, the hallway, and the return were not driven. No lap movie. `up_z` 1.000.

## Walkway center, then the bathroom

The straight aim is the existing rule. A turn at the mouth mapped the south cells. A* on odometry y 4.88 crossed.

| place | truth (x, y) | pos err |
|---|---|---|
| start, living | (6.096, 4.688) | 0.005 |
| door G, living side | (5.050, 5.079) | 0.022 |
| door G, at the face | (4.183, 5.134) | 0.042 |
| bedroom | (3.341, 5.137) | 0.061 |
| walkway mouth | (1.214, 4.941) | 0.020 |
| bathroom, just through | (1.181, 4.373) | 0.131 |
| bathroom | (1.176, 4.063) | 0.140 |
| bathroom, `imu_stuck` | (1.311, 3.430) | 0.116 |

From t=54.8 s to t=55.5 s the pose crept and snapped back to the same x. No new freeze line. Error 0.027 → 0.131 m. The next leg was still moving, no wall contact, `up_z` 1.000, and stopped `imu_stuck`. The hallway and the return were not driven. No lap movie.

Before that rerun, door H on the previous process stopped `0.56 m short` at the plane. The path was A*. `reverse_left` at t=93.5 s, then the stall at t=97.5 s while the shell was closing again. `0b55545` measures that retry from the backed-up pose. The rerun did not reach the door.

## Net yaw, then out of the bathroom

`0b55545` is in the logbook. The bathroom `imu_stuck` was a yaw command that flipped every step. Net wheel yaw −0.011 rad/s, gyro still, shell moving, no contact. `f187036` uses the net rate. The same point then arrived.

The t=54.8–55.5 s snap is the 0.13 m. Error 0.027 m at t=53.4 s, 0.045 m at t=54.8 s, 0.137 m at t=55.5 s. Truth moved 0.091 m and the pose stepped back 0.013 m. It is not what fired `imu_stuck`.

| place | truth (x, y) | pos err |
|---|---|---|
| start, living | (6.096, 4.688) | 0.005 |
| bedroom | (3.338, 5.106) | 0.059 |
| walkway mouth | (1.218, 4.918) | 0.024 |
| bathroom, just through | (1.178, 4.383) | 0.123 |
| bathroom, old stuck point | (1.317, 3.417) | 0.116 |
| hall | (2.377, 1.759) | 0.113 |
| hall, east | (3.409, 1.746) | 0.115 |
| kitchen | (4.096, 1.809) | 0.132 |

The kitchen goal was still free. A* was 0.32 m. The shell slid east of it, closest 0.285 m, then `0.30 m short`. No contact. `up_z` 1.000. The living-room start was not reached. No lap movie.

## Home

The 0.30 m short was the driver. The goal was beside the shell and inside the 0.25 m lookahead, and pursuit kept a point that was still ahead. `a5bc64a` steers at the goal. The walkway snap at t=54.8–55.5 s is left open.

`f187036` is in the logbook. `RBM_MOVIE=1`. `docs/drives/2026-11-19-hand/lap.mp4`.

| room | truth (x, y) | pos err |
|---|---|---|
| living, start | (6.096, 4.688) | 0.005 |
| bedroom | (3.338, 5.146) | 0.062 |
| bathroom | (1.360, 1.900) | 0.106 |
| hall | (2.164, 1.734) | 0.123 |
| kitchen | (3.792, 1.743) | 0.117 |
| living, home | (5.915, 4.511) | 0.223 |

`up_z` 1.000. No contact on the return.

## Frontier assist

The morning lap's map is not in the repo. `logs/mapping_runs/` is gitignored, and this machine has no `map.npy` or pose log from that run. `lap.mp4` is the pose logger's viewpoint, not an occupancy grid, so its seen % cannot be scored. No rerun.

This session is stock spawn, depth seed 1, `RBM_DRIVER=1`, headless, no movie. At each stop the mapper runs the existing `select_goal` and prints `MAPPING frontier proposal`. The green path on `map.png` is that path. A magenta cell on it is occupied. The goal is blue. Seen % is `coverage_pct` on that stop's own grid. Pose error is the pose log.

| stop | t | what happened | proposal (odom) | decision | pos err | seen % |
|---|---|---|---|---|---|---|
| 0 | 3.5 | ready | (0.03, −0.12), 4 points | override, `look` | 0.000 | 0.15 |
| 1 | 17.3 | look arrived | (−0.07, 3.17), 33 points, bedroom | taken | 0.005 | 67.53 |
| 2 | 56.9 | `stopped_short` 0.60 m | (−0.33, −0.47), living room | override, `go_to 0.50 4.40` | 0.165 | 76.48 |
| 3 | 62.3 | `stopped_short` 1.63 m | (−0.33, −0.47), living room | taken | 0.155 | 76.57 |
| 4 | 69.7 | `stuck`, no motion for 2 s | (−0.57, −1.57), living room | taken | 0.163 | 76.91 |
| 5 | 91.0 | arrived | (−3.02, 3.14), hall, last cell unknown | override, last free cell `go_to -2.93 2.25` | 0.177 | 76.92 |
| 6 | 96.1 | `stopped_short` 4.48 m | (−3.02, 3.14) again | stopped | 0.185 | 76.93 |

Stop 0 is 29 free cells. The proposal sits 12 cm to the side of the shell. A look is what makes a doorway exist on the map.

Stop 1's path crosses the shared wall at world y 5.07. Door G's opening is 4.72–5.54. The goal is in the bedroom.

Stop 2's proposal goes back to the living room. The same map has a free path west, toward the bath walkway. The override aimed there. From t=33 s to t=34 s the error went 0.030 → 0.105. The line is `floor blank, wheel translation held` at explorer t=29.4 s, std 0.2. The forward picture did not count as moved, so the existing hold rewound. That hold was left in place. The shell then rocked near truth (3.05, 5.08) until the 2 s clock. `up_z` 1.000. No contact.

Stop 3 is that west aim. The shell was already in the corner, nose about 0.5 m, and the command ended 1.63 m short. The living-room proposal was the way out, so the next one was taken.

Stop 4 did not move. `floor still, translation frozen` at explorer t=64.6 s. Over that second the truth path is about 0. The line matches a shell that was sitting. The following proposal, further into the living room, arrived. Three `floor blank` lines at explorer t=70.1 s, std 0.5. Error 0.160 → 0.174, then 0.125 at t=76 s, 0.177 at the stop.

Stop 5's goal cell is unknown, so a `go_to` of that cell is a straight aim. The cell before it is free and the driver's path to it is A*. That cell is world (3.85, 1.77), on the kitchen side of the hall. The command moved about 0.6 m and stopped 4.48 m short beside the long living-room frontier. Nose 0.55 m. No new freeze line. Error 0.185. The same hall proposal was still the next one. The drive stopped there.

End, room seen %: living 98.9, bedroom 93.6, kitchen 99.3, bath 4.9, laundry 46.8, closet 33.1, hall 20.5, storage 0, pantry 0. Visited 6.6. The bath was not entered. Pantry and storage were not opened. `up_z` minimum 0.99992. No wall-contact sample. The walkway snap at t=54.8–55.5 s was not reached. No new controller bug. No movie.

## Corner, then the door hold

Stops 2–4 of that session. Pose error 0.165, 0.155, 0.163. From t=67.0 s to t=69.7 s the yaw command flipped 139 times. `194b88f` keeps one heading for the turn after a reverse, and a mapped wall is not a straight aim.

The same commands after that change. The bedroom goal stopped 1.08 m short at t=28.2 s, pose (0.20, 2.13), instead of rocking until t=57 s. `go_to 0.50 4.40` then said `imu_stuck`. Net wheel yaw in the 2.5 s before it was about 0.22 rad/s and the gyro about 0.06 rad/s, which is the existing check. `go_to -0.33 -0.47` arrived at t=58.0 s.

The pose step at the bedroom door, from the first session. At t=33.0 s the error was 0.030 m and the estimate was (0.368, 2.353). It was still there at t=33.4 s. The truth moved 0.066 m. The error was 0.095 m, and 0.105 m at t=34.0 s. `floor blank, wheel translation held` at explorer t=29.4, 29.6, and 29.8 s. The yield did not run. A still picture still rewinds. No new threshold. Not changed.

## After the corner fix

Stock spawn, depth seed 1, `RBM_DRIVER=1`, no movie. The route was living, bedroom, back through the corner, hall, bath door. It stopped on the living side of door G.

| stop | t | result | proposal (odom) | decision | pos err | seen % |
|---|---|---|---|---|---|---|
| 0 | 3.5 | ready | (0.03, −0.12), 4 points | override, `look` | 0.000 | 0.15 |
| 1 | 17.3 | look arrived | (−0.07, 3.17), bedroom | taken | 0.005 | 67.53 |
| 2 | 28.0 | 1.08 m short | (0.46, 6.68), off the floor | override, `go_to 0.40 2.80` | 0.021 | 70.81 |
| 3 | 34.5 | `imu_stuck` | (3.21, 5.17), straight aim | override, door center `go_to 0.43 2.39` | 0.034 | 72.95 |
| 4 | 49.5 | 0.58 m short | (0.98, 3.73) | stopped | 0.036 | 73.78 |

Stop 2 is the new wall rule. The goal cell was occupied, so the path ended on the last free cell, world (3.97, 4.90), still on the living side of the shared wall. The next proposal is outside the floor.

Stop 3 is the existing imu check. In the 2.5 s before t=35 s the net wheel yaw was 0.22 rad/s and the gyro 0.06 rad/s. The truth moved 0.04 m. Not a new threshold.

Stop 4's first command was `reverse_right`. Pursuit wanted a hard right. Every forward right turn and the right spin were `map:corner`. The reverse is the turn that still fits, and the 2 s clock then carried the shell from world (3.91, 4.89) to (4.10, 4.69). Changing that score would be a new weight. The drive stopped there. Hall and the bath door were not reached. Bedroom visited 0.

Room seen % at stop 4: living 98.4, bedroom 85.3, kitchen 99.3, bath 1.8, laundry 46.8, closet 4.6, hall 20.5, storage 0.0, pantry 0.0. Bridge bedroom 100.0, bridge bed-bath 97.8, bridge entry 94.7, bridge laundry 58.3, bridge bath 0.0, bridge closet 0.0. Visited overall is the shell on the pose log; bedroom visited 0.0. Pantry and storage were not opened.
