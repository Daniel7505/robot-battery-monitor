# Lessons learned

Read this before changing apartment mapping. Append what the latest sim actually showed. Do not write a cause the logs did not support.

## What this robot is

The apartment machine is `ButlerBotSkid`: four driven wheels, no casters. Hubs at y = ±0.17 m (track 0.34 m), wheel radius 0.08 m, nose and tail 0.19 m, side face 0.227 m. Planning inflation stays 0.20 m. The depth camera is an OAK-D Lite at (0.14, 0, 0.78), pitched 5°, minimum range 0.35 m. A reading of nose = 0.12 m is the blind placeholder, not a measured gap.

The warehouse still uses the caster `ButlerBot`. That proto keeps its casters and its device depth noise. Do not edit it to fix the apartment.

`apartment.wbt` instances `ButlerBotSkid` directly. A mapping launch aborts if the world is the caster. The mapper never reads the ground-truth floor plan, the room rectangles, or the pose logger.

## Rooms and seeds

The scored floors, from `apartment.wbt`, used only after the run: living, kitchen, bedroom, bath, hall, plus smaller closets. The bath opening (`APT_FLOOR_BRIDGE_BATH`) is 0.81 m across. The kitchen floor starts at x = 3.76, y = 0 to 2.51. The bedroom is x = 0 to 3.66, y = 4.62 to 8.33. Its southwest corner is the west wall meeting the south wall near (0, 4.62). The walkway gap is east of about x = 0.81. That corner is not the doorway.

The four 600 s seeds, spawn (x, y, yaw): stock (6.10, 4.70, 1.5708), +0.40 (6.10, 4.70, 1.9708), −0.35 (6.10, 4.70, 1.2208), north (5.80, 5.60, 2.2). Odometry goals convert to the apartment frame with that spawn. The ground-truth pose log is already in the apartment frame.

## Metrics

- Seen %: share of ground-truth free floor the robot's map has labeled. Rays through a door raise this without the shell entering.
- Visited %: share of that floor the rectangular shell (0.19 m nose and tail, 0.227 m half-width) drove over, from `gt_pose_log.csv`. Scoring only.
- Pile: longest time the ground-truth body stayed inside 0.45 m of one point. The bar is 60 s.
- Stuck: decision-log reasons `camera_stuck`, `imu_stuck`, and `camera_imu_stuck` in `summary.txt`. `unknown` is the older relief action and is counted separately. A missing `stuck, next goal` line does not mean the count is zero.
- Wall and corner steps: physics-log steps with a tire, or a front tire, on a wall.

## How to run one check

From the repo root, with Webots R2025a on `PATH`. Copy the apartment world, keep `ButlerBotSkid`, set the spawn, and launch headless. `eval/run_skid_seeds.py` does this. Env: `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MAP_DIR` per seed, `RBM_MOVIE=1` if you need frames. Delete a previous seed folder first. A leftover `mapping_done` makes the pose logger quit immediately.

Score with `eval/score_apartment_map.py` and `--spawn X Y YAW` for that seed. The stock world yaw is not the seed yaw. Warehouse sanity: `RBM_MISSION_LAPS=1` on `butlerbot_warehouse.wbt`, `--no-rendering`. Pass is the 201 m cap and `MISSION complete`. Kill that process by its port, not with a broad pattern.

## How a failed seed was read

This is the method behind the kitchen and pile notes. Use it before changing the planner.

1. Confirm the run reached 600 s: `mapping_done` exists and `gt_pose_log.csv` has about 75,000 rows (8 ms steps).
2. Stuck, back-ups, `heading_off`, and bans are the counts in `summary.txt`. From round 7, stuck is `camera_stuck`, `imu_stuck`, and `camera_imu_stuck`. `unknown` is the older relief and is a separate line. A missing `stuck, next goal` line in `webots.log` does not mean the stuck count is zero. Round 5 north had 198 `action=stuck` rows in `decisions.txt` and no `stuck, next goal` line.
3. Pile: walk `gt_pose_log.csv` and take the longest span whose points stay inside 0.45 m of the first point of that span. Do not sum separate visits to the same neighbourhood.
4. Room visited: warp `map.npy` with the seed spawn for seen %. Stamp the shell on the ground-truth pose for visited %. The two numbers answer different questions.
5. Goals are odometry. Convert with the seed yaw before comparing them to a floor box. Round 4 −0.35 had eight converted goals inside the kitchen box and the path touched the box, so the failure was not "no goal." North had three goals inside the bath and the path stayed 0.45 m short, so the failure was not "door too narrow." The bath opening measures 0.81 m.
6. After a `pile, leaving` line, measure ground-truth motion over the next 30 s. Round 4 north moved 0.00 m and the command was `(0.00,0.00)`. Round 5 north's long stay is a slow spin (`-0.64,0.64`) at one pose with nose 0.12. That is the evidence. A hypothesis that the rear sensor was closed the whole time is not in the log.

## Standing rules

- No map, room size, or floor-area foresight to the robot. Ground truth is for scoring only.
- Camera-first. No lidar.
- Keep the skid-steer. Keep the caster proto for the warehouse.
- Never commit run artifacts or secrets. Preview PNGs under `webots/meshes/previews/` are generated by `build_skid_mesh.py` and are gitignored.
- Delete superseded code. Decision logs use the fixed reason-code pool, one folder per seed.

## Caster branch used on the skid robot

Symptom: fixes from PR #38 (`cursor/apartment-mapping-5970`) were treated as the robot under test. That branch still instances the caster.

How it was diagnosed: the world file and the proto on that branch were read. The apartment instance was `ButlerBot`, with casters. The skid head at the time was `cursor/jamb-veto-5970`.

Root cause, confirmed: two robots share the repo. Launching the caster instance runs a different drivetrain. The mapping ideas still applied. The motion stack did not.

Fix: port onto the skid head. Every mapping launch aborts unless the world is `ButlerBotSkid` and not the caster. `apartment.wbt` now instances the skid proto. The runtime swap of the caster instance was deleted.

Why not the alternative: editing `ButlerBot.proto` would change the warehouse hash and the caster tests.

Result: apartment runs print `MAPPING front hubs slaved`. The warehouse lap still reaches the 201 m cap.

Dead ends: do not run from the caster branch. Do not delete `ButlerBot.proto`. Do not put OAK depth noise on the caster device. The mapper applies the quadratic model. The skid proto's device noise stays 0.

## Seen from the doorway, never driven in

Symptom: round 2 coverage was about 93% on −0.35 while the bath was 95% seen and 0% visited, and the kitchen centre only crossed 0.37 m past the entry. Round 3 kitchen visited was 0% on all four seeds. Round 4: −0.35 kitchen 2%, bath 0%; north kitchen 0%, bath 0%, bedroom 14%.

How it was diagnosed: scoring warped the robot map with the seed spawn, and stamped the shell along `gt_pose_log.csv`. Goals in `webots.log` were converted with the same spawn. Round 5's check of the round 4 logs: the bath opening is 0.81 m, wider than the shell. −0.35 had 8 goals inside the kitchen box, the closest at (3.84, 1.94), 8 cm inside x = 3.76, and the path touched the box. North had 3 goals inside the bath, including (1.40, 3.92), and the path stayed 0.45 m short of the bath floor. So the door is wide enough, and goals were chosen. They were not driven through.

Root cause, confirmed for those logs: a stall fired while the shell was still on the threshold (`stall, backing out` within a few seconds of the room goal), and the goal was replaced. The unvisited target was also the nearest other-region cell about a metre out, which landed on the kitchen edge rather than inside the room. Not confirmed: that every failed entry is a smear that closes the door in the robot's map. The goals existed, so A* had a path at selection time.

Fix chosen: a doorway line-up (`approach` straight or pivot) is not a translation stall, so the goal is not dropped while the robot is still aiming through the opening. The unvisited goal is a cell about 2 m into the other region of the robot's own segmentation, not the first cell across the line. The settle step passes the robot's visited grid, so "no frontier" is not done while that region is undriven.

Why not the alternatives: feeding the apartment room rectangles to the planner breaks the standing rule. Widening the 0.20 m inflation would not explain a goal that was already inside the bath. Treating seen % as the finish condition is the bug.

Result, round 5 two-seed check: −0.35 kitchen visited 5%, bedroom 26%, bath 0%, pile 71 s. North kitchen 0%, bedroom 0%, bath 0%, pile 383 s. The bar was not cleared. Full four seeds were not run. Still open for kitchen and bath as a reliable visit.

Dead ends: do not report seen % as "went there." Do not stand at the 1–3 m viewpoint and call the room finished. Do not give the robot the floor plan.

## Bedroom southwest blind corner

Symptom: round 2, seed −0.35, 171 still-floor events and 206 s at (0.35, 4.97). Heading about −161°. Nose 0.12 m on 84% of the status lines. Wheels at 0.080 m, wall-contact columns zero.

How it was diagnosed: ground-truth pose, the phase lines (`nose=` and `cmd=`), and the apartment solids. The south wall ends near x = 0.81. The pile is west of that, at the inside corner. The depth camera's minimum range is 0.35 m and the lens is at x = 0.14, so a west-facing nose 0.36 m from the wall is inside the minimum.

Root cause, confirmed: 0.12 m is the blind placeholder. Frontier failures skipped every blind look, including a room with no translation. The relief reverse was 0.45 s, about 4 cm, then the same laundry goal was chosen again (24 times in that window).

Fix: a blind view plus no translation, place tag room, counts as a frontier failure. Back up until depth is at least 5 cm past the minimum and the shell has moved 0.30 m. A doorway exemption remains only for an actual crossing: doorway tag and a straight creep or a forward drive. An open-floor pivot stays exempt.

Why not the alternative: banning every doorway tag also banned the crossing the robot was already on. That had been tried and reverted in the tests.

Result: the next −0.35 run spent 0 s inside 0.45 m of (0.35, 4.97).

Dead ends: do not treat 0.12 m as an open hall or as a measured 12 cm gap. Do not use only the 12 s avoid-weight fade to leave a patch.

## Visible stall and the 60 s pile

Symptom: round 3 stock sat 527 s at (3.29, 1.60), 258 events, nose about 1 m. −0.35 sat 528 s at (3.85, 5.16). Round 4, after a guard that only flipped the spin and reset its clock: piles 181 s and 348 s. North's second `pile, leaving` was followed by `cmd=(0.00,0.00)` and 0.00 m of ground-truth motion in 30 s. Round 5: −0.35 pile 71 s, north 383 s. North's decision log has 198 `action=stuck` lines. From about t = 321 s the command is a slow spin (`-0.64,0.64`) at one pose with nose 0.12, and the pose does not change. `stuck, next goal` does not appear. The stuck count is the relief action, not a goal change.

How it was diagnosed: `gt_pose_log.csv` for the stay, phase lines for nose and wheel command, and `decisions.txt` for the stuck reason. Floor-still was compared with the ground-truth pose, not assumed.

Root cause, confirmed: on the round 3 hall pile the ground-truth body was not translating, and the floor picture did not shift, so the freeze matched the body. A blank, low-texture frame does not freeze (it resets the anchor). The miss was the response: a 0.45 s reverse, then another goal in the same region. Round 4's zero speed is also confirmed: both straight sweeps failed, the code returned `(0, 0)`, and the shell stayed. Round 5 replaced that zero with a spin when the rear trail was not taken. On north the spin did not yaw the body. Unconfirmed: that the rear sensor was closed for that whole 383 s. The log shows the spin command and a frozen pose, not a rear-range trace for every step.

Fix chosen: never return zero speed from the pile leave. If the map sweep closes both straights and the rear check is open, reverse toward a crumb the shell already drove (a pose at least 0.35 m back on a 0.25 m trail). If the crumb is beside the robot, turn only when the corner sweep allows. If the rear check is closed and the forward camera still has a real range of at least 0.5 m, creep forward. A doorway line-up is not a 2 s stall.

Why not the alternatives: turning off floor-still would let wheel slip walk the pose. The round 3 body was actually still. A fixed 0.45 s reverse was already too short. Spinning when the sweep fails put corner steps back (round 3: +0.40 had 140, north 1410). Round 4's refusal of that spin dropped north's corner steps to 0, then to 0 again in round 5, but the held-zero and the later in-place spin did not leave the ball.

Result: corner steps stayed 0 on both round 5 seeds. Piles did not go under 60 s. The four-seed run was not started.

Dead ends: do not disable floor-still without a ground-truth pose that moved while the log said frozen. Do not restore the 0.45 s reverse. Do not spin into a failed corner sweep. Do not return `(0, 0)` from the pile leave. An in-place spin that the body does not follow is the same pile. Do not add another detector for one nose value or one wheel command. Round 6's check only fired when the nose was the 0.12 m placeholder, so north's `cmd=(1.06,-1.06)` spin with nose 99 was invisible. The round 6 path trim, which had no log line, is what regressed −0.35. Both are deleted. See the round 7 entry.

## Pivot swept a corner into the wall

Symptom: before the rectangle sweep, corner-contact steps were 89 / 204 / 74 / 416 on the four seeds. `heading_off` was 85 / 222 / 216 / 186.

How it was diagnosed: the physics log's front-tire wall columns, and the pivot command (`v = 0`, `|w| = 0.70`) continuing after the nose was close to the path.

Root cause, confirmed by the contact drop after the change: turn checks used a disk. The shell is a rectangle. The corner swings outside the tires. A one-cell heading error restarted the pivot.

Fix: pivots and near-wall arcs sweep the rectangle. A corner that meets the map becomes the existing straight backup. A pivot ends at 12° and does not start again until 25°. The jamb veto, the 0.23 / 0.30 m approach clearance, and the goal-ahead rule on ordinary backups stayed.

Result: the next four seeds had 0 corner steps. Later escape spins brought some back, then the sweep refusal took north back to 0. Do not raise the 0.20 m planning radius to 0.227 m or 0.26 m. That closes a doorway.

## Other traps the logs already hit

- A seed folder that still has `mapping_done` quits the next launch at once. The pose logger sees the file and calls `simulationQuit`. Delete the folder first.
- Killing Webots by a broad pattern killed another seed's X server. Kill by port.
- Odd video sizes fail in ffmpeg. Scale to an even width and height.
- PR #40 (pytest-xdist on `main`, idle since 27 Oct 2026) is not safe to merge. CI failed: workers raced on Postgres and hit a duplicate `allocation_snapshots` type. The speedup is real when database tests stay on one worker. Do not close it from a mapping round.

## Round log

- Round 3, 2026-11-02. Any-stall escape, unvisited goals, skid instance in `apartment.wbt`. Piles 95 / 235 / 439 / 250 s. Kitchen visited 0% on all four. Worst pile −0.35 at (3.99, 5.04), 439 s.
- Round 4, 2026-11-02. Straight leave at 60 s, clipped spins refused. Two seeds only. Piles 181 s and 348 s. North commanded zero speed and moved 0.00 m. Corner steps on north 0. Kitchen 2% and 0%.
- Round 5, 2026-11-02. Trail reverse instead of zero speed. Doorway line-up is not a stall. Unvisited goal about 2 m into the other region. Two seeds only, bar missed. −0.35 pile 71 s, kitchen visited 5%, bedroom 26%, bath 0%, corner steps 0, stuck 3. North pile 383 s, kitchen 0%, bedroom 0%, bath 0%, corner steps 0, stuck 198 from a slow spin at one frozen pose. Four seeds not run.
- Round 6, 2026-11-02. A path that ends with the nose in one mapped wall and a side against another is cut back to the last pose where a pivot fits. A spin with no yaw change and nose 0.12 m reverses along the trail. Two seeds only, bar missed. The `(-0.64,0.64)` command is gone (0 lines). North's pile is now `cmd=(1.06,-1.06)` with nose 99, 193 phase lines, pose frozen, pile 446 s at (3.90, 4.97) from t=154 s, stuck 193, kitchen/bath visited 0. −0.35 pile 361 s, kitchen 0, bath 0, bedroom visited 15%, corner steps 0. Unconfirmed: that nose 99 means the depth frame is empty rather than inside minimum range. Confirmed: the body is not yawing under that spin, and the new detector only fires when the nose is the 0.12 m placeholder, so this spin is invisible to it. Four seeds not run. The path trim and the nose check were deleted in round 7.
- Round 7, 2026-11-02. See the entry below. Two seeds only. Bar missed. Four seeds not run. Warehouse lap passed.
- Round 8, 2026-11-02. See the entry below. An 80% body was a diagnostic only and was deleted. Two full-size seeds. North beat the round 5 bar. −0.35 did not. Four seeds not run. Warehouse lap passed.

## One stuck check, not one pose

Symptom: round 6 −0.35 visited fell from 23.4% to 17.0%, the pile grew from 71 s to 361 s, and kitchen visited went from 5% to 0. North's frozen spin moved from `cmd=(-0.64,0.64)` with nose 0.12 to `cmd=(1.06,-1.06)` with nose 99.

How it was diagnosed: round 6 −0.35 has two `nose=0.12` lines, so the nose check barely ran. The planner change on every `select_goal` was `_trim_wedge`, and that function writes no log line. The three `no room to turn` lines in that log are the older tight-frontier refusal, not the trim. North's round 6 pile is a frozen pose under a spin the nose check ignores.

Root cause of the −0.35 regression, confirmed by that count: the silent path cut, not the nose detector. The cut was deleted. `wedge_entry` and `_trim_wedge` are gone. Nothing flags them off.

Fix chosen: two stuck checks, either one starts a straight reverse along the driven trail. A spin is not the escape. The camera check fires when a motion command is on and the depth picture (median range, finite fraction, and a coarse block grid) has not changed for 2.5 s. An empty frame is a stable picture, so nose 99 counts. Odometry yaw does not reset it, and ground truth is not read. The second check is the wheel encoders against an IMU. Encoders were already the hub position sensors, quantized in software at 0.0015 rad. `oak_gyro` was already there. Round 7 adds `oak_accel` on `ButlerBotSkid` only: about 0.03 m/s² of lookup bias, plus 0.04 m/s² of absolute noise and a clamped drift in the mapper, because a fractional lookup is silent near zero. Wheels turning with no matching gyro rate, or a short wheel-speed change with no horizontal acceleration, is `imu_stuck`. Reasons in the fixed pool: `camera_stuck`, `imu_stuck`, `camera_imu_stuck`.

Why not the alternatives: another nose value or another wheel pair would miss the next command, which is what round 6 already did. Leaving the path trim behind a flag keeps the code that regressed −0.35. Putting the accelerometer on `ButlerBot.proto` would touch the warehouse robot.

Result, two seeds, compared with round 5. The bar was both seeds beating round 5 on visited % and on pile time. Neither did. Four seeds were not run.

- −0.35: seen 97.1% (round 5 was 89.5), visited 14.1% (23.4), pile 267 s at (1.68, 4.13) from t=233 s (71 s). Kitchen visited 0, bedroom 30, bath 14, living 13, hall 0. Corner steps 0. Wall steps 1727. Detectors: `camera_stuck` 27, `imu_stuck` 1, both 0. Older `unknown` relief 85.
- North: seen 79.7% (69.6), visited 5.6% (3.4), pile 516 s at (3.99, 4.90) from t=84 s (383 s). Kitchen 0, bedroom 0, bath 0, living 13, hall 0. Corner steps 0. Wall steps 0. Detectors: `camera_stuck` 6, `imu_stuck` 15, both 0. `unknown` 5.

What the logs show about those piles, confirmed: on −0.35 the ground-truth body stays inside 0.45 m while the path inside that window is 8.4 m long and the yaw changes by 25 rad. The commands alternate reverse `(-1.25,-1.25)`, forward `(1.25,1.25)`, and spins. Nose is about 0.55 m. `camera_stuck` fires through the stay. The reverse starts and a later command drives back in. On north, from t=200 s to t=600 s the phase is `spin`, the command is `(-1.49,1.49)`, the ground-truth point stays inside 0.37 m of the pile start, and the yaw only moves from 1.66 to 1.69 rad (2.1 rad of absolute change over 400 s). The last camera or IMU decision is at t=203 s. The check runs in the go phase. The spin phase does not call it.

Dead ends: do not patch one stuck pose, one nose reading, or one wheel command. Do not bring back the wedge path cut. Do not treat a reverse that is followed by a forward command as having left the 0.45 m ball. Do not assume a detector that lives only in the go phase can leave a stay whose phase is spin. The warehouse caster lap still reached the 201 m cap and printed `MISSION complete`.

## An 80% body, then the escape on every phase

Symptom: round 7 −0.35 visited the bedroom and a bit of the bath, and none of the kitchen, then sat 267 s reversing and driving back in. North sat in `spin` from t=200 s with the stuck check not running.

How the size question was diagnosed: one 600 s run of seed −0.35 on a copy of `ButlerBotSkid` whose collision footprint, wheel radius, track, and the planner shell and clearance margins were all at 80%. The decision code was the round 7 code. The proto was not the robot in `apartment.wbt`, and it was deleted after the run. Scoring used the same full shell as round 7, on that run's ground-truth centerline, so a high visited % is not a smaller stamp.

Result of that one run, against round 7 in parentheses: seen 92.6% (97.1), visited 16.5% (14.1), pile 181 s at (6.92, 6.33) in the living room (267 s at the bedroom side). Kitchen visited 26% (0), bath 0 (14), bedroom 0.2 (30), living 30 (13), hall 6.5 (0). Detectors: `camera_stuck` 2 (27), `imu_stuck` 0 (1), `unknown` 6 (85).

Conclusion, confirmed for this seed: the full-size kitchen zero is clearance. The same decisions on an 80% body drove the centerline across a quarter of the kitchen and almost stopped firing the camera check. The bath and the bedroom are not "too narrow to enter." Round 7's full-size path visited them, and this smaller body did not go there on this one run. Unconfirmed: that a second 80% seed would also skip the bedroom. Do not shrink the shipped robot from this one run. The kitchen nick on the full-size robot is what the margins allow. The 26% is what a smaller body did.

Fix chosen on the real-size robot: the camera and IMU check runs before spin, settle, and go, and a stuck spin leaves that phase. After an escape, that spot and heading stay blocked for 18 s, or until the shell is 0.70 m away. A forward command along that heading becomes a reverse. A reverse, and a heading more than about 50° off, are still allowed. The goal that was active is held so it is not scored again until the shell has moved.

Why not the alternatives: leaving the check in go only is the round 7 north pile. A permanent ban of the heading would trap a robot that had turned around after the cooldown. Scaling the shipped proto would change the robot the warehouse tests are not using, and one seed is not a reason to ship a smaller body.

Result, two full-size seeds, round 5 in parentheses. Both had to beat round 5 on visited % and pile time. −0.35 did not. Four seeds were not run.

- −0.35: seen 91.6% (89.5), visited 19.3% (23.4), pile 152 s at (4.47, 2.69) from t=224 s (71 s). Kitchen visited 6, bedroom 9, bath 0, living 40, hall 0. Corner steps 0. Wall steps 143. Detectors: `camera_stuck` 1, `imu_stuck` 0, `unknown` 12. The log has 11 `blocked heading` lines. Round 7 was visited 14.1, pile 267 s, `camera_stuck` 27.
- North: seen 78.2% (69.6), visited 7.4% (3.4), pile 372 s at (3.18, 5.28) from t=228 s (383 s). Kitchen 0, bedroom 22, bath 0, living 7, hall 0. Corner steps 0. Wall steps 485. Detectors: `camera_stuck` 15, `imu_stuck` 19, `unknown` 109. `blocked heading` 117 lines. Phase `spin` lines 25, against the long spin in round 7. This seed beat the round 5 bar. The pile is still most of the run, inside the bedroom edge, with the block refusing the drive back in.

Dead ends: do not ship the 80% proto. Do not read the kitchen 26% as a stuck-detector bug on open floor. Do not put the stuck check back into go only. Do not treat the heading block as having left the 0.45 m ball. North's block fired through a 372 s stay. The warehouse caster lap still reached the 201 m cap and printed `MISSION complete`.
