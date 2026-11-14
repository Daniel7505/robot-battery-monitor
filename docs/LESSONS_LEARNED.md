# Lessons learned

Read this before changing apartment mapping. Append what the latest sim actually showed. Do not write a cause the logs did not support. The choice that followed a lesson goes in [`docs/DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md). Update that file at the end of every round.

## What this robot is

The apartment machine is `ButlerBotSkid`: four driven wheels, no casters. Hubs at y = ±0.17 m (track 0.34 m), wheel radius 0.08 m, nose and tail 0.19 m, side face 0.227 m. The side faces are 0.454 m apart (17.9 in). The tires stick out to about 0.195 m from the centreline, 0.390 m across (15.4 in). Planning inflation stays 0.20 m at cruise. A creep through a throat the robot itself sees uses the side face plus 2 cm (0.247 m) instead of the 0.30 m pivot aim. The depth camera is an OAK-D Lite at (0.14, 0, 0.78), pitched 5°, minimum range 0.35 m. A reading of nose = 0.12 m is the blind placeholder, not a measured gap.

A centred path through a common US door, spare on each side of the 0.454 m side-face width: a 24 in door (0.610 m) leaves 0.078 m, a 30 in door (0.762 m) leaves 0.154 m, and a 32 in ADA clear opening (0.813 m) leaves 0.180 m. The bath opening in this apartment is 0.81 m, about that 32 in clear width. The 0.30 m pivot aim wants the centre 0.30 m from a mapped wall, so a centred path needs about 0.60 m of gap before a smeared cell. A 24 in door does not keep that aim. The creep aim of 0.247 m still fits a centred 24 in door.

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
- PR #40 (pytest-xdist on `main`, idle since 27 Oct 2026) is not safe to merge. Re-read on 2026-11-08: still open, still `MERGEABLE`, CI still the 27 Oct failure. `gw1` created `allocation_snapshots` while `gw0` already had it. The `xdist_group("db")` pin did not cover every database test. +93/−12 across the workflow, pytest.ini, and a set of test files. Do not merge it, and do not close it from a mapping round.

## Round log

- Round 3, 2026-11-02. Any-stall escape, unvisited goals, skid instance in `apartment.wbt`. Piles 95 / 235 / 439 / 250 s. Kitchen visited 0% on all four. Worst pile −0.35 at (3.99, 5.04), 439 s.
- Round 4, 2026-11-02. Straight leave at 60 s, clipped spins refused. Two seeds only. Piles 181 s and 348 s. North commanded zero speed and moved 0.00 m. Corner steps on north 0. Kitchen 2% and 0%.
- Round 5, 2026-11-02. Trail reverse instead of zero speed. Doorway line-up is not a stall. Unvisited goal about 2 m into the other region. Two seeds only, bar missed. −0.35 pile 71 s, kitchen visited 5%, bedroom 26%, bath 0%, corner steps 0, stuck 3. North pile 383 s, kitchen 0%, bedroom 0%, bath 0%, corner steps 0, stuck 198 from a slow spin at one frozen pose. Four seeds not run.
- Round 6, 2026-11-02. A path that ends with the nose in one mapped wall and a side against another is cut back to the last pose where a pivot fits. A spin with no yaw change and nose 0.12 m reverses along the trail. Two seeds only, bar missed. The `(-0.64,0.64)` command is gone (0 lines). North's pile is now `cmd=(1.06,-1.06)` with nose 99, 193 phase lines, pose frozen, pile 446 s at (3.90, 4.97) from t=154 s, stuck 193, kitchen/bath visited 0. −0.35 pile 361 s, kitchen 0, bath 0, bedroom visited 15%, corner steps 0. Unconfirmed: that nose 99 means the depth frame is empty rather than inside minimum range. Confirmed: the body is not yawing under that spin, and the new detector only fires when the nose is the 0.12 m placeholder, so this spin is invisible to it. Four seeds not run. The path trim and the nose check were deleted in round 7.
- Round 7, 2026-11-02. See the entry below. Two seeds only. Bar missed. Four seeds not run. Warehouse lap passed.
- Round 8, 2026-11-02. See the entry below. An 80% body was a diagnostic only and was deleted. Two full-size seeds. North beat the round 5 bar. −0.35 did not. Four seeds not run. Warehouse lap passed.
- Round 9, 2026-11-03. Creep through a seen throat, and leave a repeated pile for another goal. Two seeds. North beat the round 5 bar. −0.35 did not (visited 10.5%, pile 417 s, kitchen 0). Four seeds not run. Warehouse lap passed.
- Variance check, 2026-11-04. No controller change. Five repeats of −0.35 and five of north on the round 9 code. Visited % spanned 12.5 points on −0.35 (mean 14.7, sample sd 5.4) and 10.2 points on north (mean 9.0, sample sd 3.9). See "Hitting the wall: idea debt."
- One recovery rule, 2026-11-04. Flag `RBM_RECOVERY_MODE`, commit `d264bfa`. Five repeats of each seed in `one_rule`, then five in `trail_only`. The one rule visits less floor on −0.35 than the stack and sits longer. The cost mark does not change that. See "One recovery rule."
- Two footprints, 2026-11-04. Flag `RBM_FOOTPRINT_ALIGN`, off unless set. The planner disk and the veto rectangle are written up under "Planner disk and veto rectangle" in `docs/DESIGN_DECISIONS.md`. The scored runs are in "One footprint for the planner and the veto."
- Quit dump and planner mask, 2026-11-04. Two reviews agreed on the same quit list. The dump is always on. Flag `RBM_SHARED_FOOTPRINT` stays off. See "Quit reasons and one planner mask."
- True start and pile dump, 2026-11-04. Flag `RBM_TRUE_START`, off unless set. The pile line is always on. See "Where a long pile spends its ticks."
- Trusted travel and a best-center pile, 2026-11-06. The dump now follows the estimate's own best center. Stall progress uses wheel travel the step trusted. Five runs of each seed, flags off. Visited means 10.2 and 7.6, inside the noise. The long sits are a `stuck` reverse the pose does not follow. See "A stuck reverse the pose does not follow."
- Stuck reverse trace, 2026-11-06. The north doorway sit was re-read from the wheel, physics, and pose logs. The motors turn. The body does not, because a wheel leaves the floor. See "The wheels turn and the shell does not."
- Floor slots, 2026-11-06. Dan approved filling the apartment floor. The 0.102 m slot is the 4 in partition. The doorway plates were removed in `0cd3694`. See "The partition slot was not a real floor gap."
- Stuck reverse yield, 2026-11-07. A reverse the hinges have already turned, while the forward picture stays frozen, yields after the existing 1 m backup cap. See "A stuck reverse that the picture ignores."
- Collision check, 2026-11-08. No controller change. The tire collider meets the floor in ordinary driving. A corner sit can lift the rear axle or scrub both rear tires on a wall. See "The collider is on the floor until a corner lifts it."
- Island prop and pile-leave log, 2026-11-08. The 14 mm rear lift is the rear sonar box against the island. Pile-leave now logs its branch. See "What held the rear axle up."
- Escape-block rewrite, 2026-11-08. Three probes, no scoring batch. The sustained reverse is a forward pile-leave branch rewritten to `v=-0.10`. See "The forward branch is rewritten into the failed reverse."
- Blocked reverse that does not move, 2026-11-09. The rewrite still starts. A reverse whose planner pose stays put is not sent again. See "A blocked reverse that does not move the shell."
- Island climb counted as travel, 2026-11-09. North run 4's reverse onto the island moved the planner pose with the wheels. No driving change. See "The island climb was wheel travel."
- Pitch gate on every wheel step, 2026-11-09. Tried and reverted. The IMU pitch crossed 0.12 rad while the body was level. See "The pitch gate fired on level driving."

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

## Creep the throat, leave a repeated pile

Symptom: the 80% body visited 26% of the kitchen on −0.35. The full-size round 8 path nicked that kitchen (6%) and north's blocked-heading line fired 117 times without leaving the bedroom edge.

How it was diagnosed: the size run is in the entry above. The body width against 24 / 30 / 32 in doors is in the opening section. Round 8's wall steps were 143 on −0.35 and 485 on north, with 0 corner steps. Those are the contact numbers this round has to stay near.

Fix chosen: when the robot's own map puts the centre in a throat (clearance under the 0.30 m pivot aim and at least the side face plus 2 cm) and a live shoulder sees a surface, a forward command is capped at the 0.12 m/s creep. The path aim for that plan drops from 0.30 m to 0.247 m. A cruise arc still uses 0.30 m. The footprint sweep is unchanged, so a creep that would hit a mapped cell is not sent. The reason code is `narrow_gap`. A second escape within 0.45 m of the last one, or a fourth blocked-heading refusal, drops that goal, marks it low priority, and reverses along the driven trail until the shell is 0.70 m away. The reason code is `pile_leave`. The short heading block remains for a single refusal.

Why not the alternatives: shrinking the shipped body was the diagnostic, not the robot. Dropping the 0.20 m inflation below the tire would plan through cells the wheels hit. Using the tight aim at cruise is how a fast arc clips a jamb.

Result, two full-size seeds, round 5 in parentheses. Both had to beat round 5 on visited % and pile time. −0.35 did not. Four seeds were not run. The 26% kitchen figure was not repeated on the full-size −0.35 seed.

- −0.35: seen 91.9% (89.5), visited 10.5% (23.4), pile 417 s at (1.36, 4.81) from t=183 s (71 s). Kitchen visited 0, bedroom 23, bath 4, living 13, hall 0. Corner steps 0. Wall steps 477. Bumps (`contact_bump`) 0. `narrow_gap` 2, `pile_leave` 3, `camera_stuck` 11, `imu_stuck` 4, `unknown` 180. Blocked-heading lines 8. The three `narrow gap` lines are at t=50, 51, and 177 s. All 477 wall steps are in the 200–300 s window, during the bedroom pile, after those creeps.
- North: seen 86.6% (69.6), visited 14.4% (3.4), pile 146 s at (2.92, 1.06) from t=453 s (383 s). Kitchen visited 16, bedroom 10, bath 0, living 17, hall 51. Corner steps 0. Wall steps 528. Bumps 10. `narrow_gap` 16, `pile_leave` 4, `camera_stuck` 8, `imu_stuck` 2, `unknown` 37. Blocked-heading lines 46, down from 117. This seed beat the round 5 bar. Wall steps are 528 against round 8's 485, all in the last 100 s. Round 8 kitchen visited on this seed was 0.

Dead ends: do not read the −0.35 wall-step rise as the creep. The creep lines are earlier than every wall step on that seed. Do not treat north's 16% kitchen as the 80% diagnostic's 26%, and do not claim the creep caused it without a trace that those `narrow_gap` lines are the kitchen entry. The full-size −0.35 kitchen is still 0. Do not lower the cruise aim to 0.247 m. The warehouse caster lap still reached the 201 m cap and printed `MISSION complete`.

## Hitting the wall: idea debt

Idea debt here means a stack of recovery rules, about 15 of them, each added to fix the last failure. Fixing one side of a pile has been breaking the other side. Rounds 5–9 each ran a seed once, then patched Explorer again. Visited floor moved by about 10–14 percentage points from round to round. Visited floor is the share of open floor the robot's body actually drove over. A pile is the longest time the true body stayed inside a circle of radius 0.45 m. The bar on a pile is 60 s. A bump is a `contact_bump` line in `summary.txt`. Stuck events are `camera_stuck`, `imu_stuck`, and `camera_imu_stuck`. `unknown` is the older relief and is counted on its own.

The reusable pattern is identify, adapt, overcome. Identify the stall: the rules now fight each other, and a one-run score cannot separate an edit from noise. Adapt: take a fresh outside review, and measure the run-to-run spread before the next patch. Overcome: strip the recovery rules back to a smaller set and remeasure. The third-party review said stop patching Explorer and measure first. That measurement is this entry. The strip is recorded in "One recovery rule."

How it was measured: commit `00dae73`, the round 9 controller, unchanged. Five 600 s runs of −0.35 (harness name `m35`, yaw 1.2208) and five of north, through `eval/run_skid_seeds.py`. Settings matched the recent rounds: `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`, Webots R2025a fast and headless. The depth-noise seed stayed fixed per spawn (63 on −0.35, 64 on north), so the spread is the simulator's own run-to-run difference. Every scored run reached 600 s (75,000 rows in `gt_pose_log.csv`). Sample standard deviation divides by 4. The run figures below are rounded; the mean and the sample sd use the unrounded scores. Logs stayed out of git, one folder per seed and repeat, with `summary.txt`, `decisions.txt`, and `webots.log`.

One launch pair was killed near sim time 500 s by a wall-clock cutoff during a clock jump, then run again. Those partial folders are not in the five.

−0.35, runs 1–5. Visited % 18.0, 9.6, 14.1, 9.9, 22.1 (mean 14.7, sample sd 5.4, range 12.5). Pile 68.6 s at (3.03, 4.99) from t=481, 349.8 s at (2.36, 1.82) from t=250, 52.0 s at (4.67, 3.22) from t=401, 459.5 s at (2.82, 1.24) from t=140, 135.6 s at (2.40, 6.44) from t=97 (mean 213 s, sample sd 182 s). Time in separate stays of at least 60 s: 136, 444, 0, 460, 276 s. The 0 belongs to the 52.0 s pile, which is under the bar, so it adds no 60 s stay. Bumps 30, 4, 5, 9, 62. `camera_stuck` 4, 6, 2, 15, 0. `imu_stuck` 5, 0, 0, 7, 0. `camera_imu_stuck` 0 on every run. `unknown` 7, 154, 13, 142, 18. `pile_leave` 4, 3, 3, 4, 2. `narrow_gap` 25, 20, 21, 5, 22. Blocked-heading lines 10, 154, 3, 37, 2. `stall, backing out` 13, 7, 9, 27, 9. Wall steps 0, 0, 336, 965, 0. Corner steps 0. Kitchen visited 0, 16, 1, 5, 0. Bedroom 49, 0, 22, 0, 36.

North, runs 1–5. Visited % 3.7, 6.5, 10.7, 10.2, 13.9 (mean 9.0, sample sd 3.9, range 10.2). Pile 541.0 s at (4.01, 5.73) from t=59, 446.5 s at (1.40, 4.80) from t=153, 332.9 s at (4.13, 0.64) from t=267, 291.0 s at (3.02, 5.80) from t=309, 334.0 s at (4.30, 5.48) from t=266 (mean 389 s, sample sd 103 s). Time in separate stays of at least 60 s: 544, 446, 333, 356, 334 s. Bumps 3, 2, 0, 2, 2. `camera_stuck` 21, 15, 8, 12, 3. `imu_stuck` 0, 5, 1, 0, 6. `camera_imu_stuck` 0. `unknown` 226, 194, 158, 126, 138. `pile_leave` 1, 3, 2, 2, 4. `narrow_gap` 2, 16, 10, 12, 8. Blocked-heading lines 233, 11, 162, 135, 145. `stall, backing out` 21, 11, 12, 12, 11. Wall steps 55673, 16, 40014, 31369, 105. Corner steps 0. Kitchen visited 0, 0, 32, 0, 0. Bedroom 0, 18, 0, 17, 0.

Published overall visited %, rounds 5–9, one run each. −0.35: 23.4, 17.0, 14.1, 19.3, 10.5 (span 12.9 points). North: 3.4, 5.6, 7.4, 14.4 (span 11.0 points; round 6 published no overall visited %). Those spans are the same size as the same-code ranges above. Round 9's −0.35 numbers (visited 10.5, pile 417 s) sit inside this −0.35 sample. Round 9's north pile of 146 s is shorter than all five north piles here, and its visited 14.4 sits just above this sample's high of 13.9.

Wall steps are 8 ms physics steps with a tire on a wall, so a long lean inflates the count. Corner steps stayed 0 on all ten runs. A kitchen percentage from one run is a draw: north run 3 visited 32% of the kitchen, and the other four north runs visited 0.

Dead ends: do not add another recovery rule from one seed. Do not read a 10 point visited change between rounds as an effect of the last patch. Do not treat round 9's north pile of 146 s as what this code usually does.

Outcome: the strip was measured. See "One recovery rule." One rule visits less of the −0.35 floor than this stack and sits longer. The cost mark does not close that gap.

Two later reviews, read independently, agreed on one list: the start cell from `_nearest_free` can sit in a free component that does not reach a frontier, a frontier ban does not lift, the top-8 cut happens before that filter, and A* can hit its 25,000 expansion cap. They also agreed the map veto refuses about a third of planner-accepted paths. That agreement is a hypothesis list. The measurement, before any of those paths is patched, is "Quit reasons and one planner mask." The follow-up, starting A* in the robot's own component and writing down what a pile actually commands, is "Where a long pile spends its ticks." The next measurement, after that dump missed two long sits, is "A stuck reverse the pose does not follow." The trace of that reverse is "The wheels turn and the shell does not." The floor repair, after Dan approved it, is "The partition slot was not a real floor gap." Do not add another recovery rule on top of this sample. The gapped-floor runs are not a clean baseline for a later controller change.

## True pose for the planner, same map

Symptom: the same controller visits about 15% of the open floor on −0.35 and about 9% on north, and the true body sits inside 0.45 m for minutes. That could be a wrong pose, or it could be the choices the explorer makes once it has a pose.

How it was diagnosed: an opt-in flag, `RBM_ORACLE_POSE`, off unless set. The flag is commit `234734e`. On, the pose logger writes the robot's own simulator pose to `oracle_pose.txt`. The mapper converts that into the odometry frame, first sample as the origin, and the planner uses it for goals, path following, the survey spin, and the stuck and pile clocks. The occupancy grid, the footprint stamp, the visited-cell mask, fusion, the wall nudge, loop closure, the pin, and the zero-velocity update stay on the wheel and gyro estimate. No floor plan is read. Five 600 s runs of −0.35 and five of north, depth-noise seeds 63 and 64, the same harness settings as the variance entry (`RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`). Webots was R2025a in fast mode with `--no-rendering`, so the main view was not drawn. The sensors still updated. Every run latched the oracle and wrote 75,000 rows. `mapping_done` is 600 s. Sample standard deviation divides by 4. The run figures below are rounded; the mean and the sample sd use the unrounded scores. A gap under about 6 visited points is the noise from the entry above.

−0.35, runs 1–5. Visited % 5.5, 11.5, 16.4, 11.5, 9.7 (mean 10.9, sample sd 3.9, range 10.9). The variance mean was 14.7. The gap is 3.8 points. Pile 483.4 s at (4.06, 3.86) from t=117, 211.1 s at (2.73, 1.23) from t=389, 97.3 s at (7.94, 3.32) from t=503, 371.5 s at (4.08, 3.40) from t=228, 366.4 s at (7.76, 3.81) from t=234 (mean 306 s, range 97 s to 483 s). The variance pile mean was 213 s. Bumps 0, 22, 7, 5, 9. Stuck events (`camera_stuck` + `imu_stuck` + `camera_imu_stuck`) 3, 16, 6, 9, 14. `camera_stuck` 3, 9, 6, 8, 11. `imu_stuck` 0, 7, 0, 1, 2. `camera_imu_stuck` 0, 0, 0, 0, 1. `unknown` 210, 80, 21, 154, 149. `pile_leave` 1, 2, 6, 3, 3. `narrow_gap` 13, 12, 15, 8, 0. `heading_off` 60, 92, 79, 38, 41. `too_tight_to_fit` 212, 15, 34, 164, 19. `cant_see_behind_backing_out` 222, 137, 89, 181, 205. `contact_bump` 0, 22, 7, 5, 9. `goal_changed` 7, 33, 32, 16, 13. `dead_end` 0, 0, 3, 1, 4. `no_room_to_turn` 2, 3, 2, 2, 4. `frontier_ban` 0, 0, 2, 0, 0. `frontier_clear` 0, 0, 2, 0, 0.

North, runs 1–5. Visited % 11.2, 11.2, 3.3, 6.6, 8.0 (mean 8.1, sample sd 3.3, range 7.9). The variance mean was 9.0. The gap is 0.9 points. Pile 292.1 s at (3.84, 2.69) from t=308, 279.1 s at (4.16, 2.45) from t=321, 470.4 s at (3.97, 5.89) from t=130, 507.5 s at (4.77, 1.16) from t=92, 270.4 s at (0.68, 4.86) from t=330 (mean 364 s, range 270 s to 507 s). The variance pile mean was 389 s. Bumps 14, 8, 0, 5, 9. Stuck events 6, 5, 20, 10, 5. `camera_stuck` 6, 5, 13, 8, 5. `imu_stuck` 0, 0, 7, 2, 0. `camera_imu_stuck` 0. `unknown` 144, 100, 204, 224, 109. `pile_leave` 2, 2, 1, 1, 2. `narrow_gap` 12, 13, 2, 8, 26. `heading_off` 39, 103, 26, 34, 102. `too_tight_to_fit` 12, 109, 204, 9, 77. `cant_see_behind_backing_out` 187, 147, 234, 250, 157. `contact_bump` 14, 8, 0, 5, 9. `goal_changed` 14, 23, 13, 10, 25. `dead_end` 3, 0, 0, 0, 0. `no_room_to_turn` 3, 0, 1, 0, 0. `frontier_ban` 0, 0, 0, 0, 1.

The estimate was still wrong, so the flag was not a no-op. End position error of the estimate against the true body: −0.35 0.4 m, 1.2 m, 0.3 m, 3.1 m, 1.6 m. North 30.1 m, 0.4 m, 0.3 m, 1.4 m, 1.4 m. North run 1 was also reversed in heading, and it visited 11.2%.

Dead ends: do not build the grid from the true pose and call that this test. Do not turn the flag on in the harness. Do not read the 3.8 point visited gap on −0.35 as a loss. Do not add a localization patch to chase these piles.

Outcome: the decision logic is the flaw. A true pose did not raise visited floor above the noise and did not bring a pile under 60 s. The flag stays off.

## One recovery rule

Symptom: the same controller sits inside 0.45 m for minutes, and a true pose did not stop it. The remaining suspect was the stack of recovery rules, about 15 of them, each added after the last failure.

How it was diagnosed: an opt-in flag, `RBM_RECOVERY_MODE`, empty unless set. Empty is the stack. The scored code is commit `d264bfa`. `one_rule` has one trigger and one response. The trigger is the body: it has stayed inside 0.45 m for 60 s, or a translation command or a spin command has not moved it for 2 s. A survey spin that changes yaw does not count. The response is a reverse along the trail the robot drove, which is known-clear space. `one_rule` also raises the planner cost of that 0.45 m disk for the whole 600 s run, and the next goal has to sit outside the disk. The occupancy grid is not painted. `trail_only` reverses the same way and does not mark the disk. Camera and IMU escapes, the pile-forced branch, the blocked heading, the unknown relief, the contact recover, the creep, the stall escape, the wide escape, and the dead-end back remain in the file and do not run. No floor plan is read. Oracle pose stays off.

Five 600 s runs of −0.35 (harness name `m35`) and five of north, for `one_rule`, then the same ten for `trail_only`. Depth-noise seeds 63 and 64, ports 1711 and 1712. Settings: `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Webots R2025a, fast, `--no-rendering`, headless. The main view was not drawn. The sensors still updated. Every run wrote 75,000 rows and `mapping_done` is 600 s. Sample standard deviation divides by 4. The visited figures below are the scorer's percent to two decimals. The mean and the sample sd are computed from those figures. A gap under about 6 visited points is the noise from the variance entry. The pile is the longest time the true body stayed inside 0.45 m of a pose it occupied, checked every 0.25 s. Logs stayed out of git.

An earlier clock treated the survey sit as a stall and quarantined the spawn. Those two partial runs were deleted. They are not in the tables.

`one_rule`, −0.35, runs 1–5. Visited % 3.45, 7.78, 5.65, 3.46, 7.77 (mean 5.6, sample sd 2.2, range 4.3). The variance mean was 14.7. The gap is 9.1 points. Pile 567.3 s at (4.00, 4.84) from t=33, 519.1 s at (1.57, 5.87) from t=81, 528.1 s at (4.93, 3.16) from t=72, 552.4 s at (4.44, 5.06) from t=48, 490.4 s at (1.10, 6.16) from t=110 (mean 531 s, range 490 s to 567 s). The variance pile mean was 213 s. Its longest pile was 459.5 s. Bumps 0. Stuck events 0. Blocked-heading lines 0. `one_rule` 1, 3, 2, 2, 2. `camera_stuck` 0. `imu_stuck` 0. `camera_imu_stuck` 0. `unknown` 0. `narrow_gap` 0. `pile_leave` 0. `contact_bump` 0. `heading_off` 30, 36, 19, 4, 46. `goal_changed` 2, 5, 4, 2, 6. `no_room_to_turn` 2, 2, 2, 2, 2. `too_tight_to_fit` 0, 0, 1, 1, 0. `cant_see_behind_backing_out` 0.

`one_rule`, north, runs 1–5. Visited % 2.94, 3.58, 4.08, 4.83, 5.08 (mean 4.1, sample sd 0.9, range 2.1). The variance mean was 9.0. The gap is 4.9 points. Pile 505.0 s at (5.15, 5.78) from t=95, 549.6 s at (3.47, 5.16) from t=50, 454.2 s at (4.48, 5.90) from t=146, 547.9 s at (2.43, 4.98) from t=52, 474.0 s at (5.26, 6.28) from t=126 (mean 506 s, range 454 s to 550 s). The variance pile mean was 389 s. Bumps 0. Stuck events 0. Blocked-heading lines 0. `one_rule` 3, 1, 3, 2, 2. `camera_stuck` 0. `imu_stuck` 0. `camera_imu_stuck` 0. `unknown` 0. `narrow_gap` 0. `pile_leave` 0. `contact_bump` 0. `heading_off` 4, 7, 60, 21, 35. `goal_changed` 4, 3, 5, 4, 6. `no_room_to_turn` 1, 2, 1, 1, 2. `too_tight_to_fit` 10, 0, 0, 0, 0. `cant_see_behind_backing_out` 0, 1, 0, 0, 0.

`trail_only`, −0.35, runs 1–5. Visited % 6.91, 4.60, 3.89, 3.07, 8.12 (mean 5.3, sample sd 2.1, range 5.1). The variance mean was 14.7. The gap is 9.4 points. The gap to `one_rule` is 0.3 points. Pile 510.5 s at (0.87, 6.19) from t=90, 464.1 s at (4.56, 5.56) from t=136, 526.6 s at (4.13, 5.05) from t=73, 569.2 s at (4.24, 4.83) from t=31, 426.1 s at (4.30, 2.27) from t=174 (mean 499 s, range 426 s to 569 s). Bumps 0. Stuck events 0. Blocked-heading lines 0. `trail_only` 2, 3, 2, 2, 3. `camera_stuck` 0. `imu_stuck` 0. `camera_imu_stuck` 0. `unknown` 0. `narrow_gap` 0. `pile_leave` 0. `contact_bump` 0. `heading_off` 23, 20, 21, 6, 40. `goal_changed` 4, 4, 2, 1, 6. `no_room_to_turn` 2, 2, 2, 2, 2. `too_tight_to_fit` 0. `cant_see_behind_backing_out` 0.

`trail_only`, north, runs 1–5. Visited % 5.29, 5.91, 8.19, 5.03, 3.43 (mean 5.6, sample sd 1.7, range 4.8). The variance mean was 9.0. The gap is 3.4 points. The gap to `one_rule` is 1.5 points. Pile 520.1 s at (3.19, 5.17) from t=80, 336.9 s at (3.47, 5.05) from t=263, 476.2 s at (4.48, 3.26) from t=124, 509.0 s at (3.39, 5.15) from t=91, 485.9 s at (4.42, 5.57) from t=114 (mean 466 s, range 337 s to 520 s). Bumps 0. Stuck events 0. Blocked-heading lines 0. `trail_only` 3, 7, 1, 1, 4. `camera_stuck` 0. `imu_stuck` 0. `camera_imu_stuck` 0. `unknown` 0. `narrow_gap` 0. `pile_leave` 0. `contact_bump` 0. `heading_off` 7, 40, 19, 15, 14. `goal_changed` 4, 4, 7, 4, 3. `no_room_to_turn` 1, 2, 2, 1, 1. `too_tight_to_fit` 1, 0, 0, 0, 0. `cant_see_behind_backing_out` 1, 0, 0, 0, 0.

`heading_off` is still counted because an ordinary pivot logs it. The blocked-heading recovery is the separate log line, and that line is 0.

Dead ends: do not delete the stack from this result. Do not turn the flag on in the harness. Do not read the 4.9 point north gap, or the 3.4 point `trail_only` north gap, as a loss. Do not read the 0.3 and 1.5 point gaps between `one_rule` and `trail_only` as an effect of the cost mark. Do not add another recovery rule on top of the stack from one of these sits.

Outcome: one rule does not match the stack and does not beat it. On −0.35 the robot covers less floor and sits longer. Marking the spot it failed in does not send it somewhere else. The reverse fires a few times, and the body stays inside 0.45 m for most of the run. The flag stays off.

## One footprint for the planner and the veto

Symptom: the same controller sits inside 0.45 m for minutes, and neither a true pose nor one recovery rule fixed it. A path the planner accepts can still be a turn the veto refuses, because the two checks use different shapes.

How the shapes differ, with the flag off. The planner inflates occupied cells by a disk of radius 0.20 m. That is the tire (0.195 m from the centreline) plus a centimetre, and it is round. The turn veto sweeps a rectangle: nose and tail 0.19 m, side face 0.227 m, plus 0.02 m on every side. The front corner of that rectangle is about 0.32 m from the centre, so it sticks out past the disk. A straight command does not use the rectangle. It looks for a wall that spans the centre and both points 0.12 m to the side. Live range is a separate sonar check.

How it was diagnosed: an opt-in flag, `RBM_FOOTPRINT_ALIGN`, off unless set. The flag code is commit `9ed7ff2`. On, the planner and the map veto both use that rectangle and that 0.02 m clearance. A planner step is legal only when the rectangle at the step's heading misses mapped occupied cells. A turn of more than about 57° is legal only when the rectangle stays clear through the yaw change. Straight commands use the same rectangle sweep. Goal scores are unchanged. The 2.5 and 6.0 clearance weights stay. The recovery stack stays on. Oracle stays off. No floor plan is read.

Five runs of −0.35 (harness name `m35`) and five of north. Depth-noise seeds 63 and 64, ports 1711 and 1712. Settings: `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`, `RBM_FOOTPRINT_ALIGN=1`. Webots R2025a, fast, `--no-rendering`, headless. The main view was not drawn. The sensors still updated. The cap is 600 s. The mapper still stops when it finds no reachable frontier, and the pose logger quits, so several runs ended sooner. `mapping_done` is the run length. Sample standard deviation divides by 4. The visited figures are the scorer's percent to two decimals. The mean and the sample sd are computed from those figures. A gap under about 6 visited points is the noise from the variance entry. The pile is the longest time the true body stayed inside 0.45 m of a pose it occupied, checked every 0.25 s. A plan/veto disagreement is one issued path whose follow command the map footprint then rejected. Live range is not in that count. The variance folders are not on this machine, so there is no flag-off disagreement count. Logs stayed out of git.

−0.35, runs 1–5. Visited % 24.99, 6.01, 9.88, 11.00, 17.57 (mean 13.9, sample sd 7.5, range 19.0). The variance mean was 14.7. The gap is 0.8 points. Pile 52.7 s at (2.05, 5.00) from t=473, 519.9 s at (4.05, 4.97) from t=80, 34.0 s at (6.10, 4.70) from t=0, 114.2 s at (1.22, 6.28) from t=98, 142.1 s at (4.04, 2.22) from t=346 (mean 173 s, range 34 s to 520 s). The variance pile mean was 213 s. Its longest pile was 459.5 s. Run length 576 s, 600 s, 127 s, 336 s, 488 s. Bumps 2, 0, 0, 0, 5. Stuck events 1, 4, 0, 2, 15. `camera_stuck` 0, 4, 0, 1, 15. `imu_stuck` 1, 0, 0, 1, 0. `camera_imu_stuck` 0. `unknown` 6, 211, 0, 3, 7. `narrow_gap` 23, 2, 4, 7, 10. `pile_leave` 1, 2, 0, 3, 3. `heading_off` 103, 53, 10, 84, 134. `too_tight_to_fit` 4, 211, 0, 10, 26. `cant_see_behind_backing_out` 37, 236, 12, 30, 48. `contact_bump` 2, 0, 0, 0, 5. `goal_changed` 36, 8, 9, 10, 19. `dead_end` 0. `no_room_to_turn` 0. Blocked-heading lines 1, 5, 0, 3, 19. Plans issued, then map-veto disagreements: 61 and 24, 9 and 3, 11 and 3, 13 and 5, 31 and 7.

North, runs 1–5. Visited % 5.73, 10.90, 6.09, 22.84, 21.52 (mean 13.4, sample sd 8.3, range 17.1). The variance mean was 9.0. The gap is 4.4 points. Pile 65.0 s at (3.07, 5.88) from t=69, 355.6 s at (4.38, 1.46) from t=244, 43.5 s at (4.15, 5.30) from t=118, 56.1 s at (5.80, 5.60) from t=0, 80.6 s at (2.99, 0.82) from t=519 (mean 120 s, range 44 s to 356 s). The variance pile mean was 389 s. Run length 134 s, 600 s, 161 s, 484 s, 600 s. The two runs that lasted 600 s had piles of 356 s and 81 s. Bumps 0, 4, 0, 0, 2. Stuck events 0, 8, 0, 1, 1. `camera_stuck` 0, 6, 0, 1, 0. `imu_stuck` 0, 2, 0, 0, 1. `camera_imu_stuck` 0. `unknown` 1, 160, 1, 4, 11. `narrow_gap` 6, 3, 3, 22, 19. `pile_leave` 0, 4, 0, 0, 2. `heading_off` 16, 70, 6, 80, 154. `too_tight_to_fit` 0, 159, 0, 0, 9. `cant_see_behind_backing_out` 11, 195, 11, 32, 41. `contact_bump` 0, 4, 0, 0, 2. `goal_changed` 9, 18, 12, 34, 42. `dead_end` 0. `no_room_to_turn` 1, 1, 1, 2, 2. Blocked-heading lines 0, 156, 0, 0, 5. Plans issued, then map-veto disagreements: 12 and 6, 26 and 4, 13 and 1, 43 and 13, 55 and 18.

Dead ends: do not turn the flag on in the harness. Do not read the 4.4 point north gap as a gain. Do not read the north pile mean of 120 s as the sit ending. Three north runs and four −0.35 runs stopped early, and a short run cannot grow a long pile. The 520 s pile on −0.35 is longer than the variance sample's longest pile. Do not add another recovery rule from these sits.

Outcome: one shared footprint does not end the stall. The −0.35 visited mean matches the variance mean. The north visited gap is inside the noise. Issued paths are still rejected by the map footprint. The flag stays off.

## Quit reasons and one planner mask

Symptom: two reviews, read independently, agreed the mapper can stop early because the start cell from `_nearest_free` sits in a free component that does not reach a frontier, because a frontier ban does not lift, because the top-8 cluster cut happens before that filter, or because A* hits its 25,000 expansion cap. They also agreed the map veto refuses about a third of planner-accepted paths. The rectangle flag above did not end that refusal.

How it was diagnosed: a quit line, always on, logging only. When `select_goal` returns None, and again at the end of `_no_frontier` and `_settle`, one line records the start cell, the jump from the robot cell, whether the longest frontier shares the start's free component, raw frontier cells, clusters of at least 6 cells before and after the top-8 plus door cut, clusters dropped by bans, banned cells and banned regions, clusters with a viewpoint, A* attempts, failures, and cap hits, and which return fired. The line's `reason` is one of `start_component`, `all_banned`, `astar_cap`, `cut`, or `other`. `frontier_ban` and `frontier_clear` event counts are written to `summary.txt` and `plan_veto.txt`. The code is commit `3472086`.

The veto trial is `RBM_SHARED_FOOTPRINT`, off unless set. On, the planner free mask (the 0.20 m disk) is the map test. A swept pose is rejected when its centre has left that mask. Unknown cells stay open. Live range still applies. The rectangle is not applied a second time. No new clearance number. Recovery, oracle, and `RBM_FOOTPRINT_ALIGN` stay off.

Five runs of −0.35 (harness name `m35`) and five of north with the flag on, then two of each seed with the flag off, for the reason dump on the baseline. Depth-noise seeds 63 and 64, ports 1711 and 1712. Settings match the rectangle runs: `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`, Webots R2025a, fast, `--no-rendering`, headless. Every run wrote 75,000 pose rows. `mapping_done` is 600 s on all fourteen. Sample standard deviation divides by 4 for the five-run sets. The visited figures are the scorer's percent to two decimals. The mean and the sample sd use those figures. A gap under about 6 visited points is the noise from the variance entry. The pile is the longest time the true body stayed inside 0.45 m of a pose it occupied, checked every 0.25 s. A plan/veto disagreement is one issued path whose follow command the map test then rejected. Live range is not in that count. Logs stayed out of git.

Flag on, −0.35, runs 1–5. Visited % 24.08, 5.94, 25.45, 15.92, 9.72 (mean 16.2, sample sd 8.6, range 19.5). The variance mean was 14.7. The gap is 1.5 points. The rectangle-flag mean was 13.9. Pile 80.4 s at (4.50, 2.49) from t=357, 517.1 s at (3.85, 5.10) from t=83, 96.0 s at (4.50, 5.81) from t=416, 239.4 s at (3.07, 5.17) from t=360, 363.3 s at (4.49, 2.46) from t=237 (mean 259 s, range 80 s to 517 s). The variance pile mean was 213 s. Plans issued, then map-veto disagreements: 52 and 5 (10%), 35 and 20 (57%), 49 and 8 (16%), 47 and 6 (13%), 23 and 9 (39%). Pooled 48 of 206 (23%). `frontier_ban` / `frontier_clear`: 0/0, 0/0, 3/3, 0/0, 1/1. `select_goal` returned None on run 2 only, twice, both `reason=start_component`.

Flag on, north, runs 1–5. Visited % 10.13, 11.10, 11.95, 5.25, 14.14 (mean 10.5, sample sd 3.3, range 8.9). The variance mean was 9.0. The gap is 1.5 points. The rectangle-flag mean was 13.4. Pile 331.0 s at (3.86, 5.27) from t=67, 306.4 s at (3.73, 1.53) from t=293, 364.5 s at (3.94, 1.75) from t=235, 524.3 s at (4.22, 5.51) from t=76, 291.1 s at (6.00, 7.90) from t=309 (mean 363 s, range 291 s to 524 s). The variance pile mean was 389 s. Plans issued, then map-veto disagreements: 46 and 15 (33%), 42 and 7 (17%), 20 and 3 (15%), 30 and 20 (67%), 26 and 2 (8%). Pooled 47 of 164 (29%). `frontier_ban` / `frontier_clear`: 1/1, 0/0, 0/0, 1/1, 1/1. `select_goal` returned None four times on run 1 and three times on run 4, every line `reason=start_component`.

Flag off, two runs, the reason dump on the unchanged veto. −0.35 visited 18.65 and 9.51. Pile 159.0 s and 485.9 s. Veto 11 of 41 (27%) and 3 of 13 (23%). `frontier_ban` / `frontier_clear` 1/0 and 0/0. One `select_goal` miss, `reason=start_component`. North visited 22.69 and 7.42. Pile 68.1 s and 462.1 s. Veto 19 of 52 (37%) and 8 of 30 (27%). `frontier_ban` / `frontier_clear` 1/1 and 1/1. No `select_goal` miss.

Early-quit count: 0 of 14. No run has an `end_run` other than the 600 s cap (`mapping_done` is 600 s, 75,000 rows). The ten flag-on misses and the one flag-off miss are the dump. Every one of those eleven lines is `reason=start_component`, `return=no_candidate`, `start_none=0`, `astar_cap=0`, and `banned_drop=0`. Clusters of at least 6 cells were 26 to 43 before the top-8 plus door cut and 11 or 12 after it, so the cut is on the same line, and the label is still the component split because that check is first. A* failed every attempt on those lines (20 to 24 failures) and did not hit the cap. No line is `all_banned`, `astar_cap`, or `cut`. North run 4's first miss had 26 banned cells and 1 banned region; the next miss in that run had 0 banned cells, and the run's clear count is 1. The searches resumed. The body was still driving at the cap.

Dead ends: do not patch `_nearest_free`, the ban lifetime, the top-8 cut, or the 25,000 cap from these lines. The label was the component split, and the run still reached 600 s. Do not read the 1.5 point visited gaps as an effect of the mask. Do not read the pooled veto rate (23% and 29% with the flag on, 26% and 33% with it off) as the refusal going away. One flag-on run still rejected 20 of 30 paths. Do not turn the flag on in the harness.

Outcome: the mask flag stays off. The quit line stays. The next patch, if one is made, has a measured label to aim at, and this sample did not show that label ending a run.

## Where a long pile spends its ticks

Symptom: the quit dump's label was `start_component`, and those searches still reached the 600 s cap. The open question was what the wheels are actually being told during a long sit, and whether starting A* in the robot's own free component changes the visited floor.

How it was diagnosed: a pile line, always on, logging only. The existing pile ball is 0.45 m. The existing window is 60 s (`PILE_LIMIT_S`). Once a stay has lasted that long, one line a second records the phase, the goal cell and its age and attempt count, the path length and next waypoint, the pure-pursuit target, every `choose_motion` candidate with its veto reason, the chosen `(v, w)` and its source, the live sensor blocks, and the odometry delta against the commanded distance. When the estimate leaves the ball, a summary gives the share of ticks that were hold or zero, the share where every candidate was vetoed, the share where a nonzero command did not show up in the pose, and which recovery rules fired, plus one plain sentence. The trial flag is `RBM_TRUE_START`, off unless set. On, the robot cell is treated as free and a walk that does not step on an occupied cell finds a planner-free cell in that same component. The walk uses the same 40-cell limit as `_nearest_free`. No new clearance, and the recovery stack and the scorer are unchanged. The code is commit `33bf583`.

Five runs of −0.35 (harness name `m35`) and five of north with the flag on, then two of each seed with it off. The pile line is on in all fourteen. Depth-noise seeds 63 and 64, ports 1711 and 1712. Settings match the 3b runs: `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`, Webots R2025a, fast, `--no-rendering`, headless. 3b is the planner-mask trial (means 16.2 and 10.5). Sample standard deviation divides by 4. The visited figures are the scorer's percent to two decimals. A gap under about 6 visited points is the noise from the variance entry. The scored pile is the true body inside 0.45 m, checked every 0.25 s. The dump's pile is the estimate's anchor. Logs stayed out of git.

Flag on, −0.35, runs 1–5. Visited % 19.70, 17.40, 14.16, 10.81, 11.12 (mean 14.6, sample sd 3.9, range 8.9). The variance mean was 14.7. The gap is 0.1 points. The 3b mean was 16.2. Pile 122.4 s at (2.63, 1.93) from t=477, 225.5 s at (0.95, 6.06) from t=374, 184.6 s at (3.99, 2.79) from t=415, 442.9 s at (1.97, 6.29) from t=157, 85.5 s at (4.54, 1.29) from t=175 (mean 212 s). Run length 600 s, 600 s, 600 s, 600 s, 329 s. `start_component` count 0. Run 5's 31 `select_goal` misses are `return=start_none` `reason=other`: the escape walk found no planner-free cell inside 40 cells, and the run ended.

Flag on, north, runs 1–5. Visited % 22.88, 26.51, 13.52, 7.54, 4.69 (mean 15.0, sample sd 9.5, range 21.8). The variance mean was 9.0. The gap is 6.0 points. The 3b mean was 10.5. The gap to 3b is 4.5 points. Pile 96.8 s at (4.68, 3.07) from t=172, 70.1 s at (1.00, 6.19) from t=241, 299.0 s at (4.19, 5.69) from t=301, 92.7 s at (1.17, 5.02) from t=151, 536.1 s at (3.99, 5.33) from t=64 (mean 219 s). Run length 600 s, 600 s, 600 s, 244 s, 600 s. `start_component` count 0. Run 4's 14 misses are the same `start_none` / `other` ending.

Flag off, two runs, the pile dump on the straight-line start. −0.35 visited 20.63 and 9.36, piles 194.6 s and 421.9 s, both 600 s, `start_component` 0. North visited 22.66 and 8.17, piles 229.4 s and 450.0 s, both 600 s, `start_component` 0. No `select_goal` miss on these four.

What the dump shows on the piles it opened. Hold or zero is 0–4% of ticks. A tick where every motion candidate was vetoed is 0% on every summary. The large shares are recovery commands. Commanded-but-still (a nonzero command the pose did not follow) is usually 1–5%, and it rises inside a stall loop.

Three reads:

North run 3, the summary that opened at t=298 s and lasted 60 s. 7,522 ticks. Hold 4%, veto 0%, commanded-but-still 33%. Recovery 91%: `stall` 6,203, `pile_leave` 402, `stuck` 252. The sentence is that this pile was a recovery command. The goal at the end was (2.88, 0.48). The stall escape was issuing the command, and on a third of the ticks the pose did not follow it.

−0.35 run 3, the summary from t=284 s, 65 s, 8,146 ticks. Hold 0%, veto 0%, commanded-but-still 1%. Recovery 78%: `back` 3,990, `narrow_gap` 1,722, `pile_leave` 645. The shell was backing and creeping inside the ball. The odometry followed almost all of those commands. The veto was not what held it.

North run 5, ground-truth pile 536 s from t=64 s, and −0.35 run 4, ground-truth pile 443 s from t=157 s. Neither opened a summary. The estimate's logged poses in those windows span 0.36 m and 0.41 m, and the 60 s anchor kept resetting, so the dump's window never latched. The true body was the one sitting. The estimate was creeping just enough to look like a new stay.

Dead ends: do not read the north mean of 15.0 as a gain. The sample sd is 9.5 and two of the ten flag-on runs stopped early because the escape walk returned no start. Do not turn the flag on in the harness. Do not add a longer walk than the existing 40 cells to chase those two stops. Do not treat a 0% veto share as "the map veto never fires"; it means that once a stay has already lasted 60 s, the tick that is running is a recovery command, not a fresh `choose_motion` ballot.

Outcome: `RBM_TRUE_START` stays off. The pile line stays. On the stays the estimate held for 60 s, the dominant tick is a recovery command (`stall`, `pile_leave`, `back`, `contact`), not a veto hold and not a zero command. `start_component` did not appear. The two early stops are `start_none`.

## A stuck reverse the pose does not follow

Symptom: two ground-truth piles in the true-start round, 536 s and 443 s, never opened a dump. The estimate's poses in those windows spanned 0.36 m and 0.41 m, and the entry anchor reset whenever a chord passed 0.45 m. The same round's opened summaries were recovery commands, often `stall`, with hold at 0–4% and a fully vetoed ballot at 0%.

How it was diagnosed: the code was read before the runs. Three different clocks were in play. The stall escape ended, and its "did not move" phases tripped, on the distance between `plan_pose` and a saved pose. Fusion and loop closure can move that pose without the wheels rolling, and a dropped wheel step (frozen view, slip, spin scrub) does not move it. The pile dump and the pile guard used the entry anchor, while the scorer uses a center the body occupied. `_stuck_escape` is called before the stall machine and clears `_blind_back`, so a camera or IMU stuck tick never reaches the stall travel counter. Ground truth is not an input to any of those.

The logging change uses the robot's own pose only. Every 0.25 s it keeps the longest suffix that still sits inside 0.45 m of one of its samples, and it opens the existing 1 Hz line once that stay reaches 60 s, even if the entry anchor has reset. The summary adds commanded distance (integral of `|v|`) and the estimate's path length. It does not arm the pile guard. The behavior change is only the stall escape: success, and the phase checks that used to read the pose, now sum wheel travel the step actually trusted. The thresholds are the existing ones (0.30 m to leave, 1.0 m back cap, 0.02 m and 0.05 m "did not move"). No new knob. Flags stayed off: `RBM_TRUE_START`, `RBM_SHARED_FOOTPRINT`, `RBM_FOOTPRINT_ALIGN`, `RBM_ORACLE_POSE`, and `RBM_RECOVERY_MODE` were unset. The code is commit `2774b24`.

Five 600 s runs of −0.35 (harness name `m35`) and five of north. Depth-noise seeds 63 and 64, ports 1711 and 1712. `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Webots R2025a, fast, `--no-rendering`, headless. Every run wrote 75,000 pose rows and `mapping_done` is 600 s. The `time cap` print is not in these `webots.log` files; the done file and the row count are the cap. Sample standard deviation divides by 4. Visited figures are the scorer's percent to two decimals. The mean and the sample sd use those figures. A gap under about 6 visited points is the noise from the variance entry. The scored pile is the true body inside 0.45 m of a pose it occupied, samples every 0.25 s. Logs stayed out of git.

−0.35, runs 1–5. Visited % 18.05, 12.38, 4.19, 3.76, 12.64 (mean 10.20, sample sd 6.12, range 14.3). The variance mean was 14.7. The gap is 4.5 points. The 3b mean was 16.2. Pile 103.9 s from t=78, 298.2 s from t=302, 541.4 s from t=58, 553.0 s from t=47, 323.3 s from t=276 (mean 364 s). Early-quit count 0. Run 2 logged `reason=start_component` at 18 distinct times (38 lines; some timestamps are repeated). The search resumed and the run reached 600 s. No `start_none`.

North, runs 1–5. Visited % 3.40, 10.87, 9.16, 9.69, 5.09 (mean 7.64, sample sd 3.22, range 7.5). The variance mean was 9.0. The gap is 1.4 points. The 3b mean was 10.5. Pile 564.5 s from t=35, 431.4 s from t=168, 378.4 s from t=221, 418.3 s from t=182, 527.1 s from t=73 (mean 464 s). Early-quit count 0. No `select_goal` miss.

What the dump shows. The best-center clock opened on these sits. The entry-anchor miss from the previous round did not repeat: a 564 s true sit produced 483 one-hertz lines. A summary is still missing when the shell is inside the ball at the cap, because the pose logger quits in the same step that marks the run done, before the leaving-the-ball summary is written. The one-hertz lines are the record for those.

Three reads:

North run 1. Ground-truth pile 564.5 s from t=35. The dump is 483 s of `src=stuck`, commanded distance 48.2 m, estimate path 1.01 m, and 482 of those seconds commanded at least 0.05 m while the estimate moved under 0.02 m. The command on those lines is `v=-0.10`. The follow frames show the shell in a doorway. The true body was sitting. The estimate was not drifting through a large cloud. `_stuck_escape` owns the tick, clears the stall latch, and keeps reversing while the depth picture stays frozen. A frozen picture drops the wheel step, so the pose does not integrate the command, the picture stays frozen, and `camera_stuck` stays true.

−0.35 run 1, the summary that did close, 84 s from t=77. Hold 1%, veto 0%, commanded-but-still 11%, commanded 1.81 m, estimate path 1.61 m. Recovery 58%: `contact` 979, `stall` 642, `narrow_gap` 98. The odometry followed a local shuffle inside the ball. That is a different sit from the frozen reverse.

−0.35 run 4. Ground-truth pile 553.0 s from t=47. 491 dump seconds, 448 of them `src=stuck`, commanded 45.6 m, estimate path 0.39 m. No summary line, because the sit lasted to the cap. Same pattern as north run 1. `stall` is 35 of those seconds, not the owner.

Dead ends: do not read 10.2 or 7.6 as a gain or a regression. Both gaps are inside the 6 point noise, and this −0.35 sample's own sd is 6.1. Do not turn a flag on. Do not delete the recovery stack; the one-rule trial already visited less floor. Do not treat the trusted-travel stall exit as the fix for these sits. The long ones are `stuck`, and that path does not use the travel counter. Do not add a threshold or a new recovery rule from this sample.

Outcome: the best-center dump stays. It sees the sits the entry anchor missed, and it separates a shuffle the pose follows from a reverse the pose does not. The stall travel rule is in the measured commit and did not shorten the piles. The next question is why a stuck reverse of 0.10 m/s moves neither the estimate nor the body, and why the summary is dropped when that sit lasts until the cap.

## The wheels turn and the shell does not

Symptom: a stuck reverse of `v=-0.10` lasted for minutes while the estimate moved about a metre and the true body stayed put. The open question was whether the body was blocked, the wheels were spinning, the motor command never arrived, the contact geometry let go, or the depth check froze the pose and that freeze was being mistaken for a stopped body.

How it was diagnosed: no new simulation. North run 1 from the trusted-travel round is the doorway sit (ground-truth pile 564.5 s from t=35, pose about (3.88, 4.96)). Its `wheel_enc.csv` is the post-safety setpoint and the hinge sensor. `physics_log.csv` is the true body, hub height, and contact counts. `pose_log.csv` is the estimate. Ground truth was used only for this read. The mapper did not receive it. Flags were already off on that run.

The command that reaches the hinges is the command `_stuck_escape` returns. From t=60 s to t=600 s every 10 s bin is `cmd=(-1.25,-1.25)` rad/s, which is `-0.10` m/s at the 0.08 m wheel. The hinge sensors match it: `-12.49` rad on each side per 10 s, so about 1.00 m of wheel travel per 10 s and about 55 m over the sit. True body path in those bins is 0.000 m and true speed is 0.000. The estimate moves about 0.03–0.05 m per 10 s, so it rejects almost all of that wheel travel. That rejection matches the body. Accepting the encoders would have walked the pose away from the shell.

The drop is earlier than the reverse. At t=38–41 s the command is still straight, about `(+1.50,+1.50)`, all four hubs are at z=0.080, all four report the floor, `up_z` is 1.000, and the body is moving at about 0.12 m/s near (3.94, 5.15). At t=42 s the right hub falls to z=0.049 and `up_z` to 0.982. At t=44 s the right hub is z=-0.003 and the front-right hub is z=-0.004, `up_z` 0.965. At t=47 s the command becomes the equal reverse and stays there. From t=60 s, `up_z` is 0.953, the right hub stays at z=0.002, wall contacts stay 0, and the floor count on all four wheels stays 0. The motors are not stalled: a loaded hinge would lag the velocity command, and these hinges match it.

The place is the seam between floor plates. The living plate's west face is x=3.7592. The bedroom plate's east face is x=3.6576. The open slot is 0.102 m, and at y=4.96 no bridge solid covers it. The shell centre is x=3.88, about 0.12 m onto the living plate, so the west-side wheels (0.17 m from the centreline) can drop into that slot. The follow frame at that moment shows the shell in the doorway. This is contact geometry. It is not a missing `setVelocity`, and it is not the depth check inventing a stop.

`_stuck_escape` does own the command after the drop. It runs before the stall machine and clears `_blind_back`, and the equal `-1.25` rad/s is that reverse. It does not own the drop. The wheels left the floor while the command was still a forward mix and the body still had speed.

The same ten runs have two other contact shapes, both with the same actuator fact (the hinge matches a `-1.25` rad/s reverse and the body speed is about 0):

- North run 2, t=200–220 s, at (3.97, 1.28). Level, `up_z` 1.000, every hub z=0.080. Only the right wheel reports a floor contact. The other three wheels and the casters report none. Encoder travel is `-2.00` m in 20 s. True path is 0.
- North run 5, t=200–220 s. Both rear wheels report a wall contact for the whole window, the front wheels report the floor, hubs stay at z=0.080, `up_z` 1.000. Encoder travel is `-2.00` m. True speed is 0.007 m/s. The same rear-wall window shows up on −0.35 runs 3 and 4. The hinges still reach the commanded speed, so the wall is not stopping the motor. The tire is scrubbing.

Dead ends: do not treat the frozen depth picture as the reason the body stopped. The picture is frozen because the body has stopped, and the estimator's rejection of the spinning wheels agrees with the true path. Do not trust those encoder steps into the pose. Do not add a recovery rule that issues a different reverse; the hinges are already doing the reverse they were given. Do not edit the floor plates or the proto from this round. The slot and the wall scrub are geometry, and that change waits on Dan. Do not read the earlier "commanded 48 m, pose moved 1 m" line as "the motors did not turn."

Outcome: the pile summary is written when the run is marked done, before `mapping_done`, so a sit that lasts to the cap is recorded. The summary now includes `wheel_m`, the hinge travel, beside `cmd_m` and `pose_m`. No driving change. The five-by-five was not repeated. The next decision is Dan's, on the floor seam and on a wheel that is against a wall and still free to spin.

## The partition slot was not a real floor gap

Symptom: a wheel fell through the floor at door G. The real apartment has no such gap. The modeled slot was 0.102 m, from the living plate's west face at x=3.7592 to the bedroom plate's east face at x=3.6576.

Where it came from: that distance is the 4 in partition, `0.1016` m, the estimated wall thickness. Room plates have stopped at the wall faces since the apartment world was added in `9e335e7`. The doorways in those partitions had plates. `APT_FLOOR_CARPET_EDGE` covered door G (0.8128 m along the wall, 0.1056 m across it) and `APT_FLOOR_CARPET_EDGE_BATH` covered the bedroom-bath walkway. Commit `0cd3694` ("Drop the carpet ramps") removed both. Their collision meshes were `IndexedFaceSet`s. The bedroom floor was lowered to the same top as the laminate, and the doorway slots were left empty. The real 10 mm carpet lip was not the hole. Deleting the plates was.

How the rest of the floor was checked: every `APT_FLOOR` visual box matches its collision box, and every top is z=0. Every gap between collinear wall pieces, including a 4 in post on that line, was sampled at 1 cm, inset 1 cm from the jambs. An opening that contains a closed door (`APT_DOOR`, the front door and the patio door) is not walkable and is not required to have a floor plate. Every other opening must be covered. That list is door G, the bedroom-bath walkway, door H, the entry opening, the laundry door, and the closet opening off the bath. The last four were already covered by the laminate bridges from `9e335e7`. Door G and the walkway were the only uncovered ones. A point outside the exterior walls is not a hole. The living and kitchen plates share an edge at y=2.5146 with no gap. This is not a claim from door G alone. It is also not a survey of cracks thinner than 1 cm, and it does not drive a robot along every seam.

The repair puts those two plates back, flush, carpet, the same footprint as the removed edges: `APT_FLOOR_BRIDGE_BEDROOM` and `APT_FLOOR_BRIDGE_BED_BATH`. Doors, walls, furniture, friction, motors, and the controller were not touched. The warehouse world was not touched. Commit `7590d16`.

A 60 s spawn just east of door G, facing the slot, crossed it from x=3.90 to x=3.55 between t=23.3 s and t=25.8 s. On the slot the lowest hub was z=0.080, `up_z` was 1.000, body speed was 0.12 m/s, and all four wheels reported the floor. Before the repair the same slot put a hub at z=-0.003.

Five 600 s runs of −0.35 and five of north, flags off, same harness as the gapped-floor round. Depth seeds 63 and 64. Every run reached 600 s and 75,000 pose rows. No hub fell below z=0.078. Where a run crossed a repaired slot, the hub on that crossing stayed at or above z=0.078. Early-quit count 0. North run 2 logged one `start_component` and still finished.

−0.35 visited % 16.73, 19.07, 12.77, 9.51, 15.85 (mean 14.79, sample sd 3.71). Piles 102, 183, 325, 464, 82 s (mean 231 s). The gapped-floor mean was 10.20, piles mean 364 s. The variance mean was 14.7. The gap to variance is 0.1 points.

North visited % 7.05, 19.53, 9.31, 7.25, 13.31 (mean 11.29, sample sd 5.25). Piles 519, 116, 294, 498, 217 s (mean 329 s). The gapped-floor mean was 7.64, piles mean 464 s. The variance mean was 9.0. The gap is 2.3 points.

Those visited gaps are inside the 6 point noise. The variance runs and the gapped-floor runs were the same broken world, so they are not a clean before for the driving. The clean driving result is the hub height.

Sits that remain are not the slot. North run 1 sat 428 s from t=172 at (4.70, 2.49), level, one rear wheel on a wall about half the window, four-wheel floor contact 0.02. −0.35 run 4 sat 415 s from t=185 at (0.34, 5.64) with a rear wheel on a wall. North run 4 sat 399 s from t=201 at (0.82, 6.62) with both rear wheels on a wall. −0.35 run 3 sat 311 s from t=289 at (8.19, 1.37), level, hub z stayed 0.079, and no wheel reported the floor for that window. That last one is the unloaded-wheel shape, not a fall through a plate.

Dead ends: do not read 14.8 or 11.3 as a coverage gain. Do not treat the old 14.7 / 9.0 or the 10.2 / 7.6 as a driving baseline for a controller change; the world under them had an open slot. Do not widen a door to hide a slot. Do not fill under a closed exterior door. Do not claim the floor is free of every crack thinner than the 1 cm sample.

Outcome: the two doorway slots are plates again. The regression fails if an interior opening loses its floor. Wall scrubbing and a wheel with no contact are still in the logs, and they are a separate problem.

## A stuck reverse that the picture ignores

Symptom: on the repaired floor, three long sits still had a rear tire on a wall, and one had ride-height hubs with no floor contact. The question was why the reverse kept going, using the robot's own sensors, with the contact log only as a check afterward.

How it was diagnosed: the repaired-floor logs, flags off. In each of those four windows the post-safety command was `(-1.25,-1.25)` rad/s, which is `v=-0.10`, and every 1 s bin of the hinge sensors matched it. True speed was about 0. The forward nose in the mapper log was 0.9–2.2 m, so the forward beam was open. The pile source was `stuck` for the whole sit. `_stuck_escape` sends that reverse when the rear beam is not within 0.08 m, and it keeps sending it while `camera_stuck` is true. `camera_stuck` is a frozen forward picture plus a nonzero command. The escape is that command, so the picture stays frozen and the reverse does not end. The encoders say the hubs turned. The forward camera says the shell did not. That is the onboard split. The wall-contact counts are the supervisor's, and they are not an input.

The press test still wants that reverse to start: a still picture with an open nose is a press. The fault is that the reverse never ends. After the hinges have turned the existing `BLIND_BACK_CAP_M` (1 m) of that straight reverse while the picture stays frozen, `_stuck_escape` yields. A picture change of 0.05 m in the depth signature clears the yield. No new distance and no ground truth. Commit `6281d58`.

Five runs of each seed, same repaired world, same spawns, depth seeds 63 and 64, flags off. The repaired-floor five was not rerun; it is the baseline, and Webots still varies run to run. Every new run reached 600 s. Early-quit count 0. `start_component` still appears and does not end the run.

−0.35 visited % 26.56, 15.46, 9.61, 24.84, 18.00 (mean 18.89, sample sd 6.94). Piles 75, 257, 437, 157, 112 s (mean 208 s). The repaired-floor mean was 14.79, pile mean 231 s. The gap in visited floor is 4.1 points.

North visited % 22.68, 17.93, 10.77, 23.32, 10.27 (mean 16.99, sample sd 6.27). Piles 144, 127, 325, 119, 408 s (mean 225 s). The repaired-floor mean was 11.29, pile mean 329 s. The gap is 5.7 points. Both sample sds are about 6 points. Neither gap is a coverage result.

Driving: some runs no longer spend the whole cap in `src=stuck`. −0.35 run 1 had 0 s of stuck and a 75 s pile. North run 1 had 0 s of stuck. The long ones moved from a pure stuck reverse to a mix. −0.35 run 3 still had 289 s of `v=-0.10` (139 s of it `stuck`, the rest `pile_leave`) and both rear wheels on a wall around (3.40, 7.42). North run 5 had 259 s of that reverse and both rear wheels on a wall around (3.95, 0.88). North run 3 had 217 s of reverse with no wall contact and four-wheel floor contact only 0.30 of the window, hubs still near z=0.08. Yielding the stuck escape hands the tick to pile-leave, and pile-leave issues the same reverse.

Dead ends: do not treat an open nose as a reason to drive forward on the first stuck tick. The press test is that case, and the reverse has to start. Do not read 18.9 or 17.0 as a gain. Do not edit friction, motors, or the world from this sample. Do not add a second recovery rule in the same round.

Outcome: the stuck reverse now has the same 1 m cap as a blind backup, measured by the hinges, and only while the forward picture stays frozen. The sit is not gone. The next command in the same sit is pile-leave, and it reverses too.

## The collider is on the floor until a corner lifts it

Symptom: after the stuck escape yields, pile-leave sends the same `v=-0.10` reverse. Dan's hypothesis, from earlier Webots work, is that the visible tire sat on the floor while the hidden collision shape was about 0.35 mm above it, so the hubs spun and the body did not. A second hypothesis is intermittent wedging: a chassis or tire shape loads and unloads at a corner. A permanent gap would also show up in ordinary driving.

How it was checked: the proto and the wheel mesh, then the saved physics logs. No new simulation. No geometry edit.

The collision tire is a cylinder of radius 0.08 m and thickness 0.05 m, center at z=0.08, so the authored tread is at z=0. The visual mesh vertices lie on that same radius within 0.001 mm (`butlerbot_skid_wheel.obj`, radius 0.080001). The mesh builder rejects a wheel whose vertex radius differs from 0.08 m by more than 0.1 mm. The chassis boxes end at z=0.105 (deck and side sonars). The floor pods end at z=0.055. None of those are the support when the tires are at z=0.08.

Ordinary driving, yield −0.35 run 1, t=20–40 s, speed about 0.15 m/s. All four hubs at z=0.08000, body z median 0.00000, contact point z median 0.00000, every wheel reporting the floor, no wall. The same medians hold for the free-drive windows of yield north run 5 and the repaired-floor north run 1. The body z in those windows swings about ±0.35 mm around zero. That is the size of Dan's figure, and it is the bounce while the tire is carrying the robot, not a resting gap. A collider that stayed 0.35 mm off the floor would not leave the hub median at 0.08000 and the contact median at z=0 while the shell moves.

Two different sits, both with hinges matching the reverse:

Repaired-floor north run 1, t=250–260 s, before the yield change. Front hubs z=0.08000 and on the floor. Both rear hubs z=0.09425, rear floor contact 0, left rear wall contact 0.49. Body z=0.00729, `up_z` 0.9979, about 3.7° nose-down over the 0.22 m axle spacing. The rear axle is 14 mm off the floor. The hub positions say that. The missing rear floor count agrees with the lift; it is not the only evidence.

Yield north run 5, t=300–320 s. All four hubs at z=0.080, body z=0.000, `up_z` 1. Both rear wheels report a wall on 0.80 of the samples and the floor on 0.45. The tire collider is on the floor and on the wall at the same time. The hinges still match `v=-0.10`. The body speed in the longer window around that sit is about 0.03 m/s. This is scrubbing, not an airborne wheel. A missing contact report is not what this window shows.

The handoff does not carry the failed reverse. `_stuck_yield` is read only inside `_stuck_escape`. Once that returns nothing, a pile that has already lasted 60 s calls `_straight_leave`. That function does not see the yield or the hinge distance. It still emits `v=-0.10` when it prefers backing. On yield north run 5, 30 of the pile-leave reverse seconds had a mapper nose of at least 0.5 m, so the open-nose check alone did not force a forward command. Which of the remaining checks fired (a map clip, a short forward bubble, or a rear gap larger than the forward gap) is not in the log.

Dead ends: do not treat the 0.35 mm figure as an authored radius error. The mesh and the resting hub disagree with that. Do not treat a missing floor count, by itself, as an airborne wheel. Do not patch pile-leave in this round. The rear tire is mechanically against a wall, or the rear axle is lifted, and another reverse rule would be guessing which command the contact will allow. Do not edit the proto or the friction without Dan.

Outcome: ordinary driving is the collision cylinder on the floor. A corner can lift the rear axle by about 14 mm or plant both rear tires on a wall while they stay at ride height. Pile-leave repeats the reverse because it does not know the stuck reverse already failed. No controller change followed this read.

## What held the rear axle up

Symptom: repaired-floor north run 1 held both rear hubs at z=0.09425 from t=175 s through t=600 s. The fronts stayed at z=0.08000. Dan asked what pair of shapes was actually carrying that pitch. A permanent 0.35 mm tire gap was already inconsistent with free-drive hub medians of 0.08000.

How it was measured: the saved physics and wheel logs, the robot pose at t=175.2 s, and the bounding boxes. No new world. The pose logger does not name the other body in a contact, so the pair is the overlap of those boxes, not a force sensor.

At t=175.2 s the shell is at (4.696, 2.489), yaw 3.119, body z 0.00729, `up_z` 0.9979 (3.7° nose-down). The command for the previous 1.5 s, and then for the rest of the cap, is `(-1.25,-1.25)` rad/s, a straight reverse. The island cabinet `apt_furn_island` has its west face at x=4.8768, y from 1.600 to 2.515, z from 0 to 0.914. The rear-left tire center transforms to (4.796, 2.317, 0.094). The cylinder of radius 0.08 m is 0.56 mm clear of that face. The rear sonar box, body center (−0.172, 0, 0.12), size 0.032×0.040×0.030, has a corner 0.005 mm from the same face. That is contact, within the solver slop. The deck's closest corner is 34 mm clear. The rear-right tire is 80 mm clear. The front tires are on the floor.

The sequence before the lock: from t=153 s the rear wheels report no floor while their hubs are still z=0.080 and the command is reverse. At t=156 s the left rear wheel reports a wall and the hubs are still z=0.080. At t=157 s both rear hubs rise through 0.088 to 0.093, then drop back. At t=174.5 s the reverse is on again and by t=175.0 s the hubs are at 0.09425 and stay there, to five decimals, until t=600 s. From t=180 s to t=600 s, z, both rear hub heights, both front hub heights, and `up_z` each have one unique value. x and yaw still jitter. Over t=180–190 s the hinges turn −12.50 rad each, matching the reverse. The rear tires are spinning. The pitch is not.

That is a climb, then a prop. The reverse drives the tail into the cabinet. The rear-left tire meets the face while it is still at ride height (the wall count). The body pitches. The rear sonar box, which reaches almost as far back as the tire and sits higher, takes the cabinet face. The rear tires unload by 14 mm. Gravity does not drop them because the sonar box and the front tires are the supports. A hidden 0.35 mm tire clearance does not produce a 14 mm lock, and it does not appear in the free-drive windows.

The ride-height scrub is a different pair. Yield north run 5 at t=300.5 s, pose (3.949, 0.877), yaw 0. Both rear hubs z=0.080. Both rear tire centers sit 80 mm from `APT_WALL_SHARED_A`, which is the tire radius: the tread is on that wall. The log reports wall contacts and, at that instant, no floor contacts. The hub height says the tires are not 14 mm in the air.

Pile-leave does not receive `_stuck_yield`. Commit `6ac2026` writes one line a second once a pile-leave command is chosen: branch, nose, forward gap, rear gap, blind, forward clip, back clip, yield, and the hinge metres already counted. A unit test with forward gap 0.40 m and rear gap 2.00 m records `straight_back:rear_clearer` while yield is still set. A 240 s north run on this code, depth seed 64, did not open a pile and did not lift a hub (maximum left hub 0.081 m). The island sit was not reproduced. The branch that chooses reverse when the rear gap is the larger one is the tested decision. Which of those checks fired in the old 259 s sit is still not in that sit's log.

Dead ends: do not call the rear tires the support of the 14 mm lock. They are 0.56 mm clear, and the sonar box is the contact. Do not call a missing floor count the proof by itself; the hub z is the proof they are up, and the box overlap is the proof of what holds them. Do not patch pile-leave from the unit test alone. The live sit was not re-read with the new line.

Outcome: the lift is the rear sonar collider against the island cabinet, after a reverse drove the rear tire into that face. The ride-height case is both rear tires on `APT_WALL_SHARED_A`. The pile-leave line is in the tree. One 240 s probe did not reach a pile. `eval/score_rooms.py` (82 lines) had no caller in tests, docs, or the CLI; room seen/visited percent stays in `eval/score_apartment_map.py`. Removed in `233ed4b`.

## The forward branch is rewritten into the failed reverse

Symptom: the 240 s north probe on the new pile-leave line did not stall, so the unit test `straight_back:rear_clearer` is not the branch of a live sit. Dan asked for one real sustained failed reverse, with the branch, the gaps, the clips, and the yield read against the command, the hinges, the body, and the contacts. No driving change.

How it was run. Three probes, flags off, Webots R2025a `--mode=fast --no-rendering --batch`, `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. No five-by-five. Ground truth and the supervisor contacts were not given to the controller.

1. North, depth seed 64, spawn (5.80, 5.60, yaw 2.2), port 1712, the 1 Hz line from `6ac2026`. Finished at 600 s. A straight reverse (`(-1.25,-1.25)` rad/s) repeats from about t=166 s to the cap. The longest unbroken reverse command is 24.2 s (t=383.7–407.9). The body stays inside 3 cm of (3.403, 7.456), yaw −1.861, for 176 s, and the left hinge turns 201 rad in that ball. Hubs stay at z=0.080 (maximum 0.0815). When the yaw is −1.57, both rear treads meet `APT_FURN_NIGHTSTAND`'s south face (sampled gap under 1 mm) and both rear wheels report a wall. That is ride-height tread on the nightstand, not the island lift. The 1 Hz branch line in those windows says `nose_forward` or `straight_forward`. The separate line `blocked heading, not driving back in` is written every 2 s through the same windows. One `fallback_reverse` tick at t=380.2 s has nose 0.46, both clips set, rear 0.02, yield 0. No `straight_back` line.

2. −0.35, depth seed 63, spawn (6.10, 4.70, yaw 1.2208), port 1711, commit `be9bac7` (a branch change is logged immediately). Finished at 600 s. No sustained failed reverse. The longest reverse command is 11.9 s and the body moves 1.02 m at 0.10 m/s, hubs z=0.080, on the floor. Branches: `straight_forward` 109, `trail_reverse` 100, `straight_back:clip_fwd` 1. The `trail_reverse` lines have both clips set, rear often `none`, yield 0. That reverse is the trail branch, and the shell follows it. Maximum left hub 0.0824. Not a stall, and not a 14 mm lift.

3. North again, same seed 64 and spawn, port 1712, commit `be9bac7`, started 2026-11-08T20:28:16Z, finished at 600 s (`mapping_done`). A straight reverse is commanded for 340 s of the 600. From t=416 s to t=560 s the reverse is on for 133 s, the left hinge turns −166 rad (13.3 m of tread), and the body stays near (3.86, 5.39). At t=506.3 s the pose is (3.861, 5.392), yaw −1.114, hub z=0.08005, body z=0.000, `up_z` 1.000, speed 0.008, wheel wall counts 0/0. The branch log for the whole run is `nose_forward` 237 and `straight_forward` 182. No `straight_back`, no `trail_reverse`, no `fallback_reverse`.

The same tenth of a second carries both lines. At t=436.9 s: `branch=straight_forward nose=99 fwd=0.43 rear=0.04 clip_fwd=0 clip_back=1 yield=0 yield_m=0.00` and `src=pile_leave v=-0.10`. `straight_forward` returns `wheel_speeds(0.12, 0)`. `_log_pile_leave` runs before that return. `_remember` then calls `_respect_escape_block`, which replaces a forward command along the stuck heading with `wheel_speeds(-0.10, 0)` for 18 s, inside 0.70 m and 0.9 rad of the pose `_stuck_escape` armed. The phase line reads `_commanded` after that replacement, and the source stays `pile_leave` because the block does not mark again. The log line of the replacement is `blocked heading, not driving back in`, every 2 s from t=413.8 s through the rest of the cap. Yield is 0 on these lines, so this sit is not the yield handoff. Pile-leave chose forward. The block turned it into the reverse.

The shape at that pose is the rear sonar box, body center (−0.172, 0, 0.12), size 0.032×0.040×0.030. Its closest sampled corner is 0.93 mm from `APT_WALL_SHARED_C`. The rear-right tire stays at least 19 mm clear of that wall. Hubs at z=0.080 say the tires are at ride height, so this is not the 14 mm axle lift. The wheel contact counter stays 0 because the close shape is the sonar box, not a tire. The pose logger does not name the other body; the pair is the box overlap.

A separate 1.2 s pitch at the same jamb, t=189.4–190.6 s, holds both rear hubs at z=0.09491, body z=0.0076, `up_z` 0.9977, with the sonar box on `APT_WALL_SHARED_C` and the rear tires about 92 mm clear. The hubs are back at z=0.080 by t=190.6 s. That is the island kind of prop, at the doorway jamb, and it does not lock. The island cabinet (west face x=4.8768) is not this pose.

Dead ends: `straight_back:rear_clearer` is the unit test, and it is not a line in these three logs. The throttled `nose_forward` line is the pre-rewrite branch, not the wheels that were sent. A missing wheel-wall count is not clearance while the sonar box is under 1 mm from the wall and the body is still. The 1.2 s pitch is not the island lock and not the sustained stall. Not reproducing the island sit does not clear it; these probes never entered that cabinet. No clearance knob, no friction edit, no proto edit. The command that keeps the reverse going is already in the controller.

Outcome: the sustained reverse in the re-logged north run is `_respect_escape_block` replacing a forward pile-leave command. The shell is at ride height with the rear sonar box against the doorway jamb. The nightstand sit on the 1 Hz log is the same rewrite against a different shape, the rear treads on `APT_FURN_NIGHTSTAND`. The smallest next change, not made here, is to stop that block from sending `v=-0.10` once the reverse has already failed to move the shell.

## A blocked reverse that does not move the shell

Symptom: Dan agreed the next change was to stop the blocked-heading safeguard from substituting a reverse again after that reverse had failed to move the robot. Forward along the blocked heading, and an endless stop or spin, were not the replacement.

How it was changed. Commit `9ebb7cd`. The first substitute is still the reverse. Progress is the planner pose, the same `STILL_M` (0.04 m) and `STALL_WINDOW_S` (2 s) a command-the-body-did-not-follow already uses. Wheel angle is not that clock. A frozen picture does not use it either: the hinge cap already owns that reverse, and the pose is held on purpose while it runs. Once the pose has stayed put, the reverse is not sent again, including from `_stuck_escape`. The other legal leave is a turn off the heading, one sign then the other, each dropped when yaw does not change by 0.12 rad inside the same 2 s. After both are quiet the caller's forward is no longer rewritten. A straight reverse from the caller is not sent. Leaving the 0.70 m spot, or yawing past the existing 0.9 rad release, clears the latch so a later block can back up again.

Matched north seed 64, spawn (5.80, 5.60, yaw 2.2), depth seed 64, flags off, 600 s. The run before this change (`/tmp` `leave_repro/north_b`, code `be9bac7`) commanded a straight reverse for 340 s. The longest spans were 18–37 s and the body moved about 0.1 m, hubs at z=0.080, at the doorway. The run after (`block_fix/north`) commanded that reverse for 81 s. Its longest span, 18.1 s from t=545 s, moved the body 1.81 m. Two lines say `blocked reverse did not move the shell`. Maximum hub z=0.081. The shell leaves the doorway corner and later spins near the nightstand. That spin is not the block: the block never logged `blocked turn did not yaw` on this run.

Five runs of each seed, flags off, 600 s cap, depth seeds 64 and 63. No movie. This is not a second five of the old code, so the visited percents are the sample, not a win against 14.7 / 9.0 or the repaired-floor means.

| seed | visited % | mean | sd | longest pile | early stop | straight reverse |
| --- | --- | ---: | ---: | --- | --- | --- |
| north | 26.28, 3.15, 15.31, 9.06, 11.86 | 13.13 | 8.59 | 63 s, 543 s, 115 s, 352 s, 140 s | 0 | 71 s, 47 s, 156 s, 102 s, 201 s |
| −0.35 | 22.28, 10.57, 15.87, 7.89, 14.89 | 14.30 | 5.51 | 196 s, 369 s, 67 s, 169 s, 210 s | run 3 at 456 s | 183 s, 91 s, 68 s, 141 s, 184 s |

Every run exited 0. The early stop is `end_run reason=start_component` after a final scan found no reachable frontier. The longest straight reverse that barely moved the body is north run 2, 20.6 s and 0.04 m, hubs z=0.080. The other long reverses in the ten moved the body by a metre or more, except −0.35 run 1's 14.4 s span (0.13 m) and −0.35 run 5's 12.8 s span (0.03 m). North run 4 drove a reverse into the island from t=306 s, the pose moved with the climb, and the shell finished inverted (`up_z` −0.01, hub z=0.30) through t=600 s. North run 3's hub touched z=0.45 for 0.7 s and came back down.

Dead ends: deleting the block, which would drive the forward along the stuck heading on the first tick. A new distance or a new recovery rule. Treating encoder radians as the progress check. Using the 2 s pose clock while the picture is frozen, which cut off the hinge-cap yield the existing test requires. Reading 13.13 or 14.30 as a coverage gain.

Outcome: the repeated doorway reverse is shorter. A reverse the pose believes is moving, including a climb onto the island, is still sent. A frozen-picture reverse still runs out the hinge cap. Spins that are not this block still sit. No further rule in this round.

## The island climb was wheel travel

Symptom: north run 4 of `9ebb7cd`, depth seed 64, spawn (5.80, 5.60, yaw 2.2), finished inverted after a reverse into the kitchen island. The planner pose moved during the climb, so the 2 s still-pose gate kept the reverse. Diagnosis only. The saved logs are the run. A fresh 50 s start at the pre-climb ground-truth pose (4.652, 2.339, yaw −2.948), same depth seed, drove away from the island and stayed upright (hub z ≤ 0.081, `up_z` 1). The full north spawn was not rolled again. That draw is not this trajectory.

Timeline, ground truth beside the estimate. Command is `(-1.25,-1.25)` rad/s, a straight reverse, from t=304.9 s through the tip. At t=301.4 s pile-leave had chosen `straight_forward`; the blocked-heading rewrite turned that into the reverse. From t=304 s to t=310 s the planner pose moves 0.49 m and the wheel arcs move 0.51 m. Ground truth moves 0.18 m across the floor while the body rises from z=0 to z=0.185 and `up_z` falls from 1.000 to 0.057. Both rear hubs go 0.080 → 0.098 at t=306.5 s (both rear wheels on a wall, floor counts already 0) and 0.300 by t=310 s. From t=311 s the body is still (`up_z` −0.011, z=0.191, hubs 0.300) and the planner pose keeps walking, another metre by t=320 s, still matched to the wheel arcs. `visual_applied_yaw`, `floor_applied_yaw`, and `wall_applied_yaw` change by 0.000 from t=304 s to t=312 s. Fused yaw follows the gyro (−0.190 rad), not the wheel yaw (+0.301 rad). No `slip`, `floor still`, `zero velocity`, or `wheels lifted` line is logged between t=304 s and t=320 s. The nose is 0.87 m at t=305.5 s, 0.97 m at t=307.5 s, 99 at t=309.5 s, then the 0.12 m blind placeholder. The still-pose latch logs at t=318.6 s, after the shell is already inverted and the picture has been the blind placeholder. The pose itself is still moving then. That line is the frozen-picture hinge cap marking the reverse dead, not a pose that had stopped during the climb.

What moved the estimate: wheel translation, accepted. Nothing else added a shift. The forward-range slip check treats a clearance outside 0.2–3 m as no evidence, and a blind reverse is not dropped. The floor stall does not freeze a blank frame, and the code says a climb that stays blank is not a stall. The pitch check that would call a 0.12 rad tip "wheels lifted" runs only after the floor image is confidently still. That branch did not run. `oak_imu` pitch is already wired into the step. It is not logged, and it is not a translation veto on its own. Ground-truth `up_z` 0.957 at t=307 s is about 0.29 rad of tilt, past that 0.12 rad gate, while the reverse continues.

North run 3's hub z=0.445 is not this. From t=332.8 s to t=333.7 s the body flips (`up_z` to −0.26, z to 0.34) near (1.00, 4.67) and is upright again by t=334 s, about a metre away. The planner pose stays put through the tumble (wheel arcs 0.20 m, planner displacement 0). Translation was rejected. The shell then drives on from the landed place. A short physics toss, not a trusted climb.

Dead ends: calling the planner motion visual odometry or a scan match. Those yaw channels are zero, and the metre of pose equals the metre of wheel arc. Calling north run 3 the same climb. Treating the 50 s reconstruction as the scored run. A driving change in this round.

Outcome: the climb is trusted wheel travel. The pose gate saw that travel and kept the reverse. The pitch the robot already measures did not get a vote, because the floor image never became the still picture that gate waits for. The smallest next change, not made here, is to let that existing pitch gate drop wheel translation when the shell is tipped, without waiting for a still floor image.

## The pitch gate fired on level driving

Symptom: the note above said to drop wheel translation when `oak_imu` pitch is more than 0.12 rad from the learned rest, without waiting for a still floor image. That change was committed as `2db10f5` and then taken back.

How it was checked. The threshold stayed 0.12 rad. The line logs the IMU pitch. Unit tests: a 0.20 rad pitch drops the wheel step and the blocked-heading reverse; a level reading and a 10 mm rise over the 0.22 m axle spacing (about 0.045 rad) do not. Mapping, floor, twin, skid, and the warehouse proto test passed, 176. Postgres was not up, so database tests were skipped.

A 50 s start at north run 4's pre-climb pose stayed upright and never logged a pitch drop. The same north spawn was then run five times with the change and five times on `9ebb7cd`, depth seed 64, and the same five for −0.35, depth seed 63. Flags off. All twenty reached 600 s. No run had `up_z` below 0.5. The movie run (new north 1) never came closer than 2.9 m to the island face, so there is no clip of a tip.

| code | seed | visited % | mean | sd | min up_z | max hub z |
| --- | --- | --- | ---: | ---: | --- | --- |
| pitch gate | north | 10.52, 16.59, 3.17, 18.25, 19.67 | 13.64 | 6.81 | 0.998, 1, 1, 1, 1 | 0.094, 0.082, 0.081, 0.082, 0.082 |
| `9ebb7cd` | north | 16.41, 19.66, 23.70, 26.12, 23.78 | 21.93 | 3.86 | 1, 1, 1, 1, 1 | 0.083, 0.081, 0.083, 0.081, 0.083 |
| pitch gate | −0.35 | 19.88, 16.19, 16.23, 19.53, 12.34 | 16.83 | 3.06 | all 1 | 0.084, 0.081, 0.081, 0.084, 0.083 |
| `9ebb7cd` | −0.35 | 10.26, 8.32, 13.31, 12.23, 10.16 | 10.86 | 1.95 | 1, 1, 0.999, 1, 1 | 0.082, 0.083, 0.083, 0.082, 0.081 |

On new north run 1 the log says `shell pitched 0.11 rad, wheel translation dropped` at t=105.2, 111.9, 125.9, 130.6, 351.9, 483.9, 537.6, 589.5 s. At each of those times the ground-truth body is level: `up_z` 1.000, hubs z=0.080, and the speed is often 0.10 m/s. The printed 0.11 rad is the IMU pitch, and the gate compares it with the learned rest, so a rest a little below zero plus a reading near 0.11 rad crosses 0.12. The body had not tipped. Dropping the wheel step there freezes the planner pose while the shell is driving. North's visited mean fell from 21.93 to 13.64 on the matched seeds. −0.35 moved the other way. Neither difference is a coverage result to keep. No run in this pair inverted, so the gate was not shown stopping the island climb.

Dead ends: keeping the gate because −0.35's mean rose. Lowering 0.12 rad. A new knob. Treating the unit-test pitch of 0.20 as what the device reports on a level floor.

Outcome: `2db10f5` is reverted. The 0.12 rad check is still only the floor-still branch. A pitch veto on every wheel step needs a rest that is not one noisy sample, and this round does not add that.

## A flat floor frame does not confirm wheel travel

Symptom: on `9ebb7cd`, north run 4's reverse into the island moved the planner pose with the wheels, so the 2 s still-pose gate kept the reverse until the shell inverted. The floor image during that climb was blank, and the pitch check waits for a confidently still picture. A pitch gate against one rest sample false-fired on level driving and was reverted.

What the downward camera reports. A level reverse stays textured (std about 6–11) and the closed steps come back `moving`. A forced reverse into the island face goes flat (std about 0.4–0.6) and every step is `blank` while the wheels keep turning. The blank begins while the body is still partway up: earlier in that same reverse the tracker is still saying `moving` at std about 9–11 as `up_z` falls through 0.8 toward 0.6. `oak_imu` is a Webots InertialUnit, roll/pitch/yaw with noise 0.01. That is ground-truth attitude. It was not wired in as a new controller input.

How it was changed. Commit `b1827e7`. When the floor step is `blank` and the frame std is under 2, and the hubs have claimed 0.02 m since the anchor, translation is put back on that anchor. Both numbers already belong to the floor tracker (a flat frame is not a track; a confident still needs 2 cm). A step that comes back `moving` still accepts the travel and moves the anchor. A blank that still has texture, the wrapped correlation, still restarts the anchor. No new threshold.

The forced reverse used the pre-climb pose (4.652, 2.339, yaw −2.948), depth seed 64. The backup was told to ignore the rear gap for that probe, and that edit was removed before the commit. Once the image is flat, the planner pose stays on the anchor, about 0.13 m from the start of the reverse, while the wheels continue. The shell still inverts, because the probe keeps commanding the reverse. The pose does not follow that remainder.

Matched five runs of each seed, flags off, 600 s. North is depth seed 64 at (5.80, 5.60, yaw 2.2). −0.35 is depth seed 63 at (6.10, 4.70, yaw 1.2208). The `9ebb7cd` column is a rerun beside this commit, not the earlier 21.93 / 10.86 sample. All twenty reached 600 s. No run had `up_z` below 0.5. No run climbed the island, so this sample does not say whether a real climb would still count as progress. The closest north approaches were 0.40–0.51 m with the hold and 0.24–0.40 m on `9ebb7cd`, all upright. Sample sd divides by n−1.

| code | seed | visited % | mean | sd | min up_z | holds |
| --- | --- | --- | ---: | ---: | --- | --- |
| floor blank | north | 14.62, 15.81, 10.78, 13.10, 16.92 | 14.25 | 2.40 | all 1.000 | 1, 0, 2, 0, 8 |
| `9ebb7cd` | north | 27.07, 3.44, 20.97, 20.14, 11.62 | 16.65 | 9.21 | 1, 0.999, 1, 1, 1 | — |
| floor blank | −0.35 | 13.11, 13.39, 15.40, 13.96, 9.19 | 13.01 | 2.31 | all 1.000 | 2, 3, 8, 6, 0 |
| `9ebb7cd` | −0.35 | 22.51, 9.12, 9.90, 8.85, 14.10 | 12.90 | 5.78 | 1, 0.998, 1, 1, 1 | — |

North's mean is 2.4 points under a baseline whose sd is 9.2. −0.35 differs by 0.1. Neither gap is a coverage result. This is not the pitch gate's drop, where the matched north mean fell from 21.93 to 13.64.

Flat frames do occur on a level floor. North run 5 logs eight holds while `up_z` is 1.000 and both hubs are z=0.080. After one of them the body moves 0.26 m in two seconds and the planner moves 0.16 m. The next textured step accepts travel again. Several of those frames sit near (3.7, 5.2) and (1.1, 4.5). The pose does not stay put for the rest of the drive.

Dead ends: retrying the single-rest pitch gate. A rest pitch that keeps learning while the robot drives, which would absorb a slow climb. Using `oak_imu` as a controller input. Treating an ambiguous, textured blank as a stall. Reading the 2.4 point north gap as a loss.

Outcome: a flat blank is no longer wheel travel. A moving floor still is, and that includes the part of a climb where the picture still has texture. The scored runs did not climb.

## A held pose cuts the blocked reverse

Symptom: the forced island reverse kept commanding reverse after the planner pose stuck, and the shell inverted. That probe was a scripted backup. It never armed the blocked heading, so the 2 s pose clock was not consulted.

How it was checked. No controller change. A step loop with a flat floor, the heading armed, and a forward picture that keeps changing: the pose stays under 0.04 m, a straight reverse starts, and 2.0 s later the log is `blocked reverse did not move the shell` and the command is a turn. The test is `test_a_held_floor_cuts_the_blocked_reverse`. Mapping tests passed, 151. Postgres was down, so database tests were skipped.

The same pairing on the pre-climb pose (4.652, 2.339, yaw −2.948), depth seed 64. The backup was told to ignore the rear gap, and the heading was armed at explorer t=0.4 s. Both of those were probe-only and were removed. At explorer t=2.4 s the log is `blocked reverse did not move the shell`. The command changes from a straight reverse to a turn. `up_z` is 1.000 when the reverse starts, about 0.63 when it is cut, and reaches a minimum of 0.395 during the turn. By sim t=12.5 s the shell is upright again (`up_z` 1.000, hubs z=0.080). It does not finish inverted. The earlier unarmed probe finished at `up_z` −0.01 with both hubs at 0.30. This probe did not log `floor blank`; an earlier pin (`depth floor and visual still`) is what held translation. The step loop is the run that uses the flat-floor hold itself.

Dead ends: a shorter stall window. The forward picture did not freeze before the stop, so the frozen-picture exemption was not what kept this reverse going. That exemption stays: a frozen picture already belongs to the 1 m hinge cap, and deleting it would cut a backup whose pose is held on purpose. A pile-leave that has cleared the block is a different reverse. The island command was a forward rewritten while the block was still on, which is the path that stops. A stall reverse with no block counts wheel travel the pose rejected, so its own 0.8 s check does not see the hold. That clock was not this reverse, and it was not retimed.

Outcome: once the pose is held and the blocked heading is armed, the existing 0.04 m / 2 s rule cuts the reverse. The shell on this probe pitched and settled back upright. No matched five, because the command path is unchanged.

## A short floor shift is not the wheel step

Symptom: the floor tracker can say `moving` at the start of an island climb, while `up_z` is already falling, and that bit was treated as agreement with the whole wheel step.

What the camera reported, same forced reverse, pre-climb pose (4.652, 2.339, yaw −2.948), depth seed 64. The backup ignored the rear gap for the probe only. On open floor the same reverse is a different picture: std about 7, the peak 8–11 px on a 0.010–0.013 m wheel step, and planner, wheels, and ground truth all move together (0.73 m) with `up_z` at 1.000.

Against the island the peak is 2–5 px on a step of about 0.010–0.013 m, which at this camera is about 8–11 px of floor. From sim t=5.5 s to t=6.5 s, `up_z` goes 0.730 → 0.469, the hubs rise, the peak stays 2.2–3.0 px, and the planner pose moves 0.085 m while the wheels move 0.088 m. That window was trusted. The blank hold starts at sim t=6.5 s, after the pose has already taken the climb. The shell then inverts, because this probe is not the blocked-heading reverse.

How it was changed. Commit `fec4ff8`. A `moving` step confirms the wheel translation only when the peak is within the existing 2 px still gate of the distance that step should have moved. The scale is the camera's own metres per pixel. A shorter peak puts x,y back on the anchor, the same rewind a flat blank already uses. The log is `floor shift short of the wheels`. No new threshold. The 2 s reverse clock was not touched.

After that check, the same forced reverse keeps the planner pose at 0 while the wheels turn through the climb (`up_z` still falls, because the script keeps reversing). The open-floor reverse still matches the wheels, and it does not log a short shift. Tests: a floor rolled about as far as the wheel step keeps the travel; a 2 px roll on a centimetre of wheels does not. Mapping, floor, twin, skid, and the warehouse proto test passed, 197. Postgres was down, so database tests were skipped.

Matched five runs per seed against `222da06`, flags off, 600 s. North depth seed 64, −0.35 depth seed 63. All twenty reached 600 s. No `up_z` below 0.5. No run climbed. Sample sd divides by n−1.

| code | seed | visited % | mean | sd |
| --- | --- | --- | ---: | ---: |
| short shift | north | 10.15, 5.24, 14.34, 12.87, 23.44 | 13.21 | 6.69 |
| `222da06` | north | 12.39, 11.29, 17.28, 4.76, 7.85 | 10.71 | 4.74 |
| short shift | −0.35 | 16.17, 9.54, 17.92, 14.78, 13.42 | 14.37 | 3.17 |
| `222da06` | −0.35 | 22.22, 18.99, 16.52, 17.92, 8.00 | 16.73 | 5.31 |

North is 2.5 points over a baseline sd of 4.7. −0.35 is 2.4 points under a baseline sd of 5.3. Neither gap is a coverage result. Short-shift holds on the new runs were 0–17 per 600 s. The straight open-floor reverse logged none.

Dead ends: the 30% stall ratio on this peak. The climb's ratios sit at 0.27–0.36, so that line still accepts the steps that moved the pose. A new fraction. The pitch gate. An `oak_imu` input. Calling the north mean a win.

Outcome: a floor shift that misses the wheel step by more than 2 px is not travel. A shift that matches the wheels still is. The forced climb's early `moving` window no longer moves the planner pose.

## The two holds together, and where that stops

The short-shift hold and the blocked-heading stop were checked together on the pre-climb pose, depth seed 64. The backup ignored the rear gap and the heading was armed at explorer t=0.4 s. Both hooks were removed. At explorer t=0.2 s the log is `floor shift short of the wheels` (4.4 px on 0.013 m). The planner pose stays at 0. At explorer t=2.4 s the log is `blocked reverse did not move the shell` and the command becomes a turn. `up_z` is 0.625 when the reverse is cut, reaches 0.395 during the turn, and is 1.000 again by sim t=12.5 s with the hubs at z=0.080. The shell does not finish inverted. The unarmed reverse of the same face does.

The matched runs on `fec4ff8` were not climbs. In north run 5 the command is a turn on 44% of samples, forward on 21%, and a straight reverse on 18%. Over sim t=31–300 s the pose path is 25.3 m and the wheel path is 26.6 m. The long net-encoder spans are the shell driving back and forth, not a pose that swallowed a climb.

iRobot's stasis patent (US 9,317,038) treats drive wheels that spin while an independent wheel shows no transitions as stasis, and uses that to escape. Their optical-odometry patent (US 9,744,670) takes distance from the underside camera when its quality is high enough, and marks wheel data unreliable when it disagrees with the inertial measurement. The floor-camera check is the first of those. The second needs an inertial unit we are not using as a controller input, or a stasis wheel we do not have. Neither is a threshold change.

Dead ends: another fraction on the same peak. The 30% stall line still accepts this climb. Shortening the 2 s clock. A new recovery rule from the backup counts. Those backups move the shell.

Outcome: the early `moving` window is not planner travel, and with the heading armed the reverse stops and the shell comes back upright. The pitch to `up_z` 0.395 during the turn is the 2 s the existing clock allows. No further command change.

## The apartment job is not finished at this tip

What the job already is. The mapper declares itself done only when no reachable frontier remains after a final scan and the occupied-cell count stays within 8 cells for 4 s while the shell is stopped (`done: no reachable frontier after a final scan, map stable while stopped`). The apartment reward beside that is visited %, the free floor the shell actually drove over, overall and per floor solid. Seen % is the map label and is not that reward. The floor solids are living, kitchen, hall, bath, bedroom, closet, laundry, pantry, storage, and the bridge plates. There is no second score and no visited-percent knob.

Evidence, the ten 600 s runs already scored for the short-shift change, which is this controller (`fec4ff8`, notes through `0351bf4`). Flags off. North depth seed 64, −0.35 depth seed 63. Every run wrote `mapping_done` at 600 s. None logged the done line. The last phase is still `go`. End reason, where the log flushed it, is `time_cap`.

Visited floor, and the rooms that matter, in percent:

| run | all | living | kitchen | hall | bath | bedroom | laundry |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 10.15 | 15 | 25 | 0 | 0 | 3 | 0 |
| n2 | 5.24 | 11 | 0 | 0 | 0 | 2 | 0 |
| n3 | 14.34 | 6 | 0 | 0 | 0 | 60 | 0 |
| n4 | 12.87 | 22 | 8 | 70 | 0 | 0 | 0 |
| n5 | 23.44 | 47 | 17 | 0 | 0 | 7 | 0 |
| m1 | 16.17 | 18 | 17 | 74 | 0 | 18 | 0 |
| m2 | 9.54 | 7 | 0 | 0 | 0 | 33 | 0 |
| m3 | 17.92 | 12 | 0 | 32 | 34 | 32 | 25 |
| m4 | 14.78 | 34 | 2 | 0 | 0 | 0 | 0 |
| m5 | 13.42 | 7 | 0 | 0 | 16 | 44 | 0 |

Pantry and storage are 0 on all ten. Closet is 0 except m3 at 1. Bath is 0 on eight of the ten. The shell does enter a second room on some seeds (bedroom 60 on n3, hall 74 on m1) and still has not finished the apartment. North run 5, the highest visited draw at 23%, is a turn on 44% of samples and a straight reverse on 18%, and the pose path matches the wheel path. That time is real motion, not a climb the pose accepted.

Dead ends: another coverage batch aimed at the 0.395 pitch. A new visited threshold. A recovery rewrite chosen because `cant_see_behind_backing_out` is a common log line. That line is every logged reverse, and those reverses move the shell.

Outcome: at this tip the job is blocked by the 600 s cap while the mapper is still going and most rooms are unvisited. That is not an honesty bug and not a single command change the logs pick out. No controller edit.

## Why pantry, storage, closet, and the bath stay unvisited

The same ten runs. No new batch. A frontier is free space next to unknown space. Depth through a door paints the next room free and removes that frontier. `_unvisited_stand` can still aim about 2 m into undriven free floor, and its score is capped (`min(area, 4) / length * 0.45`) so it does not beat a real frontier. Goals in the log were warped with the seed spawn. The shell's distance is ground truth.

Pantry: 0 goals inside the box on all ten. Seen is 0 on n2 and m2, where the shell stayed 1.7 m and 1.9 m away, so no way in was on the map. On the other runs seen is 9–69% and the shell's closest pass is 0.33–1.67 m. The room was looked into and never chosen.

Storage: one goal lands in the box, on n4, and at that moment the shell was 1.66 m away, so it was not an approach. Seen is 94–99% on n4, m4, and n5, with visited 0 and the shell 0.43 m short. The camera finished the room from the door. The final map has no frontier left inside it on those runs.

Closet: one goal inside, on m5, with the shell 0.59 m away, and the closest pass is 0.33 m. Visited stays 0. m3's 1% is a brush at 0.22 m with no goal inside the box. Seen is 73–99% on m2, m3, m4, m5, n5.

Bath: the final map still has a frontier inside the bath on every run that did not enter it (42–239 cells in a cluster at least `MIN_FRONTIER_M`). Goals inside the box, with the shell actually there: m3 and m5 entered (visited 34% and 16%). m1 had three such goals, the shell came to 0.19 m, and visited stayed 0. On n1 and m4 a warped goal falls in the bath rectangle while the shell was 2.5–2.8 m away, which is the pose, not a visit. The other five runs never selected a bath goal. The frontier was there and lost the score to other frontiers before the cap.

Dead ends: one patch for all four rooms. Raising the unvisited-stand weight so a seen room beats a real frontier. That weight is already the cap that keeps an unscanned room from outranking a frontier, and these logs do not show it failing at a door the shell was on, except m1's bath, which is one run.

Outcome: pantry, storage, and closet are mostly seen from the doorway and then drop out of the frontier list, so they are not selected. The bath usually still has a frontier and it is not selected before 600 s. m1 is the one bath goal the shell reached and did not cross. No controller change.

## What is left at the time cap

The same ten maps, at 600 s. A frontier the picker will score is a cluster at least 0.30 m long. Its score is that length divided by the path length plus 0.25 m. The unvisited stand is capped at 4 m² and multiplied by 0.45, so a stand 2 m away scores about 0.80. Cells were warped with the seed spawn and compared with the floor plates.

Of the cells in those clusters, 48% land on a floor plate, 11% are within 0.35 m of one, and 42% are farther out. The outside cells form clusters that run around the building, 0.4–1 m past the plates, on every side. A single pose shift cannot put every side outside at once. Rank below uses straight-line distance in place of the A* path. North run 5's outside cluster is 51 m long and 1.6 m from the final odom pose, so that score is 28. The best bath cluster on that run scores 0.28. The outside cluster is the top score on eight of the ten runs. On m2 and m4 a real bedroom frontier is the top score instead (3.9 and 6.1). The gap on the runs the outside cluster wins is large enough that a longer path would not hand the win to the bath.

Real unknown is still there. Inside the plates, the big clusters are bedroom (15% of big-cluster cells), living (14%), bath (10%), and kitchen (5%). Bath clusters that remain score 0.1–2.1. That already beats the capped stand, and the outside cluster beats the bath. Clusters shorter than 0.30 m are 1887 cells and are not scored, so speckle is not what starves the bath.

Dead ends: raising the 0.45. Rejecting every off-plate cell. The 0.4–1 m band is wide enough that a yaw error can move a real edge into it, so a hard reject would also drop real floor. Pantry and storage stay out of this. They are closed.

Outcome: the leftover is both. The score is won by a perimeter of free cells the map drew past the floor, glued into one frontier. Real unexplored floor remains in the bedroom, the living room, and the bath, and it would lose to that perimeter on most of these runs. No controller change.

## The outside loop no longer takes the whole length

Same ten maps, rescored with the planner, not a new batch. A cluster keeps its cell count unless two things are both true: more than half its cells are over 0.35 m from every floor plate, and those cells average more than 1.2 m from their own center. That is the loop. Its credit is the longest 0.40 m strip, and only the share of the strip still within 0.35 m of a plate. A local edge keeps the cell count, including one that sits entirely off the plate. The 0.45 stand cap is unchanged. The plates are the apartment floor solids. The spawn is read once, as odom zero. The inertial unit is not an input.

At the final pose the planner was choosing a majority-off-plate cluster on n1, n2, n3, m3, and n5. North run 5's was the 51 m loop, planner score 19.5. After the credit, that goal is the living room (score 3.76). n2's goal becomes the living room, n3's the bedroom, and n1's the bath (score 2.15). m5 was already the bath, and it stays the bath. m1, m2, n4, and m4 were already on a plate and stay there (living, bedroom, kitchen, bedroom).

m3 is the one that does not land on a plate. The old goal was 80% off the plates, stand in the kitchen, score 2.86. That one loses. The goal that replaces it is a tighter cluster, 51% off the plates, stand in the living room, score 1.91. Its cells sit close to their own center, so the rule leaves the length alone. That is the same case as a wall a yaw error has pushed off the plate.

The bath is the chosen frontier on two of the ten final poses (n1, and m5 as before). On the runs the loop used to win, a real room now wins instead, and on n2, n3, and n5 that room is the living room or the bedroom, which still outrank the bath. No new 600 s batch.

Dead ends: deleting every cell past a plate. Splitting every cluster, on-plate or not. The bedroom frontier on m4 is 37 m and mostly on the plate; a split of every long cluster would have cut that too.

## Live ten under the loop credit

The rescore above used maps the old score had already built. These ten are new 600 s runs, same seeds, flags off. North depth seed 64, spawn (5.80, 5.60, yaw 2.2). −0.35 depth seed 63, spawn (6.10, 4.70, yaw 1.2208). The controller is the floor-blank hold, the short-shift match, and the blocked-heading reverse-stop, plus the loop credit. Each run latched 15 floor plates. `oak_imu` was not a planner input. No `up_z` below 0.5. The lowest is 0.999.

Nine logs end `time_cap` still in `go`, except m2, which is in `spin` at the cap. n4's log stops at phase `go`, t=599.5 s, and `mapping_done` is 600.000; the reason line was not flushed. None log the mapper done line.

Visited %, then living, kitchen, hall, bath, bedroom, closet, laundry, pantry, storage:

| run | visited | liv | kit | hall | bath | bed | closet | lau | pan | sto |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 18.48 | 30.5 | 0 | 0 | 0 | 27.7 | 0 | 0 | 0 | 0 |
| n2 | 10.73 | 11.2 | 0 | 0 | 0 | 30.4 | 0 | 0 | 0 | 0 |
| n3 | 12.55 | 23.3 | 0 | 0 | 0 | 13.0 | 0 | 0 | 0 | 0 |
| n4 | 9.14 | 18.6 | 9.4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n5 | 20.69 | 23.8 | 0 | 0 | 44.9 | 26.0 | 1.2 | 0 | 0 | 0 |
| m1 | 14.97 | 25.6 | 32.8 | 0 | 0 | 0.1 | 0 | 0 | 0 | 0 |
| m2 | 14.14 | 27.2 | 19.1 | 0 | 0 | 0.6 | 0 | 0 | 0 | 0 |
| m3 | 7.96 | 8.1 | 0 | 0 | 0 | 22.8 | 0 | 0 | 0 | 0 |
| m4 | 9.57 | 7.5 | 0 | 0 | 0 | 32.1 | 0 | 0 | 0 | 0 |
| m5 | 11.40 | 22.2 | 15.4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

North mean 14.32, sd 5.02. The pre-fix five were 10.15, 5.24, 14.34, 12.87, 23.44 (mean 13.21, sd 6.69). −0.35 mean 11.61, sd 2.97. The pre-fix five were 16.17, 9.54, 17.92, 14.78, 13.42 (mean 14.37, sd 3.17). Both gaps sit inside that scatter.

Bath visited is 0 on nine runs. n5 is 44.9, and its walk-in is 1.2. The pre-fix bath entries were m3 at 34 and m5 at 16; those two are 0 here. Pantry and storage are 0 on all ten. Hall and laundry are 0. The goal stand, warped with the seed spawn, sits on a floor plate for most of the 600 s. The longest stretch with the stand more than 0.35 m off every plate is m2 at 125 s, then n5 at 76 s. n2's stand sits in the bath for 296 s and the shell never drives on that floor.

Outcome: the night is in-room stands, not the outside loop. Overall visited does not move past the old scatter, and the bath is entered on one run of ten. No further scoring change.

## A bath stand with the shell held still

Same ten logs. No new run. n2 holds one goal from explorer t=317 s to the cap, 283 s. Warped with the seed spawn, that stand is (1.07, 2.50), inside the bath, about 0.8 m west of the east wall and 1.1 m from door H. The true shell at that moment is (0.77, 4.92), in the bedroom, 0.38 m north of the bath floor at the bedroom walkway. Pose error is 0.24 m, so the stand is not a spawn-warp illusion and the shell is not somewhere else. From there to the cap the true path is 0.04 m. The pile summary is 314 s, command hold or zero on 100% of ticks, `cmd_m` 0, `wheel_m` 0, `pose_m` 0, and the goal at the end is still that stand. The lines just before the zeros are `blocked reverse did not move the shell` and `blocked turn did not yaw`.

The other bath, hall, and laundry stands are a different shape. A stand held 20 s or more in one of those rooms, other than n2: the shell is in another room, 1.0–2.5 m from the target floor, and it is still driving (zero command under 3% of the phase lines). Hall is selected for at most 45 s (n3). Laundry for 49 s (n5), while the shell is already in the bath and moving. Those goals change before the shell arrives. n5 is the run that does drive the bath, 44.9%.

m3, n4, m4, and m5 also sit for 232–414 s with a zero command. Their pile summaries that flushed do not keep a bath stand; m3 and m4 end with no goal. The n2 case is the one where the unvisited room's stand stays selected through the sit.

Dead ends: calling n2 a stop 0.19 m short of door H. The shell is at the other opening, the bedroom walkway, and then the command is zero rather than a creep that quits at the jamb.

Outcome: the stand is a real point in the bath. The shell is in the bedroom, and after the blocked turn does not yaw the command stays zero until the cap, with that goal still selected. No controller change.

## The dead reverse was requested again

The two lines are one path. `_respect_escape_block` sends the existing reverse along a blocked heading. When that reverse has not moved the planner pose for the 2 s stall window, it logs `blocked reverse did not move the shell` and does not send the reverse again. `_block_turn_away` then tries one yaw, and the other way. Each quiet try is the same 2 s window. The second quiet try logs `blocked turn did not yaw`. A straight reverse after that returns wheel speeds `(0, 0)`.

What kept the zero until the cap: `_leaving_command` and a forced pile both send that straight reverse on the next tick. The block turns it into zero again, so the shell never moves the 0.70 m that would drop the leave, and `_go` never reaches the path it already planned. n2's path was still 4.11 m, with the bath stand still selected. m3, n4, m4, and m5 are the same branch. Each logs the failed reverse, then `blocked turn did not yaw`, then `pile, leaving the spot`, and the long sit is `src=pile_leave` with the command at zero. m3 and m4 end that sit with no goal. n2 is the one that keeps the bath stand.

The correction does not add a motion. On that zero return, the leave latch and the forced pile are dropped, and the pile clock starts again. An empty path replans on the next tick. A path that is already there is driven by the existing pursuit. A forward the caller chose is still left alone. The 2 s clocks are unchanged.

## Live ten after the dead reverse is released

Same seeds, flags off, 600 s, on `7d07869`. Each run latched 15 floor plates. All ten end `time_cap` still in `go`. None log the mapper done line. No `up_z` below 0.5. The lowest is 0.999.

Visited %, then living, kitchen, hall, bath, bedroom, closet, laundry, pantry, storage:

| run | visited | liv | kit | hall | bath | bed | closet | lau | pan | sto |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 17.74 | 12.6 | 20.1 | 1.1 | 0 | 49.9 | 0 | 0 | 0 | 0 |
| n2 | 12.81 | 22.7 | 25.5 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n3 | 22.73 | 35.8 | 0 | 0 | 31.7 | 18.3 | 0 | 0 | 0 | 0 |
| n4 | 14.58 | 22.6 | 0 | 0 | 0 | 24.9 | 0 | 0 | 0 | 0 |
| n5 | 18.62 | 25.3 | 4.7 | 0 | 0.1 | 36.7 | 0 | 0 | 0 | 0 |
| m1 | 17.27 | 23.0 | 1.5 | 0 | 0 | 36.8 | 0 | 0 | 0 | 0 |
| m2 | 19.86 | 12.1 | 32.3 | 49.2 | 56.5 | 11.7 | 0 | 0 | 0 | 0 |
| m3 | 15.34 | 34.2 | 3.6 | 0 | 0 | 0.8 | 0 | 0 | 0 | 0 |
| m4 | 11.18 | 16.7 | 32.8 | 0.9 | 0 | 0 | 0 | 0 | 0 | 0 |
| m5 | 9.14 | 7.3 | 0 | 0 | 0.2 | 30.2 | 0 | 0 | 0 | 0 |

North mean 17.30, sd 3.84, against 14.32 (sd 5.02) on the previous live ten. −0.35 mean 14.56, sd 4.38, against 11.61 (sd 2.97). Both gaps are about 3 points. Pantry, storage, laundry, and the walk-in are 0. Bath is entered on m2 (56.5) and n3 (31.7). Hall is entered on m2 (49.2).

`blocked reverse did not move the shell` appears on every run. `blocked turn did not yaw` appears on n1, n2, n3, m3, n4, m4, and m5. After that second line the next phase command is not zero: n2 is a turn at t=415.5, two seconds later. No run has a `src=pile_leave` stretch with both v and w at zero. n2's following pile is 209 s of pile-leave and stall, goal none, and the true shell moves 0.6 m. m4 and m5 spend the late run in `src=stall` with a turn or a zero and almost no translation (m4's longest v=w=0 stall stretch is 101 s). That is not the old pile-leave zero that kept the bath stand.

Outcome: the 314 s zero with the bath stand held did not repeat, and neither did the 232–414 s pile-leave zeros. The shell still does not finish the apartment. No further change.

## Stall zero and the trail spin

Same ten logs. No new run. `src=stall` is `_stall_escape`. It backs up, then turns, then drives a short way. The tag is set before `_respect_escape_block` sees the speeds.

m4, from the turn failure at t=257 s to the cap, is that stall. 304 of 312 pile lines are v=0 and w=0. Eight are a turn at w=0.30. The longest run of those zero lines is 101 s, t=449.7 s to t=550.2 s. The last `stall, backing out` is t=242.9 s, then the failed reverse and both quiet turns. From t=242.9 s to t=246.4 s the wheels' translation is 0.30 m. After t=257 s it is 0.000 m, and 95% of the wheel-command samples are (0, 0). The rest are pivots shorter than half a second, w=0.30 or w=±0.70. The back phase hands off to a turn when the reverse is closed, when trusted wheel travel hits 1 m, or when 0.8 s have passed with under 0.02 m of that travel. The travel from the reverse that was still allowed sits above 0.02 m, and once the command is zero it stops there, under 1 m and under the 0.30 m that would end the stall. The handoff does not come. The block turns the next reverse into zero as well. Planner yaw over that window changes by 0.00 rad. The true path is a few centimetres. The goal is none.

m5 is the same stall, from the same two lines at t=190 s. 226 pile lines are a turn (w=0.30, −0.70, or −0.30) and 176 are the zeroed reverse. Of the wheel samples, 56% are those pivots and 44% are (0, 0). None is a drive. Planner yaw changes by −0.02 rad. The true path is 0.9 m. A stall turn whose heading has not moved 0.2 rad in 4 s flips the spin and starts that clock again. That check comes before the branch that would drive after 4 s, so the flip never lets the drive start.

n2's 209 s pile is not that branch. After the blocked turn, the phase command at t=415.5 s is already a turn. Of 149 pile lines from t=414 s, 122 are `src=pile_leave` and 27 are stall. The pile-leave branch after that time is `trail_reverse` 120 times and `straight_forward` 3 times. `trail_reverse` is a ±0.50 spin when the crumb is off to the side, with v=0. The goal is none and the path is empty. Planner yaw changes by 0.01 rad, so the spin never lines up and never becomes a drive. The true path is 0.6 m.

Dead ends: calling n2 another stall zero. The speeds are a spin, the source is pile-leave, and the goal was already dropped. Calling the m4 zero a pile-leave latch. The latch from `7d07869` is clear; this reverse comes from the stall's back phase.

Outcome: two sits. Stall keeps requesting a reverse the block has already refused, and its own 0.8 s handoff does not see that as no motion. The n2 pile is the trail spin, which does not yaw, so it does not start driving. No controller change.

## A refused stall reverse moves to the turn

When the block turns that stall reverse into (0, 0), the back phase moves to the turn the stall already has. The same step the 0.8 s handoff uses. Trusted travel is left where it is. The next tick asks for that turn, which the block does not rewrite. No new speed and no new spin. The pile-leave `trail_reverse` spin is unchanged. m5's 4 s flip, which restarts before the drive, is unchanged. Mapping tests: 157 passed. No new 600 s batch.

## Live ten after the stall reverse advances

Same seeds, flags off, 600 s, controller `4dc9e25`. All ten latch 15 plates and end `time_cap` in `go`. No mapper done line. Lowest `up_z` is 0.975 (m5). None go below 0.5.

| run | visited | living | kitchen | hall | bath | bedroom | closet | laundry | pantry | storage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 19.48 | 28.7 | 6.7 | 0 | 0 | 32.7 | 0 | 0 | 0 | 0 |
| n2 | 11.66 | 17.7 | 12.1 | 73.9 | 0 | 0 | 0 | 0 | 0 | 0 |
| n3 | 15.75 | 31.8 | 16.2 | 2.0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n4 | 12.18 | 24.3 | 14.1 | 0.2 | 0 | 0 | 0 | 0 | 0 | 0 |
| n5 | 16.60 | 29.3 | 15.7 | 61.4 | 0 | 0 | 0 | 0 | 0 | 0 |
| m1 | 12.59 | 18.8 | 0 | 0 | 0 | 23.1 | 0 | 0 | 0 | 0 |
| m2 | 14.13 | 16.2 | 59.3 | 0 | 0 | 0.1 | 0 | 0 | 0 | 0 |
| m3 | 16.44 | 21.1 | 6.2 | 0 | 0 | 33.9 | 0 | 0 | 0 | 0 |
| m4 | 16.39 | 6.9 | 0 | 45.3 | 52.7 | 28.5 | 0 | 0 | 0 | 0 |
| m5 | 12.07 | 7.4 | 0 | 0 | 4.5 | 42.2 | 0 | 0 | 0 | 0 |

North mean 15.14, sd 3.25, against 17.30 (sd 3.84). −0.35 mean 14.33, sd 2.05, against 14.56 (sd 4.38). Both gaps are inside the earlier scatter. Bath is entered on m4 (52.7) and m5 (4.5). Hall on n2 (73.9), n5 (61.4), and m4 (45.3). Pantry, storage, laundry, and the walk-in are 0.

No `src=pile_leave` stretch has both v and w at zero. The three runs that log `blocked reverse did not move the shell` and then `blocked turn did not yaw` do not stay at zero on the next phase line: m1 drives `(1.50,1.50)` at t=483.5, m2 turns `(1.49,-1.49)` at t=455.5, and m5 drives at t=273.5 and turns at t=521.5. m4's 101 s stall zero does not repeat. m2 is the run that still sits. After that turn the phase command is zero, and stall pile lines stay v=0 w=0 for 38 s (t=493.3 to t=531.5), then one line at w=0.30, then zeros again through t=596.5 except one w=−0.30 at t=561.5. The goal is none. From t=490 s the true path is 0.00 m. Wheel commands in that window are zero on 99% of samples, with brief w=±0.30 pivots. The turn was sent and the heading did not leave.

`trail_reverse` is 33 lines on n2 (span 16 s) and 346 lines on m2 (span 11 s). The 209 s no-goal trail spin does not repeat. m5's late window moves: true path 6.4 m from t=510 s to the cap, so the 4 s flip does not hold the shell on this draw. No further change.

## m2's brief pivots are not a pin

Same m2 log. From t=490 s to the cap the true pose stays at (8.147, 0.888), z=0. True yaw goes from +2.937 to +2.936 rad, a change of −0.0007 rad. Planner yaw changes by −0.0014 rad. `up_z` is 1.000 and both hubs stay at z=0.080. Every physics step has a floor contact. Wall-contact steps are 0, so no logged part of the shell is touching a wall, the counter, or furniture.

The left-rear tire rim is 3.5 cm from `APT_FURN_COUNTER`. The right-rear tire is 6.5 cm from `APT_WALL_EAST`. The deck box is 10.3 cm from the counter. The sonar face at 0.227 m, which has no bounding object, crosses that counter box by 8 mm. The shell is sitting on the floor beside the counter. It is not held by a contact.

A held w=0.30 pivot just before this window, t=451.04 s to t=454.13 s, turned the wheels 1.97 rad on each side, matching the command, and the true yaw changed by +0.45 rad. The planner yaw changed by the same amount. That rate is enough to yaw the shell. After t=490 s there are 171 pivots, all w=±0.30 (wheel commands ±0.637 rad/s). Each lasts 8–24 ms, one to three steps. The wheel positions do not move on those steps. None of them is a w=±0.70 stall turn.

The branch is `_stall_escape` after both `_spin_safe` checks fail on the ±0.70 spin. The phase is set back to `back`. A rear that reads closed returns the existing w=±0.30 pivot for that step. Otherwise the function returns the straight reverse, and `_respect_escape_block` turns it into (0, 0) because that reverse was already refused. `_advance_stall_turn` arms the turn again, and the next step repeats. The block does not rewrite the pivot. The 4 s yaw flip does not run, because the safe spin is not held. The stall does not end, and the source stays stall, so the pile clock is not the command. The pivot is too short to yaw. The same shape for 20 s or longer appears on m2 only, from t=456 s to the cap. No controller change.

## The small pivot stays on when the reverse is refused

Commit `8f9efb0`. When both ±0.70 spins fail `_spin_safe` and `_block_reverse_dead` is set, the stall stays in the turn and holds `_corner_pivot`: 0.40 rad/s if that sweep is clear, otherwise 0.30 rad/s toward the open side. The phase clock is not reset, so the existing yaw target and the 4 s limit still end the turn. A reverse that has not been refused still drops back. Mapping tests: 158 passed.

## Live ten with the small pivot held

Same seeds, flags off, 600 s. All ten latch 15 plates. Eight log `time_cap` in `go`. n1 and m4 reach t=599.5 s and the process exits 0 without an `end_run` line; n1's last phase is `spin`, m4's is `go`. No mapper done line. Lowest `up_z` is 0.999. None go below 0.5.

| run | visited | living | kitchen | hall | bath | bedroom | closet | laundry | pantry | storage |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 16.96 | 17.2 | 14.5 | 0 | 26.6 | 23.6 | 0 | 0 | 0 | 0 |
| n2 | 19.60 | 25.3 | 21.6 | 1.1 | 0 | 30.8 | 0 | 0 | 0 | 0 |
| n3 | 19.28 | 33.5 | 19.5 | 35.7 | 0 | 0 | 0 | 31.3 | 0 | 0 |
| n4 | 20.81 | 19.5 | 23.3 | 59.8 | 21.4 | 24.8 | 0 | 0 | 0 | 0 |
| n5 | 13.26 | 20.7 | 29.0 | 6.9 | 0 | 2.2 | 0 | 0 | 0 | 0 |
| m1 | 19.33 | 20.2 | 8.6 | 0 | 0.1 | 49.0 | 0 | 0 | 0 | 0 |
| m2 | 20.22 | 27.7 | 0 | 0 | 7.5 | 36.0 | 5.3 | 0 | 0 | 0 |
| m3 | 18.38 | 19.8 | 0.2 | 0 | 0 | 50.5 | 0 | 0 | 0 | 0 |
| m4 | 31.44 | 26.7 | 0 | 39.0 | 36.2 | 59.7 | 4.6 | 60.3 | 0 | 0 |
| m5 | 13.55 | 16.5 | 12.7 | 40.2 | 0 | 0.9 | 0 | 80.2 | 0 | 0 |

North mean 17.98, sd 2.99, against 15.14 (sd 3.25). −0.35 mean 20.58, sd 6.59, against 14.33 (sd 2.05). The −0.35 sd is wide because m4 is 31.44; the other four are 19.33, 20.22, 18.38, and 13.55. Bath is entered on n1 (26.6), n4 (21.4), m2 (7.5), and m4 (36.2). Laundry on n3 (31.3), m4 (60.3), and m5 (80.2). Pantry and storage are 0.

No run has a zero-command stretch of 20 s with the true shell still. The longest exact zero is 14 s on m2. The m2 shape from t=456 s, a refused reverse ping-ponging with 8–24 ms pivots, does not repeat. No further change.

## The cap line and what the ten was still doing

n1 and m4 reach t=599.5 s, print `Terminating`, and have no `end_run` line and no traceback. Both `mapping_done` files contain `600.000`, so the time-cap branch did run. The pose logger calls `simulationQuit` as soon as that file exists, and Webots dropped the controller's stdout from that step. Eight of the ten still show the line. `record_end_run` now writes `end_run.txt` and flushes it before `mapping_done` is created. The motion code is unchanged.

The cap is what stops the run. Over the last 120 s the true path is 10.9, 9.1, 9.5, 8.3, 9.9 m on the north runs and 10.0, 6.1, 12.4, 10.6, 3.1 m on −0.35. That is 0.026–0.103 m/s. Share of the 600 s:

| run | stall | pile_leave | no goal | go with progress | waste of 10 s |
| --- | ---: | ---: | ---: | ---: | --- |
| n1 | 0.03 | 0.10 | 0.11 | 0.56 | none |
| n2 | 0.00 | 0.02 | 0.04 | 0.57 | none |
| n3 | 0.01 | 0.04 | 0.04 | 0.60 | none |
| n4 | 0.02 | 0.15 | 0.22 | 0.48 | none |
| n5 | 0.00 | 0.23 | 0.23 | 0.55 | none |
| m1 | 0.09 | 0.06 | 0.21 | 0.58 | 49 s of spin |
| m2 | 0.04 | 0.07 | 0.12 | 0.49 | 52 s, mostly spin |
| m3 | 0.01 | 0.12 | 0.17 | 0.57 | none |
| m4 | 0.00 | 0.02 | 0.01 | 0.73 | none |
| m5 | 0.13 | 0.09 | 0.27 | 0.39 | 100 s of spin |

m5 is the slow end (3.1 m in the last 120 s) and the long spin. The other nine are still covering ground when the clock stops.

The largest unvisited reachable floor is the living room, 21.3 m², with about 17–18 m² still unvisited on m5 and n1 and about 99% seen. The interior is free on the map, so it is not a frontier. On m5 the longest remaining frontier is a 6.2 m bath edge (124 cells). The living edge is 4.0 m, mixed with cells off the floor, and ranks behind that bath edge. m5's last goal, t=558 s, is in the bath. On n1 the last four goals are in the bath, and the longest frontier is a 2.7 m kitchen edge. A seen room does not keep a frontier, and the capped unvisited stand loses to those unknown edges. Pantry and storage stay closed.

## An 1800 s cap still does not finish

Three runs of each seed, cap 1800 s, same worlds and flags. No mapper done line. All six write `end_run.txt` with `time_cap`. Lowest `up_z` is 0.998. Map snapshots are the save just after 600, 1200, and 1800 s. The pair in each cell below is seen % / visited %.

| run | 600 s | 1200 s | 1800 s | frontiers at the cap | stall | pile_leave | no goal | progress |
| --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| n1 | 83/10 | 87/12 | 88/12 | 47, longest 4.0 m living, 26 of 79 cells off the floor | 0.78 | 0.01 | 0.79 | 0.12 |
| n2 | 91/16 | 91/24 | 91/24 | 41, longest 7.0 m bath, 9 of 140 off the floor | 0.54 | 0.04 | 0.59 | 0.25 |
| n3 | 93/20 | 98/30 | 98/31 | 36, longest 4.4 m laundry | 0.05 | 0.07 | 0.14 | 0.46 |
| m1 | 91/15 | 85/16 | 21/17 | 11, longest 35.8 m after the map leaves the floor | 0.12 | 0.20 | 0.38 | 0.38 |
| m2 | 80/16 | 96/27 | 97/28 | 37, longest 6.1 m bedroom, 48 of 121 off the floor | 0.06 | 0.05 | 0.11 | 0.57 |
| m3 | 90/12 | 98/23 | 98/40 | 47, longest 4.4 m bedroom | 0.06 | 0.06 | 0.15 | 0.49 |

Rooms at 1800 s, seen/visited. n1: living 99/19, bath 21/0, bedroom 91/18, closet 86/0. n2: living 98/35, bath 67/0, bedroom 93/46, closet 100/0. n3: living 99/17, bath 93/62, bedroom 99/35, closet 99/0, laundry 93/34. m2: living 99/11, bath 91/68, bedroom 97/79, closet 97/0. m3: living 97/47, bath 97/63, bedroom 98/39, closet 99/0. Visited is flat after 1200 s on n1 and n2. The others still gain a few points.

n2 from t=805 s to the cap commands the stall's w=−0.30 pivot on 94% of samples. The wheels match the command. True path is 0.52 m and yaw does not change. Bath visited stays 0, and the 7 m bath edge is still there. n1's command from t=450 s is a stall pivot as well (w=+0.30 on 60%, w=−0.70 on 28%). The planner moves 0.16 m. The true shell creeps 8.0 m and the pose error grows from 1.4 m to 2.6 m. m1's seen percent falls from 91 at 600 s to 21 at 1800 s, with pose error 1.1 m, so the 35.8 m edge is the spawn warp of a map that has left the floor. Other waste of 10 s or more is spin on m2 (1535–1623 s), n3 (1325–1385 s and 1699–1736 s), and m3 (longest 309–361 s), plus a 36 s zero on m2.

What stays alive is an unknown edge on a real room, sometimes with a share of its cells off the floor plate. n2's is the bath. n3's is the laundry. m3's is the bedroom. n1's late edge is a living-room strip with a third of its cells off the floor. The walk-in is seen and not driven. The shell does not run out of frontiers in 1800 s.

## The pose walks off, and n2's pivot never ends

Same 1800 s logs. The 8.0 m figure for n1 was the sum of every 8 ms step. At 1 s samples the path is 3.5 m and the net move is 2.1 m, of which 1.9 m is forward in the body frame and 0.9 m is to the side. Yaw changes by −0.04 rad. The shell starts at (3.40, 5.22) in the bedroom, 41 cm from `APT_WALL_SHARED_C`, and ends at (1.38, 4.71), still in the bedroom, 26 cm from `APT_WALL_BED_SOUTH_B`. Hubs stay at ride height.

From t=450 s, 99.8% of wheel commands are a pivot, so `spin_cmd` is true and `trust_translation` is false on those steps. The encoders' own translation over the whole window is 0.26 m. `floor still, translation frozen` appears 106 times. There is no loop closure, no floor-slip line, and no blocked-reverse line in that window. The floor gate only holds or releases wheel travel, and the wheels never reported the 3.5 m. The step that drops the motion is the pivot rule in `odometry.update`.

m1 is the same kind of miss and then a rebuild. Pose error is 0.39 m at t=1200 s, when seen is 85%. At t=1209 s the log is `stall, backing out` and `blocked reverse did not move the shell`, and from there through t=1400 s the true path is 4.5 m while the planner path is 2.0 m and the error is 1.09 m. In that stretch the log has `floor slip, wheel motion rejected` and `floor still, translation frozen` on the same ticks as the blocked heading. Loop closure then resets the grid and replays keyframes at t=1435 s, t=1699 s, and t=1792 s. The closure at t=1435 s leaves the planner at (−3.60, −1.52); the true shell is (5.42, 1.41), about 1.0 m away in the spawn frame. Seen percent at the 1800 s snapshot is 21. n1 never rebuilds, so its seen percent stays near 88 while the live pose is the part that is wrong.

n2 sits at (7.29, 8.07) in the living room from t=805 s to the cap, yaw +2.754. The right-rear tire is 1.6 cm from `APT_FURN_SOFA` and the right-front tire is 2.8 cm from `APT_WALL_NORTH_B`. The left-rear tire is 11 cm from the sofa. Over those 1000 s the right wheel is the only part with a wall contact, on 2.4% of steps. `up_z` stays 0.999. A one-minute slice of the pivot has wheel rotation 76.69 rad against 76.69 rad of command. The wheels are slipping on the floor. The sofa is close, and the contact is a brush, not a wedge that locks the shell.

The commanded speed is `_corner_pivot`, not `_escape_spin`. Every 4 s, `yaw_stuck` flips `_escape_spin` and sets the phase clock back to now, so the branch that would drive after 4 s does not run. The yaw target of 0.8 rad is never met. Stall travel does not grow because a pivot sets `applied` to 0. The sign of the 0.30 rad/s pivot comes from the side gaps and does not change, which is why 94% of the commands are the same wheel pair. No controller change.

## The small pivot hands off, and two runs sit on a blocked reverse

Commit `1a66938`. When the stall turn is sending `_corner_pivot` and yaw has not moved 0.2 rad in 4 s, the phase becomes `drive` and the existing drive code runs. Flipping `_escape_spin` stays the path for the ±0.70 spin, which is the command only when that spin is safe. Mapping tests: 161 passed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Lowest `up_z` is 0.998, on n2 and m3.

North visited 12.30, 18.61, 15.15, 20.72, 17.42 (mean 16.84, sd 3.24) against 17.98 (sd 2.99). −0.35 visited 25.34, 17.28, 8.12, 17.92, 5.98 (mean 14.93, sd 7.89) against 20.58 (sd 6.59). Pantry and storage are 0. Bath is 47.5 and 47.3 on n2 and n3, and 26.9 on m1. Hall is 52.1 on n4 and 69.3 on m1. Laundry is 38.0 on n4. Closet is 31.6 on m1. The other bath, hall, laundry, and closet figures are 0.

Pose error at the cap: n1 0.31, n2 1.94, n3 1.26, n4 0.94, n5 0.63, m1 0.71, m2 2.08, m3 0.35, m4 0.71, m5 0.56.

The longest stretch of a pivot command whose yaw stays within 0.2 rad of the start is 5.2 s on n1, 4.0 s on n2, 5.0 s on n3, 2.6 s on n4, and 1.8 s on n5. On −0.35 it is 12.0 s on m1, 47.0 s on m2, 21.6 s on m3, 17.4 s on m4, and 9.4 s on m5. m2's 47 s is the ±0.70 pair, 49% `(1.49, -1.49)` and 47% `(-1.49, 1.49)`, pile `src=stall`. m1's 12 s and m4's 17 s are the ±0.30 pair, both signs. m3's 22 s is 71% the ±0.30 pair. The log on m3 and m5 shows the handoff: `src=stall` goes from `w=0.30` to `v=0.12` within a few seconds.

The 10 s wastes that are not the opening survey spin:

- n1, 26 s of spin from t=201 s, `pile_leave` and `stall`. n1, 10 s of spin from t=514 s, `pile_leave`.
- n3, 29 s of the ±0.70 spin from t=570 s to the cap. Yaw change +0.10 rad. The log is `blocked turn did not yaw, other way`.
- n5, 16 s of `(0, 0)` from t=332 s, `src=hold` then `pile_leave`, and another 14 s of `(0, 0)` from t=364 s.
- m2, the 47 s ±0.70 stall above, then 12 s more of the same spin from t=526 s.
- m3, 332 s of `(0, 0)` from t=268 s to the cap. Pile `src=back`, `v=0`. The line just before is the stall drive, then a goal, then `src=back`.
- m4, 43 s of `(0, 0)` from t=557 s to the cap.
- m5, 354 s of `(0, 0)` from t=245 s to the cap. Pile `src=back`, `v=0`, after the stall drive and a new goal.

The ±0.70 flip is unchanged, and it is what fills m2's long flat spin. The two long zeros are a reverse the block has already refused. The tag stays `back` and the speeds stay `(0, 0)`.

## The path reverse is still requested after the block zeros it

No controller change. On m3 and m5 the stall drive does hand off, a goal is taken, and the command is then `(0, 0)` with pile `src=back`.

m5: at t=236–239 s the stall is `w=±0.30`. At t=240.3 s it is `v=0.12`, the drive. At t=242.0 s the goal is (−0.72, 3.83). At t=242.3 s the pile is `src=back v=0.00`, and it stays there until the cap, 354 s, 353 lines. The goal's bearing is 1.49 rad off the planner yaw, so it is not ahead. The true shell stays at (3.91, 5.00) in the living room, yaw −1.36. The right-front tire is 13.7 cm from `APT_WALL_SHARED_B`. The other tires are 35–49 cm from that wall or `APT_WALL_SHARED_C`. No wall contact on those 354 s. `up_z` is 1.000 and the hub stays at z=0.

m3: the dead reverse was logged at t=223.0 s and `blocked turn did not yaw` at t=231.8 s. At t=263.4 s the stall is `v=0.12`. At t=264.2 s the goal is (−0.71, 3.56). At t=264.4 s `src=back v=0.00` starts and runs 332 s. The goal bearing is 1.20 rad off the yaw. The true shell stays at (4.14, 2.39) in the kitchen, yaw −2.87. The right-front tire is 22.5 cm from `APT_WALL_SHARED_B`. The others are 33–53 cm out. No wall contact. `up_z` is 1.000.

The speeds come from `_continue_back`, which returns `wheel_speeds(-0.10, 0)` while `_backing` is set. `_go` marks that `back` before `_remember` runs. `_respect_escape_block` sees a straight reverse on a heading whose reverse is already dead and whose two turns were already quiet, sets the body speeds to 0, and calls `_release_dead_reverse`. There is no `blocked heading` line during the sit. That line is logged only while the reverse is still being substituted, which is before it is marked dead.

`_release_dead_reverse` clears `_leave_xy` and `_pile_forced`, replans only when the path is empty, and calls `_advance_stall_turn` only when `_blind_back` is set and the stall phase is still `back`. This caller is the path's `_backing` latch, after the stall has left that phase. The latch ends when the pose moves 1 m, the clearance opens, the rear closes, or the goal is ahead. None of those happen while the command is zero, so the next tick asks again. `4dc9e25` is that stall advance. It does not run here.

The saved live tens for `8f9efb0` (`hold_pivot`), `4dc9e25` (`stall_turn`), and `7d07869` (`release_live`) have no `src=back v=0.00` stretch of 20 s. The code path is older than `1a66938`. Those runs did not reach it. The drive handoff is what gets m3 and m5 to a goal, and the goal path then requests the reverse the block already refused.

m2 from t=538 s to t=585 s is not the 4 s flip. The wheel command swaps between `(1.49, -1.49)` and `(-1.49, 1.49)` every step or two. Of 2311 runs of one sign, 1498 last one or two steps, and the longest is 0.78 s. The 4 s flip would hold one sign. This is the turn's first check: `_spin_safe` rejects the spin just sent, the opposite sign is sent the same tick, and the next tick rejects that one. The 1 Hz pile line shows one sign for several seconds only because it samples a single tick.

## The path latch is cleared with the dead reverse

Commit `a1604df`. `_release_dead_reverse` clears `_backing` and `_back_from`. `_begin_back` does not start on a heading whose reverse is already dead. Mapping tests: 162 passed. The ±0.70 same-tick swap was not changed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Lowest `up_z` is 0.997, on n1 and m3.

North visited 25.10, 17.70, 14.86, 14.95, 15.04 (mean 17.53, sd 4.40) against 16.84 (sd 3.24) after the handoff and 17.98 after `8f9efb0`. −0.35 visited 18.06, 17.93, 9.30, 21.40, 19.77 (mean 17.29, sd 4.69) against 14.93 (sd 7.89) and 20.58. Pantry and storage are 0. Bath is 4.6, 20.1, 34.0, 20.6, and 57.7 on n1, n2, m1, m2, and m4. Hall is 83.4 on n3 and 58.4 on n4. Laundry is 11.5 on n3. Closet is 66.5 on m1. Bedroom is above 40 on n2, m1, m2, and m4.

Pose error at the cap: n1 0.67, n2 1.53, n3 1.27, n4 0.55, n5 2.15, m1 0.32, m2 0.38, m3 0.74, m4 0.29, m5 0.78.

The longest `(0, 0)` stretch is 10.1 s on n1, from t=95 s to t=106 s, while a stall is backing out. At t=107 s the wheels are `(-1.25, -1.25)` and the shell moves. The other nine runs' longest zero is 3.5 s at the start. No `src=back` zero of 20 s.

The longest ±0.70 sign-swap, commands alternating within one step or two, is 2.2 s on m1, 2.1 s on m3, m5, and n4, and 1.3 s on n2. n1, n3, n5, m2, and m4 have none. The 47 s loop from the previous ten does not appear.

## Where the pose error comes from

No controller change. The `a1604df` ten, position error only. Each number is the source's share of the final error vector, in metres. The four shares sum to the final error. Yaw at the cap is within 4° on every run. Loop closure is about 0 (one closure each on n4 and n5).

| run | final | pivot creep | floor slip | loop | drive |
| --- | ---: | ---: | ---: | ---: | ---: |
| n1 | 0.67 | −0.04 | 0.00 | 0.00 | 0.71 |
| n2 | 1.53 | −0.02 | 0.00 | 0.00 | 1.55 |
| n3 | 1.27 | −0.14 | 0.00 | 0.00 | 1.41 |
| n4 | 0.55 | 0.15 | 0.00 | 0.00 | 0.40 |
| n5 | 2.15 | 0.02 | 0.00 | 0.00 | 2.13 |
| m1 | 0.32 | 0.15 | −0.13 | 0.00 | 0.30 |
| m2 | 0.38 | 0.08 | 0.00 | 0.00 | 0.30 |
| m3 | 0.74 | 0.36 | 0.07 | 0.00 | 0.31 |
| m4 | 0.29 | 0.01 | 0.00 | 0.00 | 0.28 |
| m5 | 0.78 | −0.01 | 0.00 | 0.00 | 0.79 |

A negative share points against the final miss. Pivot creep is the part of the final miss from a pivot command while the true shell moved and the pose did not. It is the largest share only on m3. Floor-slip rejections are logged (50 times on m3, 0 on n2, n3, n5, m4, m5) and do not leave the miss.

The drive share is mostly the shell moving while the pose stays. The largest rises, with the phase and the pile src when one is open:

- n5 t=387 s, +0.34 m, `go`, no pile. t=54 s, +0.14 m, `go`. t=382 s, +0.13 m, `go`.
- n2 t=310 s, +0.28 m, `go`. t=197 s, +0.18 m, `go`. t=271 s, +0.16 m, `go`.
- n3 t=310 s, +0.16 m, `go`, `stall`. t=144 s, +0.15 m, `go`. t=313 s, +0.14 m, `go`, `hold`.
- n1 t=385 s, +0.12 m, `go`. t=233 s, +0.09 m, `go`.
- n4 t=560 s, +0.25 m, `go`, the pose running ahead of the shell.
- m5 t=441 s, +0.20 m, `go`. t=444 s, +0.17 m, `go`. t=246 s, +0.15 m, `go`.
- m1 t=564 s, +0.17 m, `go`, `back`. t=159 s, +0.08 m, `go`. t=598 s, +0.08 m, `go`, `stall`.
- m3 t=113 s, +0.12 m, `go`. t=21 s, +0.08 m, `go`.
- m4 t=389 s, +0.11 m, `go`. t=21 s, +0.10 m, `go`.
- m2 t=227 s, +0.11 m, `go`, `hold`.

On n5 from t=384 s to t=392 s the command is a straight reverse, wheel travel is −0.80 m, the true shell moves 0.80 m, and the pose moves 0.17 m. The forward nose range goes from 0.82 m to 1.45 m. The log repeats `pinned, depth floor and visual still, wheel translation frozen`. The same shape is on n3 from t=305 s (true 0.78 m, wheels −0.71 m, pose 0.08 m, `src=stall`) and on m5 from t=438 s (true 0.75 m, wheels −0.72 m, pose 0.12 m). A pin rewinds the pose and then holds translation for 1.2 s, and the next pin rearms. The part of the final error inside those holds is 0.09, 0.40, 0.89, 0.01, 0.50 m on n1–n5 and −0.01, 0.00, 0.11, 0.01, 0.65 m on m1–m5.

m3 visited 9.30 with pose error 0.74 m and no long zero. Phase samples are 97.7% `go` and 2.3% `spin`. Wheel commands are 55% drive, 39% pivot, 4% mixed, 2% zero. True path 28.2 m, planner path 25.4 m. A pile is open for 281 s: `pile_leave` 182 s, `stuck` 42 s, `stall` 31 s, `hold` 11 s, `pivot` 9 s, `contact` 5 s. It adopts 17 goals. Warped, 6 are in the bedroom, 5 are off the floor plates, 3 are in the kitchen, 2 in the hall, 1 in the laundry. The true shell never comes within 0.6 m of any of them. The closest miss is 1.16 m. Visited floor is the living room at 20% and the kitchen at 5%.

The change that cuts the pose error, not made here: do not pin a straight reverse when the forward range has increased since the pin anchor. The wheels and the nose already show that reverse. m3's visited score is the goals it never reached, not this 0.74 m.

## The opening nose is not a pin

Commit `5a546bf`. A straight reverse whose forward range has risen more than 0.04 m since the pin anchor is not rewound and is not held for 1.2 s. That margin is six standard deviations of the depth noise at 1 m and more than two at 1.5 m. The frozen reverses in the previous ten rose by several tenths of a metre. A flat range still pins. A pitch more than 0.12 rad from the learned rest still pins. Mapping tests: 165 passed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Lowest `up_z` is 0.996, on m3. No sample is below 0.95.

North visited 17.92, 13.60, 15.94, 24.64, 21.26 (mean 18.67, sd 4.36) against 17.53 (sd 4.40). −0.35 visited 19.80, 13.96, 17.91, 20.02, 18.27 (mean 17.99, sd 2.44) against 17.29 (sd 4.69). Pantry and storage stay under 0.5. Bath is 28.3 on n5, 16.0 on m1, and 59.2 on m5. Hall is 19.4 on n5 and 55.4 on m5. Bedroom is above 40 on n4, n5, m1, m3, and m4.

Pose error at the cap: n1 1.28, n2 0.79, n3 0.95, n4 0.85, n5 0.35, m1 0.24, m2 1.19, m3 0.93, m4 1.06, m5 0.34. The previous ten was 0.67, 1.53, 1.27, 0.55, 2.15 and 0.32, 0.38, 0.74, 0.29, 0.78. The pin's share of that error is 0.12, −0.01, 0.01, 0.26, −0.00 on the north runs and −0.02, 0.19, 0.17, 0.18, −0.00 on −0.35. Previously it was 0.09, 0.40, 0.89, 0.01, 0.50 and −0.01, 0.00, 0.11, 0.01, 0.65.

The longest `(0, 0)` stretch is 24 s on n3, from t=70 s to t=95 s, phase `go`, command zero, nose about 0.64 m. The other nine are 3.5 s at the start.

## What is left after the pin release

No controller change. Same `5a546bf` logs. Shares below are metres of the final error from the steps outside a pin hold. Yaw at the cap is −1.7°, +1.1°, +1.2°, +2.8°, +1.4° on n1–n5 and −1.2°, −3.9°, +5.0°, +0.7°, −0.6° on m1–m5. Loop closure is one event on n4 and one on m1, and that share is 0.00 m.

| run | final | drive | reverse | pivot | stall | pile | zero |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 1.28 | +0.40 | +0.62 | −0.02 | −0.09 | +0.27 | +0.01 |
| n2 | 0.79 | +1.27 | −0.69 | 0.00 | 0.00 | +0.24 | −0.01 |
| n3 | 0.95 | +0.22 | +0.35 | +0.02 | 0.00 | +0.32 | −0.01 |
| n4 | 0.85 | +0.57 | −0.07 | +0.04 | 0.00 | +0.06 | +0.01 |
| n5 | 0.35 | +0.79 | −0.23 | −0.12 | −0.02 | −0.08 | −0.01 |
| m1 | 0.24 | 0.00 | +0.21 | +0.18 | 0.00 | −0.16 | +0.03 |
| m2 | 1.19 | +0.44 | +0.29 | +0.02 | −0.03 | +0.24 | +0.02 |
| m3 | 0.93 | −0.11 | +0.74 | +0.21 | −0.04 | −0.05 | 0.00 |
| m4 | 1.06 | +0.23 | +0.42 | +0.32 | −0.09 | −0.07 | +0.02 |
| m5 | 0.34 | +0.35 | −0.05 | −0.01 | 0.00 | +0.04 | +0.01 |

The outside-pin sum is not the final error. The pin hold is the rest. On the four large misses the mechanism is the same: the wheels and the true shell move together, and the pose does not. That rejected-translation share is +0.30, +0.69, +0.58, +0.23, +0.20 on n1–n5 and −0.54, +0.64, +0.26, +0.84, +0.44 on m1–m5. The pose-ahead share is smaller except on n1 (+0.38). A slide the wheels did not report is about 0. Floor-slip rejections are 9, 27, 0, 0, 0 and 0, 0, 149, 31, 0, and their share of the final error is about 0 except +0.18 on m4 and +0.07 on m3.

Largest rises outside a pin hold:

- n1 t=287–289 s, +0.18 m, drive. True 0.18 m, pose 0.02 m, wheels +0.20 m. Command `(-0.34, 2.84)`, an arc. t=182–183 s, +0.16 m, the same shape. t=536–538 s, +0.15 m, reverse, wheels −0.15 m, pose 0.05 m.
- m2 t=263–266 s, +0.29 m, reverse. True 0.30 m, pose 0.00 m, wheels −0.30 m. Nose goes from 0.54 m at t=261.5 s to 0.92 m at t=265.5 s while the pose stays at (−1.43, 2.61). The pin logs stop at t=261.5 s. `floor still, translation frozen` is at t=269 s, after the pose has already missed the reverse. t=71–73 s and t=21–23 s are +0.17 m drives, wheels matching the shell, pose under 0.04 m.
- m4 t=152–154 s, +0.20 m, drive. True 0.17 m, pose 0.04 m, wheels +0.20 m. `floor still` is at t=156 s. t=21–23 s, +0.17 m, same shape. t=141–142 s, +0.16 m.
- n3 t=352–354 s, +0.16 m, pile `src=hold`, command `v=0.10 w=0.75`. True 0.17 m, pose 0.00 m, wheels +0.20 m. `floor still, translation frozen` is at t=348.3 s. t=514–516 s, +0.15 m, reverse, wheels −0.15 m, pose 0.00 m. t=348–350 s, +0.15 m, same hold.

n3 from t=70 s to t=95 s is not that miss. The true shell stays near (4.47, 5.66), about 3 cm of motion, yaw +2.00. The planner stays at (0.61, 1.10). The phase command is `(0, 0)`. Nose stays about 0.64 m and the bubble about 0.40 m. There is no blocked-heading line. The previous decision is `back` at t=65.9 s. The next is a goal and `stall, backing out` at t=91 s. A straight or pivot approach does not count as a translation stall, so a shell that has already stopped there waits until the goal changes. The sit does not add pose error.

The next fix, not made here: a wheel step the two hubs agree on, and whose forward nose has moved more than the existing 0.04 m since the picture was called still, should not be dropped by the frozen-view check or rewound by the floor-still hold. A flat nose still drops. A pivot still drops. A pitched shell still drops. The test is a frozen depth signature with a straight reverse and a nose that opens by more than 0.04 m, which keeps the wheel step, against the same signature with a flat nose, which still drops it.

## An agreed step with a moving nose is kept

Commit `6b8ce43`. The frozen-view drop and the floor-still rewind both keep a wheel step the two hubs agree on when the forward nose has moved more than 0.04 m since the picture was called still. A flat nose still drops. Opposite hubs, and a commanded pivot, still drop. A pitch more than 0.12 rad from the learned rest still drops. The pin release, the floor-blank hold, and the weak-shift hold are unchanged. Mapping tests: 170 passed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Lowest `up_z` is 0.998, on m5. No sample is below 0.95.

North visited 16.42, 13.01, 17.52, 13.27, 10.15 (mean 14.07, sd 2.94) against 18.67 (sd 4.36). −0.35 visited 18.94, 16.10, 23.72, 13.19, 11.05 (mean 16.60, sd 4.97) against 17.99 (sd 2.44). Pantry and storage stay at 0. Laundry and closet stay at 0. Bath is 14.2 on m1 and 17.7 on m5. Hall is 58.4 on m1, 10.1 on n2, 5.6 on m3, 3.8 on n1, and 3.2 on n3. Bedroom is 45.0 on n4 and 49.9 on m2. Kitchen is 57.6 on n3.

| run | living | bedroom | kitchen | bath | laundry | closet | hall | storage | pantry |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 19.7 | 35.3 | 6.8 | 0 | 0 | 0 | 3.8 | 0 | 0 |
| n2 | 21.4 | 1.3 | 24.8 | 0 | 0 | 0 | 10.1 | 0 | 0 |
| n3 | 24.1 | 0 | 57.6 | 0 | 0 | 0 | 3.2 | 0 | 0 |
| n4 | 10.5 | 45.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n5 | 22.5 | 2.1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m1 | 17.0 | 31.6 | 13.2 | 14.2 | 0 | 0 | 58.4 | 0 | 0 |
| m2 | 14.6 | 49.9 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m3 | 36.6 | 20.0 | 31.8 | 0 | 0 | 0 | 5.6 | 0 | 0 |
| m4 | 27.3 | 7.3 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m5 | 7.4 | 29.2 | 0 | 17.7 | 0 | 0 | 0 | 0 | 0 |

Pose error at the cap: n1 1.60, n2 0.45, n3 0.37, n4 1.33, n5 0.28, m1 0.32, m2 1.49, m3 0.71, m4 0.35, m5 0.40. The `5a546bf` ten was 1.28, 0.79, 0.95, 0.85, 0.35 and 0.24, 1.19, 0.93, 1.06, 0.34. Yaw at the cap is +2.4°, +0.4°, −1.6°, −5.4°, +0.3° on n1–n5 and −2.4°, +0.1°, 0.0°, +2.4°, +0.6° on m1–m5.

The rejected-translation share, outside the 1.2 s after each `pinned` line, is the part of the final error from a 0.5 s step whose wheels moved more than 0.02 m, whose planner moved less than 0.4 of that, and whose true shell moved more than 0.5 of that. It is +0.61, +0.09, +0.83, +1.12, +0.21 on n1–n5 and +0.32, +0.51, +0.73, +0.05, −0.20 on m1–m5. The same gate on the `5a546bf` logs reads +0.30, +0.64, +0.64, +0.21, +0.22 and −0.43, +0.62, +0.26, +0.83, +0.44. The notes for that ten listed +0.30, +0.69, +0.58, +0.23, +0.20 and −0.54, +0.64, +0.26, +0.84, +0.44. n2 and m4 are the runs this keep cuts. n1, n3, n4, and m3 are larger.

The longest `(0, 0)` stretch is 55 s on n3, from t=67 s to t=122 s, phase `go`, true motion 0.6 cm. The command is zero after a reverse. The next decision that moves it is `pile_leave` at t=122 s, after `camera_stuck`. n4 sits 40 s from t=560 s to the cap, pile `src=hold`, bubble 0.02 m, dead-end refusals, true motion 0.3 cm. m4 sits 40 s from t=523 s to t=563 s, the same hold, bubble 0.03 m, true motion 2.2 cm. n5 sits 20 s from t=185 s to t=205 s and then logs `stall, backing out`. The other six stretches are 3.5 s at the start.

## The nose keep was a regression

Commit `6b8ce43` is reverted in `e929f76`. Visited means were 14.07 and 16.60 against 18.67 and 17.99. Pose-error means were 0.81 m and 0.65 m against 0.84 m and 0.75 m. The 55 s n3 zero and the 40 s n4 and m4 zeros came back. `5a546bf` stays the baseline. The opening-nose test is kept with the drop restored. A flat nose, a pivot, a pitched shell, and the floor-still rewind still do what they did before that commit, so those tests were dropped.

The n4 and m4 sits on that ten were a pivot whose issued command was `(0, 0)` with the path still set. `src=hold` is the mark on that zero, not a separate leave. The same 5 s clock covers them. The n3 24 s and 55 s zeros followed a `back` and then an empty path (`goal=none`). The approach was not straight or pivot, so the clock leaves that sit alone.

## A zero straight or pivot takes the stall

Commit `e929f76`. In phase `go`, a straight or pivot approach whose command has been `(0, 0)` for more than 5 s, with the same goal and a shell that has not moved 0.04 m, takes the existing stall escape (`stall, backing out`). A doorway crossing, the stall cooldown, or a reverse the block has already refused drops the goal instead. Mapping tests: 169 passed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Lowest `up_z` is 0.999, on m2 and m3. No sample is below 0.95.

North visited 15.56, 14.82, 8.80, 22.92, 17.67 (mean 15.95, sd 5.10) against 18.67 (sd 4.36). −0.35 visited 24.09, 18.63, 16.47, 27.00, 16.82 (mean 20.60, sd 4.70) against 17.99 (sd 2.44). Pantry, storage, laundry, and closet stay at 0. Bath is 20.4 on n5, 23.0 on m3, 13.9 on m2, and 2.1 on n2. Hall is 67.8 on n2, 48.2 on m1, and 49.2 on m2.

| run | living | bedroom | kitchen | bath | laundry | closet | hall | storage | pantry |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 33.0 | 5.6 | 2.5 | 0 | 0 | 0 | 0 | 0 | 0 |
| n2 | 12.2 | 25.3 | 16.1 | 2.1 | 0 | 0 | 67.8 | 0 | 0 |
| n3 | 6.0 | 31.8 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n4 | 23.5 | 66.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n5 | 9.0 | 58.4 | 0 | 20.4 | 0 | 0 | 0 | 0 | 0 |
| m1 | 31.1 | 14.4 | 51.3 | 0 | 0 | 0 | 48.2 | 0 | 0 |
| m2 | 18.0 | 31.5 | 10.3 | 13.9 | 0 | 0 | 49.2 | 0 | 0 |
| m3 | 17.0 | 32.4 | 0 | 23.0 | 0 | 0 | 0 | 0 | 0 |
| m4 | 42.5 | 45.3 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m5 | 16.0 | 51.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Pose error at the cap: n1 0.37, n2 1.03, n3 0.53, n4 0.50, n5 0.81, m1 1.31, m2 0.19, m3 0.51, m4 0.43, m5 1.62. The `5a546bf` ten was 1.28, 0.79, 0.95, 0.85, 0.35 and 0.24, 1.19, 0.93, 1.06, 0.34. Means are 0.65 m and 0.81 m, against 0.84 m and 0.75 m.

The longest `(0, 0)` is 243 s on n3, from t=357 s to the cap, phase `go`. The branch is contact recovery: `action=recover`, `contact_bump`, pile `src` flipping between `hold` and `contact`, goal `(2.23, 0.93)` unchanged, bubble about 0.5 m. That look returns `(0, 0)`. It is not a straight or pivot approach, so the 5 s clock does not take it. m4 sits 49 s from t=532 s to t=581 s after a `back` at t=527 s, then `goal=none` and `src=hold`, until `pile_leave` at t=577 s. That approach is `back`, so the clock leaves it. n1's 14 s from t=578 s to t=592 s includes `stall, backing out` at t=588 s. The other seven longest stretches are 3.5 s at the start.

## The zero clock covers contact and the empty path after a back

Commit `bc55896`. The same 5 s clock now also runs while contact recovery is sending `(0, 0)`, and while the path is empty after a back with `goal=none`. Expiry takes the stall escape. A blocked back-out takes the pile leave. If that cannot start, the goal is dropped. Contact uses the existing 2 s cooldown a reverse that did not move already sets, so the look does not start again on the next tick. The empty path uses the ordinary 4 s replan gap and clears the back latch. Mapping tests: 175 passed.

Five 600 s runs of each seed, flags off, against the `998e957` ten. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Lowest `up_z` is 0.998, on n3, m1, and m3. No sample is below 0.95.

North visited 11.86, 17.42, 14.62, 12.21, 18.39 (mean 14.90, sd 2.96) against 15.95 (sd 5.10). −0.35 visited 22.65, 13.78, 9.58, 16.77, 11.67 (mean 14.89, sd 5.09) against 20.60 (sd 4.70). Pantry and storage stay at 0. Closet stays at 0. Laundry is 4.5 on n1. Bath is 29.7 on n5. Hall is 59.3 on n1 and 18.2 on n3.

| run | living | bedroom | kitchen | bath | laundry | closet | hall | storage | pantry |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 19.4 | 0 | 10.0 | 0 | 4.5 | 0 | 59.3 | 0 | 0 |
| n2 | 25.5 | 33.2 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n3 | 27.5 | 0.2 | 16.5 | 0 | 0 | 0 | 18.2 | 0 | 0 |
| n4 | 14.1 | 31.3 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n5 | 8.6 | 57.0 | 0 | 29.7 | 0 | 0 | 0 | 0 | 0 |
| m1 | 33.3 | 38.8 | 6.7 | 0 | 0 | 0 | 0 | 0 | 0 |
| m2 | 13.7 | 40.3 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m3 | 13.5 | 19.4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m4 | 21.1 | 22.5 | 27.3 | 0 | 0 | 0 | 0 | 0 | 0 |
| m5 | 7.4 | 43.4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Pose error at the cap: n1 0.42, n2 1.18, n3 0.96, n4 0.96, n5 1.11, m1 0.76, m2 0.92, m3 0.61, m4 0.63, m5 1.79. The `998e957` ten was 0.37, 1.03, 0.53, 0.50, 0.81 and 1.31, 0.19, 0.51, 0.43, 1.62. Means are 0.93 m and 0.94 m, against 0.65 m and 0.81 m.

The 243 s contact sit and the 49 s empty path after a back do not repeat. Nine of the ten longest `(0, 0)` stretches are 3.5 s at the start. m5's next stretch is 5.0 s, from t=304 s to t=309 s, and it logs `stall, backing out` at t=306 s. The longest is 244 s on m3, from t=356 s to the cap, phase `go`, pile `src=hold`, goal still set, path still set. The last decision is `pile_leave` and `back` at t=357.6 s, then the same goal. The approach stays `back` while a goal is held, so neither the contact branch nor the `goal=none` branch matches, and the clock does not take it.

## One watchdog on the issued command

Commit `b975857`. The straight, pivot, contact, and empty-back clocks are gone. `_remember` is where the final wheel command is stored, after the escape block. If that output has been `(0, 0)` for more than 5 s and the planner pose has not moved 0.04 m, the watchdog fires. The first rung is the stall escape. A blocked back-out takes the pile leave. The next rung drops the goal and quarantines that spot for the existing 30 s. A fire inside 20 s of the last one starts one rung higher. The opening 3.5 s, `done`, and `settle` do not count. Contact still uses its 2 s cooldown. Mapping tests: 175 passed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Lowest `up_z` is 0.996, on m2. No sample is below 0.95.

North visited 16.07, 15.91, 17.47, 18.76, 11.97 (mean 16.04, sd 2.55) against 14.90 (sd 2.96) on `bc55896` and 15.95 (sd 5.10) on `998e957`. −0.35 visited 17.41, 22.75, 12.16, 11.58, 12.64 (mean 15.31, sd 4.76) against 14.89 (sd 5.09) and 20.60 (sd 4.70). Pantry and storage stay at 0. Closet is 69.6 on m3. Laundry is 36.0 on m4 and 21.0 on m2. Hall is 53.3 on m4 and 31.9 on m2.

| run | living | bedroom | kitchen | bath | laundry | closet | hall | storage | pantry |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 19.7 | 39.1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n2 | 6.2 | 67.1 | 0 | 0.6 | 0 | 0 | 0 | 0 | 0 |
| n3 | 25.2 | 33.1 | 0 | 0.5 | 0 | 0 | 0 | 0 | 0 |
| n4 | 32.9 | 20.4 | 5.4 | 0 | 0 | 0 | 0 | 0 | 0 |
| n5 | 11.0 | 37.0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m1 | 31.4 | 5.9 | 22.9 | 0 | 0 | 0 | 0 | 0 | 0 |
| m2 | 30.2 | 28.1 | 17.9 | 0 | 21.0 | 0 | 31.9 | 0 | 0 |
| m3 | 7.1 | 25.7 | 0 | 12.2 | 0 | 69.6 | 0 | 0 | 0 |
| m4 | 16.0 | 0.6 | 9.5 | 0 | 36.0 | 0 | 53.3 | 0 | 0 |
| m5 | 16.5 | 9.1 | 30.7 | 0 | 0 | 0 | 0 | 0 | 0 |

Pose error at the cap: n1 0.14, n2 0.30, n3 1.18, n4 0.97, n5 0.94, m1 0.45, m2 0.70, m3 0.56, m4 0.87, m5 0.78. Means are 0.71 m and 0.67 m. `bc55896` was 0.93 m and 0.94 m. `998e957` was 0.65 m and 0.81 m.

Watchdog fires: n4 one `stall`, m4 one `stall`, and zero on the other eight. No `pile_leave` rung and no `drop` rung. The longest `(0, 0)` is 5.0 s, on n4 from t=273 s to t=278 s and on m4 from t=90 s to t=95 s. Both are phase `go` after a `back`, and the stall rung ends them. The other eight longest stretches are the 3.5 s start-up, which the watchdog does not count. The 244 s m3 sit does not repeat.

## Four modes, one command writer

Commits `c3e7031` and `081cddc`. The controller names the mode on each tick: `explore` (spin, settle, idle), `approach` (`phase == go`), `recover` (blind back, contact recovery, relief, the stuck escape, or the one-rule command), `leave_area` (a leave target, a forced pile, or the pile-leave command). `_issue` is the function that stores the wheel command leaving the controller. `__init__` still sets the starting `(0, 0)`. The watchdog ladder is the `b975857` ladder: 5 s of `(0, 0)` with the planner pose inside 0.04 m, then stall, then pile leave, then a goal drop with the existing 30 s quarantine. A repeat inside 20 s starts one rung higher. The 3.5 s start-up, `done`, and `settle` stay excluded.

`c3e7031` also deleted the stuck escape, the pile-leave helpers, relief, and contact recovery. Those branches issue a non-zero command, so the watchdog, which only replaces `(0, 0)`, did not stand in for them. Five 600 s runs of each seed on that commit: north visited 2.43, 2.52, 9.39, 3.68, 4.86 (mean 4.58, sd 2.87) and −0.35 visited 8.95, 6.23, 4.31, 12.25, 3.69 (mean 7.09, sd 3.54). `081cddc` puts those drives back and keeps the mode names and the single writer. Against `b975857` the controller diff is 13 lines removed and 42 lines added (3,809 lines to 3,838). Mapping tests: 177 passed. The new tests check that `_issue` is the only writer of the outgoing command and that the mode order is explore, approach, recover, leave_area.

Five 600 s runs of each seed on `081cddc`, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. Seen % is `coverage_pct` from `eval/score_apartment_map.py` on the saved map: the share of ground-truth free cells the map labels known.

North visited 20.28, 15.70, 17.77, 16.26, 11.32 (mean 16.27, sd 3.29) against 16.04 (sd 2.55) on `b975857`. Seen 92.03, 94.45, 96.54, 86.45, 87.85 (mean 91.46, sd 4.28). −0.35 visited 11.36, 19.35, 24.66, 20.84, 23.82 (mean 20.01, sd 5.30) against 15.31 (sd 4.76). Seen 88.92, 94.27, 96.35, 97.72, 93.28 (mean 94.11, sd 3.38). The ten-run visited mean is 18.14 against 15.68. North is inside the noise. The −0.35 mean is 4.7 points higher. The command each branch returns is the same command it returned on `b975857`. Pantry and storage stay at 0 visited. Closet and laundry stay at 0 visited in this sample.

| run | living | bedroom | kitchen | bath | laundry | closet | hall | storage | pantry |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 42.1 | 11.0 | 0.5 | 0 | 0 | 0 | 0 | 0 | 0 |
| n2 | 8.7 | 36.8 | 0 | 31.0 | 0 | 0 | 27.9 | 0 | 0 |
| n3 | 9.6 | 40.5 | 0 | 45.6 | 0 | 0 | 7.5 | 0 | 0 |
| n4 | 11.8 | 57.5 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| n5 | 21.3 | 0.9 | 9.1 | 0 | 0 | 0 | 19.4 | 0 | 0 |
| m1 | 16.1 | 22.6 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m2 | 25.8 | 29.5 | 15.9 | 0 | 0 | 0 | 15.2 | 0 | 0 |
| m3 | 30.0 | 60.7 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| m4 | 18.0 | 19.7 | 0 | 62.1 | 0 | 0 | 52.1 | 0 | 0 |
| m5 | 41.1 | 31.8 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Pose error at the cap: n1 0.58, n2 0.75, n3 0.89, n4 0.89, n5 0.31, m1 0.90, m2 0.99, m3 0.65, m4 1.33, m5 1.87. Means are 0.68 m and 1.15 m. `b975857` was 0.71 m and 0.67 m.

Watchdog fires: n1 one `stall`, n5 six `drop`, and zero on the other eight. No `pile_leave` rung. The longest `(0, 0)` is 26.6 s on n5, from t=505.3 s to t=531.9 s, phase `go`, `src=hold`, goal already none. The drop rung returns `(0, 0)`, so each fire starts another 5 s of the same command. The six drop lines are t=497.8, 506.8, 511.8, 516.8, 521.8, and 526.8 s. Pile leave takes the wheels at t=528.4 s. n1's longest zero is 5.0 s, from t=429.6 s to t=434.6 s, ended by the stall rung. The other eight longest stretches are the 3.5 s start-up.

Lowest `up_z` is 0.933, on n5, for about 0.4 s at t=493.5–493.9 s. The shell is back above 0.98 by t=494.3 s. The other nine stay at or above 0.998. No run finishes inverted.

## The drop rung leaves, or it spins

Commit `9c58a66`. The drop rung still quarantines the goal for the existing 30 s. It then hands the wheels to `leave_area` when that leave command is non-zero, and to `explore` (the survey spin) when the leave is blocked. The stall rung and the pile-leave rung are unchanged. A log line, `done would fire: no reachable frontier, map stable`, is written when the planner has no reachable frontier and the known and occupied counts stay inside 8 cells for the existing 4 s settle hold. The line does not set `done` and does not end the run. Seen % (`coverage_pct`) is the first number in the scorer's text. Visited % stays beside it. Mapping tests: 179 passed. Against `081cddc`, `explore.py` is 71 lines added and 2 removed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. The would-fire line does not appear.

Seen, primary. North 94.88, 38.88, 65.69, 92.11, 93.36 (mean 76.98, sd 24.48) against 91.46 (sd 4.28) on `081cddc`. −0.35 95.55, 93.01, 95.87, 74.63, 93.47 (mean 90.51, sd 8.96) against 94.11 (sd 3.38). The ten-run seen mean is 83.75 against 92.79.

Visited, secondary. North 18.18, 12.15, 18.94, 19.45, 15.40 (mean 16.82, sd 3.04) against 16.27 (sd 3.29). −0.35 19.41, 14.75, 17.58, 18.72, 15.92 (mean 17.28, sd 1.93) against 20.01 (sd 5.30). The ten-run visited mean is 17.05 against 18.14.

| run | living | bedroom | kitchen | bath | laundry | closet | hall | storage | pantry |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| n1 | 6.0 | 35.2 | 0 | 63.0 | 0 | 0 | 35.5 | 0 | 0 |
| n2 | 17.6 | 0 | 5.6 | 0 | 37.0 | 0 | 68.4 | 0 | 0 |
| n3 | 6.6 | 62.7 | 0 | 15.8 | 0 | 53.1 | 0 | 0 | 0 |
| n4 | 34.7 | 0 | 7.1 | 0 | 56.7 | 0 | 39.2 | 0 | 0 |
| n5 | 26.1 | 2.8 | 27.7 | 0 | 0 | 0 | 5.3 | 0 | 0 |
| m1 | 21.6 | 28.1 | 14.6 | 0.6 | 0 | 0 | 78.0 | 0 | 0 |
| m2 | 16.2 | 39.6 | 0.4 | 0 | 0 | 0 | 0 | 0 | 0 |
| m3 | 7.4 | 36.5 | 0 | 51.3 | 0 | 24.1 | 3.2 | 0 | 0 |
| m4 | 14.7 | 26.7 | 0 | 60.6 | 0 | 0 | 0 | 0 | 0 |
| m5 | 17.0 | 0 | 9.2 | 17.0 | 59.2 | 0 | 88.7 | 0 | 0 |

Pose error at the cap: n1 0.09, n2 0.45, n3 0.94, n4 0.42, n5 1.83, m1 0.51, m2 1.17, m3 0.97, m4 0.85, m5 0.54. Means are 0.75 m and 0.81 m. `081cddc` was 0.68 m and 1.15 m.

Watchdog fires: m5 one `stall`, and zero on the other nine. No `pile_leave` rung and no `drop` rung. The longest `(0, 0)` is 5.0 s on m5, from t=280.0 s to t=285.0 s, phase `go`. The stall rung at t=281.5 s ends it, and the command is a reverse by t=285.5 s. The other nine longest stretches are the 3.5 s start-up. The 26.6 s n5 sit does not repeat. This sample never reached the drop rung, so the new handoff was not the command on these ten. Lowest `up_z` is 0.999, on m5. No run finishes inverted.

## A still pose is a sit even when the wheels turn

Commit `317f719`. The watchdog no longer requires `(0, 0)`. Five seconds with the planner pose inside the existing 0.04 m takes the stall rung, then pile leave, then the drop with the existing 30 s quarantine. A repeat inside 20 s still starts one rung higher. Startup, `done`, `settle`, and the survey spin stay out of the clock. The survey spin is the opening full turn. The `heading_off` loop is phase `go`. Mapping tests: 183 passed. Against `9c58a66`, `explore.py` is 13 lines added and 11 removed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `end_run.txt` with `time_cap`. No mapper done line. The would-fire line does not appear. Lowest `up_z` is 0.998. No run finishes inverted.

Seen, primary. North 89.31, 94.57, 93.64, 96.48, 94.36 (mean 93.67, sd 2.65) against 76.98 (sd 24.48) on `9c58a66` and 91.46 (sd 4.28) on `081cddc`. −0.35 93.48, 90.40, 95.91, 95.42, 95.76 (mean 94.20, sd 2.34) against 90.51 (sd 8.96) and 94.11 (sd 3.38). The ten-run seen mean is 93.93 against 83.75 and 92.79.

Visited, secondary. North 18.69, 11.14, 17.79, 21.80, 13.94 (mean 16.67, sd 4.17) against 16.82 and 16.27. −0.35 14.23, 19.26, 12.47, 17.87, 15.08 (mean 15.78, sd 2.75) against 17.28 and 20.01. The ten-run visited mean is 16.23 against 17.05 and 18.14.

A wing crossing is at least 2 s in the bedroom, bath, or closet and 2 s in the hall, kitchen, or laundry. North crosses on n1, n4, and n5. −0.35 crosses on m1, m4, and m5. n2 stays in the living room and the kitchen and still scores seen 94.57. n3 stays in the bedroom, the living room, and the bath.

Pose error at the cap: n1 0.55, n2 1.19, n3 1.48, n4 0.21, n5 1.75, m1 0.54, m2 1.22, m3 0.91, m4 0.34, m5 1.16. Means are 1.04 m and 0.83 m. `9c58a66` was 0.75 m and 0.81 m. `081cddc` was 0.68 m and 1.15 m.

The longest planner-pose sit, xy inside 0.04 m, is 56.6 s on n2, from t=52.4 s to t=109.0 s. The others are n1 37.8 s, n3 31.5 s, n4 41.8 s, n5 31.5 s, m1 35.8 s, m2 30.0 s, m3 26.4 s, m4 36.2 s, m5 38.0 s. The clock restarts after each fire, and the next command often does not translate, so one frozen pose is several 5 s rungs. Every run fires the drop rung. North fires are 31 `stall`, 13 `pile_leave`, 79 `drop`. −0.35 fires are 40 `stall`, 19 `pile_leave`, 65 `drop`. The drop handoff runs. It does not end the frozen pose by itself.

## Where the 317f719 pose error comes from

No controller change. The ten above, commit `317f719`. Estimate and ground truth are compared every 0.2 s. A bin inside a planner span that stays within 0.04 m for at least 5 s is frozen. Otherwise the wheel command names the bin: pivot, straight, contact, zero, or other. The step is the change in the world-frame error vector. Final position error averages 0.93 m. Final yaw averages 2.5° in magnitude, so the miss is xy.

Sum of the absolute error steps across the ten: frozen 32.8 m, straight 25.7 m, other 18.7 m, pivot 3.4 m, zero 0.3 m, contact 0.2 m. The net vector that remains is also largest in the frozen bins on seven runs (0.66, 0.99, 0.77, 0.18, 0.72, 0.34, 0.51, 0.41, 0.13, 0.56 m). Straight driving is second (net 0.08 to 0.66 m). Contact does not move the total.

The long frozen spans themselves add little. On n2 from t=52 s to t=109 s the planner path is 0.10 m, the true path is 0.70 m, and the error goes from 0.12 m to 0.17 m. The command is a pivot on 88% of the samples. Wheel translation in that window is 0.60 m and is not kept: the planner stays inside the 0.04 m ball. `floor still, translation frozen` is logged 479 times inside frozen spans and 334 times outside. The floor check is already refusing that wheel arc. Stopping integration a second time would not be a new hold.

## A rung that does not translate takes the open straight

Commit `cb637d8`. When the stall or pile-leave command is a pivot, or the reverse side is blocked, the rung takes `_straight_leave`, which already picks the open side. The following ticks stay on that straight until the planner pose moves 0.04 m. If neither side can translate, the fire falls through to the next rung, and the drop still ends in the survey spin. Wheel integration is unchanged. Mapping tests: 184 passed. Against `317f719`, `explore.py` is 60 lines added and 11 removed.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `time_cap`. No mapper done line. The would-fire line does not appear. Lowest `up_z` is 0.998. No run finishes inverted.

Seen, primary. North 95.79, 97.80, 89.34, 87.52, 94.37 (mean 92.96, sd 4.36) against 93.67 (sd 2.65). −0.35 96.74, 59.92, 91.65, 94.82, 78.73 (mean 84.37, sd 15.37) against 94.20 (sd 2.34). The ten-run seen mean is 88.67 against 93.94. m2 at 59.92 and m5 at 78.73 are the drop. Both spend the run in the bedroom and the living room.

Visited, secondary. North 20.44, 23.98, 16.55, 14.57, 21.14 (mean 19.33, sd 3.76) against 16.67. −0.35 19.82, 16.27, 15.42, 21.33, 9.40 (mean 16.45, sd 4.64) against 15.78. The ten-run visited mean is 17.89 against 16.23.

The longest planner-still span is 47.9 s on n1, t=339–387. The others are n2 31.5 s, n3 31.5 s, n4 33.5 s, n5 31.5 s, m1 30.7 s, m2 35.0 s, m3 21.0 s, m4 27.3 s, m5 33.5 s. The 31.5 s spans that start at t=0 are the survey spin, which the clock does not count. North fires 30 `stall`, 18 `pile_leave`, 60 `drop`. −0.35 fires 30 `stall`, 18 `pile_leave`, 95 `drop`. Pose means are 0.68 m and 0.58 m, against 1.04 m and 0.83 m. A wing crossing, 2 s in the bedroom wing and 2 s in the hall or kitchen, happens on n3, m1, and m2.

## A doorway sit creeps and keeps the goal

Commit `76933cb`. If the watchdog fires while `_narrow_gap` is true, the command is the existing throat creep at `DOOR_SPEED_M_S`. The goal is not quarantined and the ladder step does not advance. An open floor still takes the straight leave and the drop. The following ticks stay on the creep until the pose moves 0.04 m or the throat test fails. Mapping tests: 186 passed. Against `cb637d8`, `explore.py` is 23 lines added.

Five 600 s runs of each seed, flags off. Depth seed 64 on north and 63 on −0.35. All ten write `time_cap`. No mapper done line. The would-fire line does not appear. Lowest `up_z` is 0.861, on n1, for about 4 s at t=258–262 s. The shell is upright again by t=262 s. No run finishes inverted.

Seen, primary. North 96.89, 92.88, 90.73, 96.29, 90.15 (mean 93.39, sd 3.10) against 92.96 on `cb637d8` and 93.67 on `317f719`. −0.35 91.63, 92.38, 95.72, 57.78, 81.14 (mean 83.73, sd 15.50) against 84.37 and 94.20. The ten-run seen mean is 88.56 against 88.67 and 93.94. m4 spends 460 s in the bath and scores 57.78. m5 stays in the kitchen and the hall and scores 81.14.

Visited, secondary. North 16.14, 17.05, 13.18, 16.29, 11.43 (mean 14.82, sd 2.40) against 19.33 and 16.67. −0.35 18.80, 18.25, 14.43, 15.95, 11.66 (mean 15.82, sd 2.92) against 16.45 and 15.78. The ten-run visited mean is 15.32 against 17.89 and 16.23.

Doorway-creep lines: n1 12, n2 9, n3 3, n4 8, n5 2, m1 10, m2 5, m3 13, m4 11, m5 8. North ladder fires are 33 `stall`, 20 `pile_leave`, 88 `drop`. −0.35 fires are 34 `stall`, 20 `pile_leave`, 70 `drop`. The longest planner-still span is 48.7 s on n2, t=501–550. The others are n1 32.8 s, n3 38.9 s, n4 31.5 s, n5 34.6 s, m1 40.4 s, m2 30.7 s, m3 29.8 s, m4 23.5 s, m5 24.2 s. Pose means are 0.89 m and 0.90 m, against 0.68 m and 0.58 m on `cb637d8`.

## The clean −0.35 set on 317f719 does not repeat

No controller change. Depth seed 63 is the seed behind the earlier −0.35 fives, so these five use new depth seeds 71, 72, 73, 74, and 75, the same five on both commits. Spawn stays (6.10, 4.70), yaw 1.2208. 600 s, flags off, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Each run writes `time_cap`.

`317f719`, seeds 71–75. Seen 50.37, 77.50, 94.51, 95.90, 92.47 (mean 82.15, sd 19.23). Visited 9.57, 14.69, 14.51, 16.99, 14.22. Pose 0.26, 0.56, 0.70, 1.04, 1.72 m. Longest planner-still span: 26.0 s (t=460–486), 45.2 s (t=555–600), 48.5 s (t=513–561), 42.4 s (t=211–253), 22.0 s (t=324–346). Lowest `up_z` is 0.843, on seed 71, for 4.7 s at t=549–554. The shell is upright again by t=554 s and ends at 1.0. The other four stay at or above 0.992. Seed 71 spends 465 s in the hall after a 19 s bath visit and scores 50.37. Seed 72 spends 469 s in the bedroom, never enters the hall or the kitchen, and scores 77.50.

`76933cb`, the same seeds. Seen 92.65, 71.80, 86.02, 84.94, 92.89 (mean 85.66, sd 8.57). Visited 16.46, 21.84, 10.49, 6.86, 24.90. Pose 0.08, 0.58, 0.66, 0.40, 0.61 m. Longest planner-still span: 30.1 s (t=502–532), 26.9 s (t=304–330), 30.8 s (t=226–257), 93.7 s (t=258–352), 24.8 s (t=417–442). Lowest `up_z` is 0.998. Seed 72 spends 294 s in the bedroom and never enters the south wing (71.80). Seed 73 spends 408 s in the hall and never enters the north wing (86.02). Seed 74 spends 438 s in the kitchen and never enters the north wing (84.94).

Pooled with the depth-63 fives, ten runs each. `317f719` seen mean 88.17, sd 14.39 (the earlier five were 93.48, 90.40, 95.91, 95.42, 95.76, mean 94.19, sd 2.33). `76933cb` seen mean 84.70, sd 11.85 (the earlier five mean 83.73, sd 15.50). The gap is 3.5 points. The standard error of that difference is 5.9. On seeds 71–75 the paired difference (`317f719` minus `76933cb`) is −42.28, +5.70, +8.49, +10.96, −0.42, mean −3.51. The all-at-or-above-90 set on depth seed 63 was that seed. The gap between `317f719` and `76933cb` is inside the noise. `cb637d8` was not re-run; its depth-63 −0.35 mean of 84.37 sits with the `76933cb` ten.

## What keeps a −0.35 run in one wing

No controller change and no new runs. The twenty −0.35 logs of `317f719` and `76933cb`, depth seeds 63 and 71–75. A crossing is 2 s on a bedroom-wing floor and 2 s on a hall or kitchen floor. Thirteen of the twenty never do that. Four spend 30 s in both wings: `317f719` seeds 63 m4 and m5 and seed 71, and `76933cb` seed 63 m1. The other three crossings are 13 s, 5 s, and 3 s.

The planner does score the other wing. `select_goal` ranks cluster length over path length, with a 6× bonus when the path leaves the robot's region, and only the longest eight clusters plus four door-touching ones are scored. On the final map, with the floor-plate credit the live run uses, a far-wing cluster is in that scored set on every trapped run. It is the winner at the cap on `317f719` seed 63 m3 (bath) and seed 73 (laundry bridge), and on `76933cb` seeds 71 and 74 (bath). It is the runner-up, behind a nearer frontier or an outside patch, on the others. Far-wing goals are also selected during the run: 9 to 19 times on eleven of the thirteen trapped runs. Two runs barely select one (`76933cb` seed 63 m3, two goals, then the closet; seed 74, two goals). The far wing still has hundreds to thousands of unknown cells at the cap, so the distance penalty is not an empty wing. It does let a short local frontier outscore a long far one. On seed 63 m5 of `76933cb` the living winner scores 123 against a bedroom runner at 29.

What ends the far goal is the ladder, at the doorway. Across the far-wing selections the next event is a drop quarantine 25 times, a pile-leave 21 times, a nearer home-wing frontier 18 times, `stuck, next goal` 14 times, `too_tight` 11 times, and a frontier ban 5 times. The drop quarantines the goal point for 30 s inside 0.8 m, so a kitchen or bath point is banned while the shell is still on the near side of the opening. `76933cb` creeps and keeps the goal only when `_narrow_gap` is already true. That creep almost never arms at the opening where the drops fire: seed 72 has 9 drops and 1 creep at door G, seed 63 m5 has 19 drops and 0 creeps at the west-hall entry, seed 74 has 69 drops and 0 creeps there.

The opening the body sits in:

- Door G, the living–bedroom opening at (3.71, 5.13). `317f719` seed 63 m2 for 77 s, seed 72 for 166 s, seed 73 for 180 s, seed 75 for 263 s. `76933cb` seed 63 m2 for 123 s and seed 72 for 138 s. The far goals are kitchen and hall, and the drop or the pile-leave clears them.
- The west-hall entry at (3.71, 1.78), and door H at (1.88, 1.78). `317f719` seed 63 m3 reaches door H to 0.31 m and is still holding a bath goal at the cap. `76933cb` seed 63 m5 spends 205 s at the entry and 167 s at the laundry door, with the bedroom goal drop-quarantined. Seed 71's last bath goal is drop-quarantined at door H. Seed 73 spends 299 s at the entry; the bath goal is selected 19 times and the creep does fire there, and the shell still does not pass door H.
- `76933cb` seed 63 m4 spends 461 s in the bath. Door H is within 0.55 m for 8 s. Kitchen goals are selected from the bath and cleared by the pile-leave, the drop, and `too_tight`.
- `76933cb` seed 63 m3 and seed 74 are the goal-selection pair. m3 selects the south twice, then stays in the closet. Seed 74 selects the bath twice; at the cap the bath score is 11.2 and the kitchen score is 10.5, and 69 of 82 drops are at the west-hall entry on local goals.

The runs that do cross use those same openings. `317f719` seed 63 m5 spends 236 s within 0.55 m of door H and still spends 132 s in the bath. The opening is passable. The far goal has to survive the sit.

The twenty −0.35 seen scores have sd 13.0 around a mean of 86.4. A trapped run often still scores above 90, because the living room sees the other wing, so seen is a noisy proxy for the crossing. The standard error of a mean of n seeds is 13/sqrt(n). A 2-point change is past that noise when the standard error is about 1 point, which is about 170 seeds of one spawn. A difference between two commits needs about 340 seeds of that spawn on each commit for a standard error of 1 point. Five seeds leave a standard error near 6 points.
