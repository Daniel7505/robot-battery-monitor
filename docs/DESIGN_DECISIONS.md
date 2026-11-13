# Design decisions

This file is training material for a later system that proposes the robot's own hardware and software build from mission parameters. It is not a lab notebook of every commit. Each entry is one choice a future builder should be able to reuse or refuse.

Read [`docs/LESSONS_LEARNED.md`](LESSONS_LEARNED.md) for how a failed seed was diagnosed. This file records the choice that followed.

Append one entry at the end of every mapping round. Use the fields below. If the sims did not show it, write "unconfirmed" instead of a cause.

- **Date / round**
- **Mission parameter.** The constraint the choice is answering (door width, no foresight, camera-first, coverage that means the shell entered, cost of sensors).
- **Change.** What moved in the body or in the decision logic.
- **Why.**
- **Evidence.** The sim or the repo fact that supports it. Name the seed and the number.
- **Rejected.** Options that were considered and not kept, and the reason.
- **Outcome.** What is true after the choice. Include the miss if the bar was not met.

## 2026-11-02 — Four-wheel skid for the apartment, casters for the warehouse

**Mission parameter.** The apartment mapper and the warehouse lap are different machines. The warehouse energy trace and caster tests stay on `ButlerBot.proto`. The apartment drive is skid-steer.

**Change.** `apartment.wbt` instances `ButlerBotSkid` directly. A mapping launch aborts if the world is the caster. The runtime swap that rewrote a caster instance into a skid was deleted. Hubs at y = ±0.17 m, track 0.34 m, wheel radius 0.08 m, nose and tail 0.19 m, side face 0.227 m.

**Why.** Fixes written against the caster branch (`cursor/apartment-mapping-5970`) were being treated as the robot under test. That world still instances `ButlerBot` with casters. The mapping ideas still applied. The drivetrain did not.

**Evidence.** The world and the proto on that branch were read. The apartment node was `ButlerBot`. The skid head at the time was `cursor/jamb-veto-5970`. Later apartment runs print `MAPPING front hubs slaved`. The warehouse lap still reaches the 201 m cap.

**Rejected.** Editing `ButlerBot.proto` to fix the apartment. That changes the warehouse hash and the caster tests. Deleting the caster proto. Running mapping seeds from the caster branch.

**Outcome.** Two protos stay. Apartment mapping is skid-only. Warehouse stays on the caster.

## 2026-11-02 — Camera-first, no lidar, no floor-plan foresight

**Mission parameter.** The mapper builds its map from its own sensors. It does not receive the apartment floor plan, the room rectangles, or the ground-truth pose. Ground truth is scoring only. No lidar.

**Change.** The apartment depth camera is the OAK-D Lite model at (0.14, 0, 0.78), pitched 5°, minimum range 0.35 m. Device noise on `oak_depth` is 0. The mapper applies the quadratic depth error. `ButlerBotSkid` has no lidar node. The mapper does not import the scorer.

**Why.** A map painted by rays through a doorway looks finished while the shell is still outside. Feeding the floor plan to the planner would hide that. A lidar would change the sensor budget and the failure mode this project is trying to learn.

**Evidence.** `ButlerBotSkid.proto` and `src/mapping/` contain no lidar. The standing rule is in `docs/LESSONS_LEARNED.md`. Seen % and visited % diverge on the same runs (round 2, seed −0.35, coverage about 93% with the bath 95% seen and 0% visited).

**Rejected.** Room rectangles or floor area as planner inputs. Treating seen % as "went there." Putting range-finder device noise on the caster proto so the warehouse robot changes.

**Outcome.** The robot's map is its own. Scoring may use the ground-truth pose and the floor solids. The planner may not.

## 2026-11-02, round 2 — Visited floor, not only seen floor

**Mission parameter.** Coverage that counts is floor the shell drove over. A depth ray through a door is not a visit.

**Change.** Scoring stamps the rectangular shell (0.19 m nose and tail, 0.227 m half-width) along `gt_pose_log.csv` and reports visited % next to seen %, overall and per room. The mapper does not import that scorer. Unvisited cells on the robot's own map become goals about 2 m into another region of that map, not the first cell across the threshold.

**Why.** Round 2 coverage was about 93% on −0.35 while the path had not entered the bath or the kitchen. Goals existed inside those rooms on later logs. They were dropped during line-up. The door was not the limit: the bath opening measures 0.81 m.

**Evidence.** Round 4 logs, converted with the seed yaw: −0.35 had 8 goals inside the kitchen box, the closest 8 cm past x = 3.76. North had 3 goals inside the bath and the path stayed 0.45 m short. `stall, backing out` fired within seconds of those goals.

**Rejected.** Reporting seen % as the finish condition. Widening the 0.20 m planning radius to explain a goal that was already inside the bath. Giving the robot the room rectangles.

**Outcome.** Visited % is a scoring output. Kitchen and bath visited stayed near 0 on the full-size robot through round 5 (−0.35 kitchen 5%, bath 0). The metric is what later rounds are judged on, together with pile time.

## 2026-11-02, rounds 6–8 — General stuck detection, not one pose

**Mission parameter.** A skid robot wedges. The detector has to use the robot's own sensors. Ground truth is scoring only. One nose reading or one wheel command is not a class of failure.

**Change.** Round 6 added a silent path trim for a mapped wedge and a stuck check that fired only on a blind spin with nose 0.12 m. Round 7 deleted both. A depth picture that does not change for 2.5 s of commanded motion is `camera_stuck`. Wheels turning without a matching gyro rate, or a short wheel-speed change with no horizontal acceleration, is `imu_stuck`. Either one reverses along the driven trail. `oak_accel` was added on `ButlerBotSkid` only, next to the existing `oak_gyro` and the wheel encoders already quantized at 0.0015 rad. Round 8 runs that check in spin, settle, and go. A stuck spin leaves the spin. After an escape, that heading stays blocked for 18 s, or until the shell is 0.70 m away, so the next command cannot drive straight back in.

**Why.** Round 6 −0.35 had two `nose=0.12` lines, so the narrow detector cannot explain visited falling from 23.4% to 17.0% and the pile growing from 71 s to 361 s. The silent path trim was the remaining planner change. North's round 6 pile was `cmd=(1.06,-1.06)` with nose 99 and a frozen pose, which the nose check ignores. Round 7's north pile then sat in `phase=spin` from t=200 s, and the check lived only in go.

**Evidence.** Round 7, two seeds, round 5 in parentheses. −0.35: visited 14.1% (23.4), pile 267 s (71), `camera_stuck` 27, `imu_stuck` 1. North: visited 5.6% (3.4), pile 516 s (383), `camera_stuck` 6, `imu_stuck` 15. The bar (both seeds beat round 5 on visited % and pile time) was missed. Round 8 north: blocked-heading lines 117 during a 372 s stay. The spin-phase lines fell to 25. North beat the round 5 bar. −0.35 did not (visited 19.3%, pile 152 s).

**Rejected.** Another detector keyed to one nose value or one wheel pair. Leaving the wedge path trim behind a flag. Putting the accelerometer on `ButlerBot.proto`. Treating a reverse that is followed by a forward command as having left the 0.45 m ball.

**Outcome.** Stuck is `camera_stuck`, `imu_stuck`, or `camera_imu_stuck`. `unknown` is the older relief and is counted separately. The check is not limited to the go phase. The heading block does not by itself end a pile.

## 2026-11-02, round 8 — 80% body, diagnostic only

**Mission parameter.** Door clearance versus a false stuck decision. The question was whether the full-size robot misses the kitchen because the body and the margins do not fit, or because the stuck logic fires on a path it could already drive.

**Change.** One 600 s run of seed −0.35 on a copy of `ButlerBotSkid` whose collision footprint, wheel radius, track, and the planner shell and clearance margins were all at 80%. The decision code was the round 7 code. The proto was not `apartment.wbt` and was deleted after the run. Scoring used the same full shell as round 7, on that run's ground-truth centerline.

**Why.** Round 7 full-size −0.35 visited 0% of the kitchen, 14% of the bath, and 30% of the bedroom, then sat 267 s. A smaller body with the same decisions separates clearance from the detector.

**Evidence.** Against round 7 in parentheses: visited 16.5% (14.1), kitchen 26% (0), bath 0 (14), bedroom 0.2 (30), living 30 (13), pile 181 s in the living room (267 s on the bedroom side). `camera_stuck` 2 (27), `imu_stuck` 0 (1), `unknown` 6 (85).

**Rejected.** Shipping the 80% proto. Reading the kitchen 26% as a stuck detector firing in open floor. A second 80% seed was not run, so it is unconfirmed that the smaller body would also skip the bedroom every time.

**Outcome.** The full-size kitchen zero on that seed is clearance. The bath and the bedroom are not "too narrow to enter": the full-size round 7 path visited them. The shipped robot stays full size. Side-face width is 0.454 m (17.9 in). See the door table in `docs/LESSONS_LEARNED.md`.

## 2026-11-03, round 9 — Speed-dependent clearance

**Mission parameter.** Keep the full body and the full 0.30 m pivot aim at cruise. Through a narrow gap the robot can perceive itself, creep and use a buffer closer to the real side face, without raising wall contacts. No ground truth in the detector. Target suggested by the 80% run: kitchen visited near 26% on a full-size −0.35.

**Change.** If the robot's map puts the centre in a throat (clearance under 0.30 m and at least the side face plus 2 cm, 0.247 m) and a live shoulder sees a surface, a forward command is capped at 0.12 m/s and that plan uses 0.247 m instead of 0.30 m. The footprint sweep is unchanged. Reason code `narrow_gap`. An open hall, both shoulders at the no-return sentinel, is not a throat.

**Why.** The 80% body got through the kitchen. The full-size round 8 path only nicked it (6% on −0.35, 0% on north). The extra buffer past the side face is the 0.30 m aim, not the 0.20 m tire inflation.

**Evidence.** Two full-size seeds, round 5 in parentheses. −0.35: visited 10.5% (23.4), pile 417 s (71), kitchen 0, wall steps 477, bumps 0, `narrow_gap` 2. The three creep lines are at t=50, 51, and 177 s. All 477 wall steps are in the 200–300 s bedroom pile, after those lines. North: visited 14.4% (3.4), pile 146 s (383), kitchen 16, wall steps 528 (round 8 was 485), bumps 10, `narrow_gap` 16. Corner steps 0 on both. North beat the round 5 bar. −0.35 did not. Four seeds were not run.

**Rejected.** Lowering the cruise aim to 0.247 m. Shrinking the 0.20 m inflation below the tire. Shipping a smaller body. Claiming north's 16% kitchen was caused by the creep: `narrow_gap` fired 16 times on that seed, and no trace yet ties those lines to the kitchen entry.

**Outcome.** The creep did not reproduce the 80% kitchen result on full-size −0.35 (kitchen stayed 0). The −0.35 wall-step rise is the bedroom pile, not the creep lines. Cruise aim stays 0.30 m.

## 2026-11-03, round 9 — Leave a repeated pile for another goal

**Mission parameter.** After an escape, do not sit reversing into the same spot. North's round 8 blocked-heading line fired 117 times during a 372 s stay.

**Change.** A second escape within 0.45 m of the last one, or a fourth blocked-heading refusal, drops the current goal, marks it low priority, and reverses along the driven trail until the shell is 0.70 m away. The next plan skips a goal still within 0.80 m of that spot for 30 s. Reason code `pile_leave`. The single-refusal heading block remains.

**Why.** Reversing, then accepting a forward command along the same heading, was the round 7 −0.35 pile (8.4 m of path inside a 0.45 m ball). The round 8 block stopped the immediate return and did not select a different goal.

**Evidence.** −0.35: `pile_leave` 3, blocked-heading lines 8 (round 8 was 11), pile 417 s at (1.36, 4.81). North: `pile_leave` 4, blocked-heading lines 46 (round 8 was 117), pile 146 s at (2.92, 1.06). North's pile is shorter than round 8. −0.35's pile is longer. The bar was not cleared on −0.35.

**Rejected.** A permanent ban of the heading. Deleting the short heading block and only switching goals. Treating `pile_leave` as proof the shell left the 0.45 m ball.

**Outcome.** Repeated refusals now change the goal. They do not guarantee the shell leaves. −0.35 still sat 417 s.

## 2026-11-04 — Same-code variance, before another explorer patch

**Mission parameter.** A change in visited floor has to clear the spread of repeating the same program. Visited floor is the share of open floor the robot's body drove over. A pile is the longest time the true body stayed inside 0.45 m of one point. The 60 s pile bar stays the bar. No new sensor and no new rule in this step.

**Change.** None in the mapper, the body, or the world. Five 600 s runs of seed −0.35 (spawn yaw 1.2208, harness name `m35`) and five of north (spawn 5.80, 5.60, yaw 2.2), on commit `00dae73`, the round 9 controller. Launch was `eval/run_skid_seeds.py`: `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`, Webots R2025a `--mode=fast`, headless. Each seed kept one depth-noise seed across its five runs (63 for −0.35, 64 for north), from the harness formula `1 + (port % 97)` on ports 1711 and 1712. The simulator's own run-to-run differences are what moved.

**Why.** A third-party review said stop patching Explorer and measure first. Rounds 5–9, one run per seed, moved overall visited floor by about 10–14 percentage points from round to round. That gap might be the edit. It might be the spread of a single run.

**Evidence.** All ten runs hit the 600 s cap (75,000 ground-truth pose rows). Sample standard deviation divides by 4. The run figures below are rounded; the mean and the sample sd use the unrounded scores. Per-run counts are in `docs/LESSONS_LEARNED.md`.

−0.35 visited %: 18.0, 9.6, 14.1, 9.9, 22.1. Mean 14.7, sample sd 5.4, range 12.5 points (9.6 to 22.1). Pile: 68.6 s, 349.8 s, 52.0 s, 459.5 s, 135.6 s. Mean 213 s, sample sd 182 s.

North visited %: 3.7, 6.5, 10.7, 10.2, 13.9. Mean 9.0, sample sd 3.9, range 10.2 points (3.7 to 13.9). Pile: 541.0 s, 446.5 s, 332.9 s, 291.0 s, 334.0 s. Mean 389 s, sample sd 103 s.

Published overall visited % in rounds 5–9. −0.35: 23.4, 17.0, 14.1, 19.3, 10.5 (span 12.9 points). North: 3.4, 5.6, 7.4, 14.4 (span 11.0 points; round 6 did not publish an overall visited %). Those spans match the same-code ranges. Round 9's −0.35 run (visited 10.5, pile 417 s) sits inside this −0.35 sample. Round 9's north run (visited 14.4, pile 146 s) is a high visited draw and a short pile: 14.4 is just above this sample's 13.9 high, and 146 s is shorter than every north pile here (shortest 291 s).

**Rejected.** Another Explorer rule on one run per seed. Treating round 9's north pile of 146 s as the typical outcome of this code. Reading the climb from north's round 5 visited 3.4% to round 9's 14.4% as the effect of the creep or of `pile_leave`.

**Outcome.** A one-run swing of about 10–14 visited points is the size of the noise on this controller. −0.35 pile time runs from 52 s, under the 60 s bar, to 460 s, with no code change. The recovery strip was measured afterward. See "One recovery rule."

## 2026-11-04 — Oracle pose for the planner, map left on the estimate

**Mission parameter.** Poor coverage is either the pose the planner believes or the choices it makes with that pose. The test may hand the planner the robot's own simulator pose. It may not hand over the floor plan, the room rectangles, or any other map. The occupancy grid stays on the wheel and gyro estimate. A visited-floor change under about 6 points is the same-code noise measured above.

**Change.** `RBM_ORACLE_POSE` is off unless set. When it is on, the pose logger writes `oracle_pose.txt` (the robot's own x, y, yaw). The mapper reads that file and converts it into the odometry frame, using the first sample as the origin. Goal choice, path following, the survey spin, and the stuck and pile clocks use that pose. Occupancy integration, the footprint stamp, the visited-cell mask, fusion, wall nudge, loop closure, pin, and the zero-velocity update stay on the estimate. The default path does not open the file.

**Why.** The same controller visits about 15% of the floor on −0.35 and about 9% on north, and it sits for minutes. If a true pose fixed that, the estimate would be the flaw. If it did not, the decision logic would be.

**Evidence.** Five 600 s runs of −0.35 and five of north, oracle on, flag commit `234734e`, same seeds and depth-noise seeds as the variance entry (63 and 64). Every run latched the oracle and wrote 75,000 ground-truth pose rows (`mapping_done` is 600 s). Sample standard deviation divides by 4. Run figures are rounded; the mean and the sample sd use the unrounded scores. Counts are in `docs/LESSONS_LEARNED.md`.

−0.35 visited %: 5.5, 11.5, 16.4, 11.5, 9.7. Mean 10.9, sample sd 3.9, range 10.9 points (5.5 to 16.4). The variance mean was 14.7. The gap is 3.8 points. Pile: 483.4 s, 211.1 s, 97.3 s, 371.5 s, 366.4 s. Mean 306 s, range 97 s to 483 s. The variance pile mean was 213 s.

North visited %: 11.2, 11.2, 3.3, 6.6, 8.0. Mean 8.1, sample sd 3.3, range 7.9 points (3.3 to 11.2). The variance mean was 9.0. The gap is 0.9 points. Pile: 292.1 s, 279.1 s, 470.4 s, 507.5 s, 270.4 s. Mean 364 s, range 270 s to 507 s. The variance pile mean was 389 s.

The estimate was not already the true pose. End position error, estimate transformed by the seed spawn against the true body: −0.35 0.4 m, 1.2 m, 0.3 m, 3.1 m, 1.6 m. North 30.1 m, 0.4 m, 0.3 m, 1.4 m, 1.4 m. North run 1 was also reversed in heading. That run still visited 11.2%.

**Rejected.** Building the occupancy grid from the true pose. That would answer a different question, and the no-foresight rule for the map would not hold. Shipping the flag on. Treating the 3.8 point drop on −0.35 as a real loss.

**Outcome.** A true pose for the planner does not fix coverage and does not end the piles. Every oracle pile was over the 60 s bar. The decision logic is the flaw. The flag stays off.

## 2026-11-04 — One recovery rule

**Mission parameter.** The limit cycle may be the stack of recovery rules fighting itself. The test replaces that stack with one rule, behind a flag, and repeats the variance seeds. The occupancy grid still comes from the wheel and gyro estimate. The run may not hand the robot the floor plan or its own simulator pose. A visited-floor change under about 6 points is noise.

**Change.** `RBM_RECOVERY_MODE` is empty unless set. Empty is the current stack. `one_rule` treats a stuck body as the only trigger: the shell has stayed inside 0.45 m for 60 s, or a translation or a spin command has not moved the body for 2 s. It reverses along the driven trail, raises the cost of that 0.45 m disk for the whole 600 s run, and the next goal has to lie outside the disk. `trail_only` does the same reverse and does not mark the disk. Camera and IMU stuck escapes, the pile-forced branch, the blocked heading, the unknown relief, the contact recover, the creep, the stall escape, the wide escape, and the dead-end back stay in the file and do not run in either mode. The default path is unchanged. Commit `d264bfa`.

**Why.** Rounds 5–9 each added a recovery after one run per seed. The variance check showed that a 10 point visited swing is what the same code does by itself. A true pose did not fix the sit. The remaining suspect is the decision stack.

**Evidence.** Five 600 s runs of −0.35 and five of north, for `one_rule`, then the same ten for `trail_only`. Oracle off. Depth-noise seeds 63 and 64. Ports 1711 and 1712. `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Webots R2025a, fast, `--no-rendering`, headless. Every run wrote 75,000 pose rows. Sample standard deviation divides by 4. Counts are in `docs/LESSONS_LEARNED.md`.

`one_rule`, −0.35 visited %: 3.45, 7.78, 5.65, 3.46, 7.77. Mean 5.6, sample sd 2.2, range 4.3 points. The variance mean was 14.7. The gap is 9.1 points. Pile: 567.3 s, 519.1 s, 528.1 s, 552.4 s, 490.4 s. Mean 531 s, range 490 s to 567 s. The variance pile mean was 213 s, and its longest pile was 459.5 s.

`one_rule`, north visited %: 2.94, 3.58, 4.08, 4.83, 5.08. Mean 4.1, sample sd 0.9, range 2.1 points. The variance mean was 9.0. The gap is 4.9 points. Pile: 505.0 s, 549.6 s, 454.2 s, 547.9 s, 474.0 s. Mean 506 s, range 454 s to 550 s. The variance pile mean was 389 s.

`trail_only`, −0.35 visited %: 6.91, 4.60, 3.89, 3.07, 8.12. Mean 5.3, sample sd 2.1, range 5.1 points. The gap to the variance mean is 9.4 points. The gap to `one_rule` is 0.3 points. Pile mean 499 s, range 426 s to 569 s.

`trail_only`, north visited %: 5.29, 5.91, 8.19, 5.03, 3.43. Mean 5.6, sample sd 1.7, range 4.8 points. The gap to the variance mean is 3.4 points. The gap to `one_rule` is 1.5 points. Pile mean 466 s, range 337 s to 520 s.

Bumps, stuck events, and blocked-heading lines are 0 on all twenty runs. `camera_stuck`, `imu_stuck`, `camera_imu_stuck`, `unknown`, `narrow_gap`, and `pile_leave` are 0. The one rule itself fires 1 to 7 times in a run.

**Rejected.** Deleting the old recovery code before this measurement. Turning the flag on in the harness. Reading the 4.9 point north gap as a loss. Reading the 0.3 and 1.5 point gaps between `one_rule` and `trail_only` as an effect of the cost mark.

**Outcome.** One rule does not match the stack and does not beat it. On −0.35 the visited floor drops by more than the noise, and every pile is longer than the stack's longest pile on that seed. Marking the failed spot does not change the result. The flag stays off. The old recovery code stays.

## 2026-11-04 — Planner disk and veto rectangle

**Mission parameter.** The stall may be two footprint models. The planner can accept a path the map veto then refuses. The test gives both the same outline and the same clearance. It does not change which goal wins, and it does not add a penalty. Recovery stays the stack (`RBM_RECOVERY_MODE` empty). Oracle stays off. A visited-floor change under about 6 points is noise.

**How they differ with the flag off.** The planner's free cells are known-free cells outside a disk of radius 0.20 m (`ROBOT_RADIUS_M`, `traversable`). That radius is the tire, 0.195 m from the centreline, plus a centimetre. The disk is round, so the heading does not matter. Path costs still prefer 0.30 m of clearance and charge 6 times inside 0.23 m. Those weights are not the footprint.

The turn veto sweeps a rectangle (`footprint_clips`). The outline is the ButlerBotSkid shell: nose 0.19 m, tail 0.19 m, side face 0.227 m. One clearance of 0.02 m is added on every side, so the box is 0.21 m forward, 0.21 m back, and 0.247 m to each side. The front corner of that box is about 0.32 m from the centre. A path whose centre stays 0.20 m off a mapped cell can still swing that corner into the cell. The veto drops the turn (`map:corner`).

A straight command does not use the rectangle. It is refused only when a mapped wall spans the centre and both samples 0.12 m to either side. A smear narrower than the shell does not stop a straight pass. That bar is a third shape, narrower than the disk and narrower than the rectangle. Live range is a sonar bubble. It is not either footprint.

**Change.** `RBM_FOOTPRINT_ALIGN` is off unless set. On, a planner step is legal only when that same rectangle, at the step's heading, does not cover a mapped occupied cell. A turn of more than about 57° is legal only when the rectangle stays clear through the yaw change. Straight commands use the rectangle sweep as well (`map:footprint`), not the 0.12 m bar. The 0.02 m clearance is the one the sweep already had. The 2.5 and 6.0 clearance weights, the goal score, the recovery stack, and the oracle are unchanged.

**Why.** If the stall is "plan says yes, executor says no", one shape should cut the refusals and the time spent sitting. If the visited floor does not move by more than the noise, the two shapes are not the stall.

**Evidence.** Five runs of −0.35 and five of north, flag on, code commit `9ed7ff2`. Oracle off. `RBM_RECOVERY_MODE` unset, so the recovery stack ran. Depth-noise seeds 63 and 64. Ports 1711 and 1712. `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Webots R2025a, fast, `--no-rendering`, headless. The cap is 600 s. The mapper still writes `mapping_done` when it finds no reachable frontier, and the pose logger then quits, so a run can end sooner. Sample standard deviation divides by 4. The visited figures are the scorer's percent to two decimals. The mean and the sample sd use those figures. Counts are in `docs/LESSONS_LEARNED.md`. The variance run folders are not on this machine, so there is no flag-off plan/veto count.

−0.35 visited %: 24.99, 6.01, 9.88, 11.00, 17.57. Mean 13.9, sample sd 7.5, range 19.0 points (6.0 to 25.0). The variance mean was 14.7. The gap is 0.8 points. Pile: 52.7 s, 519.9 s, 34.0 s, 114.2 s, 142.1 s. Mean 173 s, range 34 s to 520 s. The variance pile mean was 213 s, and its longest pile was 459.5 s. Run lengths (`mapping_done`): 576 s, 600 s, 127 s, 336 s, 488 s.

North visited %: 5.73, 10.90, 6.09, 22.84, 21.52. Mean 13.4, sample sd 8.3, range 17.1 points (5.7 to 22.8). The variance mean was 9.0. The gap is 4.4 points. Pile: 65.0 s, 355.6 s, 43.5 s, 56.1 s, 80.6 s. Mean 120 s, range 44 s to 356 s. The variance pile mean was 389 s. Run lengths: 134 s, 600 s, 161 s, 484 s, 600 s. The two north runs that lasted 600 s had piles of 356 s and 81 s.

Plan/veto disagreements, as paths issued then rejected by the map footprint. −0.35: 24 of 61, 3 of 9, 3 of 11, 5 of 13, 7 of 31. North: 6 of 12, 4 of 26, 1 of 13, 13 of 43, 18 of 55. Live range is not in that count.

**Rejected.** Turning the flag on in the harness. Reading the 4.4 point north gap as a gain. Reading the lower north pile mean as the sit ending: three north runs and four −0.35 runs stopped before 600 s, and a short run cannot grow a long pile. Treating one −0.35 pile of 53 s as the typical run.

**Outcome.** One shared footprint does not end the stall. The −0.35 visited mean matches the variance mean. The north visited gap is inside the noise. A full-length run still sat 520 s on −0.35 and 356 s on north. Issued paths are still rejected by the map veto. The flag stays off.

## 2026-11-04 — Quit reasons before another explorer patch

**Mission parameter.** Two reviews, read independently, agreed on one list: the start cell can sit in a free component that does not reach a frontier, a frontier ban does not lift, the top-8 cut happens before that filter, A* can hit 25,000 expansions, and the map veto refuses about a third of planned paths. Agreement is not a measurement. A visited-floor change under about 6 points is noise. The run may not hand the robot the floor plan or its own simulator pose.

**Change.** A quit line, always on, logging only. It fires when `select_goal` returns None and at the end of `_no_frontier` and `_settle`. The line carries the start cell, the jump from the robot cell, whether the longest frontier shares that component, raw frontier cells, clusters of at least 6 before and after the top-8 plus door cut, clusters dropped by bans, banned cells and regions, clusters with a viewpoint, A* attempts, failures, and cap hits, and which return fired. `reason` is `start_component`, `all_banned`, `astar_cap`, `cut`, or `other`. `frontier_ban` and `frontier_clear` event counts go to the run summary. Commit `3472086`.

`RBM_SHARED_FOOTPRINT` is off unless set. On, the planner's inflated free mask is the map test. The veto rejects a swept centre that has left that mask, and live range still rejects a real obstacle. The rectangle is not applied again. No new clearance. Goal scores, the recovery stack, and the oracle are unchanged.

**Why.** The rectangle flag made the planner use the veto's shape and the refusals remained. This trial makes the veto use the planner's mask, and records why a search returns nothing, before either mechanism is edited.

**Evidence.** Five runs of −0.35 and five of north, flag on, then two of each seed with the flag off. Code commit `3472086`. Oracle off. `RBM_RECOVERY_MODE` and `RBM_FOOTPRINT_ALIGN` unset. Depth-noise seeds 63 and 64. Ports 1711 and 1712. `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Webots R2025a, fast, `--no-rendering`, headless. All fourteen runs wrote 75,000 pose rows and `mapping_done` is 600 s. Sample standard deviation divides by 4. Counts are in `docs/LESSONS_LEARNED.md`.

Flag on, −0.35 visited %: 24.08, 5.94, 25.45, 15.92, 9.72. Mean 16.2, sample sd 8.6, range 19.5 points. The variance mean was 14.7. The gap is 1.5 points. The rectangle-flag mean was 13.9. Pile mean 259 s, range 80 s to 517 s. Map veto rejected 48 of 206 issued paths (23%).

Flag on, north visited %: 10.13, 11.10, 11.95, 5.25, 14.14. Mean 10.5, sample sd 3.3, range 8.9 points. The variance mean was 9.0. The gap is 1.5 points. The rectangle-flag mean was 13.4. Pile mean 363 s, range 291 s to 524 s. Map veto rejected 47 of 164 issued paths (29%). One run rejected 20 of 30.

Flag off, two runs each, for the dump. −0.35 visited 18.65 and 9.51, veto 11 of 41 and 3 of 13. North visited 22.69 and 7.42, veto 19 of 52 and 8 of 30.

Early-quit count: 0. The eleven `select_goal` misses (ten with the flag on, one with it off) are all `reason=start_component`. Each of those lines also shows the top-8 cut (26 to 43 clusters before, 11 or 12 kept) and `astar_cap=0`. None is `all_banned`, `astar_cap`, or `cut`. `banned_drop` is 0 on all eleven. The searches resumed. The body was still driving at 600 s.

**Rejected.** Editing `_nearest_free`, the ban lifetime, the top-8 cut, or the expansion cap from this sample. The label was the component split, and it did not end a run. Turning the mask flag on. Reading either 1.5 point visited gap as an effect. Reading 23% and 29% rejected paths as the veto having lined up with the planner.

**Outcome.** The mask flag stays off. The quit line stays, so the next patch has a measured label. This sample's label is the start component, with the cut visible on the same line and the cap unhit, and the robot still ran to the cap.

## 2026-11-04 — Start in the robot's component, and read the pile

**Mission parameter.** The quit dump named `start_component` and the runs still reached 600 s. The next measurement is whether A* should start in the component the robot is actually standing in, and what command a long sit is made of. No new clearance. Recovery and scoring stay as they are. A visited-floor change under about 6 points is noise. 3b is the planner-mask trial (means 16.2 and 10.5).

**Change.** A pile line, always on, logging only. After the existing 60 s inside 0.45 m, one line a second records phase, goal cell, age, attempts, path length, next waypoint, pure-pursuit target, every motion candidate and its veto, the chosen `(v, w)` and its source, live sensor blocks, and odometry delta against the command. Leaving the ball writes the hold, veto, and commanded-but-still shares, the recovery rules that fired, and one plain sentence. `RBM_TRUE_START` is off unless set. On, the robot cell counts as free and a walk that does not step on occupied cells, limited to the same 40 cells as `_nearest_free`, picks the planner-free cell in that component. Commit `33bf583`.

**Why.** The straight-line snap can start in another room. The pile line answers a different question: once the shell has already sat for a minute, is the tick a veto, a zero command, a recovery loop, or a command the pose does not follow.

**Evidence.** Five runs of −0.35 and five of north, flag on, then two of each with the flag off. Pile line on in all fourteen. Oracle off. `RBM_RECOVERY_MODE`, `RBM_FOOTPRINT_ALIGN`, and `RBM_SHARED_FOOTPRINT` unset. Depth-noise seeds 63 and 64. Ports 1711 and 1712. `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Webots R2025a, fast, `--no-rendering`, headless. Counts are in `docs/LESSONS_LEARNED.md`.

Flag on, −0.35 visited %: 19.70, 17.40, 14.16, 10.81, 11.12. Mean 14.6, sample sd 3.9, range 8.9 points. The variance mean was 14.7. The gap is 0.1 points. The 3b mean was 16.2. Pile mean 212 s. Run 5 ended at 329 s.

Flag on, north visited %: 22.88, 26.51, 13.52, 7.54, 4.69. Mean 15.0, sample sd 9.5, range 21.8 points. The variance mean was 9.0. The gap is 6.0 points. The 3b mean was 10.5. Pile mean 219 s, including one pile of 536 s. Run 4 ended at 244 s.

`start_component` count: 0. The two early stops are `return=start_none` `reason=other` (31 misses on −0.35 run 5, 14 on north run 4). The escape walk found no free cell inside 40 cells. The four flag-off runs all lasted 600 s and had no `select_goal` miss. −0.35 flag off visited 20.63 and 9.36. North flag off visited 22.66 and 8.17.

On the summaries the dump did open, hold is 0–4% and a fully vetoed candidate list is 0%. The large share is recovery. North run 3's 60 s summary is 91% recovery (`stall` 6,203 of 7,522 ticks) and 33% commanded-but-still. −0.35 run 3's 65 s summary is 78% recovery (`back` 3,990, `narrow_gap` 1,722, `pile_leave` 645) with commanded-but-still at 1%: the odometry followed a local back-and-creep. North run 5's 536 s ground-truth pile and −0.35 run 4's 443 s pile never opened a summary. The estimate's poses in those windows span 0.36 m and 0.41 m, and the 60 s anchor kept resetting.

**Rejected.** Turning the flag on. Reading the north mean of 15.0 as a gain: the sample sd is 9.5 and two runs stopped early. Lengthening the 40-cell walk. Treating the 0% veto share as proof the map veto is idle outside a pile. The summaries start only after 60 s, and by then the tick is already a recovery command.

**Outcome.** `RBM_TRUE_START` stays off. The pile line stays. A stay the estimate holds for a minute is a recovery loop, chiefly `stall`, `pile_leave`, `back`, and `contact`. It is not a veto hold and it is not a zero command. The component-start trial removed `start_component` from this sample and added two `start_none` stops.

## 2026-11-06 — Trusted wheel travel, and a best-center pile

**Mission parameter.** The previous pile line missed a 536 s sit and a 443 s sit because the entry anchor reset. The opened summaries were recovery, often `stall`. The question for this round is whether that sit is the stall state machine, a command the pose does not follow, or the estimate drifting while the body sits. One behavior change, and only if that read supports it. No new clearance. Flags stay off unless a run shows a reason to set one. A visited-floor change under about 6 points is noise. The variance means are 14.7 and 9.0. The 3b means are 16.2 and 10.5.

**Change.** The pile dump follows the estimate's own best center: a sample every 0.25 s, the longest suffix inside 0.45 m of one of those samples, the 1 Hz line once that stay reaches the existing 60 s. The summary records commanded distance against the estimate's path. Ground truth is not read, and this clock does not arm the pile guard. Stall escape progress uses wheel travel the step trusted, at the existing 0.30 m, 1.0 m, 0.02 m, and 0.05 m checks. A pose jump is not travel. Commit `2774b24`.

**Why.** The missed sits spanned 0.36 m and 0.41 m on the estimate, so a center on that pose would have opened the dump. The stall exit was a pose distance, which a match correction can satisfy without the shell leaving.

**Evidence.** Five runs of −0.35 and five of north, all flags off. Depth-noise seeds 63 and 64. Ports 1711 and 1712. `RBM_MAP_MAX_S=600`, `RBM_DECISION_LOG=1`, `RBM_MOVIE=1`. Webots R2025a, fast, `--no-rendering`, headless. All ten runs wrote 75,000 pose rows and `mapping_done` is 600 s. Sample standard deviation divides by 4. Counts are in `docs/LESSONS_LEARNED.md`.

−0.35 visited %: 18.05, 12.38, 4.19, 3.76, 12.64. Mean 10.20, sample sd 6.12, range 14.3 points. The variance mean was 14.7. The gap is 4.5 points. The 3b mean was 16.2. Pile mean 364 s, range 104 s to 553 s.

North visited %: 3.40, 10.87, 9.16, 9.69, 5.09. Mean 7.64, sample sd 3.22, range 7.5 points. The variance mean was 9.0. The gap is 1.4 points. The 3b mean was 10.5. Pile mean 464 s, range 378 s to 565 s.

Early-quit count: 0. `start_none` count: 0. −0.35 run 2 has `reason=start_component` at 18 distinct times. The run still reached 600 s.

The best-center line opened on the long sits. North run 1's 564 s ground-truth pile is 483 s of `src=stuck`, commanded 48.2 m, estimate path 1.01 m. −0.35 run 4's 553 s pile is 448 s of `src=stuck`, commanded 45.6 m, estimate path 0.39 m. The closed summary on −0.35 run 1 is the other kind: commanded 1.81 m, estimate path 1.61 m, veto 0%, recovery `contact` / `stall` / `narrow_gap`. A sit that lasts until the cap still writes no `pile_end`. The pose logger quits in the step that marks the run done.

**Rejected.** Reading either visited gap as an effect. Turning a flag on. Deleting the recovery stack. Adding a rule or a threshold aimed at `stuck`. Treating the stall travel change as what ended, or failed to end, these sits: `_stuck_escape` runs first and clears the stall latch.

**Outcome.** Flags stay off. The best-center dump stays, because it shows the command and the estimate path on a sit the entry anchor used to hide. The stall travel rule remains in the measured commit and is not a reason to patch Explorer again. The open mechanism is a stuck reverse of 0.10 m/s that neither the estimate nor the body follows.

## 2026-11-06 — The reverse reaches the hinges

**Mission parameter.** Trace why a stuck reverse does not move the body. Separate a blocked shell, spinning wheels, a command that never becomes a motor setpoint, contact geometry, and a depth check that only freezes the estimate. Ground truth is for the read. No new recovery rule and no new knob. A floor or proto change waits on Dan.

**Change.** When the run is marked done, the open pile summary is written before `mapping_done`. The summary adds `wheel_m`, the hinge travel over that window, next to `cmd_m` and `pose_m`. Driving is unchanged. Flags stay off.

**Why.** A sit that lasts until the cap never left the 0.45 m ball, so the summary was skipped and the pose logger then quit. The old `cmd_m` against `pose_m` could be read as "the motors did not turn." The hinge sensor is the measurement that splits that.

**Evidence.** No new Webots run. North run 1 of the trusted-travel round, the doorway sit. From t=60 s to t=600 s the post-safety setpoint is `(-1.25,-1.25)` rad/s and each hinge moves `-12.49` rad per 10 s. True path in those bins is 0.000 m. The estimate moves about 0.04 m per 10 s. The right hub falls from z=0.080 at t=41 s to z=-0.003 at t=44 s, while the command is still a forward mix and the body still has speed. `up_z` settles at 0.953. Wall contacts stay 0. The living floor's west face is x=3.759 and the bedroom floor's east face is x=3.658, a 0.102 m slot with no bridge at y=4.96. The shell centre is x=3.88.

`_stuck_escape` is the source of the equal reverse after t=47 s. It is not the source of the wheel leaving the plate.

Two other shapes in the same ten runs, same actuator fact: north run 2 at t=200 s is level with only one wheel reporting the floor, and north run 5 at t=200 s has both rear wheels on a wall while the hinges still match `-1.25` rad/s. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A new reverse or a threshold. Trusting the free-spinning encoders in the pose. Editing `apartment.wbt` or `ButlerBotSkid.proto`. Reading the depth freeze as the cause of the stop. Running another five-by-five with no driving change to score.

**Outcome.** The summary closes at the cap and names hinge travel. The body stops because contact is lost or a tire scrubs a wall, while the motor does what it was told. The geometry decision is Dan's.

## 2026-11-06 — Fill the doorway slots Dan said are not real

**Mission parameter.** The real apartment has no floor gap. Find where the 0.102 m slot came from, check every walkable seam rather than one doorway, and fill only the accidental holes. Doors, walls, furniture, friction, motors, and the controller stay. The warehouse stays. No new flag.

**Change.** Two flush carpet plates, the same footprints the removed edges had. `APT_FLOOR_BRIDGE_BEDROOM` fills door G. `APT_FLOOR_BRIDGE_BED_BATH` fills the bedroom-bath walkway. Commit `7590d16`. `tests/test_apartment_floor.py` fails if an interior wall opening loses its floor, and it does not require a plate under a closed door or outside the walls.

**Why.** Room plates have ended at the wall faces since `9e335e7`. The 0.102 m is that 4 in partition. The doorway plates were removed in `0cd3694` with the carpet ramps. A wheel then fell through door G. The lip Dan measured is about 10 mm and was already not a collider.

**Evidence.** The opening audit is in `docs/LESSONS_LEARNED.md`. Door H, the entry, the laundry door, and the closet opening were already bridged. Door G and the walkway were the only open slots. A 60 s spawn facing door G crossed the slot at t=23–26 s with hubs at z=0.080 and all four wheels on the floor. Five runs of each seed, 600 s, flags off. No hub below z=0.078. Early-quit count 0. −0.35 visited mean 14.79 (sd 3.71), pile mean 231 s. North visited mean 11.29 (sd 5.25), pile mean 329 s. Both visited gaps versus 14.7 and 9.0 are inside the 6 point noise. Long sits that remain have a tire on a wall or a wheel with no floor contact, and the hub stays at ride height.

**Rejected.** Calling the floor gap-free from door G alone. Filling the front door or the patio door. Moving a wall to close the slot. Reading the visited means as the effect of a controller change. Using the gapped-floor runs as a clean driving baseline.

**Outcome.** The accidental holes are closed. The tests name the openings. Wall scrubbing is still in the runs and was not patched.

## 2026-11-07 — Cap a stuck reverse the picture does not see

**Mission parameter.** On the repaired floor, long sits still put a rear tire on a wall, and one sit had no wall contact and no four-wheel floor contact. Find why the reverse continues, from the robot's own sensors. One correction only if that read supports it. No new distance, no new recovery rule, no world or friction edit.

**Change.** While the forward picture stays frozen, hinge travel of a straight stuck reverse is counted. At the existing `BLIND_BACK_CAP_M` (1 m) the stuck escape yields. A 0.05 m change in the depth signature clears it. The reverse still starts. Commit `6281d58`.

**Why.** The repaired-floor sits commanded `(-1.25,-1.25)` rad/s for the whole window and the hinges matched every 1 s bin. The forward nose was 0.9–2.2 m. `camera_stuck` is that frozen picture plus a nonzero command, and the escape is the command, so it does not end. The encoders are the evidence the hubs turned. The picture is the evidence the shell did not.

**Evidence.** Five runs of each seed on the repaired world, flags off, depth seeds 63 and 64. The previous repaired-floor five was not rerun. All ten reached 600 s. Early-quit count 0. −0.35 visited mean 18.89 (sd 6.94), pile mean 208 s, against 14.79 and 231 s. North visited mean 16.99 (sd 6.27), pile mean 225 s, against 11.29 and 329 s. The visited gaps are 4.1 and 5.7 points. North run 5 still reversed for 259 s with both rear wheels on a wall. −0.35 run 3 still reversed for 289 s, and much of that was `pile_leave` after the stuck escape yielded. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Driving forward on the first stuck tick because the nose is open. The press test requires the reverse to start. Reading the visited means as a result. A second patch aimed at pile-leave in this same round. Editing the world or the tire friction.

**Outcome.** The stuck reverse yields after 1 m of hinge travel with a frozen picture. Pile-leave then repeats the reverse, and the wall sit remains.

## 2026-11-08 — No command change after the collision check

**Mission parameter.** Find why pile-leave repeats the reverse after the stuck escape yields, and check the hidden collision shapes before another behavior patch. Dan's hypotheses: a collision tire about 0.35 mm above a visual tire that looks grounded, or intermittent chassis and corner wedging. A permanent gap would also appear in ordinary driving. Contact counts can be missing. No world, proto, friction, or motor edit unless the evidence is brought back first.

**Change.** None in the controller, the proto, or the world.

**Why.** The collision cylinder and the wheel-mesh vertices are both radius 0.08 m. In ordinary driving the hub median is z=0.08000 and the contact median is z=0 while the shell moves at about 0.15 m/s. The ±0.35 mm body motion in those windows is bounce around that contact, not a resting clearance. The chassis boxes bottom at z=0.105, 25 mm above the tire contact.

**Evidence.** Saved logs only. Repaired-floor north run 1, t=250–260 s: rear hubs z=0.09425, fronts z=0.08000, rear floor contact 0, left rear wall contact 0.49, body z=0.00729. The rear axle is 14 mm up. Yield north run 5, t=300–320 s: all hubs z=0.080, both rear wheels on a wall (0.80) and on the floor (0.45), body level. `_stuck_yield` is not an input to `_straight_leave`. Counts and the rejected patch are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A pile-leave patch this round. Editing the tire radius or the friction to chase 0.35 mm. Treating an absent floor count as proof the wheel is in the air when the hub height already says otherwise, or the reverse.

**Outcome.** The collider supports ordinary driving. Corner sits are a lifted rear axle or a tire scrubbing a wall. The handoff forgets the failed reverse, and that is not enough, on this evidence, to change the command.

## 2026-11-08 — The rear sonar box holds the pitched shell

**Mission parameter.** Name the contact that held the rear hubs at z=0.09425, and record why pile-leave still reverses after the stuck escape yields. Diagnosis and a log line only. No driving change, no proto or world edit.

**Change.** Pile-leave logs its branch, the forward gap, the rear gap, the map clip, and the stuck-yield flag, once a second. Commit `6ac2026`. `eval/score_rooms.py` is deleted in `233ed4b`. Nothing imported it. The room percent the runs use is still `eval/score_apartment_map.py`.

**Why.** The hub height says the rear axle is up. It does not say which shape is under it. The yield flag is not an input to `_straight_leave`, so a later reverse can be chosen for a different reason.

**Evidence.** Repaired-floor north run 1 at t=175.2 s, pose (4.696, 2.489), yaw 3.119. The rear sonar box corner is 0.005 mm from `apt_furn_island`'s west face. The rear-left tire is 0.56 mm clear of that face. Front hubs stay at z=0.08000. From t=180 s the pitch values do not change while the hinges keep turning −12.5 rad per 10 s under `(-1.25,-1.25)`. The climb starts at t=156–157 s with the left rear tire still at z=0.080 and a wall contact, command reverse. Yield north run 5 at t=300.5 s has both rear tires on `APT_WALL_SHARED_A` at hub z=0.080. A 240 s north probe with the new line did not open a pile. The unit test records `straight_back:rear_clearer` when the rear gap is 2.00 m and the forward gap is 0.40 m, with yield still set.

**Rejected.** Another reverse rule. A tire-radius or friction edit. Treating the 14 mm lock as the 0.35 mm free-drive bounce. A five-by-five to read one branch line. Merging PR #40: its CI failure is still the `allocation_snapshots` race, and this round does not merge or close it.

**Outcome.** The prop is the rear sonar collider on the island. The scrub is the rear tires on the shared wall. Pile-leave can still choose reverse because the rear gap is larger, and it does not consult the yield. The old sit was not re-logged.

## 2026-11-08 — The escape block rewrites a forward leave

**Mission parameter.** Reproduce a recovery stall with the pile-leave branch line and name the reverse that is actually sent. The unit test `straight_back:rear_clearer` is not evidence of a live sit. Diagnosis only. No driving change, no new clearance, no proto, world, friction, or motor edit.

**Change.** Commit `be9bac7` logs a pile-leave branch as soon as the branch string changes. A one-second throttle had kept the first branch of a second and dropped a command that started a tenth later. The command is unchanged.

**Why.** The branch line and the wheel command were not the same tick. Logging the change makes a forward branch sitting next to `v=-0.10` mean something.

**Evidence.** Three 600 s probes, flags off, no scoring batch. Configurations and the trace are in `docs/LESSONS_LEARNED.md`. North seed 64 on `be9bac7` commands a straight reverse for 340 s. Every pile-leave branch in that run is `straight_forward` or `nose_forward`. At t=436.9 s the branch is `straight_forward` with `clip_fwd=0` and `yield=0`, and the phase line is `src=pile_leave v=-0.10`. `_respect_escape_block` runs after the branch is logged and replaces a forward command along the stuck heading with `wheel_speeds(-0.10, 0)` for 18 s. The log of that replacement is `blocked heading, not driving back in`. The body stays near (3.86, 5.39) at hub z=0.080 while the rear sonar box is 0.93 mm from `APT_WALL_SHARED_C` and the rear tire is 19 mm clear. A 1.2 s pitch to hub z=0.09491 at the same jamb releases. The island lock is not in these probes. −0.35 seed 63 does not stall; its `trail_reverse` moves the body 1.02 m.

**Rejected.** Reading `rear_clearer` out of the unit test as the live branch. Reading a `nose_forward` line as the sent command when the wheels are `(-1.25,-1.25)`. Calling a wheel-wall count of 0 a clear tail while the sonar box is on the jamb. Treating the 1.2 s pitch as the sustained fault or as the island. Another clearance parameter. Merging or restacking PR #40.

**Outcome.** The sustained reverse is the escape block, applied after pile-leave has chosen forward. The ride-height doorway sit is the rear sonar box on `APT_WALL_SHARED_C`. The nightstand sit is both rear treads on the furniture at hub z=0.080. Neither is the island prop. No behavior change in this round. The next change, when one is wanted, is to stop the block from repeating a reverse that has not moved the shell.

## 2026-11-09 — Stop a blocked reverse that does not move the shell

**Mission parameter.** Dan authorized the correction named above. Keep the safeguard: do not drive forward along the blocked heading on the first tick, and do not replace the reverse with an endless stop or an endless spin. No new knob. Controller pose only. Ground truth stays off the controller.

**Change.** Commit `9ebb7cd`. While the heading is blocked, a straight reverse is still the first substitute. If the planner pose has not moved `STILL_M` in `STALL_WINDOW_S`, that reverse is not sent again and `_stuck_escape` does not start it either. A turn off the heading follows, then the other sign. A direction that does not change yaw by 0.12 rad in that same window is dropped. After both are quiet, a forward the caller already chose is left alone. The latch clears when the shell leaves the 0.70 m spot or yaws past `_ESCAPE_BLOCK_YAW`. A frozen picture does not use the pose clock. The hinge cap still ends that reverse.

**Why.** The doorway sit rewrote `straight_forward` into `v=-0.10` for minutes while the body stayed at hub z=0.080. The encoders were turning. The planner pose was not. The frozen-picture hinge cap never accumulated because the picture kept changing.

**Evidence.** Same north seed 64 and spawn, flags off, 600 s. Straight reverse fell from 340 s to 81 s. The longest span after the change moved the body 1.81 m. Five runs of each seed are in `docs/LESSONS_LEARNED.md`. North visited 26.28, 3.15, 15.31, 9.06, 11.86 (mean 13.13, sd 8.59). −0.35 visited 22.28, 10.57, 15.87, 7.89, 14.89 (mean 14.30, sd 5.51). −0.35 run 3 stopped at 456 s, `start_component`. North run 4 reversed into the island while the pose was moving and finished inverted. These visited figures are not a matched five against the previous code.

**Rejected.** Removing the rewrite. A new threshold. Counting wheel radians as motion. Applying the 2 s clock to a frozen picture. Another recovery rule for the island climb or for spins that are not this block. Merging PR #40.

**Outcome.** The safeguard still backs up first, then turns. It does not keep backing up once the pose has stayed put. A reverse the pose follows, and a frozen-picture reverse inside the hinge cap, are unchanged.

## 2026-11-09 — The island climb was accepted wheel travel

**Mission parameter.** Explain why north run 4's reverse onto the island moved the planner pose and continued until the shell was inverted. Diagnosis only. Ground truth stays off the controller.

**Change.** None in the controller.

**Why.** The still-pose gate can only stop a reverse the estimate does not follow. If the estimate follows the wheels up the cabinet, the gate sees progress.

**Evidence.** Saved logs of that run, commit `9ebb7cd`. From t=304 s to t=310 s, command `(-1.25,-1.25)`, planner displacement 0.49 m, wheel arc 0.51 m, ground-truth floor travel 0.18 m, body z 0 to 0.185, `up_z` 1.000 to 0.057, both rear hubs 0.080 to 0.300. Applied visual, floor, and wall yaw are 0. No slip or floor-still line in that window. The nose goes from 0.87 m to the blind placeholder as the shell pitches. A 50 s start at the pre-climb pose did not climb. North run 3's hub z=0.445 is a 0.9 s toss near (1.00, 4.67) during which the planner pose does not move. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A behavior change this round. Reading the pose shift as visual odometry. Treating run 3 as the same climb. Another full north seed in place of the saved log.

**Outcome.** The translation is the wheel step, trusted because the forward range leaves the slip band and the floor image never becomes the still picture the pitch gate waits on. The pitch is already measured. It did not veto the travel.

## 2026-11-09 — The raw pitch gate dropped level driving

**Mission parameter.** Apply the existing 0.12 rad pitch gate to wheel translation without waiting for a still floor image, so a climb does not count as progress. Revert if it does not help or if it regresses.

**Change.** Commit `2db10f5` dropped the wheel step when `oak_imu` pitch was more than 0.12 rad from the learned rest, and logged that pitch. The commit was reverted. The floor-still branch is again the only user of that threshold.

**Why.** On the saved island climb the body really did pitch past 0.12 rad, and the pose followed the wheels. On the matched runs the same comparison fired while ground truth stayed level.

**Evidence.** New north run 1 logs `shell pitched 0.11 rad` at times when `up_z` is 1.000, the hubs are z=0.080, and the speed is about 0.10 m/s. Matched five-run means: north 13.64 (sd 6.81) with the gate, 21.93 (sd 3.86) on `9ebb7cd`. −0.35 16.83 (sd 3.06) with the gate, 10.86 (sd 1.95) without. No `up_z` below 0.5 in any of the twenty runs. The movie run stayed 2.9 m from the island. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Shipping the gate. Calling the −0.35 rise a win. A new threshold. Editing the IMU noise or the proto.

**Outcome.** Wheel translation is not dropped from pitch alone. The island climb is still trusted wheel travel. A later pitch veto has to be a tip of the body, not one reading against a single rest sample.

## 2026-11-09 — A flat floor frame is not wheel travel

**Mission parameter.** Accept wheel travel into the planner pose only when the floor camera agrees. A frame that cannot confirm motion should block translation, not only a frame that is confidently still. Do not retry the single-rest pitch gate. Do not learn a rest while driving. `oak_imu` stays off the controller if it is ground-truth attitude.

**Change.** Commit `b1827e7`. A floor step that returns `blank` with frame std under 2 rewinds x,y to the last anchor once the hubs have claimed 0.02 m, and holds translation for the same 0.12 s a still floor already uses. A `moving` step accepts the travel and moves the anchor. A blank that still has texture restarts the anchor.

**Why.** On a level reverse the camera confirms motion. On the island face it goes blank while the wheels turn, and the old code restarted the anchor on that blank, so the pose followed the wheels. The std cutoff and the 2 cm claim are the tracker's existing blank and still tests.

**Evidence.** The forced reverse into the face holds the planner pose on the anchor after the image goes flat. The tracker still says `moving` while `up_z` is falling through 0.8, so that part of the climb is still accepted. The probe keeps commanding the reverse and the shell inverts; the pose does not follow the rest. `oak_imu` is an InertialUnit with noise 0.01. Matched five-run means: north 14.25 (sd 2.40) with the hold, 16.65 (sd 9.21) on a fresh `9ebb7cd`; −0.35 13.01 (sd 2.31) and 12.90 (sd 5.78). All twenty ran 600 s. No `up_z` below 0.5. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** The single-rest pitch gate, already reverted. A rest that updates during driving. An `oak_imu` translation veto. Calling a wrapped correlation a stall. Treating the 2.4 point north gap as a regression.

**Outcome.** Flat blanks are not travel. Textured motion is. The scored runs did not climb the island, and the part of a climb the camera still calls `moving` is still trusted.

## 2026-11-09 — A held pose cuts the blocked reverse

**Mission parameter.** Once the floor-blank hold pins planner x,y, check whether the existing blocked-heading rule stops that reverse. Change the controller only if the logs show it does not.

**Change.** None. The rule already stops a reverse when the planner pose has not moved 0.04 m in 2 s, then turns off the heading.

**Why.** The forced reverse that inverted was a scripted backup. It never armed the blocked heading, so the pose clock was not in the path. With the heading armed, a held pose is the lack of progress that rule already measures.

**Evidence.** The step loop with a flat floor and a changing forward picture cuts the reverse 2.0 s after it starts, and the pose stays under 0.04 m. On the pre-climb pose, with the heading armed for the probe only, the reverse is cut at explorer t=2.4 s. `up_z` falls to 0.395 during the following turn and is 1.000 again by sim t=12.5 s, hubs at ride height. The unarmed probe of the same reverse finished inverted. The probe hooks were removed. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A shorter than 2 s clock. Dropping the frozen-picture exemption; this reverse was cut before that picture froze. Folding a pile-leave that has cleared the block into the same clock. Retiming the stall escape's 0.8 s check, which counts wheel travel the pose did not take and was not the island command.

**Outcome.** A held pose during a blocked-heading reverse stops that reverse. The command path is unchanged, so there is no new matched five.

## 2026-11-09 — A short floor shift is not the wheel step

**Mission parameter.** Find whether the early island climb, while the floor tracker still says `moving`, is accepted as planner travel. Change the controller only if that window is trusted, and only with an existing signal.

**Change.** Commit `fec4ff8`. A `moving` step keeps the wheel translation when the peak is within the existing 2 px still gate of the metres that step claimed, at the camera's own metres per pixel. A shorter peak rewinds x,y to the anchor, the same hold a flat blank uses.

**Why.** `moving` meant the peak cleared 2 px, and that bit kept the whole wheel step. On open floor the peak is 8–11 px for a centimetre of wheels. On the island face it is 2–4 px for the same claim, and from sim t=5.5 s to t=6.5 s the pose moved with the wheels (0.085 m vs 0.088 m) while `up_z` fell from 0.730 to 0.469.

**Evidence.** After the check the same forced reverse holds the planner pose at 0 through that window. The open-floor reverse still matches the wheels and does not log a short shift. Matched means against `222da06`: north 13.21 (sd 6.69) vs 10.71 (sd 4.74); −0.35 14.37 (sd 3.17) vs 16.73 (sd 5.31). All twenty ran 600 s. No `up_z` below 0.5. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Raising the bar with the existing 30% stall ratio. Those climb steps are at 0.27–0.36, so the line that already means "not motion" still lets the pose walk. A new fraction. The pitch gate. `oak_imu`. Another change to the 2 s reverse clock.

**Outcome.** A peak that misses the wheel step by more than 2 px is not travel. A peak that matches the wheels still is. The early climb no longer counts as planner progress.

## 2026-11-09 — The short shift and the reverse-stop together

**Mission parameter.** The early-climb window is already closed in `fec4ff8`. Check that window together with the existing reverse-stop, and do not add a rule the matched runs do not support.

**Change.** None.

**Why.** With the heading armed, the short shift holds the pose from the first centimetre, and the 2 s clock then cuts the reverse. The matched runs' long reverse spans are the shell moving, pose path beside wheel path, not a climb accepted as travel.

**Evidence.** Pre-climb pose, depth seed 64, heading armed for the probe only. Pose stays at 0. Reverse cut at explorer t=2.4 s. `up_z` minimum 0.395, upright again by sim t=12.5 s. North run 5 of the matched set: turn 44%, forward 21%, straight reverse 18%; pose path 25.3 m and wheel path 26.6 m over sim t=31–300 s. iRobot US 9,317,038 (stasis wheel) and US 9,744,670 (underside camera, trust it when quality is high, drop wheels when they disagree with an IMU). Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A tighter peak ratio. A shorter stall window. An `oak_imu` tilt gate. A new escape because the decision log says `cant_see_behind_backing_out` often. That label is every logged reverse, and those reverses move the shell.

**Outcome.** The honesty gap in the early climb is closed. The residual pitch during the turn is the existing 2 s. The next sensor those patents use is not on this robot as a controller input.

## 2026-11-09 — The apartment job is the visited floor, and this tip does not finish it

**Mission parameter.** Say what an apartment job already means here, and whether tip `0351bf4` finishes it. Do not add a score. Do not chase the leftover pitch.

**Change.** None.

**Why.** Done, in the mapper, is no reachable frontier after a final scan and a map that stays still for 4 s while the shell is stopped. The number beside that is visited %, free floor the shell drove over, per `APT_FLOOR_*` solid. Seen % is not the job.

**Evidence.** The ten 600 s runs of this controller. Every one ends at the cap, still in `go`, with no done line. Pantry and storage are 0 on all ten. Closet is 0 except one run at 1%. Bath is 0 on eight. The best visited draw is north run 5 at 23%, and its pose path matches its wheel path. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A new completion percent. A recovery change inferred from how often a reverse is logged. Another five aimed at `up_z` 0.395.

**Outcome.** The job is blocked by time, with most rooms unvisited. The logs do not name one command change that would finish it.

## 2026-11-09 — Seen rooms are not goals, and the bath frontier loses

**Mission parameter.** Name why pantry, storage, and closet stay at 0% visited, and the bath on eight of ten runs. Use the ten runs already scored. Do not add a goal weight.

**Change.** None.

**Why.** A frontier is free next to unknown. Depth through a door paints the room free and the frontier goes away. The unvisited-stand goal is scored at `min(area, 4) / length * 0.45` so it does not beat a frontier that is still unknown. These runs die at the 600 s cap while other frontiers remain, so that stand never becomes the only choice.

**Evidence.** Pantry: 0 goals inside on all ten. Storage: one goal, and the shell was 1.66 m away. Closet: one goal, shell 0.59 m away, visited 0; m3's 1% is a 0.22 m brush. Storage and closet are often 90% seen. Bath: a frontier remains inside on the runs that stay out, and a goal is selected with the shell there only on m1 (stopped at 0.19 m), m3, and m5 (entered). Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** One tweak aimed at the bath. Raising the 0.45 so a seen room outranks a real frontier. That is a new weight, and the small rooms are not failing at a door the shell was crossing.

**Outcome.** The small rooms are seen and then not selected. The bath frontier is usually not selected before time runs out. No controller change.

## 2026-11-10 — The frontier that wins is outside the floor

**Mission parameter.** At the 600 s cap, say whether the frontiers that outrank the bath and the unvisited stand are real unknown floor or junk. Do not raise the 0.45. Pantry and storage are closed closets.

**Change.** None.

**Why.** Score is cluster length over path length. A stand capped at 4 m² and scaled by 0.45 scores about 0.80 when the path is 2 m. Anything longer and closer beats it, and beats a shorter bath cluster too.

**Evidence.** The ten maps. 48% of the cells in clusters of at least 0.30 m sit on a floor plate (bedroom 15%, living 14%, bath 10%, kitchen 5%). 42% sit more than 0.35 m outside every plate, in a ring 0.4–1 m past the walls on every side. Rank uses straight-line distance in place of the A* path. North run 5's ring is 51 m long and scores 28; its best bath cluster scores 0.28. That outside cluster is the top score on eight runs. On m2 and m4 a bedroom frontier is the top score. Clusters under 0.30 m are not scored. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Calling the leftover only real unknown, or only speckle. Raising 0.45. Dropping every cell that misses a floor plate. A yaw error can move a real edge into the 0.4–1 m band.

**Outcome.** The score is won by free space the map drew past the floor and connected into one frontier. Real unknown remains, and on these runs it loses to that perimeter. No controller change.

## 2026-11-10 — A spread-out loop past the plates does not keep its length

**Mission parameter.** One scoring change so a long exterior halo cannot beat a real in-room frontier on the ten maps. Do not raise the 0.45. Do not drop every cell that misses a plate. Do not open pantry or storage.

**Change.** `frontier_credit_m` in `src/mapping/frontiers.py`. A cluster keeps `len * res` unless more than half its cells are over 0.35 m from every floor plate and the cells average more than 1.2 m from their own center. Then the credit is the longest strip within 0.40 m of one centerline, times the share of that strip that is not far from a plate. The apartment controller passes the floor solids and the spawn, read once. No plates, and the score is the old one. The inertial unit is not read for this.

**Why.** The loop is one cluster because the free cells past the walls connect around the building. Cell count times 0.05 m turns that into 51 m on north run 5. A yaw-shifted wall is off the plate too, but it is local, so the radius test leaves it. An on-plate bedroom frontier is long and spread out, but it is not mostly past the plates, so it keeps its length.

**Evidence.** The ten saved maps, `select_goal` at the final pose. Before: a majority-off-plate cluster on n1, n2, n3, m3, n5. North run 5 scored 19.5. After: n5 living 3.76, n2 living, n3 bedroom, n1 bath 2.15. m5 was already the bath. m3's kitchen-stand loop (80% off, score 2.86) loses to a tighter cluster (51% off, score 1.91) whose cells are close to their own center. Counts are in `docs/LESSONS_LEARNED.md`. Mapping tests: 155 passed. No new 600 s batch.

**Rejected.** Raising 0.45. Zeroing each off-plate cell. Cutting every long cluster down to a strip. The m4 bedroom frontier is on the plate and is the right winner.

**Outcome.** The building loop is not the goal on those five poses. The bath is the goal on n1 and m5. On n2, n3, and n5 a living or bedroom frontier outranks it. m3 still selects a compact cluster that is barely majority-off-plate.

## 2026-11-10 — Live ten with the loop credit

**Mission parameter.** Same ten-run honesty check, live, under the new score. North seed 64 and −0.35 seed 63, five each, flags off, 600 s. Compare visited % with the pre-fix five (controller `fec4ff8`, notes through `0351bf4`). Do not raise the 0.45. Do not open pantry or storage.

**Change.** The score is `058387a`. The first launch never latched the plates: `Robot` has no `getSelf`, so those runs were the old score and were discarded. `a307819` reads the spawn from `RBM_SPAWN` or from the supervisor node, once. The ten below each log 15 plates. No geometry, clock, or inertial change.

**Evidence.** All ten reach the cap still going. Nine logs say `time_cap`. m2 is in `spin` at the cap. n4's log stops at phase `go`, t=599.5 s, with `mapping_done` 600.000 and no reason line. None log the mapper done line. No `up_z` below 0.5.

North visited 18.48, 10.73, 12.55, 9.14, 20.69 (mean 14.32, sd 5.02) against 13.21 (sd 6.69). −0.35 visited 14.97, 14.14, 7.96, 9.57, 11.40 (mean 11.61, sd 2.97) against 14.37 (sd 3.17). Bath is 0 except n5 at 44.9. The walk-in is 0 except n5 at 1.2. Pantry and storage are 0. Goal stands sit on a floor plate for most of each run. The longest off-plate stand is 125 s (m2). n2 aims into the bath for 296 s and does not drive on it. The room table is in `docs/LESSONS_LEARNED.md`.

**Rejected.** Another scoring pass on this sample. The visited gaps are inside the old scatter. One bath entry does not make a rule.

**Outcome.** Live, the shell is not spending the night on the outside loop. Coverage is not past the pre-fix sample, and the bath is still missed on nine of ten.

## 2026-11-10 — The bath stand stays selected while the command is zero

**Mission parameter.** From the live ten, say why a stand inside a room does not become a visit. The sharp case is n2: 296 s of bath stand, bath visited 0. Do not change the controller.

**Change.** None.

**Why.** A stand warped into a floor plate is not yet a visit. The visit is the true shell on that floor.

**Evidence.** n2, explorer t=317 s to the cap. The stand warps to (1.07, 2.50), inside the bath, 1.1 m from door H. The true shell is (0.77, 4.92), bedroom, 0.38 m north of the bath floor at the bedroom walkway. Pose error 0.24 m. True path over that window 0.04 m. The pile is 314 s of hold or zero, `cmd_m` 0, `wheel_m` 0, and the goal at the end is still (0.28, 5.65). Just before the zeros: `blocked reverse did not move the shell`, then `blocked turn did not yaw`. Other bath, hall, and laundry stands of 20 s or more have the shell in a different room, still driving, and the goal changes first. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A spawn warp that only looks like the bath. A stop at door H, the earlier m1 shape. n2 is at the bedroom walkway, and the command is zero after that.

**Outcome.** The stand is in the bath and the shell is in the bedroom. The command stays zero until the cap, and the bath goal is not dropped. No controller change.

## 2026-11-10 — A dead reverse is not requested again

**Mission parameter.** Trace why n2's command stays zero after `blocked reverse did not move the shell` and `blocked turn did not yaw`. Say whether m3, n4, m4, and m5 are the same branch. Do not add a knob or a new motion.

**Change.** When both of those recoveries have failed, a straight reverse still returns zero for that tick. The leave latch and the forced pile are cleared, and the pile clock starts again, so the next tick is not another copy of the same reverse. An empty path replans. A path already in hand is driven by the existing pursuit.

**Why.** `_respect_escape_block` refuses the reverse, then one yaw and the other. After the second quiet turn it returns `(0, 0)` for a straight reverse. `_leaving_command` and the forced pile send that reverse again on the next tick, so the zero repeats and the planned path never runs.

**Evidence.** n2's pile is 314 s of hold or zero, `src=pile_leave`, goal still the bath stand, path 4.11 m. m3, n4, m4, and m5 log the same two lines, then `pile, leaving the spot`, then a sit of 232–414 s on `src=pile_leave`. The unit test drives the planned path on the tick after the zero. Mapping tests: 156 passed. No new 600 s batch.

**Rejected.** Another spin. The quiet-turn rule already stops that. A new reverse speed. The 2 s clock already decided this reverse did not move the shell.

**Outcome.** Sitting still was the leave latch asking for a reverse the block had already refused. The next tick can run the path that was already planned.

## 2026-11-10 — Live ten after releasing the dead reverse

**Mission parameter.** Same ten-run check on `7d07869`. Compare with the live ten in notes `5ec530a` / `3a20f51`. Do not change the controller again.

**Change.** None this pass. The runs are the release in `7d07869`.

**Evidence.** All ten latch 15 plates and end `time_cap` in `go`. No mapper done line. No `up_z` below 0.5. North visited 17.74, 12.81, 22.73, 14.58, 18.62 (mean 17.30, sd 3.84) against 14.32 (sd 5.02). −0.35 visited 17.27, 19.86, 15.34, 11.18, 9.14 (mean 14.56, sd 4.38) against 11.61 (sd 2.97). Bath is entered on m2 (56.5) and n3 (31.7). Hall on m2 (49.2). Pantry, storage, laundry, and the walk-in are 0. No `src=pile_leave` stretch has both v and w at zero. After `blocked turn did not yaw`, the next phase command is a turn or a drive, not zero through the cap. n2's later pile is 209 s of pile-leave and stall with no goal, and the shell moves 0.6 m. m4's longest zero is 101 s of `src=stall`. The room table is in `docs/LESSONS_LEARNED.md`.

**Rejected.** Another scoring or recovery change from this sample. The old multi-minute pile-leave zero is gone. Coverage is about 3 points higher, inside the previous north scatter.

**Outcome.** The command does not stay zero until the cap after those two lines. The apartment is still not finished.

## 2026-11-10 — Stall's reverse and the trail spin

**Mission parameter.** From the `7d07869` logs, say what `src=stall` is, why it can sit with no translation, and whether n2's 209 s no-goal pile is the same branch. Do not change the controller.

**Change.** None.

**Why.** `src=stall` is `_stall_escape`: back up, then turn, then a short drive. The name is attached before the blocked-heading check. A straight reverse that check has already refused becomes wheel speeds (0, 0), and the tag stays stall.

**Evidence.** m4 after t=257 s: 304 of 312 pile lines are stall with v=0 and w=0, and the longest such run is 101 s. The back phase leaves that reverse on a closed tail, 1 m of trusted wheel travel, or 0.8 s with under 0.02 m. The reverse just before it was refused turned the wheels through 0.30 m. After the command is zero, wheel translation is 0, so that travel stays above 0.02 m and under 1 m. The handoff does not run. Yaw change 0.00 rad, true path a few centimetres, goal none. Eight pile lines are w=0.30, each a short pivot. m5 is the same stall: 226 turn lines and 176 zero lines, 56% of wheel samples a pivot, yaw change −0.02 rad, true path 0.9 m. A turn that has not moved the heading 0.2 rad in 4 s flips the spin and restarts that clock, ahead of the branch that would drive, so the drive does not start. n2 from t=414 s is `trail_reverse` 120 times under `src=pile_leave` (a ±0.50 spin, v=0), plus 27 stall lines. Goal none. Yaw change 0.01 rad, true path 0.6 m. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Treating n2 as the stall zero. The speeds are a spin and the source is pile-leave. A new motion. The stall turn is already the next action, and on these logs it does not change the heading.

**Outcome.** Stall keeps asking for a reverse the block has refused, and the 0.8 s handoff does not see the zeroed command as no motion. n2 is a different sit: the trail spin never lines up, so it never drives. No controller change.

## 2026-11-10 — A refused stall reverse moves to its turn

**Mission parameter.** The stall back phase keeps requesting the reverse the block has already refused, because wheel travel sits between 0.02 m and 1 m. Stop that request. Use the turn the stall already has. Do not touch the pile-leave trail spin, and do not add a motion for the 4 s flip.

**Change.** `_release_dead_reverse` advances a stall that is still in its back phase with `_advance_stall_turn`, the same step the 0.8 s handoff uses. The refused reverse on that tick is still (0, 0). The next tick is the existing turn.

**Why.** `7d07869` drops the pile latches and leaves `_blind_back` set. The stall tag is applied before the block, so the zero stays labeled stall and the back phase asks again.

**Evidence.** Unit test: travel 0.15 m, both block turns already quiet, the reverse returns (0, 0), the phase is `turn`, and the next `_go` is the stall pivot. Mapping tests: 157 passed. No new 600 s batch.

**Rejected.** Ending the stall and replanning. The turn is already the next action. A new spin for m5. That flip is the stall turn not changing the heading, and this pass does not add a motion for it. Treating n2's `trail_reverse` as this zero.

**Outcome.** A refused stall reverse is not requested again. The stall turn runs on the next tick. The 2 s clocks are unchanged.

## 2026-11-10 — Live ten after the stall reverse advances

**Mission parameter.** Same ten 600 s checks as `6f113f6`. Flags off. Do not change the controller.

**Change.** None this pass. The runs are the release in `4dc9e25`.

**Evidence.** All ten latch 15 plates and end `time_cap` in `go`. No mapper done line. Lowest `up_z` is 0.975. North visited 19.48, 11.66, 15.75, 12.18, 16.60 (mean 15.14, sd 3.25) against 17.30 (sd 3.84). −0.35 visited 12.59, 14.13, 16.44, 16.39, 12.07 (mean 14.33, sd 2.05) against 14.56 (sd 4.38). Bath on m4 (52.7) and m5 (4.5). Hall on n2 (73.9), n5 (61.4), and m4 (45.3). Pantry, storage, laundry, and the walk-in are 0. No `src=pile_leave` stretch has both speeds at zero. After both quiet turns the next phase command is a drive (m1, m5) or a turn (m2, and m5's second). m4's 101 s stall zero does not repeat. m2 then sits: stall v=w=0 for 38 s from t=493.3, goal none, true path 0.00 m, with two one-line pivots before the cap. `trail_reverse` spans 16 s on n2 and 11 s on m2. m5's true path from t=510 s is 6.4 m. The room table is in `docs/LESSONS_LEARNED.md`.

**Rejected.** Another controller change from this sample. The gaps are inside the earlier scatter. The turn after a refused reverse is already what `4dc9e25` sends. On m2 that turn does not change the heading.

**Outcome.** The 101 s stall zero is gone. One run still sits after the turn, with the heading unchanged. The apartment is still not finished.

## 2026-11-10 — m2's pivots are too short to yaw

**Mission parameter.** From the `4dc9e25` m2 log, say whether the shell is pinned, what the brief w=±0.30 pivots do to the wheels, and why the command returns to zero. Do not change the controller.

**Change.** None.

**Why.** After t=490 s the true pose is (8.147, 0.888) and does not move. True yaw changes by −0.0007 rad and planner yaw by −0.0014 rad. Hubs stay at z=0.080, `up_z` is 1.000, and the supervisor logs no wall contact on any shell part. The nearest tire is 3.5 cm from `APT_FURN_COUNTER`. A held w=0.30 pivot at t=451–454 s yawed the shell +0.45 rad with the wheels tracking, so the motor can turn the body. After t=490 s each w=±0.30 command lasts 8–24 ms and the wheel positions stay put.

**Evidence.** 171 such pivots, no w=±0.70 command in that window. The code path is both `_spin_safe` checks failing, the phase returning to `back`, and the straight reverse becoming (0, 0) in `_respect_escape_block`. The w=±0.30 line is the existing fallback on a step whose rear reads closed. Counts are in `docs/LESSONS_LEARNED.md`. No other run in this ten holds that shape for 20 s.

**Rejected.** Calling the sit a pin. Calling the pivot too weak: the same speed, held for 3.1 s, did yaw. A new motion. The 4 s flip is not what returns this command to zero.

**Outcome.** The shell is on the floor, clear of the counter and the east wall. The stall asks again for the refused reverse, and the pivot it inserts lasts one to three steps, which does not yaw. No controller change.

## 2026-11-10 — Hold the small pivot when the reverse is refused

**Mission parameter.** Both ±0.70 spins fail `_spin_safe`, the phase drops to back, and the refused reverse becomes (0, 0) every step. Hold the existing small pivot instead. Do not add a speed or change a clock.

**Change.** If `_block_reverse_dead` is set, the stall stays in the turn and commands `_corner_pivot` (0.40 rad/s when that sweep is clear, otherwise 0.30 rad/s toward the open side). The turn clock is not reset. The yaw target and the 4 s limit still end it. A reverse that has not been refused still drops back.

**Why.** A held w=0.30 pivot at t=451–454 s on the `4dc9e25` m2 yawed the shell +0.45 rad. The 8–24 ms pulses did not.

**Evidence.** `test_a_refused_stall_holds_the_small_pivot_when_the_spin_is_unsafe`: six `_go` calls return the same 0.30 rad/s pivot, source stall, phase still turn. Mapping tests: 158 passed.

**Rejected.** A new spin speed. Resetting the 4 s clock on each pulse. Opening the reverse again.

**Outcome.** The refused reverse is not requested again while the big spins are unsafe. The small pivot stays commanded.

## 2026-11-10 — Live ten with the small pivot held

**Mission parameter.** Same ten 600 s checks. Compare with `734b6cc` (north 15.14 sd 3.25, −0.35 14.33 sd 2.05).

**Change.** None this pass. The runs are `8f9efb0`.

**Evidence.** All ten latch 15 plates. Eight log `time_cap` in `go`. n1 and m4 reach t=599.5 s without an `end_run` line (last phase `spin` and `go`). No mapper done line. Lowest `up_z` is 0.999. North visited 16.96, 19.60, 19.28, 20.81, 13.26 (mean 17.98, sd 2.99). −0.35 visited 19.33, 20.22, 18.38, 31.44, 13.55 (mean 20.58, sd 6.59). The wide −0.35 sd is m4 at 31.44. Pantry and storage are 0. No zero-command stretch of 20 s with the shell still. The longest exact zero is 14 s. The room table is in `docs/LESSONS_LEARNED.md`.

**Rejected.** Reading the −0.35 mean as a finished apartment. m4 is one draw, and no run logs mapper done.

**Outcome.** The m2 ping-pong does not repeat. The shell still does not finish the apartment.

## 2026-11-10 — The cap line, and the living room left unseen as a frontier

**Mission parameter.** Say why n1 and m4 have no `end_run` line, what stops the ten at 600 s, and which reachable unvisited area is largest. Pantry and storage stay closed. Do not change how the shell moves.

**Change.** `record_end_run` writes `end_run.txt` and flushes it before `mapping_done` is created. The print stays. Motion code is unchanged.

**Why.** n1 and m4 have no traceback. `mapping_done` contains `600.000`. The pose logger stops the world when that file appears, and Webots dropped the stdout from that step. Eight runs still have the line in the log.

**Evidence.** Last 120 s true path is 8.3–10.9 m on the north runs and 3.1–12.4 m on −0.35. Go-with-progress is 0.39–0.73 of each run. Stall is at most 0.13 (m5). Spins of 10 s or more are m1 (49 s), m2 (52 s), and m5 (100 s). The living room is 21.3 m² and about 99% seen, with about 17–18 m² unvisited on m5 and n1. m5's longest frontier is a 6.2 m bath edge; its last goal is in the bath. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Treating the missing line as a mapper crash. Treating the cap as a stuck robot. The shell is still moving, except m5's slower end.

**Outcome.** The end reason is on disk before the quit. The run stops because 600 s arrives while unknown edges in other rooms still outrank the seen living room.

## 2026-11-10 — An 1800 s cap still does not reach mapper done

**Mission parameter.** Three runs of each seed, cap 1800 s, same worlds and flags. Ask whether the mapper done line fires when time is not 600 s. Do not change the controller.

**Change.** None. The cap is the existing `RBM_MAP_MAX_S`.

**Why.** At 600 s the shell was still moving and unknown edges remained. A longer cap is the check that those edges eventually go away.

**Evidence.** No run logs `done: no reachable frontier`. All six write `end_run.txt` with `time_cap`. Seen percent is already 80–93 at 600 s and, except m1, stays there. Visited percent at 1800 s is 12, 24, 31 on the north runs and 17, 28, 40 on −0.35. Frontier counts at the cap are 47, 41, 36, 11, 37, 47. The long edges that remain are a 7.0 m bath edge (n2), a 4.4 m laundry edge (n3), a 4.4 m bedroom edge (m3), and edges with a large off-plate share (n1 living, m2 bedroom). n2 commands the stall's w=−0.30 pivot from t=805 s to the cap; the wheels follow and the true path is 0.52 m. m1's seen percent falls from 91 to 21, so its 35.8 m edge is a misaligned map. Lowest `up_z` is 0.998. The table is in `docs/LESSONS_LEARNED.md`.

**Rejected.** Reading 1800 s as enough for the done line. The unknown edges are still there, and two runs spend most of the extra time in a stall pivot.

**Outcome.** The explorer does not converge to the mapper done line in 1800 s. Time is no longer the only limit: a stall pivot can hold a real room edge, and some of the longest edges are partly off the floor.

## 2026-11-10 — A pivot drops the creep, and the small pivot never ends

**Mission parameter.** From the 1800 s logs, say why n1's pose misses a real creep, why m1's seen percent falls to 21, and why n2's w=−0.30 pivot runs for 1000 s without a yaw. Do not change the controller.

**Change.** None.

**Why.** On n1 from t=450 s, 99.8% of commands are a pivot, so `trust_translation` is false. Wheel translation in that window is 0.26 m. The true path at 1 s samples is 3.5 m, net 2.1 m, mostly forward, yaw change −0.04 rad, in the bedroom clear of the walls. The floor gate logs `floor still, translation frozen` 106 times and does not contain the other metres, because it only holds wheel travel. m1's error grows from 0.39 m at t=1200 s to 1.09 m at t=1400 s after `stall, backing out` at t=1209 s, with floor slip and the floor-still hold rejecting steps while the shell moves. Loop closure then resets the grid at t=1435 s, t=1699 s, and t=1792 s, and seen percent falls to 21. n2 is not wedged: the right-rear tire is 1.6 cm from the sofa, the right wheel touches on 2.4% of steps, and the wheels match the pivot. `yaw_stuck` flips the unused ±0.70 spin and restarts the 4 s clock, so the drive branch does not run, and stall travel stays 0 because a pivot zeroes `applied`.

**Evidence.** Counts and the poses are in `docs/LESSONS_LEARNED.md`.

**Rejected.** One gate for both pose failures. n1 is the pivot rule on a command whose wheels have almost no translation. m1 is the floor-slip rejection during a blocked stall, followed by a grid rebuild. Calling n2 a wedge. The contact is a brush and the tread is slipping.

**Outcome.** Two faults. A stall pivot is defined as no translation, so a real creep never enters the pose. The small pivot that replaced the refused reverse is not the spin the 4 s flip turns, so it never hands off. No controller change.

## 2026-11-10 — A quiet small pivot takes the drive

**Mission parameter.** When the small pivot is the command and yaw has not moved in the existing 4 s, take the existing drive branch. Leave the ±0.70 flip in place when that spin is the command. Leave the odometry pivot rule and the loop-closure grid reset.

**Change.** In the stall turn, `yaw_stuck` used to flip `_escape_spin` and restart the phase clock even when the wheels were being sent `_corner_pivot`. That flip now runs only when `pivot` is None. When the small pivot is the command, the phase becomes `drive` and the existing drive code runs. The 4 s window, the 0.2 rad quiet test, the 0.8 rad target, and the speeds are unchanged. Commit `1a66938`.

**Why.** On n2 the held w=−0.30 was not `_escape_spin`, so flipping that spin never changed the command. The same quiet 4 s on the ±0.70 spin still flips it.

**Evidence.** Mapping tests: 161 passed. The new tests cover a flat small pivot handing to `wheel_speeds(0.12, 0)`, the ±0.70 flip still happening, and the refused-reverse hold still returning the small pivot inside the 4 s. Five 600 s runs of each seed, flags off, depth seeds 64 and 63. North visited 12.30, 18.61, 15.15, 20.72, 17.42 (mean 16.84, sd 3.24) against 17.98 (sd 2.99). −0.35 visited 25.34, 17.28, 8.12, 17.92, 5.98 (mean 14.93, sd 7.89) against 20.58 (sd 6.59). No mapper done line. Lowest `up_z` is 0.998. The longest flat pivot is 5.2 s on the north runs. On −0.35 the 47 s flat spin on m2 is the ±0.70 pair. m3 and m5 then sit on `(0, 0)` for 332 s and 354 s with `src=back`. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Changing the odometry rule that drops translation on a pivot. Resetting the loop-closure grid. Changing the 4 s value or the ±0.70 flip.

**Outcome.** A quiet small pivot leaves the turn. The long single-sign w=−0.30 hold does not repeat. Two −0.35 runs then spend the rest of the cap on a blocked reverse tagged `back`.

## 2026-11-10 — The path reverse stays requested after the block zeros it

**Mission parameter.** From the handoff logs, say what produces `src=back` with `v=0` on m3 and m5, whether that sit is new, and whether m2's ±0.70 alternation is the 4 s flip. Do not change the controller.

**Change.** None.

**Why.** After the stall drive, the goal path calls `_begin_back` and then `_continue_back`, which returns `wheel_speeds(-0.10, 0)` and `_go` marks `back`. `_remember` runs `_respect_escape_block`. The reverse on that heading is already dead and both quiet turns are already used, so the block sets the speeds to (0, 0) and calls `_release_dead_reverse`. That release clears the leave latch and a stall whose phase is still `back`. It does not clear `_backing`. The goal is behind the nose, the planner pose does not move, and the 1 m exit never arrives, so the next tick asks for the same reverse. `7d07869` and `4dc9e25` do not see this caller. The saved tens for those commits have no `src=back v=0` stretch of 20 s. The drive handoff is what reaches the goal. m2's wheels swap the ±0.70 sign every step or two. The longest same sign is 0.78 s. That is the turn's per-tick "try the other spin", not the 4 s flip.

**Evidence.** The sequence, the clearances, and the earlier tens are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Calling the zero a stall back phase. The stall has already handed off. Calling m2's alternation the 4 s flip. A 4 s flip would hold one sign.

**Outcome.** The fault is `_continue_back` keeping a reverse the block has already refused. The shell is free on both runs. No controller change.

## 2026-11-10 — A dead reverse drops the path backing latch

**Mission parameter.** Clear `_backing` when a reverse is refused, and do not start another `_begin_back` on that heading. No new speed. Leave the ±0.70 same-tick swap for a later pass.

**Change.** `_release_dead_reverse` now clears `_backing` and `_back_from` along with the leave latch. `_begin_back` returns nothing when the shell is still on a heading whose reverse is already dead. The planned path, or a replan when the path is empty, is the next command. Commit `a1604df`.

**Why.** The path latch was the caller `7d07869` and `4dc9e25` did not clear. It kept asking for the reverse, and the block turned every ask into (0, 0).

**Evidence.** Mapping tests: 162 passed. Five 600 s runs of each seed, flags off, depth seeds 64 and 63. North visited 25.10, 17.70, 14.86, 14.95, 15.04 (mean 17.53, sd 4.40) against 16.84 (sd 3.24) and 17.98. −0.35 visited 18.06, 17.93, 9.30, 21.40, 19.77 (mean 17.29, sd 4.69) against 14.93 (sd 7.89) and 20.58. No mapper done line. Lowest `up_z` is 0.997. The longest `(0, 0)` stretch is 10.1 s, on n1, at the start of a stall that then reverses. The 332 s and 354 s `src=back` sits do not repeat. The longest ±0.70 sign-swap is 2.2 s. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Changing the ±0.70 per-tick flip. Adding a new speed.

**Outcome.** A refused path reverse does not stay at (0, 0). −0.35's mean moves from 14.93 to 17.29 and the sd from 7.89 to 4.69. The same-tick ±0.70 swap does not hold a run this ten.

## 2026-11-10 — The pose miss is a reverse the pin froze

**Mission parameter.** From the `a1604df` ten, say where the planner-vs-true position error accumulates, why m3 visited only 9.30, and name one change that would cut the pose error. Do not change the controller.

**Change.** None.

**Why.** Yaw at the cap is within 4°. Loop closure adds about 0. Pivots and the logged floor-slip rejections do not account for the runs whose error is above 1 m. Those misses are a straight reverse, `cmd=(-1.25,-1.25)`, whose wheel step matches the true motion while the pose stays put. The log on that stretch is `pinned, depth floor and visual still, wheel translation frozen`, and the forward nose range is rising. `_watch_pin` rewinds the pose and holds translation for 1.2 s, then arms again.

**Evidence.** Signed metres of the final error, by source, are in `docs/LESSONS_LEARNED.md`. They sum to the final error. The pin hold alone is +0.89 m of n3's 1.27 m and +0.65 m of m5's 0.78 m.

**Rejected.** Treating the pivot rule or the loop-closure rebuild as the residual. Neither shows up in this ten. Using `oak_imu`, or shortening the 2 s reverse clock.

**Outcome.** The smallest pose change, not made here, is to not pin a straight reverse when the forward range has increased since the pin anchor. m3's low visited score is a separate fact: it drove 28 m and never came within 0.6 m of a goal.

## 2026-11-10 — A straight reverse with an opening nose is not pinned

**Mission parameter.** Do not hold the pin on a straight reverse when the forward range has opened by more than sensor noise. A flat range still pins. A pitched shell still pins. Nothing else moves.

**Change.** `PIN_NOSE_RELEASE_M` is 0.04 m. OAK-D Lite σ is 6.7 mm at 1 m and 15 mm at 1.5 m, which is where the frozen reverses sat, so 0.04 m is six standard deviations at 1 m and more than two at 1.5 m. The logged freezes rose by several tenths of a metre. On a straight reverse, if the nose has opened by more than that since the pin anchor, the rewind and the 1.2 s hold do not run. A spin, a flat range, a sentinel range, and a pitch more than 0.12 rad from the learned rest still pin. The floor-still and floor-blank holds are unchanged. Commit `5a546bf`.

**Why.** The large misses were a reverse the wheels and the nose both showed, while the pin's rear-camera vote said still.

**Evidence.** Mapping tests: 165 passed. Five 600 s runs of each seed, flags off, depth seeds 64 and 63. North visited 17.92, 13.60, 15.94, 24.64, 21.26 (mean 18.67, sd 4.36) against 17.53 (sd 4.40). −0.35 visited 19.80, 13.96, 17.91, 20.02, 18.27 (mean 17.99, sd 2.44) against 17.29 (sd 4.69). No mapper done line. Lowest `up_z` is 0.996. No sample below 0.95. The pin's share of the final error on n3 fell from 0.89 m to 0.01 m, and on n5 from 0.50 m to 0.00 m. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Releasing the pin on a spin, or while the pitch is already past the rest gate. Touching the 2 s reverse clock.

**Outcome.** The repeated pin on a real reverse is gone from the runs that had it. Visited means are 18.67 and 17.99. Pose error is lower on the north runs that were above 1 m, and higher on two −0.35 runs.

## 2026-11-11 — An agreed wheel step with a moving nose is kept

**Mission parameter.** A wheel step both hubs agree on, whose forward nose has moved more than the existing 0.04 m since the view was called still, is not dropped by the frozen-view check and is not rewound by the floor-still hold. A flat nose, a pivot, and a pitched shell still drop. Nothing else moves.

**Change.** `_keep_agreed_nose` is that test. The frozen-view drop skips it. The floor-still rewind arms a new anchor and returns instead of putting x and y back. The nose is the one stored when the depth signature last changed. Agreement is a non-zero hub step that is not a commanded spin and not opposite hub rotation. Pitch more than 0.12 rad from the learned rest refuses the keep. Commit `6b8ce43`.

**Why.** Outside the pin holds the `5a546bf` ten still dropped translation while the wheels and the true shell moved together. The coarse depth signature can stay still while the nose range moves.

**Evidence.** Mapping tests: 170 passed. An opening nose keeps the step. A flat nose drops it. A pivot drops it. A pitched shell drops it. The floor-still hold keeps the pose when the nose reads 1.08 m and rewinds it when the nose reads 1.00 m. Five 600 s runs of each seed, flags off, depth seeds 64 and 63. North visited 16.42, 13.01, 17.52, 13.27, 10.15 (mean 14.07, sd 2.94) against 18.67 (sd 4.36). −0.35 visited 18.94, 16.10, 23.72, 13.19, 11.05 (mean 16.60, sd 4.97) against 17.99 (sd 2.44). Pose error: n 1.60, 0.45, 0.37, 1.33, 0.28 and m 0.32, 1.49, 0.71, 0.35, 0.40, against 1.28, 0.79, 0.95, 0.85, 0.35 and 0.24, 1.19, 0.93, 1.06, 0.34. The rejected-translation share is +0.61, +0.09, +0.83, +1.12, +0.21 and +0.32, +0.51, +0.73, +0.05, −0.20. The same gate on the previous logs is +0.30, +0.64, +0.64, +0.21, +0.22 and −0.43, +0.62, +0.26, +0.83, +0.44. No mapper done line. Lowest `up_z` is 0.998. No sample below 0.95. The longest `(0, 0)` is 55 s on n3. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Keeping a pivot, a flat nose, or a shell pitched more than 0.12 rad from rest. Changing the pin release, the floor-blank hold, or the weak-shift hold. Reading `oak_imu` as a new input. Moving the 0.45 m cap or the 2 s reverse clock. Opening pantry or storage.

**Outcome.** The keep is in. Visited means are 14.07 and 16.60, below 18.67 and 17.99. Pose-error means are 0.81 m and 0.65 m, against 0.84 m and 0.75 m. n2 and m4 lose most of the rejected-translation share. n1, n3, n4, and m3 gain it. The long zeros are a stopped shell.

## 2026-11-11 — The nose keep is reverted, and a zero approach takes the stall

**Mission parameter.** `6b8ce43` is a regression against `5a546bf`. Put that behavior back. Then, in phase `go` on a straight or pivot approach, a command that has been `(0, 0)` for more than 5 s, with the goal unchanged and the shell still, takes the existing stall escape. If that escape is blocked, drop the goal. Do not add a separate hold-branch change.

**Change.** The frozen-view drop and the floor-still rewind are back to the `5a546bf` rule. `ZERO_APPROACH_S` is 5 s. `_zero_approach_stall` starts the same `_stall_escape` that logs `stall, backing out`. A doorway crossing, the stall cooldown, or a reverse the block has already refused calls `_drop_sitting_goal` instead. Commit `e929f76`.

**Why.** The nose keep cut visited means from 18.67 and 17.99 to 14.07 and 16.60 and brought the long zeros back. On that ten the n4 and m4 `src=hold` sits were a pivot whose issued command was already `(0, 0)`, so this clock covers them. The n3 24 s and 55 s zeros followed a `back` and an empty path. That is not this approach.

**Evidence.** Mapping tests: 169 passed. A zero command past 5 s logs `stall, backing out`. Under 5 s does not. A shell that has moved 0.10 m does not. Five 600 s runs of each seed, flags off, depth seeds 64 and 63. North visited 15.56, 14.82, 8.80, 22.92, 17.67 (mean 15.95, sd 5.10) against 18.67 (sd 4.36). −0.35 visited 24.09, 18.63, 16.47, 27.00, 16.82 (mean 20.60, sd 4.70) against 17.99 (sd 2.44). Pose error: n 0.37, 1.03, 0.53, 0.50, 0.81 and m 1.31, 0.19, 0.51, 0.43, 1.62, against 1.28, 0.79, 0.95, 0.85, 0.35 and 0.24, 1.19, 0.93, 1.06, 0.34. No mapper done line. Lowest `up_z` is 0.999. No sample below 0.95. The longest `(0, 0)` is 243 s on n3, contact recovery (`contact_bump`), which this clock does not take. m4's 49 s follows a `back`. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Keeping the nose release. A separate change to the hold mark or to contact recovery. Reading `oak_imu` as a new input. Moving the 0.45 m cap or the 2 s reverse clock. Opening pantry or storage.

**Outcome.** The nose keep is gone. The 5 s clock is in. North's visited mean is 15.95, below 18.67. −0.35's mean is 20.60, above 17.99. The long contact-recovery sit and the sit after a `back` are still there.

## 2026-11-11 — The zero clock also covers contact and the empty path after a back

**Mission parameter.** The 243 s n3 sit is contact recovery, and the 49 s m4 sit is an empty path after a back. Extend the same 5 s `(0, 0)` clock to those two branches. On expiry, stall, or pile-leave if the back-out is blocked, or drop the goal. Do not start the same branch again on the next tick.

**Change.** `_zero_branch` names contact recovery and a `goal=none` back as well as straight and pivot. `_finish_zero` runs the stall escape, then `_begin_pile_leave`, then `_drop_sitting_goal`. Contact sets the existing 2 s `_contact_until`. The empty path clears `_backing` and waits the ordinary 4 s replan. Commit `bc55896`.

**Why.** Both sits send `(0, 0)` while the shell is still, and the straight-or-pivot check never saw them.

**Evidence.** Mapping tests: 175 passed. Each new branch takes the stall after 5 s, stays put under 5 s, and stays put when the shell has moved 0.10 m. Five 600 s runs of each seed, flags off, depth seeds 64 and 63, against the `998e957` ten. North visited 11.86, 17.42, 14.62, 12.21, 18.39 (mean 14.90, sd 2.96) against 15.95 (sd 5.10). −0.35 visited 22.65, 13.78, 9.58, 16.77, 11.67 (mean 14.89, sd 5.09) against 20.60 (sd 4.70). Pose means are 0.93 m and 0.94 m, against 0.65 m and 0.81 m. No mapper done line. Lowest `up_z` is 0.998. No sample below 0.95. The 243 s contact sit and the 49 s empty-back sit do not repeat. The longest `(0, 0)` is 244 s on m3, `src=hold` with the goal and the path still set after a `back`. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A new cooldown number. A change to the hold mark while a goal is still set. Reading `oak_imu` as a new input. Moving the 0.45 m cap or the 2 s reverse clock. Opening pantry or storage.

**Outcome.** Contact recovery and the empty path after a back no longer sit out the cap. Visited means are 14.90 and 14.89, below 15.95 and 20.60. A back that still holds a goal can sit, which is m3's 244 s.

## 2026-11-12 — One watchdog on the command that is sent

**Mission parameter.** Each branch clock exposed the next branch. m3 sat 244 s on a back that still held a goal. One check, on the final `(0, 0)`, independent of which branch produced it.

**Change.** The straight, pivot, contact, and empty-back clocks are deleted. `_watchdog` runs in `_remember`, after the escape block has rewritten the speeds. Five seconds of `(0, 0)` with the planner pose inside 0.04 m of where the zero started takes the stall escape, the pile leave if that back-out is blocked, or a goal drop plus the existing 30 s quarantine. A fire within 20 s starts one rung higher. The first 3.5 s, `done`, and `settle` are not a sit. Contact keeps its 2 s cooldown. Commit `b975857`.

**Why.** The issued command is the same in every sit. Naming the branch left the next one out.

**Evidence.** Mapping tests: 175 passed. Straight, pivot, contact, an empty back, and a back that still holds a goal each take the stall after 5 s. Under 5 s does not. A shell 0.10 m away does not. Three fires inside 20 s take stall, then pile leave, then drop. The start-up idle does not fire. Five 600 s runs of each seed, flags off, depth seeds 64 and 63. North visited 16.07, 15.91, 17.47, 18.76, 11.97 (mean 16.04, sd 2.55) against 14.90 and 15.95. −0.35 visited 17.41, 22.75, 12.16, 11.58, 12.64 (mean 15.31, sd 4.76) against 14.89 and 20.60. Pose means are 0.71 m and 0.67 m, against 0.93 m and 0.94 m on `bc55896` and 0.65 m and 0.81 m on `998e957`. Watchdog fires are one `stall` on n4, one `stall` on m4, and zero on the other eight. The longest `(0, 0)` is 5.0 s, ended by that stall. No mapper done line. Lowest `up_z` is 0.996. No sample below 0.95. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** Another branch name on the clock. A new blacklist duration. Reading `oak_imu` as a new input. Moving the 0.45 m cap or the 2 s reverse clock. Opening pantry or storage.

**Outcome.** The 244 s sit does not repeat. The longest zero is the 5 s the watchdog is written to wait. Visited means are 16.04 and 15.31. −0.35 is below the `998e957` mean of 20.60.

## 2026-11-13 — Four modes and one owner of the wheel command

**Mission parameter.** Stop adding a clock per branch. Name the controller mode, and give one function the wheel command that leaves the controller. Keep the `b975857` watchdog. Remove a recovery branch only where that watchdog already replaces it.

**Change.** Each tick sets `mode` to `explore`, `approach`, `recover`, or `leave_area`. `_issue` stores `self._commanded` after the escape block and the watchdog. `__init__` still sets the starting `(0, 0)`. The ladder is unchanged: 5 s of `(0, 0)` with the planner pose inside 0.04 m, then stall, then pile leave, then a goal drop with the existing 30 s quarantine. A repeat inside 20 s starts one rung higher. Commit `081cddc`.

**Why.** The branch clocks were already one watchdog. The remaining split was which function was allowed to publish the wheels, and what the controller called the situation it was in.

**Evidence.** Mapping tests: 177 passed. One test reads the source and requires a single assignment of the outgoing command. One test walks explore, then approach, then recover, then leave_area. Deleting the stuck escape, pile-leave helpers, relief, and contact recovery (`c3e7031`) left north at visited mean 4.58 (sd 2.87) and −0.35 at 7.09 (sd 3.54), because those branches issue a non-zero speed and the watchdog only replaces `(0, 0)`. `081cddc` restores them. The controller diff against `b975857` is 13 lines removed and 42 added. Five 600 s runs of each seed, flags off, depth seeds 64 and 63. North visited 20.28, 15.70, 17.77, 16.26, 11.32 (mean 16.27, sd 3.29) against 16.04 (sd 2.55). Seen, the scorer's `coverage_pct`, is 92.03, 94.45, 96.54, 86.45, 87.85 (mean 91.46, sd 4.28). −0.35 visited 11.36, 19.35, 24.66, 20.84, 23.82 (mean 20.01, sd 5.30) against 15.31 (sd 4.76). Seen 88.92, 94.27, 96.35, 97.72, 93.28 (mean 94.11, sd 3.38). Pose means are 0.68 m and 1.15 m, against 0.71 m and 0.67 m. Watchdog fires are one `stall` on n1 and six `drop` on n5. The longest `(0, 0)` is 26.6 s on n5. The drop rung returns `(0, 0)`, and the sit runs until pile leave at t=528.4 s. No mapper done line. Lowest `up_z` is 0.933 for 0.4 s on n5; the shell is upright again by t=494.3 s. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A new timeout, blacklist, or rung. Deleting a branch that still issues a non-zero command. Changing geometry, the world, friction, motors, sensors, clearance, IMU usage, the 2 s reverse clock, the 0.45 m unvisited-stand cap, or pantry and storage. Switching the done condition to seen % (that is the next round).

**Outcome.** The mode and the single writer are in. The non-zero recovery drives stay, after the deletion ten showed the visited collapse. North's visited mean moved 0.23. −0.35's visited mean is 20.01 against 15.31. The 5 s sit cap is missed on n5, where the top watchdog rung is itself `(0, 0)`.

## 2026-11-13 — The drop rung moves, and seen % is the reported score

**Mission parameter.** The drop rung must not leave the shell at `(0, 0)`. After the goal drop and the existing 30 s quarantine, the next command is `leave_area` or `explore`. Seen % is the number the report leads with. A done condition is logged when there is no reachable frontier and the map has been stable, and the run still goes to the time cap.

**Change.** `_resume_after_drop` keeps a leave command that is still non-zero, and otherwise starts the survey spin. The stall rung and the pile-leave rung are unchanged. `done would fire: no reachable frontier, map stable` is logged when the last plan found nothing and the known and occupied counts stay inside 8 cells for the existing 4 s settle hold. `done` stays false. `format_scores` prints `coverage_pct` first and `visited_pct` second. Commit `9c58a66`.

**Why.** n5 of `081cddc` sat 26.6 s because the drop rung returned `(0, 0)` and the next tick produced the same hold. The seen fraction is the score the next round will use to decide the run is finished.

**Evidence.** Mapping tests: 179 passed. The drop test blocks the leave and requires a non-zero command on that tick or the next, with the mode `explore` or `leave_area`, and `done` still false. The stable-map test logs the would-fire line at 4.5 s and stays out of `phase == done`. A map change of 20 cells resets the hold. Five 600 s runs of each seed, flags off, depth seeds 64 and 63, against the `081cddc` ten. Seen means are 76.98 (sd 24.48) and 90.51 (sd 8.96), against 91.46 and 94.11. The ten-run seen mean is 83.75 against 92.79. Visited means are 16.82 (sd 3.04) and 17.28 (sd 1.93), against 16.27 and 20.01. The ten-run visited mean is 17.05 against 18.14. Pose means are 0.75 m and 0.81 m, against 0.68 m and 1.15 m. One `stall` fire, on m5. The longest `(0, 0)` is 5.0 s, ended by that stall. The would-fire line does not appear. No mapper done line. Lowest `up_z` is 0.999. Counts are in `docs/LESSONS_LEARNED.md`.

**Rejected.** A new duration. Ending the run on the would-fire line. Shortening the 2 s reverse clock. Reading `oak_imu` as a controller input. Changing geometry, the world, friction, motors, sensors, or clearance.

**Outcome.** The drop handoff is in and tested. This ten never reached the drop rung. The 26.6 s sit does not repeat. The ten-run seen mean is 9.0 points lower. The ten-run visited mean moved 1.1 points.
