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

**Outcome.** A one-run swing of about 10–14 visited points is the size of the noise on this controller. −0.35 pile time runs from 52 s, under the 60 s bar, to 460 s, with no code change. Stripping the stacked recovery rules is not done. Outcome of that strip: pending.

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
