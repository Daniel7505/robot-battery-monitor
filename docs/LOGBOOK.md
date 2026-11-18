# Logbook

A chapter index of the apartment mapping rounds. Each line is taken from [`LESSONS_LEARNED.md`](LESSONS_LEARNED.md) or [`DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md). A commit is listed only when those notes name it. A round can sit in more than one chapter when the same sim answered two questions.

## Contents

1. [Tip-overs and climbs](#1-tip-overs-and-climbs)
2. [Stuck, stall, and pile-leave](#2-stuck-stall-and-pile-leave)
3. [Doorways and wing crossing](#3-doorways-and-wing-crossing)
4. [Goal selection](#4-goal-selection)
5. [Pose and odometry](#5-pose-and-odometry)
6. [Floor and world audit](#6-floor-and-world-audit)
7. [Measurement, variance, and false wins](#7-measurement-variance-and-false-wins)
8. [Integrity and ground-truth leaks](#8-integrity-and-ground-truth-leaks)
9. [Reverted experiments](#9-reverted-experiments)
10. [Waypoint driver](#10-waypoint-driver)

## 1. Tip-overs and climbs

### 2026-11-09 — A blocked reverse stops, and one north run climbs the island

- **Commit:** `9ebb7cd`
- **Tried:** A blocked-heading reverse whose planner pose stays inside 0.04 m for 2 s is not sent again.
- **Result:** Straight reverse on the matched north seed fell from 340 s to 81 s. North run 4 reversed into the island while the pose was still moving and finished inverted.
- **Lesson:** The still-pose gate only stops a reverse the estimate does not follow.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-blocked-reverse-that-does-not-move-the-shell) · [Decision](DESIGN_DECISIONS.md#2026-11-09-stop-a-blocked-reverse-that-does-not-move-the-shell)

### 2026-11-09 — The island climb was trusted wheel travel

- **Commit:** `9ebb7cd` (saved log; no new controller change)
- **Tried:** Diagnosis only, from the north run 4 log.
- **Result:** From t=304 s to t=310 s the command was `(-1.25,-1.25)`. Planner travel 0.49 m matched the wheel arc. Body z rose to 0.185, `up_z` fell to 0.057, and both rear hubs reached 0.300. No floor-still line in that window.
- **Lesson:** The pose followed the wheels up the cabinet because the floor image never became the still picture the pitch gate waits on.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-island-climb-was-wheel-travel) · [Decision](DESIGN_DECISIONS.md#2026-11-09-the-island-climb-was-accepted-wheel-travel)

### 2026-11-09 — A flat floor frame is not wheel travel

- **Commit:** `b1827e7`
- **Tried:** A blank floor step with frame std under 2 rewinds planner x,y once the hubs have claimed 0.02 m.
- **Result:** The forced reverse holds the pose after the image goes flat. The tracker still says `moving` while `up_z` falls through 0.8, and that part of the climb is still accepted. The probe keeps the reverse and the shell inverts. Matched means stayed inside the old scatter. No `up_z` below 0.5 on the twenty scored runs.
- **Lesson:** A flat blank is not travel. A climb the camera still calls moving is still trusted, and the motors can invert the shell after the pose has stopped.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-flat-floor-frame-does-not-confirm-wheel-travel) · [Decision](DESIGN_DECISIONS.md#2026-11-09-a-flat-floor-frame-is-not-wheel-travel)

### 2026-11-09 — A held pose already cuts the blocked reverse

- **Commit:** not named; no controller change
- **Tried:** Check whether the existing 2 s pose clock stops a reverse once the floor hold has pinned x,y.
- **Result:** With the heading armed, the reverse is cut at explorer t=2.4 s. `up_z` falls to 0.395 during the following turn and is 1.000 again by sim t=12.5 s. The unarmed probe of the same reverse finished inverted.
- **Lesson:** The command path already stops that reverse. The island probe inverted because the heading was not armed.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-held-pose-cuts-the-blocked-reverse) · [Decision](DESIGN_DECISIONS.md#2026-11-09-a-held-pose-cuts-the-blocked-reverse)

### 2026-11-09 — A short floor shift is not the wheel step

- **Commit:** `fec4ff8`
- **Tried:** A `moving` floor peak that misses the claimed wheel step by more than the existing 2 px still gate rewinds x,y.
- **Result:** The forced reverse no longer walks the pose through the early climb. Open floor still matches the wheels. Matched means stayed inside the old scatter. No `up_z` below 0.5.
- **Lesson:** A 2–4 px peak on the island face was keeping the whole wheel step. A peak that matches the wheels still counts.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-short-floor-shift-is-not-the-wheel-step) · [Decision](DESIGN_DECISIONS.md#2026-11-09-a-short-floor-shift-is-not-the-wheel-step)

### 2026-11-09 — The short shift and the reverse-stop together

- **Commit:** `fec4ff8` already in; no new change
- **Tried:** Confirm the early-climb window is closed when the short-shift hold and the 2 s reverse-stop run together.
- **Result:** Pose stays at 0. Reverse cut at explorer t=2.4 s. `up_z` minimum 0.395, upright by sim t=12.5 s. The matched runs' long reverses are the shell actually moving.
- **Lesson:** The early-climb honesty gap is closed. The residual pitch during the turn is the existing 2 s clock.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-two-holds-together-and-where-that-stops) · [Decision](DESIGN_DECISIONS.md#2026-11-09-the-short-shift-and-the-reverse-stop-together)

### 2026-11-15 — Seed 78 of the throat-hold ten tips at the entry

- **Commit:** `d648fe8` (the run; the hold itself fired at t=56.8 and released at t=67.6)
- **Tried:** The live ten. No separate climb change.
- **Result:** At t=434 s a straight reverse `(-1.25,-1.25)` drives the tail into the laundry-north / west-hall corner. Rear hubs go to 0.300, body z to 0.19, `up_z` to −0.01 by t=439, and it stays there. Floor blank holds the planner pose at t=433.5. "Wheels lifted" is logged at t=442.4, after the shell is already over.
- **Lesson:** Same shape as the island climb, on a different wall. The floor hold froze the pose and the motors kept the reverse.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-throat-hold-fires-on-the-live-ten) · [Decision](DESIGN_DECISIONS.md#2026-11-15-the-throat-hold-on-the-live-ten)

### 2026-11-15 — Seed 79 of the no-plate ten tips

- **Commit:** `edfd172`
- **Tried:** The honesty ten. No climb change.
- **Result:** `up_z` −0.019 at (3.57, 2.17) from t=495 s through the cap. The other nine runs stay at or above 0.998.
- **Lesson:** Removing the plate credit did not remove the reverse-into-a-wall tip.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-plates-were-prior-knowledge) · [Decision](DESIGN_DECISIONS.md#2026-11-15-the-mapper-does-not-read-the-floor-plan)

## 2. Stuck, stall, and pile-leave

### 2026-11-02 — Rounds 3–5, the 60 s pile and the zero command

- **Commit:** not named
- **Tried:** Any-stall escape, then a straight leave, then a trail reverse instead of `(0, 0)`. A doorway line-up is not a 2 s stall.
- **Result:** Piles stayed over 60 s. Round 4 north commanded zero and moved 0.00 m. Round 5 north sat 383 s on a slow spin at one frozen pose, 198 stuck lines, and `stuck, next goal` never appeared.
- **Lesson:** A blank floor frame does not mean the body moved. Returning `(0, 0)`, or a spin the body does not yaw, is the same pile.
- **Notes:** [Lessons](LESSONS_LEARNED.md#visible-stall-and-the-60-s-pile)

### 2026-11-02 — A pivot swept a corner into the wall

- **Commit:** not named
- **Tried:** Pivots and near-wall arcs sweep the rectangle. A corner that meets the map becomes the straight backup. A pivot ends at 12° and does not restart until 25°.
- **Result:** Corner-contact steps fell to 0 on the next four seeds. Later escape spins brought some back.
- **Lesson:** The shell is a rectangle. A disk misses the corner. Do not raise the 0.20 m planning radius to close that; it closes a doorway.
- **Notes:** [Lessons](LESSONS_LEARNED.md#pivot-swept-a-corner-into-the-wall)

### 2026-11-02 — Rounds 6–8, one stuck check

- **Commit:** not named
- **Tried:** Round 6's silent path trim and nose-0.12 check were deleted. A depth picture unchanged for 2.5 s is `camera_stuck`. Wheels without a matching gyro are `imu_stuck`. Either reverses along the trail. Round 8 runs the check in spin, settle, and go, and blocks the heading for 18 s.
- **Result:** Round 7 missed the bar (−0.35 visited 14.1% and pile 267 s). Round 8 north beat the round 5 bar. −0.35 did not (visited 19.3%, pile 152 s).
- **Lesson:** One nose value or one wheel command is not a class of failure. The heading block does not by itself end a pile.
- **Notes:** [Lessons](LESSONS_LEARNED.md#one-stuck-check-not-one-pose) · [Decision](DESIGN_DECISIONS.md#2026-11-02-rounds-68-general-stuck-detection-not-one-pose)

### 2026-11-03 — Leave a repeated pile

- **Commit:** not named
- **Tried:** A second escape inside 0.45 m, or a fourth blocked-heading refusal, drops the goal and skips that spot for 30 s. Reason `pile_leave`.
- **Result:** North's blocked-heading lines fell from 117 to 46 and the pile from a 372 s stay to 146 s. −0.35 piled for 417 s.
- **Lesson:** Changing the goal does not mean the shell left the 0.45 m ball.
- **Notes:** [Lessons](LESSONS_LEARNED.md#creep-the-throat-leave-a-repeated-pile) · [Decision](DESIGN_DECISIONS.md#2026-11-03-round-9-leave-a-repeated-pile-for-another-goal)

### 2026-11-04 — One recovery rule

- **Commit:** `d264bfa`
- **Tried:** `RBM_RECOVERY_MODE=one_rule` and `trail_only` replace the recovery stack. The flag stays off by default.
- **Result:** `one_rule` −0.35 visited mean 5.6 against the variance mean 14.7, pile mean 531 s. `trail_only` was the same within a point. Bumps and stuck events were 0 because those branches did not run.
- **Lesson:** One rule visits less and sits longer. Marking the disk does not change that. The stack stays.
- **Notes:** [Lessons](LESSONS_LEARNED.md#one-recovery-rule) · [Decision](DESIGN_DECISIONS.md#2026-11-04-one-recovery-rule)

### 2026-11-06 — Trusted wheel travel, best-center pile

- **Commit:** `2774b24`
- **Tried:** The pile dump follows the estimate's own best center. Stall progress uses wheel travel the step trusted.
- **Result:** Visited means 10.20 and 7.64, inside the noise. North run 1's 564 s pile is 483 s of `src=stuck`, commanded 48.2 m, estimate path 1.01 m.
- **Lesson:** The open mechanism is a stuck reverse of 0.10 m/s that neither the estimate nor the body follows.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-stuck-reverse-the-pose-does-not-follow) · [Decision](DESIGN_DECISIONS.md#2026-11-06-trusted-wheel-travel-and-a-best-center-pile)

### 2026-11-06 — The reverse reaches the hinges

- **Commit:** not named; driving unchanged
- **Tried:** Write the open pile summary at the cap, and add hinge travel beside commanded and pose distance.
- **Result:** North run 1's doorway sit holds `(-1.25,-1.25)` and each hinge moves −12.49 rad per 10 s while the true path is 0. The right hub falls to z=−0.003 at the unrepaired door G slot.
- **Lesson:** The motors do what they were told. The body stops because a wheel leaves the floor or a tire scrubs a wall.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-wheels-turn-and-the-shell-does-not) · [Decision](DESIGN_DECISIONS.md#2026-11-06-the-reverse-reaches-the-hinges)

### 2026-11-07 — Cap a stuck reverse the picture does not see

- **Commit:** `6281d58`
- **Tried:** A straight stuck reverse yields after 1 m of hinge travel while the forward picture stays frozen.
- **Result:** Visited gaps versus the repaired-floor five were 4.1 and 5.7 points. North run 5 still reversed 259 s on a wall. −0.35 run 3 reversed 289 s, much of it `pile_leave` after the yield.
- **Lesson:** The stuck escape yields. Pile-leave then repeats the reverse.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-stuck-reverse-that-the-picture-ignores) · [Decision](DESIGN_DECISIONS.md#2026-11-07-cap-a-stuck-reverse-the-picture-does-not-see)

### 2026-11-08 — The forward branch is rewritten into the reverse

- **Commit:** `be9bac7` (log only)
- **Tried:** Log a pile-leave branch as soon as the branch string changes.
- **Result:** On north seed 64 every pile-leave branch is `straight_forward` or `nose_forward`, and the wheels are `v=-0.10`. The escape block rewrites the forward command for 18 s. The body stays near (3.86, 5.39).
- **Lesson:** The sustained reverse is the block, applied after pile-leave has chosen forward.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-forward-branch-is-rewritten-into-the-failed-reverse) · [Decision](DESIGN_DECISIONS.md#2026-11-08-the-escape-block-rewrites-a-forward-leave)

### 2026-11-10 — A dead reverse is not requested again

- **Commit:** `7d07869`
- **Tried:** After both recoveries have failed, clear the leave latch and the forced pile so the next tick is not another copy of the refused reverse.
- **Result:** The unit test drives the planned path on the tick after the zero. Mapping tests: 156 passed. No new 600 s batch in that note.
- **Lesson:** The zero through the cap was the leave latch asking for a reverse the block had already refused.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-dead-reverse-was-requested-again) · [Decision](DESIGN_DECISIONS.md#2026-11-10-a-dead-reverse-is-not-requested-again)

### 2026-11-10 — Live ten after the dead reverse is released

- **Commit:** `7d07869`
- **Tried:** The same ten-run check. No further controller change.
- **Result:** No `src=pile_leave` stretch has both speeds at zero. North visited mean 17.30 against 14.32. −0.35 mean 14.56 against 11.61. m4's longest zero is 101 s of `src=stall`. No mapper done line.
- **Lesson:** The command does not stay zero until the cap after those two lines. The apartment is still not finished.
- **Notes:** [Lessons](LESSONS_LEARNED.md#live-ten-after-the-dead-reverse-is-released) · [Decision](DESIGN_DECISIONS.md#2026-11-10-live-ten-after-releasing-the-dead-reverse)

### 2026-11-10 — A refused stall reverse moves to its turn

- **Commit:** `4dc9e25`
- **Tried:** A stall still in its back phase advances to the existing turn when the reverse is refused.
- **Result:** The 101 s stall zero does not repeat on the next ten. m2 then sits with the heading unchanged. Visited means 15.14 and 14.33, inside the earlier scatter.
- **Lesson:** The stall turn runs. On one run that turn does not change the heading.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-refused-stall-reverse-moves-to-the-turn) · [Decision](DESIGN_DECISIONS.md#2026-11-10-a-refused-stall-reverse-moves-to-its-turn)

### 2026-11-10 — Hold the small pivot

- **Commit:** `8f9efb0`
- **Tried:** If the reverse is already dead and both big spins fail the sweep, keep commanding the existing 0.30 or 0.40 rad/s pivot.
- **Result:** The m2 ping-pong does not repeat. North visited mean 17.98. −0.35 mean 20.58, wide because m4 is 31.44. No 20 s zero with the shell still. No mapper done line.
- **Lesson:** Holding the small pivot removes that sit. The shell still does not finish the apartment.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-small-pivot-stays-on-when-the-reverse-is-refused) · [Decision](DESIGN_DECISIONS.md#2026-11-10-hold-the-small-pivot-when-the-reverse-is-refused)

### 2026-11-10 — A quiet small pivot takes the drive

- **Commit:** `1a66938`
- **Tried:** When the small pivot is the command and yaw has not moved in 4 s, the phase becomes drive. The ±0.70 flip stays when that spin is the command.
- **Result:** The long single-sign w=−0.30 hold does not repeat. Two −0.35 runs then sit on `(0, 0)` for 332 s and 354 s with `src=back`. Visited means 16.84 and 14.93.
- **Lesson:** The quiet small pivot leaves the turn. The next fault is a path reverse the block has already refused.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-small-pivot-hands-off-and-two-runs-sit-on-a-blocked-reverse) · [Decision](DESIGN_DECISIONS.md#2026-11-10-a-quiet-small-pivot-takes-the-drive)

### 2026-11-10 — A dead reverse drops the path backing latch

- **Commit:** `a1604df`
- **Tried:** `_release_dead_reverse` clears `_backing`. `_begin_back` does not start on a heading whose reverse is already dead.
- **Result:** The 332 s and 354 s `src=back` sits do not repeat. Longest `(0, 0)` is 10.1 s. −0.35 visited mean 17.29 against 14.93. Longest ±0.70 sign-swap is 2.2 s.
- **Lesson:** The path latch was the caller the earlier releases did not clear.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-path-latch-is-cleared-with-the-dead-reverse) · [Decision](DESIGN_DECISIONS.md#2026-11-10-a-dead-reverse-drops-the-path-backing-latch)

### 2026-11-11 — A zero straight or pivot takes the stall

- **Commit:** `e929f76`
- **Tried:** In phase `go`, a straight or pivot approach at `(0, 0)` for more than 5 s, same goal, shell still, takes the existing stall. A blocked escape drops the goal.
- **Result:** North visited mean 15.95 against 18.67. −0.35 mean 20.60 against 17.99. The longest `(0, 0)` is 243 s on n3, contact recovery, which this clock does not take.
- **Lesson:** The 5 s clock covers the approach zero. Contact recovery and a back with an empty path are a different sit.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-zero-straight-or-pivot-takes-the-stall) · [Decision](DESIGN_DECISIONS.md#2026-11-11-the-nose-keep-is-reverted-and-a-zero-approach-takes-the-stall)

### 2026-11-11 — The zero clock covers contact and the empty back

- **Commit:** `bc55896`
- **Tried:** The same 5 s clock also covers contact recovery and a `goal=none` back. Expiry takes stall, then pile-leave, then a goal drop.
- **Result:** The 243 s contact sit and the 49 s empty-back sit do not repeat. Visited means 14.90 and 14.89, below 15.95 and 20.60. m3 sits 244 s on `src=hold` with the goal still set after a back.
- **Lesson:** Naming the branch left the next branch out. A back that still holds a goal can sit.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-zero-clock-covers-contact-and-the-empty-path-after-a-back) · [Decision](DESIGN_DECISIONS.md#2026-11-11-the-zero-clock-also-covers-contact-and-the-empty-path-after-a-back)

### 2026-11-12 — One watchdog on the issued command

- **Commit:** `b975857`
- **Tried:** The branch clocks are deleted. Five seconds of the final `(0, 0)`, after the escape block, takes stall, then pile-leave, then a drop. A repeat inside 20 s starts one rung higher.
- **Result:** The longest `(0, 0)` is 5.0 s. Watchdog fires: one stall on n4, one on m4, zero on the other eight. Visited means 16.04 and 15.31.
- **Lesson:** The issued command is the same in every sit. The 244 s sit does not repeat.
- **Notes:** [Lessons](LESSONS_LEARNED.md#one-watchdog-on-the-issued-command) · [Decision](DESIGN_DECISIONS.md#2026-11-12-one-watchdog-on-the-command-that-is-sent)

### 2026-11-13 — Four modes, and the drop rung was itself zero

- **Commit:** `081cddc` (after `c3e7031` deleted the non-zero recoveries and was put back)
- **Tried:** Each tick names `explore`, `approach`, `recover`, or `leave_area`. `_issue` is the only writer of the outgoing command. The `b975857` watchdog stays.
- **Result:** Deleting the non-zero recoveries collapsed visited means to 4.58 and 7.09, so those drives came back. With them restored, north visited mean 16.27 and −0.35 20.01. n5 sits 26.6 s because the drop rung returns `(0, 0)`.
- **Lesson:** The watchdog only replaces a zero. A recovery that already drives has to stay.
- **Notes:** [Lessons](LESSONS_LEARNED.md#four-modes-one-command-writer) · [Decision](DESIGN_DECISIONS.md#2026-11-13-four-modes-and-one-owner-of-the-wheel-command)

### 2026-11-13 — The drop rung moves

- **Commit:** `9c58a66`
- **Tried:** After the drop, keep a non-zero leave or start the survey spin. Log a would-fire done line and do not end the run.
- **Result:** This ten never reached the drop rung. The longest `(0, 0)` is 5.0 s. Ten-run seen mean 83.75 against 92.79. Ten-run visited mean 17.05 against 18.14.
- **Lesson:** The 26.6 s sit does not repeat. The seen mean fell 9 points on a ten that never exercised the new handoff.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-drop-rung-leaves-or-it-spins) · [Decision](DESIGN_DECISIONS.md#2026-11-13-the-drop-rung-moves-and-seen-is-the-reported-score)

### 2026-11-13 — A still pose is a sit

- **Commit:** `317f719`
- **Tried:** Five seconds with the planner pose inside 0.04 m takes the ladder, including a pivot or a reverse. Startup, done, settle, and the survey spin stay out.
- **Result:** Seen means 93.67 and 94.20. Ten-run seen mean 93.93 against 83.75 and 92.79. Every run fires `drop`. Longest planner-still span is 56.6 s on n2.
- **Lesson:** A pivot with the wheels turning was invisible to a zero-only clock. The next command is often another non-translating rung, so the 5 s clock starts again.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-still-pose-is-a-sit-even-when-the-wheels-turn) · [Decision](DESIGN_DECISIONS.md#2026-11-13-a-still-pose-is-a-sit-wheels-turning-or-not)

### 2026-11-14 — A non-translating rung takes the open straight

- **Commit:** `cb637d8`
- **Tried:** A pivot, or a reverse whose side is blocked, is replaced by the existing straight leave until the pose moves 0.04 m.
- **Result:** Pose means fell to 0.68 m and 0.58 m. Ten-run seen mean 88.67 against 93.94. −0.35 seen mean 84.37, with m2 at 59.92 and m5 at 78.73. Longest still span 47.9 s. Every run still fires `drop`.
- **Lesson:** Pose error at the cap is lower. The straight leave does not by itself end a frozen pose.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-rung-that-does-not-translate-takes-the-open-straight) · [Decision](DESIGN_DECISIONS.md#2026-11-14-a-non-translating-rung-takes-the-open-straight)

## 3. Doorways and wing crossing

### 2026-11-02 — Seen from the doorway, never driven in

- **Commit:** not named
- **Tried:** A doorway line-up is not a translation stall. The unvisited goal is a cell about 2 m into the other region of the robot's own map.
- **Result:** Round 5, two seeds, bar missed. −0.35 kitchen visited 5%, bath 0%. North kitchen, bedroom, and bath visited 0%.
- **Lesson:** Seen % is not a visit. The bath opening is 0.81 m. Goals were chosen and then dropped on the threshold.
- **Notes:** [Lessons](LESSONS_LEARNED.md#seen-from-the-doorway-never-driven-in) · [Decision](DESIGN_DECISIONS.md#2026-11-02-round-2-visited-floor-not-only-seen-floor)

### 2026-11-02 — Bedroom southwest blind corner

- **Commit:** not named
- **Tried:** A blind view plus no translation counts as a frontier failure. Back up until depth is 5 cm past the minimum and the shell has moved 0.30 m.
- **Result:** The next −0.35 run spent 0 s inside 0.45 m of (0.35, 4.97).
- **Lesson:** Nose = 0.12 m is the blind placeholder, not a measured gap.
- **Notes:** [Lessons](LESSONS_LEARNED.md#bedroom-southwest-blind-corner)

### 2026-11-03 — Creep a seen throat

- **Commit:** not named
- **Tried:** In a throat the robot's own map measures (clearance 0.247–0.30 m) with a live shoulder, cap forward speed at 0.12 m/s and plan at 0.247 m.
- **Result:** Full-size −0.35 kitchen stayed 0%. `narrow_gap` fired twice on that seed, and all 477 wall steps were in a later bedroom pile. North visited 14.4% against round 5's 3.4%, kitchen 16%.
- **Lesson:** The creep did not reproduce the 80% body's kitchen result. Cruise aim stays 0.30 m.
- **Notes:** [Lessons](LESSONS_LEARNED.md#creep-the-throat-leave-a-repeated-pile) · [Decision](DESIGN_DECISIONS.md#2026-11-03-round-9-speed-dependent-clearance)

### 2026-11-14 — A doorway sit creeps and keeps the goal

- **Commit:** `76933cb`
- **Tried:** A watchdog fire while `_narrow_gap` is true issues the existing throat creep and does not quarantine the goal.
- **Result:** The creep line fired in every run (81 lines). Ten-run seen mean 88.56 against 88.67 and 93.94. m4 spends about 460 s in the bath and scores 57.78. Lowest `up_z` is 0.861 for about 4 s on n1.
- **Lesson:** Keeping the goal did not by itself get that shell out of the bath. The shoulders are already in the throat before this creep arms.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-doorway-sit-creeps-and-keeps-the-goal) · [Decision](DESIGN_DECISIONS.md#2026-11-14-a-doorway-sit-creeps-and-keeps-the-goal)

### 2026-11-14 — A far-wing goal is dropped at the doorway

- **Commit:** not named; no controller change
- **Tried:** Diagnosis of 13 trapped −0.35 runs on `317f719` and `76933cb`.
- **Result:** On eleven of the thirteen the far wing is selected and then drop-quarantined, piled, or replaced. Door G holds the shell 77–263 s. The west-hall entry holds it 205–348 s. Two runs are goal selection.
- **Lesson:** The binding failure is the ladder at a doorway the other runs do pass.
- **Notes:** [Lessons](LESSONS_LEARNED.md#what-keeps-a-035-run-in-one-wing) · [Decision](DESIGN_DECISIONS.md#2026-11-14-a-far-wing-goal-is-dropped-at-the-doorway)

### 2026-11-14 — A door on the path keeps the goal

- **Commit:** `58157dd`
- **Tried:** A watchdog fire whose path crosses a door cell, while the shell is within 0.8 m, creeps and keeps the goal. A goal already inside 0.8 m still drops.
- **Result:** −0.35 seeds 71–80 cross 7 of 10. Seeds 71–75 are 5 of 5, against 1 of 5 on `76933cb`. Seeds 78–80 never come within 0.8 m of door G or door H. North seeds 71–75 cross 1 of 5.
- **Lesson:** The creep has nothing to keep until the shell is at the door. The north spawn does not get the −0.35 crossing rate.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-door-on-the-goal-path-keeps-the-goal) · [Decision](DESIGN_DECISIONS.md#2026-11-14-a-door-on-the-path-keeps-the-goal)

### 2026-11-15 — Seeds 78–80 lose the goal between fires

- **Commit:** `58157dd` logs; no controller change
- **Tried:** Diagnosis of the three misses.
- **Result:** They do reach door G or the west-hall entry. The 8 s, 1.25× replan replaces the far-wing goal. Door H stays beyond 1.6 m. Seeds 78 and 80 are goal selection. Seed 79 is the ladder after a creep stalls.
- **Lesson:** The replan takes the goal in the gap between watchdog fires.
- **Notes:** [Lessons](LESSONS_LEARNED.md#seeds-78-79-and-80-sat-at-the-other-openings) · [Decision](DESIGN_DECISIONS.md#2026-11-15-seeds-7880-lose-the-goal-between-fires)

### 2026-11-15 — The throat hold

- **Commit:** `d648fe8`, notes `fcc22da` and `4308e85`
- **Tried:** A throat counts as the door, and the saved goal survives a clear. The 8 s door-cell hold (`dbbe08d`) had fired 0 times.
- **Result:** The hold fired 45 times. Three fires were followed by a passage through door G. Nine fires were within 0.8 m of a named door. Thirty-six were not. Crossing 4 of 10 against 10 of 20 on `58157dd` and `dbbe08d` (Fisher p=0.71). Seed 80 crossed with the hold silent.
- **Lesson:** Most fires keep a furniture-gap goal or lose the goal inside 30 s. A seed that flips after about 40 s is run-to-run divergence.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-throat-hold-fires-on-the-live-ten) · [Decision](DESIGN_DECISIONS.md#2026-11-15-the-throat-hold-on-the-live-ten)

## 4. Goal selection

### 2026-11-09 — Seen rooms are not goals

- **Commit:** not named; no controller change
- **Tried:** From the ten already scored, say why pantry, storage, closet, and most baths stay unvisited.
- **Result:** Pantry has 0 goals on all ten. Storage and closet are mostly seen from the door and then drop out of the frontier list. The bath usually still has a frontier and is not selected before 600 s.
- **Lesson:** The 0.45 unvisited-stand weight loses to a real frontier. Raising it would be a new weight.
- **Notes:** [Lessons](LESSONS_LEARNED.md#why-pantry-storage-closet-and-the-bath-stay-unvisited) · [Decision](DESIGN_DECISIONS.md#2026-11-09-seen-rooms-are-not-goals-and-the-bath-frontier-loses)

### 2026-11-09 — The job is not finished at this tip

- **Commit:** `0351bf4` is the tip under discussion; no controller change
- **Tried:** Say what done already means, and whether this tip finishes the apartment.
- **Result:** Every one of the ten ends at the cap, still in `go`, with no mapper done line. Pantry and storage are 0. The best visited draw is 23%.
- **Lesson:** Done is no reachable frontier after a scan and a 4 s still hold. Seen % is not the job. The logs do not name one command change that would finish it.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-apartment-job-is-not-finished-at-this-tip) · [Decision](DESIGN_DECISIONS.md#2026-11-09-the-apartment-job-is-the-visited-floor-and-this-tip-does-not-finish-it)

### 2026-11-10 — The winning frontier is outside the floor

- **Commit:** not named; no controller change
- **Tried:** Rescore the ten saved maps.
- **Result:** 42% of the cells in clusters of at least 0.30 m sit more than 0.35 m outside every floor plate. North run 5's ring scores 28. The bath scores 0.28. The outside cluster is the top score on eight runs.
- **Lesson:** The score is won by free space the map drew past the walls and connected into one frontier.
- **Notes:** [Lessons](LESSONS_LEARNED.md#what-is-left-at-the-time-cap) · [Decision](DESIGN_DECISIONS.md#2026-11-10-the-frontier-that-wins-is-outside-the-floor)

### 2026-11-10 — A spread-out loop does not keep its length

- **Commit:** `058387a`, latch `a307819`
- **Tried:** A cluster that is mostly more than 0.35 m off every floor plate, and spread more than 1.2 m from its own center, keeps only a strip.
- **Result:** On the ten saved final poses the building loop is no longer the goal. The first live launch never latched the plates, because `Robot` has no `getSelf`. `a307819` reads the spawn once. The live ten's visited gaps versus the pre-fix five are inside the old scatter. Bath visited is 0 on nine of ten.
- **Lesson:** The ring can be demoted on a saved map. Live, the shell is not spending the night on that loop, and coverage is not past the pre-fix sample.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-outside-loop-no-longer-takes-the-whole-length) · [Decision](DESIGN_DECISIONS.md#2026-11-10-a-spread-out-loop-past-the-plates-does-not-keep-its-length)

### 2026-11-10 — A bath stand with the shell somewhere else

- **Commit:** not named; no controller change
- **Tried:** From the loop-credit ten, say why n2's 296 s bath stand visits 0% of the bath.
- **Result:** The stand warps into the bath. The true shell is in the bedroom. The command is zero for the rest of the cap and the bath goal is not dropped.
- **Lesson:** A goal inside a floor plate is not a visit. The visit is the true shell on that floor.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-bath-stand-with-the-shell-held-still) · [Decision](DESIGN_DECISIONS.md#2026-11-10-the-bath-stand-stays-selected-while-the-command-is-zero)

### 2026-11-10 — An 1800 s cap still does not finish

- **Commit:** not named; cap only
- **Tried:** Three runs of each seed at `RBM_MAP_MAX_S=1800`.
- **Result:** No run logs the mapper done line. Frontier counts at the cap are 47, 41, 36, 11, 37, 47. n2 holds a stall pivot from t=805 s. m1's seen percent falls from 91 to 21 after loop closure rebuilds the grid.
- **Lesson:** Time is no longer the only limit. A stall pivot can hold a real room edge.
- **Notes:** [Lessons](LESSONS_LEARNED.md#an-1800-s-cap-still-does-not-finish) · [Decision](DESIGN_DECISIONS.md#2026-11-10-an-1800-s-cap-still-does-not-reach-mapper-done)

### 2026-11-15 — Without the plates, the outside ring is chosen again

- **Commit:** `edfd172`, notes `c2cd758`
- **Tried:** Remove the floor-plate credit. Do not add a replacement rule.
- **Result:** Crossing 6 of 10 against 4 of 10 on the plate ten. Every run selects at least one goal more than 0.35 m off every floor plate. The shell spends 0 s off the plates. Seen means 84.20 and 83.92 against 85.99 and 89.66.
- **Lesson:** The ring is back in goal selection. The body does not leave the apartment. The loop is not patched in this round.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-plates-were-prior-knowledge) · [Decision](DESIGN_DECISIONS.md#2026-11-15-the-mapper-does-not-read-the-floor-plan)

## 5. Pose and odometry

### 2026-11-04 — Oracle pose for the planner

- **Commit:** `234734e`
- **Tried:** `RBM_ORACLE_POSE=1` hands the planner the simulator pose. The occupancy grid stays on the wheel and gyro estimate.
- **Result:** −0.35 visited mean 10.9 against 14.7. North 8.1 against 9.0. Every oracle pile was over 60 s. End position error was as large as 30.1 m on one north run, which still visited 11.2%.
- **Lesson:** A true pose for the planner does not fix coverage and does not end the piles. The flag stays off.
- **Notes:** [Lessons](LESSONS_LEARNED.md#true-pose-for-the-planner-same-map) · [Decision](DESIGN_DECISIONS.md#2026-11-04-oracle-pose-for-the-planner-map-left-on-the-estimate)

### 2026-11-04 — Two footprints

- **Commit:** `9ed7ff2` for the rectangle flag; `3472086` for the shared-mask flag
- **Tried:** `RBM_FOOTPRINT_ALIGN` makes the planner use the veto rectangle. `RBM_SHARED_FOOTPRINT` makes the veto use the planner mask. Both stay off.
- **Result:** Visited gaps versus the variance means were inside about 6 points, except a north sample whose sd was large and whose runs often stopped early. Issued paths were still rejected. The quit dump's label on that sample was `start_component`, and the robot still ran to the cap.
- **Lesson:** One shared outline does not end the stall. The quit line stays so the next patch has a measured label.
- **Notes:** [Lessons](LESSONS_LEARNED.md#one-footprint-for-the-planner-and-the-veto) · [Decision](DESIGN_DECISIONS.md#2026-11-04-planner-disk-and-veto-rectangle)

### 2026-11-04 — Start in the robot's own component

- **Commit:** `33bf583`
- **Tried:** `RBM_TRUE_START` starts A* in the free component the robot is standing in. A pile line records the command. The flag stays off.
- **Result:** `start_component` count 0. Two runs stopped early with `start_none`. Visited gaps were inside the noise except a north sample with sd 9.5. The opened pile summaries are recovery, chiefly stall, not a veto hold.
- **Lesson:** The long sit the estimate holds for a minute is a recovery loop.
- **Notes:** [Lessons](LESSONS_LEARNED.md#where-a-long-pile-spends-its-ticks) · [Decision](DESIGN_DECISIONS.md#2026-11-04-start-in-the-robots-component-and-read-the-pile)

### 2026-11-10 — The pose miss is a reverse the pin froze

- **Commit:** `a1604df` logs; no controller change
- **Tried:** Split the final position error by source.
- **Result:** Yaw at the cap is within 4°. The miss above 1 m is a straight reverse whose wheels match the truth while `pinned` freezes the pose. The pin hold is 0.89 m of n3's 1.27 m.
- **Lesson:** Do not pin a straight reverse whose forward range has opened since the pin anchor.
- **Notes:** [Lessons](LESSONS_LEARNED.md#where-the-pose-error-comes-from) · [Decision](DESIGN_DECISIONS.md#2026-11-10-the-pose-miss-is-a-reverse-the-pin-froze)

### 2026-11-10 — An opening nose is not a pin

- **Commit:** `5a546bf`
- **Tried:** A straight reverse whose nose has opened by more than 0.04 m since the pin anchor is not frozen. A flat range and a pitched shell still pin.
- **Result:** The pin's share of n3 fell from 0.89 m to 0.01 m, and of n5 from 0.50 m to 0.00 m. Visited means 18.67 and 17.99. No sample below `up_z` 0.95.
- **Lesson:** The repeated pin on a real reverse is gone from the runs that had it.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-opening-nose-is-not-a-pin) · [Decision](DESIGN_DECISIONS.md#2026-11-10-a-straight-reverse-with-an-opening-nose-is-not-pinned)

### 2026-11-15 — One depth seed is not one trajectory

- **Commit:** the code loaded before `d648fe8`
- **Tried:** Three identical 600 s runs of −0.35, depth seed 78.
- **Result:** Crossing Y, N, Y. Seen 95.65, 87.95, 96.04. Pose 0.15, 0.86, 0.95 m. Planner poses differ by 1 mm at t=18.0 s. Goals differ at t=42.6 s. Ground truth differs by 1 cm at t=48.6 s.
- **Lesson:** A paired comparison of one seed cannot be read past about 40 s.
- **Notes:** [Lessons](LESSONS_LEARNED.md#one-depth-seed-is-not-one-trajectory)

## 6. Floor and world audit

### 2026-11-02 — Skid for the apartment, casters for the warehouse

- **Commit:** not named in the decision; the apartment world arrives in `9e335e7`
- **Tried:** `apartment.wbt` instances `ButlerBotSkid`. A mapping launch aborts if the world is the caster.
- **Result:** Apartment runs print `MAPPING front hubs slaved`. The warehouse lap still reaches the 201 m cap.
- **Lesson:** The caster branch is a different drivetrain. Do not edit `ButlerBot.proto` to fix the apartment.
- **Notes:** [Lessons](LESSONS_LEARNED.md#caster-branch-used-on-the-skid-robot) · [Decision](DESIGN_DECISIONS.md#2026-11-02-four-wheel-skid-for-the-apartment-casters-for-the-warehouse)

### 2026-11-02 — An 80% body, diagnostic only

- **Commit:** not named; the proto was deleted after the run
- **Tried:** One 600 s −0.35 run with the collision footprint, wheels, track, and planner margins at 80%. Scoring still used the full shell.
- **Result:** Kitchen visited 26% against 0% on the full-size round 7. Bath and bedroom were not visited. `camera_stuck` fell from 27 to 2.
- **Lesson:** The full-size kitchen zero on that seed is clearance. The bath and the bedroom are not too narrow to enter. The shipped robot stays full size.
- **Notes:** [Lessons](LESSONS_LEARNED.md#an-80-body-then-the-escape-on-every-phase) · [Decision](DESIGN_DECISIONS.md#2026-11-02-round-8-80-body-diagnostic-only)

### 2026-11-06 — The doorway slots were not a real floor gap

- **Commit:** `7590d16` (the plates). The holes were opened in `0cd3694`.
- **Tried:** Flush plates in door G and the bedroom-bath walkway. Doors, walls, furniture, friction, and the controller stay.
- **Result:** A 60 s spawn crossed door G with hubs at z=0.080. Five runs of each seed: no hub below z=0.078. Visited means 14.79 and 11.29, inside the noise versus 14.7 and 9.0.
- **Lesson:** The 0.102 m slot was the 4 in partition left empty when the carpet ramps were deleted. Wall scrubbing is still in the runs.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-partition-slot-was-not-a-real-floor-gap) · [Decision](DESIGN_DECISIONS.md#2026-11-06-fill-the-doorway-slots-dan-said-are-not-real)

### 2026-11-08 — The collider is on the floor until a corner lifts it

- **Commit:** not named; no controller change
- **Tried:** Check the tire collision shape against the hub heights in the saved logs.
- **Result:** In ordinary driving the hub median is z=0.08000. One corner sit holds the rear hubs at z=0.09425. Another holds both rear tires on a wall at ride height.
- **Lesson:** There is no permanent 0.35 mm tire gap. The corner sits are a lifted axle or a scrubbing tire.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-collider-is-on-the-floor-until-a-corner-lifts-it) · [Decision](DESIGN_DECISIONS.md#2026-11-08-no-command-change-after-the-collision-check)

### 2026-11-08 — The rear sonar box holds the pitched shell

- **Commit:** `6ac2026` (the log line)
- **Tried:** Name the contact under the 14 mm rear lift, and log the pile-leave branch.
- **Result:** At t=175.2 s the rear sonar box is 0.005 mm from the island's west face. The rear-left tire is 0.56 mm clear. The hubs stay at z=0.09425 while the hinges keep turning.
- **Lesson:** The prop is the rear sonar collider. Pile-leave does not consult the stuck-yield flag.
- **Notes:** [Lessons](LESSONS_LEARNED.md#what-held-the-rear-axle-up) · [Decision](DESIGN_DECISIONS.md#2026-11-08-the-rear-sonar-box-holds-the-pitched-shell)

## 7. Measurement, variance, and false wins

### 2026-11-04 — Same-code variance

- **Commit:** `00dae73`
- **Tried:** Five repeats of −0.35 and five of north, no controller change.
- **Result:** −0.35 visited mean 14.7, sd 5.4, range 12.5 points. North mean 9.0, sd 3.9, range 10.2 points. Piles on −0.35 run from 52 s to 460 s.
- **Lesson:** A one-run swing of about 10–14 visited points is the size of the noise. Round 9's north pile of 146 s is shorter than every pile in this north sample.
- **Notes:** [Lessons](LESSONS_LEARNED.md#hitting-the-wall-idea-debt) · [Decision](DESIGN_DECISIONS.md#2026-11-04-same-code-variance-before-another-explorer-patch)

### 2026-11-14 — The clean −0.35 set was depth seed 63

- **Commit:** `317f719` and `76933cb`; no new controller change
- **Tried:** Five more −0.35 runs on each commit, depth seeds 71–75.
- **Result:** Pooled means 88.17 (sd 14.39) and 84.70 (sd 11.85). The gap is 3.5 points. The standard error of the difference is 5.9. The twenty −0.35 seen scores have sd 13.0.
- **Lesson:** The all-at-or-above-90 set was depth seed 63. About 170 seeds of one spawn put the standard error near 1 point. A commit-versus-commit difference at that bar needs about 340 seeds of that spawn on each side.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-clean-035-set-on-317f719-does-not-repeat) · [Decision](DESIGN_DECISIONS.md#2026-11-14-the-035-gap-is-inside-the-seed-noise)

### 2026-11-15 — Crossing moved, and the hold had not fired

- **Commit:** `dbbe08d`, notes `e36d1c2`
- **Tried:** Hold a goal already within 0.8 m of a door on its path when the new path crosses no door.
- **Result:** The hold line fired 0 times. North crossing 3 of 5 against 1 of 5 (Fisher p=0.52). −0.35 seeds 76–80 crossed 4 of 5 against 2 of 5 (p=0.52).
- **Lesson:** The crossing changes are run-to-run divergence. The first goals match, then the runs split.
- **Notes:** [Lessons](LESSONS_LEARNED.md#why-the-hold-missed-seeds-78-and-80) · [Decision](DESIGN_DECISIONS.md#2026-11-15-seeds-7880-lose-the-goal-between-fires)

## 8. Integrity and ground-truth leaks

### 2026-11-02 — Camera-first, no floor-plan foresight

- **Commit:** not named
- **Tried:** The mapper builds its map from its own sensors. Scoring may use the ground-truth pose and the floor solids. The planner may not.
- **Result:** The standing rule is in the lessons file. Seen % and visited % diverge on the same runs.
- **Lesson:** A map painted by rays through a doorway looks finished while the shell is still outside.
- **Notes:** [Lessons](LESSONS_LEARNED.md#standing-rules) · [Decision](DESIGN_DECISIONS.md#2026-11-02-camera-first-no-lidar-no-floor-plan-foresight)

### 2026-11-10 — The plates entered the controller

- **Commit:** `058387a`, then `a307819`
- **Tried:** Pass the apartment floor rectangles and the spawn into goal selection so an exterior loop does not keep its whole length.
- **Result:** The credit does demote the ring on saved maps. The live ten latches 15 plates. From this commit through `28b6536`, the mapper held prior map knowledge.
- **Lesson:** A scoring input that names the apartment's floors is ground truth, even when it is read once.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-outside-loop-no-longer-takes-the-whole-length) · [Decision](DESIGN_DECISIONS.md#2026-11-10-a-spread-out-loop-past-the-plates-does-not-keep-its-length)

### 2026-11-15 — The mapper stops reading the plan

- **Commit:** `edfd172`, notes `c2cd758`
- **Tried:** The mapper is a `Robot`. It does not call `getSelf`, `getPosition`, or `getOrientation`, and it does not import the world-solids file. Goal length is the cell count. The pose logger still records the simulator pose for scoring.
- **Result:** `test_mapper_has_no_pose_or_map_foresight` passes. Pytest: 1130 passed, 1 skipped. The live ten crosses 6 of 10. Every run selects an off-plate goal. The shell spends 0 s off the plates.
- **Lesson:** Honesty puts the outside ring back into goal selection. The body stays on the floor. The ring is not patched in this round.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-plates-were-prior-knowledge) · [Decision](DESIGN_DECISIONS.md#2026-11-15-the-mapper-does-not-read-the-floor-plan)

## 9. Reverted experiments

### 2026-11-02 — Round 6's silent path trim

- **Commit:** not named; deleted in round 7
- **Tried:** Cut a path that ends with the nose in one mapped wall and a side against another. A blind spin with nose 0.12 m reverses along the trail.
- **Result:** −0.35 visited fell from 23.4% to 17.0% and the pile grew from 71 s to 361 s. The nose check fired twice. North's pile was a spin with nose 99, which the check ignores.
- **Lesson:** The silent trim was the regression. Both the trim and the nose check were deleted.
- **Notes:** [Lessons](LESSONS_LEARNED.md#one-stuck-check-not-one-pose) · [Decision](DESIGN_DECISIONS.md#2026-11-02-rounds-68-general-stuck-detection-not-one-pose)

### 2026-11-09 — A raw pitch gate on every wheel step

- **Commit:** `2db10f5`, then reverted
- **Tried:** Drop wheel translation when `oak_imu` pitch is more than 0.12 rad from the learned rest, without waiting for a still floor image.
- **Result:** The gate fired while the body was level (`up_z` 1.000, hubs z=0.080). North visited mean 13.64 against 21.93 on `9ebb7cd`. No run in the pair inverted.
- **Lesson:** One rest sample plus a noisy pitch is not a tip. The 0.12 rad check stays on the floor-still branch only.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-island-climb-was-wheel-travel) · [Decision](DESIGN_DECISIONS.md#2026-11-09-the-island-climb-was-accepted-wheel-travel)

### 2026-11-11 — An agreed wheel step with a moving nose

- **Commit:** `6b8ce43`, reverted in `e929f76`
- **Tried:** Keep a wheel step both hubs agree on when the forward nose has opened by more than 0.04 m since the view was called still.
- **Result:** Visited means fell from 18.67 and 17.99 to 14.07 and 16.60. The 55 s n3 zero and the 40 s n4 and m4 zeros came back.
- **Lesson:** The keep was a regression against `5a546bf`. The frozen-view drop and the floor-still rewind went back.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-nose-keep-was-a-regression) · [Decision](DESIGN_DECISIONS.md#2026-11-11-the-nose-keep-is-reverted-and-a-zero-approach-takes-the-stall)

### 2026-11-13 — Deleting the non-zero recoveries

- **Commit:** `c3e7031`, restored in `081cddc`
- **Tried:** Delete the stuck escape, pile-leave helpers, relief, and contact recovery, and let the watchdog replace them.
- **Result:** North visited mean 4.58 (sd 2.87). −0.35 mean 7.09 (sd 3.54).
- **Lesson:** Those branches issue a non-zero speed. The watchdog only replaces `(0, 0)`.
- **Notes:** [Lessons](LESSONS_LEARNED.md#four-modes-one-command-writer) · [Decision](DESIGN_DECISIONS.md#2026-11-13-four-modes-and-one-owner-of-the-wheel-command)

### 2026-11-15 — The 8 s door hold never fired

- **Commit:** `dbbe08d`
- **Tried:** After the 8 s and 1.25× checks allow a swap, keep the goal when the shell is within 0.8 m of a door on the current path and the new path crosses no door.
- **Result:** The hold line fired 0 times on ten runs. Crossing moved (north 3 of 5 against 1 of 5, seeds 76–80 4 of 5 against 2 of 5) with Fisher p=0.52.
- **Lesson:** The condition did not match the logged sits. The crossing differences are divergence. The later throat hold is `d648fe8`, not a claim that this one worked.
- **Notes:** [Lessons](LESSONS_LEARNED.md#why-the-hold-missed-seeds-78-and-80) · [Decision](DESIGN_DECISIONS.md#2026-11-15-seeds-7880-lose-the-goal-between-fires)

## 10. Waypoint driver

### 2026-11-17 — One command, then a stop

- **Commit:** `ae27189`, notes `77c3ab8`
- **Tried:** `RBM_DRIVER=1`. Four fixed aims in the robot's odometry frame, then `stop`. Headless, stock spawn. No live ten.
- **Result:** Statuses ready, arrived, arrived, arrived, arrived, stopped. First stop estimate (0.32, 0.00), commanded 0.320 m, wheel 0.320 m, floor 0.335 m. Second (0.82, 0.00), 0.499 / 0.500 / 0.522 m. Harness pose log moved 0.82 m. Webots exited 0.
- **Lesson:** The driver can take an outside aim, stop, and wait, without reading the simulator pose.
- **Notes:** [Lessons](LESSONS_LEARNED.md#a-waypoint-waits-for-the-next-command) · [Decision](DESIGN_DECISIONS.md#2026-11-17-a-waypoint-driver-stops-and-waits)

### 2026-11-17 — Arrived was the frontier radius

- **Commit:** `a1cb535`
- **Tried:** A waypoint used `ARRIVE_M` (0.28 m), the distance at which a frontier counts as reached. The smoke stopped 0.28 m short and the estimate agreed.
- **Result:** Arrival is now 0.05 m, and the last 0.35 m is slower. The same smoke: `go_to 0.6 0` estimate (0.55, −0.00), remaining 0.050 m, wheel 0.549 m, floor 0.588 m, truth (6.103, 5.246). `go_to 1.1 0` estimate (1.05, −0.01), remaining 0.050 m, wheel 0.500 m, floor 0.552 m, truth (6.106, 5.740). Both status arrived. Turn and look arrived. Truth travel 1.04 m. A go_to that cannot close the gap is `stopped_short`.
- **Lesson:** 0.28 m is a frontier, not a named point.
- **Notes:** [Lessons](LESSONS_LEARNED.md#arrived-was-the-frontier-radius) · [Decision](DESIGN_DECISIONS.md#2026-11-17-a-waypoint-arrives-at-5-cm)

### 2026-11-17 — The turn used the 12° pivot gate

- **Commit:** `f0e942e`
- **Tried:** `turn_to 0.40` arrived at estimated yaw 0.194. That gap is `PIVOT_EXIT_RAD` (12°). During `look`, floor travel was 0.902 m while the wheels reported 0.014 m.
- **Result:** A heading arrives inside 0.05 rad. Inside the existing 12° the spin scales down with the error. The same smoke: turn estimate yaw 0.351, remaining 0.049 rad, truth yaw 1.899 (from 1.581). Look floor 0.001 m, marked unreliable; truth moved about 0.8 cm. The pose update already drops a floor shift when the yaw in that window exceeds 0.08 rad, so the controller does not write the spin into the pose.
- **Lesson:** 12° is a pivot release, not a named heading. A floor slide during a spin is the picture rotating.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-turn-used-the-12-pivot-gate) · [Decision](DESIGN_DECISIONS.md#2026-11-17-a-heading-arrives-at-005-rad)

### 2026-11-18 — Watch party, one recorded route

- **Commit:** logbook only. No controller change.
- **Tried:** `RBM_DRIVER=1`, `RBM_MOVIE=1`, rendering on, stock spawn. Sixteen commands: look, out to (1.2, 0), look, face 1.57, to (1.2, 0.8), look, face 3.14, to (0.4, 0.8), look, face −1.57, to (0.4, 0), home, face 0, look, stop.
- **Result:** Every command arrived. No backoff. No wall-contact row. Lowest `up_z` 1.000. Each look turned the estimate 6.20–6.21 rad and the truth 6.19–6.21 rad. Pose error was 0.01 m outbound and 0.24 m at the return home. Yaw error stayed inside 0.014 rad. Floor travel on the looks was 0.000–0.001 m and marked unreliable.
- **Lesson:** The route fit the open floor. The position estimate walked about a quarter metre by the time the shell came home. The heading did not.
- **Notes:** [Lessons](LESSONS_LEARNED.md#the-turn-used-the-12-pivot-gate) · [Decision](DESIGN_DECISIONS.md#2026-11-17-a-heading-arrives-at-005-rad)
