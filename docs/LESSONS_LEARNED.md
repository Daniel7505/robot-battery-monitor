# Lessons learned

Read this before changing apartment mapping. Update it at the end of every round: symptom, root cause, what fixed it (or that it is still open), and dead ends not to retry.

## Standing rules

- No map, room size, or floor-area foresight to the robot. Ground truth is for scoring only.
- Camera-first. No lidar. Depth, floor camera, gyro, and sonars are the sensors.
- The apartment robot is the 4-wheel skid-steer (`ButlerBotSkid`) on purpose. `ButlerBot.proto` stays for the warehouse and must keep working.
- Never commit run artifacts (logs, maps, frames, clips) or secrets.
- Delete superseded code. Do not leave a second path that does the old thing.
- Decision logs use the fixed reason-code pool, one file per seed (`decisions.txt`, `summary.txt`).

## Caster branch used on the skid robot

Symptom: mapping fixes from PR #38 (`cursor/apartment-mapping-5970`) were applied as if that tree were the robot under test. That branch still instances the old caster `ButlerBot`.

Root cause: two robots share the repo. The caster proto is the warehouse. Apartment mapping is `ButlerBotSkid` only. Starting from a caster head, or launching `apartment.wbt` while it still said `ButlerBot`, ran the wrong drivetrain.

What fixed it: port the mapping ideas onto the skid head (`cursor/jamb-veto-5970`, then this branch). Every mapping launch aborts unless the world instances `ButlerBotSkid` and not the caster. As of this round, `apartment.wbt` instances `ButlerBotSkid` directly. The swap shim is gone.

Dead ends: do not run Webots from the caster branch. Do not edit `ButlerBot.proto` (casters, depth noise, warehouse hash) to change apartment mapping. Do not delete the caster proto.

## Seen from the doorway, never driven in

Symptom: coverage near 93% while the path never entered the bathroom or the kitchen proper. Bath was about 95% seen and 0% visited.

Root cause: coverage is cells the depth rays labeled. A look through a doorway paints the next room free, which removes the frontier, so frontier-only exploration has nothing to drive toward. Visited is the shell rectangle swept along the ground-truth pose. The robot's own `visited` grid is the same idea and is not ground truth.

What fixed it: still open for the kitchen and the bath. The robot's own visited grid now keeps a goal inside another region of its map, and the settle step uses that same grid, so a doorway peek is not "done." On the round 4 check the bedroom was driven (−0.35 visited 24%, north 14%). Kitchen was 2% and 0%. Bath stayed 0%. The scorer's visited % is still ground truth only.

Dead ends: do not feed room rectangles or floor area into the planner. Do not treat seen % as "went there." Do not stand only at the 1–3 m viewpoint and call the room explored.

## Bedroom southwest blind corner

Symptom: on seed −0.35, 171 still-floor events and about 206 s at (0.35, 4.97). That is the bedroom's west wall meeting the south wall, not the walkway door (the gap starts near x = 0.81). Nose reading 0.12 m the whole time. Heading stuck near −161°.

Root cause: the depth camera was inside its 0.35 m minimum, so 0.12 m is the blind placeholder, not a measured gap. The frontier-failure counter skipped every blind look, including a room with no translation. The relief reverse was a fixed 0.45 s (about 4 cm), shorter than the blind zone, then the same goal was scored again.

What fixed it: a blind view plus no translation, place tag room, counts as a frontier failure. The robot backs straight out until depth returns a range at least 5 cm past the minimum and the shell has moved 0.30 m. A doorway exemption remains only for an actual crossing (doorway tag and a straight creep or a forward drive). An open-floor pivot is still exempt. The following −0.35 run spent 0 s in that corner.

Dead ends: do not ban a frontier the robot is already crossing. Do not treat the 0.12 m placeholder as an open hallway or as a 12 cm gap. Do not use the 12 s avoid-weight fade as the only way off a patch.

## Visible stall (floor still, nose about 1 m)

Symptom: round 2 stock sat 527 s at (3.29, 1.60) in the hall, 258 events. −0.35 sat 528 s at (3.85, 5.16). The camera was not blind. Floor-still froze translation, the 0.45 s reverse did not move the shell, and a new goal was scored every few seconds.

Root cause: the floor picture did not shift, and the ground-truth pose did not either, so the freeze matched a body that was actually still. It was not a low-texture false alarm (a blank frame does not freeze). The miss was the response: a short reverse, then another goal in the same region. Holding one goal point was not enough, because the next goal was a different coordinate in the same pile.

What fixed it: partial, and the 60 s bar is still open. Round 4 stops the clock from resetting on a spin. At 60 s the shell must creep straight, and a pivot that meets the map or the live side gap is refused. On the two-seed check the guard did log "pile, leaving." −0.35 then moved 2.7 m and 1.5 m. North's second trip commanded zero speed, because both straight ways failed the sweep, and the ground-truth pose moved 0.00 m in the next 30 s. Longest stays were 181 s and 348 s. North's corner steps fell from 1410 to 0. Stuck stayed 0.

Dead ends: do not turn off floor-still. Do not go back to a 0.45 s reverse. Do not spin out of the ball when the corner sweep fails; that put the corner on the wall. Holding zero speed when both straights are closed does not leave the ball either.

## Pivot swept a corner into the wall

Symptom: corner-contact steps in the hundreds. `heading_off` stayed high because a pivot kept going after the nose was almost on the path.

Root cause: turn checks used a disk. The real shell is a rectangle; the corner swings outside the tires. A heading error of one grid cell restarted the pivot.

What fixed it: pivots and near-wall arcs sweep the rectangle. A corner that meets the map becomes the existing straight backup. A pivot ends at 12° and does not start again until 25°. Corner steps on the next four seeds were 0. The jamb veto, the 0.23 / 0.30 m approach clearance, and the goal-ahead rule on ordinary backups stayed.

Dead ends: do not inflate the 0.20 m planning radius to the sonar face (0.227 m) or to 0.26 m. That closes a doorway. Do not sweep straight driving the same way.

## Other traps

- A seed folder that still contains `mapping_done` makes the pose logger quit the next launch immediately. Delete the old folder before a rerun.
- `pkill` on a broad webots pattern kills the other seed's X server. Kill by port.
- Odd frame sizes make ffmpeg fail. Scale to even width and height.
- OAK-D depth noise is applied in the mapper (quadratic, firm to 4 m, soft to 5 m). The skid proto's device noise stays 0. The caster proto's 0.002 stays for the warehouse hash.
- PR #40 (pytest-xdist, idle since 27 Oct) is not safe to merge as-is. CI failed: workers raced on Postgres and hit `allocation_snapshots` unique violations. The speedup is real when the database tests stay on one worker. Do not close it; the race wants a fix. Do not merge it from a mapping round.

## Round log

Append one line per round: date, what changed, and the four-seed pile / visited outcome.

- 2026-11-02 round 3. Stall escape, unvisited-floor goals, apartment instances `ButlerBotSkid`. Piles 95 / 235 / 439 / 250 s. Kitchen visited 0 % on all four. Worst pile −0.35 at (3.99, 5.04).
- 2026-11-02 round 4. Hard 60 s straight-leave, no clipped escape spin, done-check uses the robot's visited grid. Two-seed check only (−0.35, north): piles 181 s and 348 s, stuck 0, kitchen visited 2 % and 0 %, bedroom 24 % and 14 %, bath 0 %. North corner steps 0. Full four seeds not run. The guard stops in place when both straights are closed.
