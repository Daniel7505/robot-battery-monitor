# Cheat sheet: restart guide

For Dan, or for a new assistant session picking this up cold. Last updated
2026-10-01, after PR #8 (sharp corners) was merged. The details live in the linked docs.

## North star

This project builds an **onboard agent that designs the cheapest real robot
that can do a mission**:

1. Keep a parts catalog.
2. Take a mission or track.
3. Assemble candidates.
4. Evaluate them in the Webots twin, then on hardware.

The wheeled ButlerBot, the PMS dashboard and lane keep are the **evaluation
harness**, not the product. Warehouse travel needs ~5–20 cm accuracy; docking
needs 1–3 cm. See `docs/NORTH_STAR.md`.

## Architecture now

* **Cameras.**
  * Two nadir cameras, `nadir_left` and `nadir_right`, 128×128, fov 1.2.
  * They sit in the shoulder boxes `NADIR_BOX_L/R` at
    `translation 0.03542 ±0.41808 0.70419` and `rotation 0 1 0 0.9411`
    (pitch 53.9°, no yaw).
  * Floor rows reach 0.06 m (bottom) to 1.96 m (top) ahead of the axle.
  * At start-up the controller reads the live pose through the Supervisor.
    It falls back to the `.wbt`, then to constants, and prints `CAM POSE …`.
* **`src/lane_vision.py`: row-fit lane vision.**
  * It builds a yellow mask (`rg` mode: min(R,G) − B, so red and green bars
    are not paint).
  * Stripe-centre points come from the row and column runs, projected to the
    floor with `CameraModel`.
  * Two-line arc fit: a near model (offset and heading) plus a far model
    (curvature). The lane width is measured online.
  * `LaneTracker` keeps the priors and runs the **plausibility gate**:
    * |heading| > 45° is rejected.
    * Heading and offset jumps are rejected vs odometry and IMU.
    * A side that vanishes at low confidence is rejected.
    * A short view is held.
    * A rejected fit is held. Three lost frames brake.
  * **Corner detector** (Dan's "line widens, then disappears"):
    * A row run at least 3× the stripe width, smeared toward the turn, with no
      paint beyond it.
    * The outer line's cue is required. It must persist for 2 frames.
    * Output: `est.corner` (side, distance, conf).
  * **Junction classifier** (`src/junction_vision.py`, same line tracer):
    per side BEND_OUT (opening) / BEND_IN (corner) / END (gap) / CONT, plus
    far bars and resumed lines → `est.junction`: `t_end`, `plus`, `side_L/R`,
    `gap_straight`, `lane_lost`, `dead_end` (and `corner_L/R`), with
    openings L/S/R (`S?` = far side beyond the 1.96 m reach), near / centre
    distance, conf, 2-frame persistence. `est.view` = nearest yellow
    ahead/left/right.
  * Post-pivot: `begin_reacquire` relaxes the gate for 6 frames. Then
    `begin_settle` widens the jump limits for 5 frames. The controller calls
    it again on REACQUIRE → LANE.
* **`src/rowfit_control.py`.**
  * Pure pursuit on the fitted centre line, plus a curvature speed governor
    (0.44 m/s cruise, 0.20 min).
  * State machine `LANE → CORNER_APPROACH → PIVOT → REACQUIRE → LANE`:
    * It stops at the pivot point: outer line − lane width / 2.
    * It pivots in place on IMU yaw to ±90° (±1°, ≤ 0.9 rad/s).
    * It re-acquires slowly. If there is still no lane it brakes.
  * Intersections (`src/route_policy.py`): `LANE → JUNCTION_APPROACH`, then
    a turn (CORNER_APPROACH/PIVOT at the junction centre), straight on past a
    side branch, or `GAP_CROSS` (blind 0.15 m/s on IMU heading, up to
    `RBM_MAX_BLIND_M`) → REACQUIRE. Nothing by then: `LOOK_AROUND` (+90°, −90°,
    0°) and `STOPPED`. Route = `RBM_ROUTE`, default straight-else-left.
* **Controller glue** (`webots/controllers/butlerbot_controller/`):
  * `controller_rowfit.py`: `RowfitRuntime` and the `LaneVisionLog` writer.
  * `controller_hud.py`: shoulder HUDs (state line, fit dots, crossbar bar).
  * The GPS finish referee comes from the loaded track. It is only a referee,
    never used to steer.
* **Tracks.**
  * `tracks/*.json`: waypoints, per-corner `radius_m` (0 = sharp),
    per-waypoint lane width, 6 cm stripe, bars.
  * Tracks: `s`, `corner90`, `corner_mix`, `widen`. Sim-only:
    `corner90_right`, `corner90_wide`.
  * Intersection tracks (a road graph: `roads`, `exit`s, `routes`; lines are
    cut where another road joins): `plus`, `t_end`, `t_left`, `t_right`,
    `gap15`, `gap25`, `gap35`, `gap45` (crossing 1.5–4.5 m wide).
  * Geometry is in `src/track_geometry.py`: exact mitred and filleted offsets,
    `project()` sampler, `FinishReferee` with `crossed()` and `cross_track()`.
  * `scripts/track_builder.py <track> --world <wbt>` writes the floor paint.
    See `docs/TRACKS.md`.
* **Worlds** (`webots/worlds/`). All share the same Robot block.
  * `butlerbot.wbt`: the 16.5 m S.
  * `butlerbot_corner90.wbt`, `butlerbot_corner_mix.wbt`, `butlerbot_widen.wbt`.
  * `butlerbot_plus.wbt`, `butlerbot_t_end.wbt`, `butlerbot_gap45.wbt`
    (intersections; the referee reports which exit was crossed).
* **Offline sim.** `scripts/rowfit_sim.py`: kinematic diff-drive, the same
  camera model and tracker, a simulated IMU, and pivots.

## Env vars

Set them with `set NAME=value` in cmd before launching, or put them in the
repo `.env`. A value set with `set` wins.

| var | meaning |
|---|---|
| `RBM_LANE_MODE` | `rowfit` (default when unset) or `gap`, the old pixel-gap lane keep, not re-tuned for the box cameras |
| `RBM_TRACK` | Track name or `.json` for the finish referee and the `ct=` console value. It overrides the world's `# TRACK_FILE` tag. |
| `RBM_WORLD` | World for the launcher: `butlerbot`, `corner90`, `corner_mix`, `widen`, `plus`, `t_end`, `gap45` or a `.wbt` file. Same as `-World`. |
| `RBM_ROUTE` | Junction choices in order, e.g. `S,L,R` (S = straight, L, R). Unset = straight if possible, else left. A choice the junction does not offer falls back to that default. |
| `RBM_MAX_BLIND_M` | Max distance crossed with both lines gone (GAP_CROSS). Default `4.0`. |
| `RBM_LOOK_AROUND` | `0` = after the max blind distance just stop (no +90/−90 look-around). Default on. |
| `RBM_LOG_DIR` | Folder for `lane-vision.csv` and `steer-actions.csv`. Default: `%USERPROFILE%\OneDrive\Desktop\Grok Workspace` |
| `RBM_NADIR_GUARD_PER_FRAME` | `1` = gap-mode NadirGuard counts per camera frame (opt-in; gap mode only) |

## Commands (Windows cmd, from the repo folder)

Update:

```bat
git checkout main
git pull
pip install -r requirements.txt
```

Launch a world (the dashboard twin must be running: `scripts\start.ps1`):

```bat
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World corner90
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World corner_mix
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World widen
```

Intersections. `set` lasts for that cmd window; clear with `set RBM_ROUTE=`:

```bat
set RBM_TRACK=plus
set RBM_ROUTE=L
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World plus

set RBM_TRACK=t_end
set RBM_ROUTE=R
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World t_end

set RBM_TRACK=gap45
set RBM_ROUTE=
set RBM_MAX_BLIND_M=4.0
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World gap45
```

Score runs. `--track` must match the world, and runs are split by `run_id`:

```bat
python scripts\drift_report.py "%USERPROFILE%\OneDrive\Desktop\Grok Workspace\lane-vision.csv" --track s --last 5
python scripts\drift_report.py "%USERPROFILE%\OneDrive\Desktop\Grok Workspace\lane-vision.csv" --track corner90
```

Offline sim:

```bat
python scripts\rowfit_sim.py corner90 corner_mix corner90_right corner90_wide s widen turn90 --table
python scripts\rowfit_sim.py corner90 --turn-eff 0.8 --imu-drift 0.5 --table
python scripts\rowfit_sim.py corner90 --no-corners --table
python scripts\rowfit_sim.py plus --route L --table
python scripts\rowfit_sim.py gap45 --max-blind 2.0
python scripts\intersection_matrix.py
```

Tests (CI runs the same `pytest`; the docker test skips without docker):

```bat
python -m pytest -q
```

## Save tags (known-good points)

| tag | what |
|---|---|
| `save-2026-10-01-lanekeep-v1` | gap-mode lane keep, full S on the old camera mount |
| `save-2026-10-01-rowfit-boxcams` | rowfit default, cameras in the shoulder boxes, plausibility gate (PR #7) |
| `save-2026-10-01-corner90-v1` | waypoint tracks and worlds, sharp-corner pivot, post-pivot gate settling (PR #8) |

To go back: `git checkout save-2026-10-01-corner90-v1`. Return with `git checkout main`.

## Best scores

See `docs/RUN_HISTORY.md`.

| course | run_id | result |
|---|---|---|
| S, box cameras, rowfit | 20261001-201555 | finished, max 7.4 cm, mean 2.8 cm, rms 3.7 cm, 77.3 s |
| S, old mount, rowfit | 20261001-193237 | finished, max 9.3 cm, mean 3.2 cm, 65.7 s |
| corner90, rowfit + pivot | 20261001-224819 | finished, max 3.9 cm, mean 0.8 cm, rms 1.3 cm, 39.9 s, pivot err +0.7° |

## Gotchas

* **OneDrive locks folders during `git checkout` or `git pull`.** Git asks
  `Unlink of file … failed. Should I try again? (y/n)` or
  `Deletion of directory … failed`. Answer **n**. The checkout still
  completes; only the locked empty folder stays behind. Pausing OneDrive sync
  avoids it.
* **New CSV columns: rename `lane-vision.csv`.** New rows appended to an old
  file still line up for the old columns. The new columns
  (`state,corner_dir,…`) only get a header when the file is created. Rename or
  move the old file, and the next run writes a fresh header.
* **HUD overlays and `.butlerbot.wbproj`.** Webots saves the overlay
  positions and visibility per world in `webots\worlds\.<world>.wbproj`.
  Webots writes this file on your machine; it is not in git. The launcher
  copies `.butlerbot.wbproj` to `.<world>.wbproj` the first time a new world
  is opened. If a world's HUDs are missing, stacked or hidden, re-enable
  them in Webots (right-click the overlay / Overlays menu) and save the
  world, or delete that world's `.wbproj` and launch it again.
* **`getWorldPath` fallback: set `RBM_TRACK`.** The finish referee finds the
  track from the world's `# TRACK_FILE` tag via `robot.getWorldPath()`. If the
  console says `FINISH REFEREE finish x>=16.5 m (default S)` on a non-S world,
  that lookup failed. Then `set RBM_TRACK=corner90` before launching.
  Otherwise the GPS finish and the `ct=` values are wrong. Steering is not
  affected.
* **Gap mode** (`RBM_LANE_MODE=gap`) was tuned on the old z 1.31 m camera
  view. Do not trust it on the box cameras.
* **Webots console times** are sim seconds. `run_id` is the wall-clock
  controller start.

## Open ideas / next steps

* **Camera yaw 10° toe-out** (`docs/CAMERA_YAW_STUDY.md`): it widens lane
  coverage. Dan has not applied it yet; he likes the current look-ahead.
* **Gap mode:** retire it, or re-tune it for the box view.
* **Track importer from Paint, Unity or CAD.** It only needs to write the
  `tracks/*.json` format.
* **Intersections, next steps.** Built (see the Intersections section of
  `docs/ROWFIT_LANE_VISION.md`; the large-intersection / limited look-ahead
  case is Dan's). Open: oblique (non-90°) branches, back-to-back junctions,
  heading correction while blind (a drifting gyro limits the widest plaza),
  wide T junctions in a test course, a world for `t_left` / `t_right`.
* **Bot-body reference in the frame.** Use the robot's own visible parts
  (front caster or wheel corner) as an in-image calibration reference.
* **480 Wh hardcode.** `BATTERY_CAPACITY_WH = 480.0` at
  `src/teleop_agent.py:227` should come from the hardware profile YAML
  (see `docs/HARDWARE_PROFILE.md`).
* **`DASHBOARD_PORT` compose bug.** `docker-compose.yml` maps
  `127.0.0.1:${DASHBOARD_PORT}:5000`, but the dashboard container also gets
  `DASHBOARD_PORT` from `env_file: .env`. `src/config.py` maps it to the app
  port, so any value other than 5000 makes the app listen on a port the
  mapping does not forward. Fix: map `${DASHBOARD_PORT}:${DASHBOARD_PORT}`,
  or pin the in-container port.
* **Dependencies:** review the open Dependabot PRs and alerts, and pin or
  bump `requirements*.txt`.
* **Repo rename.** "robot-battery-monitor" is leftover from the original
  battery-monitor scope; something like "robot-design-agent" would fit better.
