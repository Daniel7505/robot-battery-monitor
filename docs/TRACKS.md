# Waypoint tracks (phase 1 of the sharp-corner project)

A track is a small data file in `tracks/`. Everything that paints, scores or
referees a course reads it. **The onboard lane keep never does**: it still
only sees the camera images, with no prior knowledge of width, shape or turns.
A future Paint / Unity / CAD importer only has to write this format.

| file | course |
|---|---|
| `tracks/s.json` | the S in `butlerbot.wbt` (826 waypoints every 2 cm, written by `python scripts/s_track.py --write-track`) |
| `tracks/corner90.json` | 4 m straight, **sharp** 90° left (true right angle), 4 m to the finish |
| `tracks/corner_mix.json` | 4 m, rounded left (centre radius 0.5 m), 4 m, sharp 90° right, 4 m |
| `tracks/corner90_right.json` | mirror of corner90: sharp 90° **right** (sim test track, no world) |
| `tracks/corner90_wide.json` | corner90 with a 1.60 m lane (sim test track, no world) |
| `tracks/widen.json` | straight; lane widens 1.30 → 1.60 m over 2 m, holds 2 m, narrows back over 2 m; finish x = 11 |

## Format

```json
{
  "name": "corner90",
  "description": "Hallway corner: 4 m straight, SHARP 90 deg left, 4 m straight to the finish.",
  "lane_width_m": 1.30,
  "stripe_width_m": 0.06,
  "paint_before_start_m": 0.3,
  "paint_after_finish_m": 0.5,
  "start_bar": true,
  "finish_bar": true,
  "waypoints": [
    [0, 0],
    {"x": 4, "y": 0, "radius_m": 0},
    [4, 4]
  ]
}
```

* `waypoints`: the lane **centre line** in world metres (ENU, x forward from the robot's start pose).
  * The robot starts at the first waypoint, facing the second. The worlds keep the robot at (0, 0) facing +x, so start there.
  * The last waypoint is the **finish**.
  * Each entry is `[x, y]` or an object `{"x", "y", "radius_m", "lane_width_m"}`.
* `radius_m` (interior waypoints, default 0): centre-line corner radius. 0 means a sharp corner.
  * Sharp corner: both lines get a true **mitre** where the offset legs meet. Corner90's inner line turns at (3.35, 0.65) and its outer line at (4.65, −0.65).
  * Rounded corner of radius r: the outer line is an arc of r + w/2. The inner line is an arc of r − w/2, or a sharp mitre if that is ≤ 0, so a line never folds back on itself.
  * A leg too short for its corners is rejected with the leg named.
* `lane_width_m` (top level = default 1.30; per waypoint = varies): linear in arc length between the waypoints that set it. The ends hold their value.
* `stripe_width_m` (0.06): width of each painted line.
* `paint_before_start_m` / `paint_after_finish_m`: straight run-in / run-out of the lines.
* `start_bar` / `finish_bar`: green / red bars across the lane at the first / last waypoint (lane width + 0.10 m long, 8 cm wide).
* `ticks` (optional): `[x, y]` or `[x, y, heading_rad]` white centre-line ticks.

Code:
* `src/track_geometry.py`: load, offset curves, centre-line sampler (`project` → arc length, signed lateral, heading), finish referee.
* `scripts/track_builder.py`: meshes and the world.

## Intersections (road networks)

A track file with `roads` is a small road graph. `waypoints` stays the main
road (the robot starts on its first waypoint, facing the second); each entry
of `roads` is another road with its own waypoints and lane width. Roads join
wherever their carriageways overlap. Painting rule, the same for every
junction shape: a road's lane lines are painted everywhere EXCEPT inside
another road's carriageway, so at a crossing the lines have gaps exactly
where the other road joins and the line ends meet at right angles.

```json
{
  "name": "plus",
  "lane_width_m": 1.30,
  "waypoints": [[0, 0], [8, 0]],
  "exit": "east",
  "roads": [{"name": "cross", "waypoints": [[4, -4], [4, 4]], "exit_start": "south", "exit_end": "north"}],
  "routes": {"S": "east", "L": "north", "R": "south", "default": "east"}
}
```

* `exit`: name of the main road's end as an exit (`null` = the main road ends
  in the junction, as in `t_end`).
* `roads[].exit_start` / `exit_end`: exits at that road's ends; a road end
  without one gets no run-out paint (a branch's start inside the junction, a
  plaza with no exits).
* Every exit gets a red bar `TRACK_EXIT_<NAME>` and is a finish line for the
  GPS referee, which prints which exit was crossed.
* `routes` (optional, for the sim and tests): which exit each `RBM_ROUTE`
  choice should end at. A key can be a whole route string
  (`"S,S,L": "shipping_1"`); otherwise the first letter is used.
* `missions` (optional, warehouse): named drives `{to, via}`. `src/route_planner.py`
  works out the `RBM_ROUTE` for each one (one entry per junction with two or more ways on).
* Old single-road files are unchanged (no `roads` = `TrackGeometry`).
* The world block has one `TRACK_LINES` Shape with all pieces, `TRACK_START`,
  and the exit bars.

| track | layout | world |
|---|---|---|
| `plus` | 4 m, a 1.30 m road crosses, 4 m on. Exits east (S), north (L), south (R) | `butlerbot_plus.wbt` |
| `t_end` | 4 m, the lane ends at a crossing road: L (north) or R (south) only | `butlerbot_t_end.wbt` |
| `t_left` / `t_right` | 4 m, a branch opens left / right, the lane carries on 4 m | sim only |
| `gap15` … `gap45` | 4 m, a crossing 1.5 / 2.5 / 3.5 / 4.5 m wide (a plaza, no exits), 4 m on | `butlerbot_gap45.wbt` |
| `warehouse` | prototype 70 × 65 m warehouse: 13 roads (every aisle taped), 32 junctions, 6 corners, a 3 m tape gap, 3 exits. Generated by `scripts/warehouse_builder.py`, see `docs/WAREHOUSE.md` | `butlerbot_warehouse.wbt` |

## Paint a world

```
python scripts/track_builder.py corner90 --world webots/worlds/butlerbot_corner90.wbt
python scripts/s_track.py --write-track --write-world      # the S (butlerbot.wbt)
```

* If the world does not exist, it is created from `butlerbot.wbt`. Robot, cameras, HUDs, lights and floor are copied unchanged (a test checks the Robot block is byte-identical). Then the track block is replaced.
* The track is one `DEF TRACK Pose` with Shapes `TRACK_LINE_L`, `TRACK_LINE_R`, `TRACK_START`, `TRACK_FINISH` (and `TRACK_TICKS` on the S). Each is a single `IndexedFaceSet` ribbon:
  * top, walls and end caps
  * `ccw TRUE`
  * top faces wound +Z
  * no `solid` field (not valid in R2025a)
* Lines top out at z 0.013, bars at 0.025.
* The block carries a `# TRACK_FILE tracks/<name>.json` tag, and WorldInfo `info` gets a `TRACK (ENU): …` line.

S check: the S built from its waypoints matches the old exact normal-offset ribbon within 0.14 mm on the lines (test: within 1 mm). Bars and ticks are identical.

## Open a world (Windows cmd, from the repo folder)

```
cd /d "%USERPROFILE%\path\to\robot-battery-monitor"
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World corner90
```

* `-World` takes:
  * `butlerbot` (default, the S)
  * `corner90`, `corner_mix`, `widen`
  * `plus`, `t_end`, `gap45` (intersections)
  * a file name such as `butlerbot_corner90.wbt`
  * a path
* `set RBM_WORLD=corner90` before the command does the same.
* Without the launcher (no dashboard twin): `"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" webots\worlds\butlerbot_corner90.wbt`, or File → Open World in Webots.
* HUD overlay placement is stored per world in `webots\worlds\.<world>.wbproj` (gitignored). The launcher copies `.butlerbot.wbproj` to the new world's name the first time, so the shoulder HUDs land where they are in the S world. If you open the world another way, arrange the overlays once.

## Finish referee (GPS, referee only)

The controller used to stop at `GPS x >= 16.5`. It now asks
`src.track_geometry.referee_for`. Source order:
1. `RBM_TRACK` (a track name or file).
2. The `# TRACK_FILE` tag in the loaded world (`robot.getWorldPath()`).
3. The built-in S rule.

A track's finish counts when the robot is across the finish line (along the last leg), within a lane width of its centre, and past the middle of the course. That last condition stops a leg that passes behind the finish line from triggering it.

Console at start-up:
* `FINISH REFEREE track corner90: finish (4, 4) heading +90deg, 8.00 m (world butlerbot_corner90.wbt) — GPS referee only, not used for steering`
* At the end: `TRACK corner90 DONE at x=… y=…` (the S still prints `FULL S DONE`).

## Score a run

```
python scripts\drift_report.py "%USERPROFILE%\OneDrive\Desktop\Grok Workspace\lane-vision.csv" --track corner90
```

* `--track` takes any track name or `.json` file.
* max / mean / rms are the signed sideways distance from that track's centre line (perpendicular, via the sampler).
* `dist m` is progress along the track.
* `done` means the run crossed the track's finish line (0.15 m tolerance, as before).
* Without `--track` it is unchanged (|y|).

## Baseline in the offline sim (before the corner logic)

> Superseded: rowfit now pivots at sharp corners. See "Sharp corners" in
> `docs/ROWFIT_LANE_VISION.md`. corner90 and corner_mix now finish. The table
> below is the pre-corner baseline, kept for comparison
> (`rowfit_sim.py corner90 --no-corners` now stops *at* the pivot point instead).

`python scripts/rowfit_sim.py corner90 widen corner_mix` (any track name or file works).

| course | result |
|---|---|
| widen | **finishes**, 29.0 s. Max 1.7 cm off centre. Measured lane width follows 1.30 → 1.60 → 1.30 (worst lag 5 cm, at the start of the narrowing). Full speed 0.44 m/s the whole way, no held frames. |
| corner90 | **does not finish; stops at the corner, stays in the lane.** Cruises at 0.44 m/s. The corner enters the fit at x ≈ 2.85 m (1.15 m before the corner vertex), and the speed governor drops to 0.23–0.29 m/s. From x = 3.24 m the plausibility gate rejects the fits (`heading jump +21deg > 17deg`), so it holds the straight lane and steers almost nothing (steer ≤ 0.07). It drives past the inner corner and brakes (lane lost 3 frames) at x = 4.29, y = 0, yaw +2.9°. That is 0.36 m short of the outer line, never turned. |
| corner_mix | Rounded 0.5 m left corner: **taken**. Max 14 cm off centre, at the arc; bend speed 0.26–0.30 m/s. Sharp right at (4, 4): gate rejects (`offset jump -45cm`) at y = 4.07, lane lost, **brakes at (4.00, 4.24), yaw 87.5°**. Does not finish; stays in the lane. |
| S (regression, built-in and `tracks/s.json`) | unchanged: worst 7.8 cm / mean 2.8 cm, parks at x = 16.50 |
| turn90 (1 m radius) | unchanged: 7.4 / 3.0 cm |

So the baseline to beat on the sharp corner is: it sees the corner about 1.2 m ahead and slows, but the fit cannot represent a corner and the gate holds the straight. Result: a safe stop at the corner, not a turn.
