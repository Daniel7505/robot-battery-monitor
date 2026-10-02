# Row-fit lane vision (`RBM_LANE_MODE=rowfit`)

An opt-in lane keep that **measures the lane in metres** from the two
shoulder cameras and steers from that, instead of counting pixels against
fixed 33/33 targets. The default `gap` mode is unchanged.

Code: `src/lane_vision.py` (camera model + detection + fit),
`src/rowfit_control.py` (steer + speed),
`webots/controllers/butlerbot_controller/controller_rowfit.py` (controller glue).
Offline closed-loop check: `python scripts/rowfit_sim.py`.

## How it works, in plain words

1. **Where each pixel lands on the floor.** The cameras are fixed on the
   robot, so with a flat floor every pixel matches one floor spot. The
   numbers (position, tilt, field of view, 128×128) are copied from
   `butlerbot.wbt`. A test reads the `.wbt` and fails if they stop matching,
   so if you move a camera, update `NADIR_LEFT_POSE` / `NADIR_RIGHT_POSE`.
   Webots R2025a cameras look along their own +X, with +Y to the left of the
   image and +Z to the top. The 1.1 rad tilt points them 63° down.

   | nadir camera (each) | value |
   |---|---|
   | bottom image row lands at | 0.55 m **behind** the axle (it looks slightly backwards) |
   | middle row | 0.27 m ahead |
   | top row | 1.96 m ahead |
   | floor per pixel, bottom row | 1.15 cm sideways × 0.97 cm along |
   | floor per pixel, top row | 2.37 cm sideways × 4.0 cm along |
   | 6 cm stripe on the image | ~5.2 px wide at the bottom, ~2.5 px at the top |

   The last row is not something we calibrated. It falls out of the model,
   and it matches what the earlier review measured on real frames.

2. **Find the paint.** The yellow test is the same one the gap mode uses
   (`yellow_score ≥ 0.22`). The scan runs along every image row *and* every
   column. For each yellow run, its two edges are mapped to the floor and the
   midpoint is taken. That midpoint sits on the stripe's centre line at any
   angle, so a line that turns sideways (a 90° bend) still gives good points.
   There is no "wider than 24 px" rejection. Runs cut off by the image border
   are skipped.
3. **Which line is which.** At start-up, left-camera points on the robot's
   left count as the left line, and the same for the right. After that, each
   frame uses the last frame's lane, shifted by how far the wheels rolled and
   how much the IMU says the robot turned.
4. **Fit the lane.** Both lines are fitted together as one centre line (a
   circle arc, or straight when curvature is 0) with paint at ±half a lane
   width. Points that don't fit are down-weighted and then dropped. Two
   fits are made:
   * near (points up to 0.9 m ahead) gives **offset** and **heading** at the axle,
   * far (points from 0.25 m ahead) gives **curvature ahead** and the steering goal point,

   so a straight that runs into a bend isn't averaged into one wrong arc.
   **Lane width is measured** whenever both lines are seen. When one line
   disappears, the last measured width places the centre. Before any width
   has been measured it falls back to 1.30 m at reduced confidence.
5. **6 cm is only a check.** Each point's width on the floor (corrected for
   the line's angle) is compared to 6 cm. The share that pass is
   `width_ok_frac`, and it lowers the confidence. It is never used as the
   scale. The camera model alone sets distances.
6. **Steer (pure pursuit).** Pick a goal point on the fitted centre line at
   look-ahead distance `0.45 m + 1.2 s × speed` (0.55 to 1.6 m, never beyond
   what the cameras can see). Then steer along the circle that reaches that
   point. If the robot is on a curve, this gives exactly the curve's own
   curvature, so offset, heading and curvature feed-forward are all one
   number. There is also a small damping term on how fast the offset
   changes, measured between camera frames.
7. **Only on new frames.** The cameras update every 320 ms, but physics runs
   every 8 ms. The command (curvature and target speed) is recomputed only
   when a new frame arrives and held in between. This fixes the gap mode's
   derivative spike for this path. Every 8 ms only the speed ramp and steer
   slew run. Wheel speeds come from the same `wheels_from_steer` (same
   0.35×cruise floor and 8 rad/s cap).
8. **Speed: slow in, speed up out.** Target speed is `sqrt(0.12 / |curvature|)`,
   kept between 0.20 and 0.44 m/s. It uses the sharpest of the near, far and
   steering curvatures, and it is also lowered when confidence is low.
   Braking is quick (0.8 m/s²) and speeding up is gentle (0.25 m/s²). On the
   S (tightest radius 1.3 m) that is about 0.40 m/s in the bends. In a 1 m
   radius 90° turn it is about 0.35 m/s.
9. **Lost lane.** One or two frames with no lane: hold the steering at
   0.20 m/s. Three frames in a row (about 1 s): brake (ABS) and wait. It
   starts again when the lane comes back.

The track shape (`s_track.py`) is **not** used for steering. It's only used
for tests, the offline sim and the GPS finish stop that was already there.

## Try it on Windows

Option A, one session (cmd):

```bat
cd C:\path\to\robot_battery_monitor_project
set RBM_LANE_MODE=rowfit
"C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe" webots\worlds\butlerbot.wbt
```

(PowerShell: `$env:RBM_LANE_MODE="rowfit"` then `.\scripts\launch_webots_twin.ps1`.)
Webots must be **started from that window**. A Webots that was already open
doesn't see the new variable.

Option B, sticky: put this line in the repo `.env` file (the controller
reads `.env` the same way it reads `RBM_API_TOKEN`):

```
RBM_LANE_MODE=rowfit
```

Then just reload the world. A value set with `set` always wins over `.env`.

How to tell it's on: the Webots console prints
`LANE MODE rowfit (run_id 2026…) — lane_vision fit, log → …lane-vision.csv`
and `ROWFIT STEER ON`. About once a second it also prints a `Rowfit … off=… hd=… k=… conf=…`
line. The shoulder HUDs show green (left-line) and magenta (right-line)
dots plus offset, heading, curvature and confidence.

**Turn it off:** `set RBM_LANE_MODE=gap` (or close that cmd window and start
Webots normally), and remove or change the `.env` line to `RBM_LANE_MODE=gap`.
Then reload. Unset means `gap`.

## Logs and comparing runs

* `lane-vision.csv` (rowfit only) sits next to `steer-actions.csv`. It gets
  one row per new camera frame (`new_frame=1`) plus one per telemetry
  publish (`new_frame=0`):
  `unix_s,x_m,y_m,mode,offset_m,heading_rad,curvature,lookahead_m,confidence,nL_pts,nR_pts,steer,target_speed,new_frame,run_id`.
  `run_id` is the controller start time, so every reload is its own run.
* `steer-actions.csv` keeps logging in both modes (`src=rowfit` in rowfit).
* Folder: `~/OneDrive/Desktop/Grok Workspace` as before, or set `RBM_LOG_DIR`
  (env or `.env`). `drift_report.py` honours the `RBM_LOG_DIR` env var too.

```bat
python scripts\drift_report.py --track s --last 5
python scripts\drift_report.py "%USERPROFILE%\OneDrive\Desktop\Grok Workspace\lane-vision.csv" --track s
```

`--track s` scores sideways error against the S centre line. Without it the
report keeps its old meaning (|y_m| from world y=0, which is only fair on a
straight). For `lane-vision.csv`, runs are split by `run_id`.

## Other switches

* `RBM_NADIR_GUARD_PER_FRAME=1` (gap mode only, opt-in): the "both gaps moved the
  same way" guard counts per camera frame. Without it, the guard is called
  every 8 ms tick on a frame that only changes every 320 ms, so its counter
  resets and it can never fire. Off by default because turning it on can
  add new stops to the gap mode.

## Not checked without Webots

* Real rendered colours: the parquet floor under the grey `colorOverride`,
  shadows, and lighting on the yellow. Synthetic tests use flat colours with
  noise. If the HUD shows dots on the floor, raise `YELLOW_THRESH`.
* The robot's own body or wheel showing in the bottom rows. It's dark, so it
  shouldn't read as yellow, but this hasn't been looked at.
* Robot pitch or bounce. The model assumes a level robot on a flat floor.
* Webots' exact pixel-centre convention (half-pixel). A test against the
  observed 5.2/2.5 px stripe widths agrees.
* Closed-loop tuning. `scripts/rowfit_sim.py` (ideal kinematics) holds the S
  and a 1 m radius 90° turn within ~8 cm. Real tyres and latency will differ.
* Sharp corners (radius ≈ 0) are fitted as tight arcs and seen late. A
  rounded 90° turn (radius ≥ 0.8 m) is what this is tested for.
