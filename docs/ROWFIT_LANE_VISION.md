# Row-fit lane vision (default lane mode)

The **default** lane keep (since 2026-10-01): it **measures the lane in
metres** from the two shoulder cameras and steers from that, instead of
counting pixels against fixed 33/33 targets. `RBM_LANE_MODE` unset (or
`rowfit`) runs it; `RBM_LANE_MODE=gap` still runs the old pixel-fan mode with
unchanged code.

Code: `src/lane_vision.py` (camera model + detection + fit),
`src/rowfit_control.py` (steer + speed),
`webots/controllers/butlerbot_controller/controller_rowfit.py` (controller glue).
Offline closed-loop check: `python scripts/rowfit_sim.py`.
Camera tilt calculator: `python scripts/aim_camera.py`.

## How it works, in plain words

1. **Where each pixel lands on the floor.** The cameras are fixed on the
   robot, so with a flat floor every pixel matches one floor spot. At
   start-up the controller **reads each camera's pose live from Webots**
   (Supervisor: the camera node's translation/rotation relative to the Robot
   node, composed through any parent Transforms) and its width, height and
   fov from the device. The node is found with `getFromDevice(camera._tag)`.
   In R2025a that call needs the integer device tag; passing the Camera
   object raises `ctypes.ArgumentError`, which is why Dan's 2026-10-01 run fell
   back to the wbt. If that fails too, it tries `getFromDef('NADIR_CAM_L'/'_R')`
   on the same Supervisor the controller created. If that fails it parses the loaded `.wbt`, then falls
   back to the `NADIR_LEFT_POSE` / `NADIR_RIGHT_POSE` constants. The console
   says which one it used (`CAM POSE nadir_left from supervisor: …`), and a
   failed live read prints the real exception text in brackets. The
   constants are still kept in sync with `butlerbot.wbt` by a test, because
   tests, the offline sim and the fallback use them.
   Webots R2025a cameras look along their own +X, with +Y to the left of the
   image and +Z to the top. `rotation 0 1 0 θ` pitches the view down by θ.

   **Mount (2026-10-01):** each camera sits in its shoulder box
   (`NADIR_BOX_L/R`, 0.05 m cube centred at 0.01042 ±0.41808 0.72919), lens on
   the box's front-bottom edge: `translation 0.03542 ±0.41808 0.70419`,
   `rotation 0 1 0 0.9411` (53.9° down) for both (a true mirror). The pitch
   comes from `scripts/aim_camera.py`: top row on the floor 1.96 m ahead of
   the axle, the same reach as the old z 1.31 m mount. With the lens on the
   box face, every box corner is at depth ≤ 0 < `near` 0.03, so the camera
   can't render its own box. (At the box centre the far corner would be at
   0.035 m, past `near`.) Predicted numbers, not yet seen in Webots:

   | nadir camera (each) | box mount (now) | old z 1.31 m mount |
   |---|---|---|
   | bottom image row lands at | 0.06 m ahead of the axle | 0.55 m behind |
   | middle row | 0.54 m ahead | 0.27 m ahead |
   | top row | 1.96 m ahead | 1.96 m ahead |
   | floor per pixel, bottom row | 0.61 cm sideways × 0.51 cm along | 1.15 × 0.97 cm |
   | floor per pixel, top row | 1.81 cm sideways × 4.36 cm along | 2.37 × 4.0 cm |
   | 6 cm stripe on the image | ~9.8 px at the bottom, ~3.3 px at the top | ~5.2 / ~2.5 px |

   Left camera, robot centred in a 1.30 m lane: the left line runs from about
   (col 26, row 127) at the bottom to (col 51, row 0) at the top, passing
   (col 33, row 89) at 0.3 m and (col 44, row 33) at 1.0 m ahead. The right
   line only clips the top-right corner (x ≥ 1.74 m). The right camera is the
   mirror image (col → 127 − col). The robot's own front caster, drive wheel
   and a sliver of torso should fill about 2% of the image, in the corner
   toward the robot (left camera: cols ~100–127, rows ~109–127), well away
   from the paint.

   The axis convention matches Dan's 2026-09-16 Webots drills, which are now
   tests:
   * identity rotation shows the horizon,
   * `0 1 0 1.5708` looks straight down,
   * `0 1 0 0.8958` with FOV 1.35 at 1.3 m puts nadir on the last row and far ground at ~5.8 m.

   `fieldOfView` is the horizontal angle (same as vertical for these square
   128×128 images). The Robot node's rotation is identity and the cameras are
   its direct children, so the camera pose is the robot-frame pose.

   **Derived from the camera, not copied:** the pursuit look-ahead window
   (`lookahead_bounds_from_coverage`: max = shortest top-row reach − 0.36 m,
   min = 0.55 m or bottom row + 0.25 m) and the fit windows (`fit_windows`)
   are computed from the live models at start-up. For the current mount they
   come out at 0.55–1.6 m (unchanged) and near fit x ≤ 0.6 m. The near fit
   used to run to 0.9 m; that only worked while the old camera also saw
   0.55 m of floor behind the axle. Without it, a bend starting 0.5 m ahead
   tilted the axle heading by ~19° in the synthetic test, so it is now 0.6 m.

2. **Find the paint.** Yellow means `min(R, G) − B ≥ 0.22` (`YELLOW_MODE="rg"`).
   Gap mode uses `(R + G)/2 − B`, which gives the same number for real yellow
   (R ≈ G). But the red finish bar (0.9, 0.15, 0.12) scores 0.41 on it, so
   rowfit fitted the red bar as lane. Under `min(R, G)` red and green bars
   score below 0. The scan runs along every image row *and* every
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
   * near (points up to 0.6 m ahead) gives **offset** and **heading** at the axle,
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
10. **Plausibility gate (no track knowledge).** A new fit is rejected when:
    * |heading| > 45°,
    * |curvature| > 2 1/m,
    * heading jumps more than 0.15 rad + curvature × distance rolled since the
      last good frame (the last lane is first moved by odometry and IMU yaw),
    * offset jumps more than 6 cm + 0.6 × distance rolled,
    * a line seen in the last good frame is suddenly empty and confidence is
      below 0.45.

    A rejected fit does not steer. The robot follows the last good lane
    (carried forward by odometry and yaw) at ≤ 0.30 m/s, and the frame counts
    as "no lane", so the 3-frame brake still applies. One exception: a fit that
    only sees < 0.5 m of paint ahead (end of the lane) is held for up to 6
    frames before it counts as lost. Dan's 2026-10-01 finish frame
    (`nL=0 nR=218 hd=-1.576 off=-0.517`) is a regression test. The console
    prints `Rowfit REJECTED fit (<reason>) — holding last lane …` and the HUD
    shows `rowfit: HELD (fit rejected)`.

The track shape (`s_track.py`) is **not** used for steering. It's only used
for tests, the offline sim and the GPS finish stop that was already there.

## Sharp corners: "the line widens, then disappears"

**The idea is Dan's.** Watching the shoulder HUDs at a sharp corner, he
noticed that in the look-ahead rows the yellow run in one row suddenly gets
much wider than a stripe, because the line now runs *across* the row. That
run is smeared toward one side, and the rows beyond it have no yellow. On the
right shoulder cam at a left turn, the outer (right) line widens to the left
and then disappears. The code below uses exactly that, with no track
knowledge: only the camera model and the yellow mask.

### Detector (`src/lane_vision.py`: `corner_cues`, `fuse_corner_cues`, `LaneTracker`)

Per camera, each line is followed up the image (near to far). The tracker
takes the run nearest the column extrapolated from the line's last rows,
including runs that touch the image edge.

* **Stripe width.** The line's own stripe is the median ground width
  (metres, via `CameraModel.pixel_to_ground`) of its nearest normal rows.
  Nominal is 6 cm.
* **Crossbar.** The first row whose run is at least
  `CORNER_WIDE_FACTOR` = 3× that width and at least `CORNER_WIDE_MIN_M` = 0.15 m.
  It must be reached:
  * within `CORNER_RAMP_MAX_M` = 0.15 m of a normal-width row, and
  * with at most `CORNER_MAX_RAMP_ROWS` = 1 in-between row.

  A rounded bend widens over many rows instead, so it is not flagged.
  corner_mix's r = 0.5 m outer line ramps over 4+ rows.
* **Direction.** The run's smear past the line: toward robot +y means a left
  turn, −y a right turn. The smear must be at least 0.12 m.
* **Role.**
  * *Outer*: the smear crosses the lane, e.g. the right line smearing left.
    This is Dan's case.
  * *Inner*: the inner line bends away. It gives the same direction, but it is
    only supporting evidence. A rounded corner still has a mitred inner line,
    so an inner cue alone never triggers.
* **"Then disappears".** The band where the line would have continued
  (±0.10 m on the ground) is checked in the farther rows: at most 2 yellow px.
  `empty_beyond` is None when the crossbar is in the top rows and there is
  nothing beyond it to look at.
* **Fusion and confidence.** An outer cue is required.
  * Outer cue: 0.50.
  * Empty beyond: +0.25. Paint beyond: −0.20.
  * An inner cue of the same direction, nearer: +0.20.
  * Both cameras see the crossbar: +0.05.
  * Two outer cues in opposite directions (a bar across the lane, or the end
    of the paint) give **no corner**.
* **Persistence.** `seen` needs two consecutive frames with the same
  direction, a distance consistent with odometry (±0.25 m), and conf ≥ 0.6.
* **Distance.** The ground x of the crossbar's stripe centre at the line,
  ahead of the axle.
* **Fit clip.** While a cue is present, fit points from 0.12 m before the
  crossbar onward are dropped. So are points past an inner cue on its side.
  The turning paint no longer bends the lane fit or trips the plausibility
  gate. This also fixes the baseline's `heading jump` rejections before the
  corner.

### Controller (`src/rowfit_control.py`)

`LANE → CORNER_APPROACH → PIVOT → REACQUIRE → LANE`. The controller gets the
IMU yaw every tick (`RowfitRuntime.command(dt, yaw=…)`).

* **CORNER_APPROACH** (on a persistent cue, conf ≥ 0.6).
  * Pivot point = cue distance − lane width / 2. That is the corner's
    centre-line vertex, so turning 90° there leaves the axle on the new leg's
    centre line.
  * The lane width is the tracker's measured one, nothing hardcoded.
    corner90_wide (1.6 m) pivots 0.8 m short of its outer line.
  * The remaining distance is re-measured on every frame that still shows the
    cue, and integrated from the speed between frames.
  * Speed = min(0.25 m/s, √(2·0.3·remaining)), down to a 4 cm/s creep, so it
    arrives nearly stopped. There is no ABS brake before the pivot.
  * It steers on the clipped near-lane fit. Missing or held frames do not
    count toward the lost-lane brake here, because the lane is *supposed*
    to end.
  * It records the approach lane's direction in IMU yaw
    (yaw − fitted heading, smoothed).
* **PIVOT** (remaining ≤ 1.5 cm).
  * Turns in place with opposite wheel speeds, closed loop on IMU yaw.
  * Target = approach-lane yaw ± 90° in the cue's direction. The rate is
    2·error, capped at 0.9 rad/s, with an acceleration limit of 1.5 rad/s²
    and a stop-in-time limit. A 0.12 rad/s floor stops it stalling short.
  * **Done** when |error| ≤ 1° at rate ≤ 0.05 rad/s for 4 ticks. Done early
    if vision sees a two-sided lane aligned within 1.5° while |error| < 12°.
    Timeout 8 s → brake.
  * At the end the tracker gets `begin_reacquire()`. It forgets the old lane,
    keeps the measured width, and relaxes the gate for 6 frames (only the
    ±45° check stays). So the expected 90° heading change is not rejected as
    a "jump".
* **REACQUIRE.**
  * ≤ 0.20 m/s on normal rowfit steering.
  * Back to LANE after 3 valid frames in a row with |heading| < 20°.
  * The fallback is unchanged: no lane for 3 frames means **brake**
    (`REACQUIRE failed …`).
  * No new corner is accepted for the first 0.5 m of the new leg.
* **No IMU yaw** (`yaw=None`): it still approaches, then brakes at the pivot
  point (`CORNER reached but no IMU yaw — braking`).

### What you see in Webots

Console, in this order (the sim's corner90 lines):

```
CORNER seen left 1.66 m conf 0.95 (nadir_right outer + nadir_left inner) — approach, pivot point in 1.01 m (lane 1.30 m)
PIVOT start left yaw -0.0deg -> target +90.1deg (lane +0.1deg, remaining +0.015 m)
PIVOT done yaw +89.4deg (target +90.1, err +0.7deg, 3.0 s, yaw on target) — REACQUIRE
REACQUIRE ok after 3 frames — off -7cm hd -0.1deg conf 0.18, back to LANE
```

* **HUD.**
  * The first text line on each shoulder HUD is the state:
    * `LANE`;
    * `LANE cue L 1.66m c1.0` (a cue that is not yet persistent);
    * `CORNER L 1.12m pv+0.47` (pv = distance to the pivot point);
    * `PIVOT L err +34d`;
    * `REACQUIRE 1/3`.
  * The crossbar row is drawn as a horizontal bar: orange for the outer line,
    yellow for the inner.
* **`lane-vision.csv`.** New columns **after** `run_id`:
  `state,corner_dir,corner_m,corner_conf,yaw_deg,yaw_target_deg`. Rows
  appended to an older file still line up for the old columns. Rename the old
  file to get the new header.
* **`drift_report.py`.** Prints a line like
  `run 3 corner states: LANE > CORNER_APPROACH > PIVOT > REACQUIRE > LANE (pivots 1)`.

### Offline sim

`python scripts/rowfit_sim.py corner90 corner_mix corner90_right corner90_wide s widen turn90 --table`

* The kinematics were already diff-drive, so a pivot is just opposite wheel
  speeds. The controller gets the simulated IMU yaw.
* `--turn-eff 0.8` models tyre scrub (80 % of the commanded yaw rate).
* `--imu-noise DEG` and `--imu-drift DEG_PER_S` corrupt the yaw.
* `--no-corners` withholds the yaw.
* Test tracks: `tracks/corner90_right.json` (mirror) and
  `tracks/corner90_wide.json` (1.60 m lane), both sim-only, no world.

Results with the box-mount cameras from the `.wbt` (worst / mean = distance
off the centre line):

| course | finish | time | worst / mean | min v | pivot err (IMU / true) | pivot time |
|---|---|---|---|---|---|---|
| corner90 | yes | 24.5 s | 1.5 / 0.2 cm | 0 (pivot) | +0.7° / +0.6° | 3.0 s |
| corner90_right | yes | 24.5 s | 1.5 / 0.2 cm | 0 (pivot) | −0.7° / −0.6° | 3.0 s |
| corner90_wide (1.6 m) | yes | 25.0 s | 3.4 / 0.8 cm | 0 (pivot) | +0.7° / +1.0° | 3.0 s |
| corner_mix (rounded L, sharp R) | yes | 34.9 s | 14.3 / 1.9 cm (at the rounded arc, as before) | 0 (pivot) | −1.1° / −1.1° | 2.9 s |
| S | yes | 41.6 s | 7.8 / 2.8 cm | 0.20 | no pivot | – |
| widen | yes | 29.0 s | 1.7 / 0.5 cm | 0.20 | no pivot | – |
| turn90 (1 m radius) | yes | 15.7 s | 7.4 / 3.0 cm | 0.20 | no pivot | – |

* **Robustness.** corner90, corner90_right, corner90_wide and corner_mix all
  still finish with each of: scrub 0.8, IMU noise 0.3° plus drift 0.5°/s,
  one frame of extra latency, and a start offset of +15 cm and 5° or −20 cm.
  The worst pivot error was 3.3°; REACQUIRE corrects it.
* **No false corners.**
  * S, widen and turn90: no corner cue at all.
  * Start and finish bars: rg yellow ignores red and green.
  * The finish bar scored *as* yellow (gap-mode yellow): both lines smear
    inward in opposite directions, so no corner.
  * corner_mix's rounded left gives only an inner cue, so no corner.

## Run it

Nothing to set: open `webots/worlds/butlerbot.wbt` and it runs rowfit.
(If your repo `.env` still has `RBM_LANE_MODE=gap` from earlier, remove it.)

How to tell it's on: the Webots console prints
`LANE MODE rowfit (run_id 2026…) — lane_vision fit, log → …lane-vision.csv`,
then one line per camera, e.g.
`CAM POSE nadir_left from supervisor: t=(0.0354, 0.4181, 0.7042) rot=(0 1 0 0.9411) pitch=53.9deg yaw=+0.0deg 128x128 fov=1.200 | floor rows bottom 0.06 m, mid 0.54 m, top 1.96 m`
(right: `t=(0.0354, -0.4181, 0.7042)`), then
`ROWFIT look-ahead 0.55–1.60 m, near fit x<=0.60 m, far fit x>=0.25 m (from camera coverage)`
and `ROWFIT STEER ON`. `from wbt …` or `from constants` instead of
`from supervisor` means the live read failed (a `WARNING rowfit camera model`
line says so). About once a second it also prints a `Rowfit … off=… hd=… k=… conf=…`
line. The shoulder HUDs show green (left-line) and magenta (right-line)
dots plus offset, heading, curvature and confidence.

**Old gap mode:** `set RBM_LANE_MODE=gap` in a cmd window and start Webots
from it (PowerShell: `$env:RBM_LANE_MODE="gap"`), or put
`RBM_LANE_MODE=gap` in the repo `.env` and reload. A value set with `set`
always wins over `.env`. Unset, empty or unknown values mean `rowfit`.
Gap-mode code is unchanged, but its 33/33 px targets and "tape → drive
wheel" ruler were tuned on the old z 1.31 m view, where the drive wheel was
in the picture. In the box view the wheel only shows in the bottom corner
next to the robot, so gap mode needs re-tuning before it can be trusted again.

## Logs and comparing runs

* `lane-vision.csv` (rowfit only) sits next to `steer-actions.csv`. It gets
  one row per new camera frame (`new_frame=1`) plus one per telemetry
  publish (`new_frame=0`):
  `unix_s,x_m,y_m,mode,offset_m,heading_rad,curvature,lookahead_m,confidence,nL_pts,nR_pts,steer,target_speed,new_frame,run_id,state,corner_dir,corner_m,corner_conf,yaw_deg,yaw_target_deg`.
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
* The robot's own body in the view. By box-model geometry the front caster,
  drive wheel and a torso sliver cover ~2% of each image (corner toward the
  robot). Arm swing was not modelled. All dark, so it shouldn't read as yellow,
  but check the HUD.
* The box mount itself (`aim_camera.py --box-center …` says the box is behind
  the near plane, but confirm no dark edge in the top/left of the HUD).
* Robot pitch or bounce. The model assumes a level robot on a flat floor.
* Webots' exact pixel-centre convention (half-pixel). On the old mount the
  model matched the observed 5.2/2.5 px stripe widths; the new 9.8/3.3 px are
  predictions.
* Closed-loop tuning. `scripts/rowfit_sim.py` (ideal kinematics, camera
  poses read from the `.wbt`) holds the S within 7.8 cm and a 1 m radius 90°
  turn within 7.4 cm with the box mount (old mount: 7.7 / 8.0 cm). Real tyres
  and latency will differ.
* Sharp corners are only verified in the offline sim. Real paint edges,
  motion blur during the pivot, tyre scrub on the parquet and the IMU in
  Webots may change the numbers. The pivot is closed loop on the IMU, so scrub
  only makes it slower, but the cue thresholds (3× stripe, 0.15 m ramp) were
  tuned on synthetic images.
