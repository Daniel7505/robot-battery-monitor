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

## Intersections: crossings, T junctions, branches and wide plazas

The north star is the same as for corners: unknown, customer-drawn tracks and
no prior track knowledge. Everything below comes from the two shoulder
cameras' yellow mask, odometry and the IMU. The large-intersection /
limited look-ahead edge case is **Dan's**: a crossing can be wider than the
1.96 m camera reach, so when the robot must decide it may not yet see whether
the lane carries on.

### Junction classifier (`src/junction_vision.py`)

It reuses the corner detector's line tracer (`lane_vision.trace_lines`,
which `corner_cues` now shares unchanged). Each lane line is followed up the
image row by row. Per side (left line from `nadir_left`, right line from
`nadir_right`) the line's event is one of:

| event | meaning in the picture |
|---|---|
| `BEND_OUT` | the run suddenly widens and smears AWAY from the lane centre: a road joins on this side (an opening) |
| `BEND_IN` | it smears TOWARDS the centre: the outer line of a corner (existing corner path) |
| `END` | the line just stops (a gap) |
| `CONT` | still running, straight, at the top row |
| `CURVE` / `EDGE` | bends gradually / leaves the picture sideways (an ordinary curve) |

Plus two far-side cues: **free bars** (wide yellow runs that no lane line
claims = lines running across the view) and **resumed lines** (new line
starts beyond the gap, at the old line's lateral position).

| kind | rule | openings |
|---|---|---|
| `corner_L/R` | any `BEND_IN` (left to the corner path) | – |
| `t_end` | both sides open at the same distance, a bar crosses the lane centre beyond | L − R |
| `plus` | both sides open, the lane resumes or far pieces with an empty centre | L S R |
| `plus` (plaza) | both sides open, nothing visible beyond yet | L S? R |
| `side_L/R` | one side opens, the other line keeps going straight at least one crossing width + 0.25 m past it | L S − / − S R |
| `gap_straight` | both lines `END`, the lane resumes | S |
| `lane_lost` | both lines `END`, nothing beyond (also: the end of the paint) | S? |
| `dead_end` | both `END`, a bar blocks | none |
| `opening_L/R` | one side opens, not yet known if branch or corner | (not acted on; slows down) |

* Distances: `near_m` = the near crossing line; `center_m` = near +
  half the crossing width (far − near when the far side is seen, else the
  own lane width). The robot pivots at `center_m`.
* Persistence: 2 frames, distance consistent within 0.3 m after odometry.
  `plus`/`t_end`/`gap_straight`/`lane_lost` count as one family (they refine
  as the far side comes into view), as do `opening_X` and `side_X`.
* Fit clipping: at a crossing the lane fit only uses points before the near
  line; at a side branch only the opening side is clipped. Near a junction
  (`LaneTracker.bar_filter_frames`, set by the controller) points on free bars
  are dropped, so the far side's cross lines do not bend the new fit.
* `est.view` (`yellow_view`): nearest yellow ahead / left / right / a bar
  across, used while crossing blind and looking around.

### Behaviour (`src/rowfit_control.py`, `src/route_policy.py`)

```
LANE -> JUNCTION_APPROACH -(L/R)-> CORNER_APPROACH -> PIVOT -> REACQUIRE -> LANE
                          -(S at a side branch)-> LANE (single-line lane past it)
                          -(S at a crossing / gap)-> GAP_CROSS -> REACQUIRE -> LANE
                                     GAP_CROSS -(nothing by max blind)-> LOOK_AROUND -> STOPPED
```

* **Route policy.** `RBM_ROUTE=S,L,R` = one choice per junction in order;
  after the list (or with none) the default is *straight if possible, else
  left, else right*. "Possible" includes *unknown* (`S?`): a wide plaza is
  crossed, not avoided. A listed choice the junction does not offer falls back
  to the default and the log says so (`route #1 wanted S — not offered,
  default L`). Corners and one-way junctions (a gap, the end of the paint)
  never consume a route entry.
* **JUNCTION_APPROACH.** Speed cap 0.25 m/s from the first persistent sighting
  (and 0.6 m of 0.25 m/s whenever one side opens while the other runs straight,
  so a side branch is never decided at full speed). Decides as soon as the
  openings are known, at the latest 0.35 m before the near line
  (odometry-dead-reckoned if vision loses it).
* **Turns** reuse CORNER_APPROACH / PIVOT / REACQUIRE, pivot point = the
  junction centre. While lining up:
  * Fresh lane fits need **conf ≥ 0.55** (`APPROACH_STEER_MIN_CONF` /
    `APPROACH_YAW_MIN_CONF`) to steer or update `lane_yaw`. Held estimates
    (odometry-propagated last-good lane) are always usable at `MIN_CONF`
    (0.15). Low-conf fresh fits near the cross-aisle paint are ignored — they
    used to pull the pivot base by several degrees (warehouse aisle-8 miss).
  * The remaining distance to the junction centre is refreshed from vision but
    **never pushed back out** (odometry countdown wins over a one-frame-late
    centre). That used to delay the pivot by ~v·frame_dt ≈ 8 cm at 0.25 m/s.
* **GAP_CROSS.** Lane pursuit up to the near line, then blind: 0.15 m/s,
  heading held on the IMU (the lane direction measured on the approach),
  distance counted by odometry. Every frame without a lane the tracker forgets
  its prior, so the next lane is taken fresh. It ends when, for 2 frames, a
  two-line lane is fitted with |heading| < 20°, |offset| < 0.45 m, width within
  ±25 % of the approach lane and both lines starting within 1.0 m. A bar across
  the centre ahead (the far side of a wide T) turns it into a turn (route L/R,
  default left). Side yellow is logged as openings.
* **Max blind distance** `RBM_MAX_BLIND_M` (default 4.0 m). Reached: stop,
  LOOK_AROUND (slow pivot to +90°, −90°, back to 0°, 0.5 rad/s; disable with
  `RBM_LOOK_AROUND=0`), report what yellow was seen, then STOPPED (brakes;
  drive on by hand). Without IMU yaw it just stops.

### What you see

Console (sim, plus with `RBM_ROUTE=S`):

```
ROUTE RBM_ROUTE=S -> S then default (straight if possible, else left); max blind 4.00 m (RBM_MAX_BLIND_M), look-around on
JUNCTION #1 seen plus near 1.77 m centre 2.42 m openings L S? R conf 0.70 — slowing to 0.25 m/s
JUNCTION #1 now plus openings L S R near 0.63 m centre 1.29 m
ROUTE junction #1 plus [L S R] -> S (route #1 S)
GAP_CROSS start: near line 0.63 m, holding heading -0.0deg, creep 0.15 m/s, max blind 4.00 m
GAP sees yellow left at 1.07 m (after 0.19 m blind) — side line = opening
GAP reacquire after 0.34 m blind — lane ahead off +0cm hd -0.0deg w 1.30 m — REACQUIRE
REACQUIRE ok after 3 frames — off +0cm hd -0.0deg conf 0.99, back to LANE
JUNCTION #2 seen lane_lost near 1.48 m centre 2.13 m openings - S? - conf 0.70 — slowing to 0.25 m/s
TRACK plus DONE via exit east at x=8.00 y=0.00 m — GPS finish, not a red camera. ...
```

The `lane_lost` sighting is the end of the paint 0.5 m past the exit: the
robot slows for it and the GPS referee stops the run at the exit first.
A turn prints `ROUTE junction #1 plus [L S R] -> L (route #1 L)` and
`TURN L at the junction centre, pivot point in 1.29 m`, then the usual
`PIVOT start/done` and `REACQUIRE ok`. Giving up prints
`GAP no lane after 4.00 m blind ... — LOOK_AROUND` and
`LOOK_AROUND done — left (+90): ..., right (-90): ..., ahead: .... STOPPED`.

HUD first line: `LANE plus 1.20m LS?R` (sighting), `JCT plus 1.29m LSR`,
`TURN L L pv+0.80`, `GAP S blind 1.20/4.0m`, `LOOK 2/3`, `STOPPED no lane`.

`lane-vision.csv` gets five more columns after the corner ones:
`junction_type,junction_m,openings,route_choice,blind_m` (rename an old
file to get the new header).

### Offline results (`python scripts/intersection_matrix.py`)

Box-mount cameras from the `.wbt`, 0.32 s frames. Success = finished through
the exit the route asks for. Worst = farthest off the nearest road's centre
line.

| course | route → exit | result | time | worst | pivot err (IMU / true) |
|---|---|---|---|---|---|
| plus | S → east | OK | 27.0 s | 0.0 cm | – (blind 0.34 m) |
| plus | L → north | OK | 30.0 s | 3.8 cm | +0.7° / +0.7° |
| plus | R → south | OK | 30.0 s | 3.8 cm | −0.7° / −0.7° |
| plus | default → east | OK | 27.0 s | 0.0 cm | – |
| t_end | L → north | OK | 29.9 s | 2.4 cm | +0.7° / +0.7° |
| t_end | R → south | OK | 29.9 s | 2.4 cm | −0.7° / −0.7° |
| t_end | S (not offered) → north | OK | 29.9 s | 2.4 cm | +0.7° / +0.7° |
| t_end | default → north | OK | 29.9 s | 2.4 cm | +0.7° / +0.7° |
| t_left | S / default → east | OK | 24.5 s | 0.1 cm | – |
| t_left | L → north | OK | 28.1 s | 1.3 cm | +1.0° / +0.8° |
| t_right | S / default → east | OK | 24.5 s | 0.1 cm | – |
| t_right | R → south | OK | 28.1 s | 1.3 cm | −1.0° / −0.8° |

Large gaps (straight; the line-to-line gap across the crossing):

| course | gap | result | time | worst | blind distance | at the decision |
|---|---|---|---|---|---|---|
| gap15 | 1.5 m | OK | 31.3 s | 0.0 cm | 0.51 m | far side seen (L S R) |
| gap25 | 2.5 m | OK | 38.0 s | 0.0 cm | 1.52 m | far side beyond reach (L S? R) |
| gap35 | 3.5 m | OK | 44.6 s | 0.0 cm | 2.53 m | L S? R |
| gap45 | 4.5 m | OK | 51.3 s | 0.0 cm | 3.53 m | L S? R |

Blind distance ≈ gap − 1.0 m (the new lines must start within 1 m), so with
the default 4.0 m any gap up to ~5 m is crossed. gap45 with
`RBM_MAX_BLIND_M=2.0` stops after 2.00 m, looks around (nothing in view)
and stays STOPPED, as intended.

Robustness (plus S/L, t_end R, t_left L, t_right S, gap25, gap45):

| condition | result |
|---|---|
| tyre scrub (turn_eff 0.8) | 7/7 OK, worst 3.8 cm, pivot err ≤ 1.1° |
| start offset +15 cm, or −15 cm and +5° yaw | 14/14 OK (worst = the start offset) |
| IMU drift +0.25°/s | 7/7 OK; gap45 ends 24 cm off after 24 s blind |
| IMU drift ±0.5°/s (and 0.3° noise) | 6/7 OK; **gap45 fails**: 12° of heading drift over 3.5 m blind puts the robot 0.58 m off, the far lane does not pass the reacquire check, it gives up at 4.0 m and stops safely. gap25 OK (14 cm). |
| one frame extra camera latency | 11/11 OK; junction turns ~5–8 cm (remaining no longer delayed by a late centre); was 8–11 cm |

So the gap crossed reliably is 4.5 m with an IMU drifting ≤ 0.25°/s and
2.5 m at 0.5°/s. Webots' InertialUnit has no drift; a real gyro's does matter
for wide plazas.

Old courses (no regressions in accuracy; slightly slower, see below):

| course | worst (before → now) | time (before → now) | pivot err |
|---|---|---|---|
| corner90 | 1.5 → 1.7 cm | 24.5 → 27.3 s | +0.7° |
| corner_mix | 14.3 → 14.3 cm | 34.9 → 37.3 s | −1.1° |
| corner90_right | 1.5 → 1.7 cm | 24.5 → 27.3 s | −0.7° |
| corner90_wide | 3.4 → 1.0 cm | 25.0 → 27.4 s | +0.7° |
| S | 7.8 → 7.8 cm | 41.6 → 42.0 s | – |
| widen | 1.7 → 1.7 cm | 29.0 → 30.6 s | – |
| turn90 | 7.4 → 7.4 cm | 15.7 → 16.2 s | – |

The extra seconds: the end of the paint (0.5 m past the finish) is now seen
as `lane_lost` ahead, so the robot slows to 0.25 m/s for the last ~1.5 m; and
a corner's inner line now slows it 0.6 m before the corner cue. corner90,
corner90_right, corner90_wide and corner_mix still finish with scrub 0.8, IMU
noise 0.3° + drift 0.5°/s, one frame latency, and start offsets (20/20).
No turn-type junction on S, widen, turn90, corner90 or corner_mix, nor on the
start/finish bars (tests).

### Limits (not handled yet)

* Junction turns are 90°. Oblique branches (45°) are classified as openings
  but the pivot angle is still 90°.
* One junction at a time; two crossings closer than ~2 m apart are untested.
* A T whose far line is beyond the reach is driven into as a plaza and only
  turned when the bar ahead shows (pivot = bar − half a lane, assumes the far
  road has the robot's lane width). Not in the test courses.
* Side lines seen while blind are reported but do not change the route.

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

## Obstacles (stereo + radar)

Lane keep itself is unchanged. When obstacle sensing is on (`docs/OBSTACLES.md`),
the gate only caps forward speed: `CLEAR` / `SLOW_FOR_OBSTACLE` /
`STOPPED_FOR_OBSTACLE` / `WAITING_FOR_MOVING` (radar saw a moving hit).
HUD line `OBS …`; CSV columns `obstacle_*`. Path-around is not in this pass.

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
