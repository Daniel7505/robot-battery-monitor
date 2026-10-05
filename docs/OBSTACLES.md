# Obstacle sensing: stereo (static) + radar (moving)

Goal: the bot sees what is in its lane ahead (height and depth, no SLAM, no
pretraining, no map of obstacles), slows down, stops at a braking distance
derived from its speed and deceleration, and resumes when the lane clears.
Stereo handles **static** nearby height/depth. Radar handles **moving** objects
(and can confirm stereo). No path-around yet — a moving hit waits; a still hit
slows then stops.

```
stereo_left / stereo_right (Webots Camera, 320x240)
  -> rectify / disparity / depth / corridor  -> ObstacleReport (static height/depth)

radar_fwd (Webots Radar, bumper height)
  -> targets (range, relative radial speed, azimuth)
  -> corridor filter + ego-speed compensation
  -> ObstacleReport (velocity_m_s, notes.moving)

  -> ObstacleFusion (nearest; prefer moving) -> ObstacleGate
  -> CLEAR / SLOW_FOR_OBSTACLE / STOPPED_FOR_OBSTACLE / WAITING_FOR_MOVING
  -> speed cap on RowfitController
```

Files:

* `src/stereo_depth.py`: mount constants, `StereoRig`, disparity backends,
  `points_from_disparity`, `CorridorParams`, `detect_obstacle`,
  `StereoObstacleSensor`.
* `src/obstacle_gate.py`: `ObstacleReport`, height bands, `ObstacleSource` /
  `ObstacleFusion`, `stop_distance_m`, `ObstacleGate` (incl. `WAITING_FOR_MOVING`).
* `src/radar_sense.py`: `RadarTarget`, ego-compensated motion classify,
  `RadarObstacleSensor` (`ObstacleSource` name `"radar"`).
* `webots/controllers/butlerbot_controller/controller_obstacles.py`: on/off
  decision, stereo + radar enable, live pose binding, per-frame harvest / fusion,
  HUD / CSV fields, the `RBM_OBS_REMOVE_AFTER_S` test harness.
* `src/stereo_synth.py`: numpy raycaster for synthetic stereo pairs (tests,
  offline sim, `scripts/stereo_accuracy.py`).
* `scripts/rowfit_sim.py --obstacles` / `--obstacle X,Y,SX,SY,H`: the same
  sensor and gate in the closed-loop offline sim.

## The mount

Every world (`butlerbot*.wbt`, identical Robot block) has a visual mast and
bar on the robot's front centre line and two `Camera` nodes:

| | value | why |
|---|---|---|
| baseline | 0.10 m (lenses at y = ±0.05) | middle of the 8–12 cm brief; fB = 19 px·m, so 1 px of disparity is 5 cm of depth at 1 m and 21 cm at 2 m |
| lens position | x 0.14, z 0.78 m | front of the body, just under the 0.80 m robot top; eye height of a cart |
| pitch | 19.9° down (`stereo_pitch_for_top`) | derived: the top image row reaches 1.0 m height 1.0 m ahead of the lens, so a waist / table top (≤ 1 m) is fully in view from 1 m out and racks are seen tall from ~3.5 m |
| image | 320 × 240, fov 1.4 rad (f = 190 px) | 64 disparities cover 0.30 m upward; SGBM runs in ~10 ms |
| floor in view | from 0.75 m ahead of the axle | the near floor is the shoulder cams' job |

The controller does not trust these constants: `ObstacleRuntime.bind` reads
the cameras' live pose from the supervisor (like the shoulder cams do) and
builds the rig from that, so moving a camera in Webots changes the maths.
It prints, e.g.:

```
STEREO rig baseline 10.0 cm, f 190.0 px, fB 19.00 px*m, 320x240, lens z 0.780 m, pitch 19.9deg, floor seen from 0.75 m, top row 1.00 m high at 1 m, rectification none (already rectified)
OBSTACLES ON — backend sgbm, corridor +-0.54 m, headroom 0.97 m, ...
```

Derived from the live setup as well: corridor half width = shoulder-cam |y|
(the widest part of the robot) + 0.025 + 0.10 m margin = ±0.54 m;
headroom = highest camera + 0.02 + 0.15 m = 0.97 m (anything lower than this
blocks the robot); front = max(0.15, lens x + 0.01).

## Pipeline details

* **Rectification.** The common orientation is the average of the two live
  camera orientations and the baseline is the true lens offset. When the
  cameras are not already parallel (one yawed in Webots), both images are
  warped by a homography. A test toes the right camera in by 0.6° and checks
  that depth stays right; without rectification it is badly off.
* **Disparity.** `sgbm` = OpenCV StereoSGBM (block 5, P1/P2 = 8/32·block²,
  uniqueness 10, LR check), then a texture mask. `numpy` = SAD on x-Sobel plus
  0.3·intensity, box filter, uniqueness ratio, parabolic sub-pixel, texture
  threshold and left-right check. Same results within a few cm (table below),
  ~15× slower. `auto` picks sgbm when `cv2` imports.
* **Ground plane.** The floor is z = 0 in the bot frame, from the live camera
  pose. A point is "above the floor" when z > 0.05 m + 3·σ_z(x), where
  σ_z(x) = 0.25 px · x · z_cam / fB (stereo height noise grows with range).
  Tape is flat, so it is floor. The logged `floor_z_med` (median z of floor
  points) checks the calibration; it is within 2 mm on the synthetic floor.
* **Corridor.** |y − κx²/2| ≤ half width, x from the robot front to 4 m,
  z between the floor tolerance and the headroom. κ is the commanded lane
  curvature, so on a curve the corridor bends with the lane.
* **Detection.** Sort corridor points by x. The nearest 0.25 m depth window
  that has ≥ 20 points, covers ≥ 0.01 m² of surface (pixel footprint
  (x/f)²) and spans ≥ 5 image rows is the obstacle. These three rules reject
  single mismatches, small blobs at occlusion edges and thin horizon bands; a
  5 cm table leg still passes. Distance = 5th percentile of that window
  (conservative). The lateral extent follows the object up to 0.5 m past the
  corridor edge. Height = 98th percentile z of the object's points.
* **Height band.** low / pallet < 0.6 m ≤ waist / table < 1.3 m ≤ tall / rack.
  If the top touches the top image row it is `top_clipped` (`0.90+` in the
  log): "at least this tall". The gate keeps the tallest band seen for a
  tracked obstacle, so a rack seen tall at 3.5 m stays tall at 1 m.
* **Confidence** = points in the window / (3 × the minimum), capped at 1.

## The gate (behaviour)

States: `CLEAR`, `SLOW_FOR_OBSTACLE`, `STOPPED_FOR_OBSTACLE`, `WAITING_FOR_MOVING`.
Numbers at the current rowfit governor (cruise 0.44 m/s, decel 0.8 m/s²):

| quantity | formula | value |
|---|---|---|
| stop distance d_stop(v) | standoff 0.35 + v·latency + v²/(2·decel) | 0.61 m at 0.44 m/s, 0.44 m at 0.20 m/s |
| slow zone | d_stop(cruise) + 1.0 m | 1.61 m (clearance from the robot front) |
| creep speed | min(0.20 m/s, √(2 · 0.5·decel · (clear − standoff))) | 0.20 m/s, tapering to 0 at the standoff |
| emergency brake | clear < v²/(2·decel) + 0.05 | ABS brake (the governor's slew is skipped) |
| resume from STOPPED | clear > d_stop(0) + 0.25 m, or obstacle gone | hysteresis 0.25 m |
| leave SLOW | clear > slow zone + 0.25 m, or gone | |

latency = the measured camera frame period. Clearance is measured from the
robot's front (x = 0.15 m), not the axle.

Behaviour target for this pass:

* lane clear → drive (`CLEAR`)
* something still in lane → slow then stop (`SLOW_FOR_OBSTACLE` / `STOPPED_FOR_OBSTACLE`)
* object **moving** in lane → wait (`WAITING_FOR_MOVING`, `v_cap = 0`) until still
* still again → same slow/stop path (path-around within the lane is a later PR)


* A detection must be seen in **2 frames** (matched within 0.4 m after
  odometry) before it counts; a one-frame false positive does nothing.
* The obstacle is **tracked by odometry** between frames.
* **Clear needs 3 frames** in which the obstacle *should* be visible. When the
  robot is close, a low box drops under the cameras' view (bottom ray; a
  0.5 m box leaves the view at ~0.2 m clearance), so "not seen" no longer
  means "gone". Then the gate holds (`blind` on the HUD) instead of resuming
  into it. While stopped the robot does not move, so the remembered obstacle
  stays where it was. At the 0.35–0.45 m stop point a 0.5 m box is still in
  view (tested), so deleting it resumes the robot.
* The cap is applied in `RowfitController` (`obstacle_v_cap`) on the lane
  speed, the corner / junction approach speed and the pivot drive. Lane
  keeping, corners and junction logic are unchanged.

## On or off by default

**Off by default; on automatically only in `butlerbot_warehouse_obstacles.wbt`**
(world tag `# OBSTACLES on`). `RBM_OBSTACLES=1` turns it on in any world,
`RBM_OBSTACLES=0` turns it off even in the obstacles world. Why:

1. Dan's validated runs (S, corner90, plus / T / gap worlds, warehouse dock
   shuttle and odd-aisle-5) stay byte-for-byte the same behaviour. The
   cameras are in every world, but disabled cameras cost nothing.
2. Two extra 320 × 240 cameras cost render time in realtime mode; no reason
   to pay that on lane-keeping tests.
3. It is a first pass validated only offline. A false stop in a lane test
   would look like a lane-keeping bug.
4. On the worlds it was tested on (offline: warehouse odd-aisle route,
   156 m, and dock shuttle with sensing on) it caused no false slow-downs, so
   turning it on by default later is a one-line change once Webots runs agree.

The console always says which and why, e.g. `OBSTACLES off (RBM_OBSTACLES
unset, world has no '# OBSTACLES on' tag)` or `OBSTACLES on (world tag
'# OBSTACLES on')`.

## Logs and HUD

* HUD (rowfit text block): `OBS clear to 4.0m`, `OBS SLOW 1.42m low c1.0`,
  `OBS STOP 0.57m waist c1.0`, `OBS WAIT 1.20m low c0.9 mov` (`blind` / `mov`
  suffixes while holding / moving).
* Console: every state change, e.g. `OBSTACLE SLOW_FOR_OBSTACLE ->
  STOPPED_FOR_OBSTACLE: stop: clearance 0.45 m <= stop distance 0.44 m
  (v 0.20 m/s) [stereo: low/pallet obstacle at 0.60 m ...]`.
* `lane-vision.csv`: four columns **appended** after the existing ones:
  `obstacle_state,obstacle_m,obstacle_band,obstacle_conf,obstacle_moving`
  (obstacle_m from the axle; empty when clear or off). Rename an old
  `lane-vision.csv` before the first run so the new header is written.
* `scripts\run_report.py` adds an `obstacles:` line per run: number of
  stops and slow-downs, nearest obstacle, and for each stop its position,
  band, distance, wait and how it resumed. Old logs without the columns are
  reported as before.

## Test props (`butlerbot_warehouse_obstacles.wbt`)

Generated by `scripts/warehouse_builder.py` into the builder's
`WH_OBSTACLES` group (the clean warehouse keeps it empty). All positions
come from the layout; see `docs/WAREHOUSE.md`. The props have a
speckle texture (`webots/worlds/textures/obstacle_speckle.png`) because
stereo needs texture to match on.

| prop | route | expected | offline sim result |
|---|---|---|---|
| box 0.6 × 0.5 × 0.5 m, aisle 5 centre | `S,S,L,S,S,R,S,L,S,S,L,S,S,S` | stop, resume when removed | slowed at 1.54 m, stopped at 0.45 m (true 0.43 m), resumed 1 s after removal, exit shipping_1 OK, max ct 1.3 cm |
| table 0.9 m, edge 0.30 m into the dock lane | `L,S` | stop, waist band | slowed at 1.59 m, stopped at 0.40 m (true 0.35 m), waist, resumed after removal, exit shipping_2 OK |
| shelf 1.8 m, corner 0.75 m left of the start lane | every mission | pass | no slow-down on either route |

## Depth accuracy (synthetic, `python scripts\stereo_accuracy.py`)

Textured wall facing the robot (median signed / 90th-percentile absolute
error of all wall points) and the detected distance of a 0.6 × 0.5 × 0.5 m
box (5th-percentile rule, so negative = reported early, the safe side).
"1 px" = depth change of one pixel of disparity.

| range | wall sgbm med / p90 | wall numpy med / p90 | box sgbm | box numpy | 1 px |
|---|---|---|---|---|---|
| 0.6 m | −0.1 / 0.4 cm | 0.0 / 0.2 cm | −0.5 cm | −0.2 cm | 1.9 cm |
| 1.0 m | −0.1 / 1.3 cm | 0.0 / 0.5 cm | −1.5 cm | −0.6 cm | 5.3 cm |
| 1.5 m | −0.4 / 2.6 cm | −0.1 / 1.1 cm | −3.2 cm | −1.5 cm | 11.8 cm |
| 2.0 m | −1.1 / 4.7 cm | −0.2 / 2.3 cm | −5.1 cm | −3.0 cm | 21.1 cm |
| 3.0 m | −4.8 / 11.6 cm | −0.3 / 5.2 cm | −14.5 cm | −9.4 cm | 47.4 cm |
| 4.0 m | +4.9 / 19.7 cm | +3.9 / 9.5 cm | −3.9 cm | −10.5 cm | 84.2 cm |

Box height was measured 0.50–0.53 m (true 0.50). Time per frame: sgbm
8–20 ms, numpy 160–190 ms (box CPU). Stops happen at 0.4–0.6 m, where the
error is well under 2 cm. These are synthetic images, not Webots renders.

## Radar (moving objects)

Mount (every `butlerbot*.wbt`): `DEF RADAR_FWD Radar` named `radar_fwd` at
translation `(0.16, 0, 0.45)`, rotation `0 1 0 π/2` (Webots Radar looks along
`-Z`; that rotation aims it along robot `+x`). Ranges 0.15–8 m, HFOV 1.0 rad,
VFOV 0.7 rad. Solids need `radarCrossSection > 0` to be visible (warehouse
builder sets 1.0 on racks / staging and 2.0 on `obs_*` props; the rolling-ball
world sets 1.0 on `WH_MOVING_BALL`).

### Motion algorithm

1. Read Webots targets (`distance`, `speed`, `azimuth`). Webots `speed` is
   radial along the radar→target LOS (**+ = receding**).
2. Keep targets whose `|distance · sin(azimuth)|` is inside the live corridor
   half-width (same number stereo uses, from shoulder-cam width).
3. Closing relative to the robot = `-speed`. Subtract ego:  
   `target_radial = closing − v_ego · cos(azimuth)`.  
   `|target_radial| ≥ 0.12 m/s` ⇒ **moving** (`notes["moving"]=True`);
   otherwise still. A static wall while cruising cancels to ~0; a ball rolling
   toward the robot does not.
4. Emit `ObstacleReport(source="radar", velocity_m_s=closing, …)`. Fusion
   prefers a moving hit over a static one at similar range; the gate enters
   `WAITING_FOR_MOVING` (hold) until still frames clear the flag, then the
   usual slow/stop path.

Test world: `butlerbot_radar_motion.wbt` (S track + `# OBSTACLES on` + a
physics ball that starts **left of the start-straight lane** at ~(2.2, 1.15) m
and rolls across y=0 in front of the bot). An earlier (5, 0) placement sat on
the S lobe outside the yellow lines and never produced `WAITING_FOR_MOVING`.
See `docs/CHEAT_SHEET.md`.

## Known limits

* **Texture.** Stereo needs texture. A plain, evenly lit wall or box gives
  few matches and may be missed or detected late; the test props are
  speckled for that reason. Real warehouses have labels and edges; Webots'
  flat colours do not. Radar is the fix.
* **Repeating patterns** (rack beams, mesh) can give wrong matches. The
  uniqueness ratio and the LR check reduce this; not tested on Webots racks.
* **Blind zone.** Below the bottom ray: at 0.35 m clearance (the
  standoff) only the part of an object above 0.32 m is in view, at 0.2 m only
  above 0.51 m. Something low that enters right in front of the robot is
  not seen. The gate holds instead of resuming, but
  something appearing there (pushed in from the side) is not seen. The
  shoulder cams see the near floor; they are not used for obstacles yet.
* **Corridor shape.** Straight, or bent by the current curvature. In a
  pivot, at a junction or before a corner, it still looks straight ahead.
  An obstacle just around a corner is seen late (after the pivot).
* **No tilt compensation.** The ground plane comes from the camera pose; the
  IMU pitch / roll is not used. Fine on a flat Webots floor.
* **Occlusion of the lane.** A big obstacle hides the lane from the shoulder
  cams only when very close; the stop point is in front of that.
* **No avoidance, no re-route.** The bot waits. A blocked aisle blocks the
  mission until it is cleared.
* **Synthetic validation only.** Accuracy and thresholds are from numpy
  renders (raycaster with value-noise texture), not Webots lighting /
  anti-aliasing. Webots runs may need the texture threshold or minimum
  point counts tuned.
* **numpy backend is slow** (~170 ms / frame). Install OpenCV.
* **Stop standoff.** The robot stops ~0.35–0.45 m before the obstacle face
  (standoff 0.35 m plus the last frame's travel at creep speed).
* Path-around / re-route is not in this pass: a still blocked lane stays
  stopped; a moving hit waits. Ultrasound is a later short-range backup.
