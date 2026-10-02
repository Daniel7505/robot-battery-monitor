# Camera yaw (toe-out) study: box-mounted nadir cameras

**Analysis only.** The cameras in `butlerbot.wbt` are unchanged (pure pitch
`rotation 0 1 0 0.9411`, yaw 0). Everything below is predicted with
`src/lane_vision.CameraModel`, the same pinhole model the controller uses.
None of it has been seen in Webots yet.

Regenerate the tables: `python scripts/camera_yaw_study.py --tracker`.
Single camera: `python scripts/aim_camera.py --yaw 10`.

## Setup
* Mount (box lens): left `0.03542 0.41808 0.70419`, right mirrored (y negated), fov 1.2 rad, 128×128.
* **Yaw > 0 = toe OUT.** The left camera turns left and the right camera turns right, as mirror images. Yaw < 0 = toe IN.
* **Order: yaw first, then pitch.** R = Rz(yaw)·Ry(pitch). The horizon stays level (no roll; the test checks this).
  * In the `.wbt` this is one axis-angle (`lane_vision.yaw_pitch_rotation`).
  * Mirroring a toed-out pair is **not** "same rotation line" any more. Negate translation y **and** the yaw, which means axis (x, y, z) → (−x, y, −z) with the same angle.
* **Pitch re-optimised per yaw.** The top-row centre still lands 1.96 m ahead of the axle, measured as robot forward x (same far reach as today). It barely moves: 53.9° at 0°, 53.6° at 10°, 52.8° at 20°.
* Robot centred in a straight lane unless noted. "In frame" means the stripe centre is at least 2 px inside the image.

## Results

| yaw (out +) | pitch | left rotation (axis-angle) | max lane w @0.5 / 1.0 / 1.5 m | own line (w 1.3) seen x | bottom-centre floor x, y | top-centre floor x, y | L/R overlap across the centre @0.5 / 1.0 / 1.5 m | sharp-corner crossing line first seen |
|---|---|---|---|---|---|---|---|---|
| -15° | 53.3° | `0.1263 0.9595 -0.2517 0.9638` | 1.63 / 1.74 / 1.86 m | 0.14..2.08 m | +0.06, +0.41 | 1.96, -0.10 | +0.70 / +1.44 / +2.18 m | 2.16 m (left) |
| -10° | 53.6° | `0.0859 0.9817 -0.1699 0.9512` | 1.72 / 1.92 / 2.13 m | 0.12..1.99 m | +0.06, +0.41 | 1.96, +0.08 | +0.52 / +1.14 / +1.76 m | 2.06 m (left) |
| -5° | 53.9° | `0.0435 0.9954 -0.0856 0.9436` | 1.82 / 2.12 / 2.41 m | 0.09..1.92 m | +0.06, +0.42 | 1.96, +0.25 | +0.38 / +0.88 / +1.38 m | 1.97 m (left) |
| +0° | 53.9° | `0.0000 1.0000 0.0000 0.9411` | 1.93 / 2.32 / 2.71 m | 0.07..1.89 m | +0.06, +0.42 | 1.96, +0.42 | +0.26 / +0.64 / +1.04 m | 1.87 m (left, right) |
| +5° | 53.9° | `-0.0435 0.9954 0.0856 0.9436` | 2.06 / 2.56 / 3.05 m | 0.05..1.88 m | +0.06, +0.42 | 1.96, +0.59 | +0.14 / +0.44 / +0.74 m | 1.97 m (left, right) |
| +10° | 53.6° | `-0.0859 0.9817 0.1699 0.9512` | 2.2 / 2.82 / 3.43 m | 0.03..1.91 m | +0.06, +0.42 | 1.96, +0.76 | +0.04 / +0.24 / +0.44 m | 2.06 m (left, right) |
| +15° | 53.3° | `-0.1263 0.9595 0.2517 0.9638` | 2.37 / 3.12 / 3.86 m | 0.02..1.96 m | +0.06, +0.43 | 1.96, +0.93 | -0.06 / +0.06 / +0.18 m | 2.16 m (left, right) |
| +20° | 52.8° | `-0.1639 0.9296 0.3301 0.9812` | 2.57 / 3.47 / >=4 m | 0.00..2.05 m | +0.07, +0.43 | 1.96, +1.12 | -0.14 / -0.10 / -0.08 m | 2.26 m (left, right) |
| +25° | 52.2° | `-0.1980 0.8930 0.4042 1.0035` | 2.82 / 3.89 / >=4 m | -0.01..2.18 m | +0.08, +0.44 | 1.96, +1.32 | -0.22 / -0.28 / -0.32 m | 2.36 m (left, right) |

Left-camera image column of the left line (robot centred; 0 = left edge, 127 = right edge, 'out' = outside the frame or within 2 px of the edge):

| yaw | w 1.0 m @ 0.3/0.5/1/1.5 | w 1.3 m @ 0.3/0.5/1/1.5 | w 1.6 m @ 0.3/0.5/1/1.5 | w 2.0 m @ 0.3/0.5/1/1.5 |
|---|---|---|---|---|
| -15° | 44 / 40 / 35 / 33 | 23 / 23 / 22 / 22 | out / 4 / 8 / 11 | out / out / out / out |
| -10° | 47 / 45 / 43 / 41 | 26 / 28 / 30 / 31 | 5 / 9 / 16 / 21 | out / out / out / 6 |
| -5° | 50 / 50 / 50 / 50 | 30 / 32 / 37 / 40 | 9 / 15 / 24 / 30 | out / out / 7 / 16 |
| +0° | 53 / 54 / 57 / 58 | 33 / 37 / 44 / 48 | 13 / 21 / 32 / 38 | out / out / 15 / 25 |
| +5° | 56 / 59 / 64 / 67 | 37 / 42 / 51 / 57 | 18 / 26 / 39 / 47 | out / 5 / 23 / 34 |
| +10° | 59 / 63 / 71 / 75 | 40 / 47 / 59 / 65 | 22 / 32 / 47 / 56 | out / 12 / 31 / 43 |
| +15° | 62 / 68 / 78 / 83 | 44 / 52 / 66 / 74 | 27 / 37 / 54 / 64 | 6 / 18 / 39 / 52 |
| +20° | 65 / 73 / 85 / 92 | 48 / 57 / 73 / 82 | 31 / 43 / 61 / 73 | 12 / 25 / 47 / 61 |
| +25° | 68 / 77 / 92 / 100 | 51 / 62 / 80 / 90 | 36 / 48 / 68 / 81 | 17 / 31 / 54 / 69 |

Sharp 90° LEFT corner, lane 1.30 m, robot centred: visible y-span of the crossing (outer) line when it is 1.0 m / 1.5 m ahead:

| yaw | left cam @1.0 m | right cam @1.0 m | left cam @1.5 m | right cam @1.5 m |
|---|---|---|---|---|
| -15° | -0.65..+0.87 | -0.65..+0.72 | -0.65..+0.93 | -0.65..+1.09 |
| -10° | -0.57..+0.96 | -0.65..+0.57 | -0.65..+1.06 | -0.65..+0.88 |
| -5° | -0.44..+1.06 | -0.65..+0.44 | -0.65..+1.20 | -0.65..+0.69 |
| +0° | -0.32..+1.16 | -0.65..+0.32 | -0.52..+1.35 | -0.65..+0.52 |
| +5° | -0.22..+1.28 | -0.65..+0.22 | -0.37..+1.52 | -0.65..+0.37 |
| +10° | -0.12..+1.41 | -0.65..+0.12 | -0.22..+1.71 | -0.65..+0.22 |
| +15° | -0.03..+1.56 | -0.65..+0.03 | -0.09..+1.93 | -0.65..+0.09 |
| +20° | +0.05..+1.73 | -0.65..-0.05 | +0.04..+2.17 | -0.65..-0.04 |
| +25° | +0.14..+1.94 | -0.65..-0.14 | +0.16..+2.11 | -0.65..-0.16 |

LaneTracker on a straight lane (points left / right line, fitted width and offset):

| yaw | w 1.3 m, robot +0.0 m | w 1.6 m, robot +0.0 m | w 2.0 m, robot +0.0 m | w 2.0 m, robot +0.3 m | w 2.0 m, robot -0.3 m |
|---|---|---|---|---|---|
| -10° | 170 / 170 pts, w 1.30, off +0.00 | 144 / 144 pts, w 1.60, off +0.00 | 44 / 44 pts, w 2.00, off +0.00 | 167 / 0 pts, w 1.30, off -0.05 | 0 / 167 pts, w 1.30, off +0.05 |
| +0° | 153 / 153 pts, w 1.30, off -0.00 | 152 / 152 pts, w 1.60, off +0.00 | 87 / 87 pts, w 2.00, off +0.00 | 154 / 31 pts, w 2.00, off +0.30 | 31 / 154 pts, w 2.00, off -0.30 |
| +10° | 165 / 165 pts, w 1.30, off -0.00 | 173 / 173 pts, w 1.60, off -0.00 | 121 / 121 pts, w 2.00, off -0.00 | 171 / 66 pts, w 2.00, off +0.30 | 66 / 171 pts, w 2.00, off -0.30 |
| +15° | 175 / 175 pts, w 1.30, off -0.00 | 183 / 183 pts, w 1.60, off -0.00 | 136 / 136 pts, w 2.00, off +0.00 | 180 / 83 pts, w 2.00, off +0.30 | 83 / 180 pts, w 2.00, off -0.30 |
| +20° | 185 / 185 pts, w 1.30, off +0.00 | 194 / 194 pts, w 1.60, off +0.00 | 149 / 149 pts, w 2.00, off -0.00 | 192 / 98 pts, w 2.00, off +0.30 | 98 / 192 pts, w 2.00, off -0.30 |

How to read it:
* **Column of the line.** At yaw 0 a 1.30 m lane's line sits at col 33–48 of 128. That is the outer third of the frame, which is what Dan saw on the HUD. A 1.6 m lane is at col 13–38, and a 2.0 m lane is out of view nearer than ~0.8 m. Toe-out moves the line toward the image centre (col 63.5).
* **Near / far coverage.** Toe-out costs no near coverage: the bottom-centre floor point stays 6 cm ahead of the axle, and the own line is seen from even closer (7 cm → 3 cm at 10°). Far reach is held at 1.96 m by the pitch re-solve. What it does cost:
  * **Overlap of the two views across the lane centre:** 0.64 m at 1 m ahead for yaw 0, 0.24 m at 10°, 0.06 m at 15°. From 20° a blind strip opens in the middle.
  * **The other side's line:** at yaw 0 the left camera sees the right line in its top-right corner (x ≥ 1.74 m). At 10° it does not, so each line is seen by one camera only.
* **Sharp 90° corner** (left turn shown; a right turn is the mirror):
  * The crossing (outer) line first enters view about 1.9–2.1 m ahead for any yaw from −10° to +10°. Yaw hardly changes how early the corner is seen, because the top row reaches 1.96 m either way.
  * It looks like a horizontal band across the top of both images.
  * The left (inner) line ends one lane width (1.3 m) before the crossing line and turns away to the left, out of the frame.
  * Toe-out widens how far along the new leg the crossing line is seen (left camera, 1.5 m ahead: up to y = +1.35 at 0°, +1.71 at 10°). But it shrinks the shared middle.
  * Toe-in widens the shared middle, but drops wide lanes.
* **LaneTracker check** (rendered frames, real fit): with toe-IN −10° a 2.0 m lane with the robot 0.3 m off-centre loses one line completely. It then falls back to a 1.30 m width and reports the offset wrong (−0.05 instead of +0.30). At 0° that case keeps only 31 points on the far line; at +10° it keeps 66.

## Recommendation: toe out 10°, pitch 53.6°

```
DEF NADIR_CAM_L Camera { translation 0.03542 0.41808 0.70419   rotation -0.0858892 0.981718 0.169862 0.9512 ... }
DEF NADIR_CAM_R Camera { translation 0.03542 -0.41808 0.70419  rotation 0.0858892 0.981718 -0.169862 0.9512 ... }
```

Why 10°:
1. **Wider lanes stay in view without fisheye or a higher mount.**
   * Widest lane whose line is in frame: 2.20 m at 0.5 m ahead (was 1.93) and 2.82 m at 1.0 m ahead (was 2.32).
   * A 2.0 m lane is visible from 0.5 m ahead (col 12) instead of only beyond ~0.8 m.
   * A 1.30 m lane sits at col 40–65, near the image centre. The line can move about 1.5× further outward before it leaves the frame (from the robot drifting away from it, or a wider lane): margin at 1 m ahead goes from 0.51 to 0.76 m, and at 0.5 m from 0.32 to 0.45 m.
2. **Nothing is lost near or far.** Bottom row 6 cm ahead, top row 1.96 m, same pixel footprint (0.61 cm/px at the bottom).
3. **The middle is still covered.** 24 cm of overlap at 1 m ahead and 44 cm at 1.5 m, so a sharp corner's crossing line is seen continuously across the lane by the pair.
   * 15° would leave only 6 cm at 1 m and a gap at 0.5 m.
   * 20° and 25° leave a blind strip in the centre, which is bad for crossing lines and for a single centre line.
4. **Toe-in is worse on every count that matters here.** It drops the widest visible lane (1.92 m at −10°), makes the off-centre wide-lane case fail in the tracker, and only marginally helps the shared middle, which yaw 0 already covers.

Costs to accept with 10°:
* Each camera stops seeing the far line.
* The near-centre overlap shrinks, so a single-side estimate relies on the measured lane width (already how rowfit works).
* Body occlusion was not re-modelled (the caster/wheel corner should move further out of frame, but this is unverified).
* The HUD frames look rotated by 10°: lines run diagonally toward the image centre instead of straight up the outer third.

## What lane_vision needs

Nothing. `CameraModel` takes any axis-angle rotation, and `parse_wbt_cameras` and the supervisor read take any rotation. Tests in `tests/test_lane_vision.py` cover yaw-then-pitch round trips (no roll), mirrored toed-out pairs giving mirror ground points, LaneTracker offset/heading/width with 10° and 20° toed-out cameras (including a 1.8 m lane), and parsing a yawed rotation from a `.wbt`.

One thing to re-derive if the cameras are changed: the look-ahead and fit windows (`lookahead_bounds_from_coverage`, `fit_windows`) come from the camera coverage at start-up, so they adapt on their own. The gap-mode pixel targets do not.

## Not verified without Webots
* The real image (lighting, parquet, motion) at a 10° toe-out.
* Body and arm occlusion.
* The HUD look.
* Whether Webots' `rotation` with these axis values renders exactly as modelled. The model matches Dan's pure-pitch drills, but no yawed drill has been done yet. Check: with the robot centred on the S start, the left line should run from about (col 26, row 127) to (col 70, row 0) in the left HUD (`aim_camera.py --yaw 10`).
