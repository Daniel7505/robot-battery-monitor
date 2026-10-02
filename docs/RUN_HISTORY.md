# Run history (current 16.5 m S track)

Best and notable full-S runs on the **current** track (5 m lobes, red finish at x=16.5 m,
since 2026-09-04 / `1ff49a5`). Older numbers in `LANE_KEEP_BASELINE_2026-08-17.md` are from
the earlier 24.5 m S and are not comparable 1:1.

Scored with:

```
python scripts\drift_report.py "%USERPROFILE%\OneDrive\Desktop\Grok Workspace\lane-vision.csv" --track s
```

max / mean / rms = sideways distance off the S-track centre line (GPS truth), metres.

| Date | run_id | Mode | Finished | Time s | Max | @x | Mean | RMS | Speed | Notes |
|------|--------|------|----------|--------|-----|----|------|-----|-------|-------|
| 2026-10-01 | 20261001-193237 | rowfit (PR #6) | yes, parked on red | 65.7 | **9.3 cm** | 5.53 m (left) | 3.2 cm | 4.3 cm | 0.44 cruise, ~0.36 in bends | First rowfit run in Webots. Old ~1 m drift at x≈7.4 gone (~7 cm there). |
| 2026-10-01 | — | gap (old default) | yes | — | ~1 m | ~7.2–7.6 m (right) | — | — | 0.44 fixed | Pre-rowfit reference from drift_report on earlier runs: line loss at the first S reversal. |

Add a row after each notable run (new mode, new tuning, new personal best).

## New baseline: cameras in the shoulder boxes (2026-10-01)

From the "Cameras into shoulder boxes, live camera pose, rowfit default" PR
on, the nadir cameras sit in `NADIR_BOX_L/R` at z ≈ 0.70–0.73 m
(`translation 0.03542 ±0.41808 0.70419`, `rotation 0 1 0 0.9411`) instead of
the old z 1.31 m mount 0.39 m behind the axle, and `rowfit` is the default
lane mode. The top row still reaches ~1.96 m, but the bottom row now lands
just ahead of the axle instead of 0.55 m behind it, and the stripe is ~2× wider in
pixels near the robot. Runs after this change are a **new baseline**: don't
compare them 1:1 with the rows above.
Gap-mode rows above are from the old view; gap mode is not re-tuned for this one.

| Date | run_id | Mode | Finished | Time s | Max | @x | Mean | RMS | Speed | Notes |
|------|--------|------|----------|--------|-----|----|------|-----|-------|-------|
| 2026-10-01 | 20261001-201555 | rowfit (PR #7, box cameras) | yes, done at x=16.45 m | 77.3 | **7.4 cm** | — | 2.8 cm | 3.7 cm | — | First run on the box mount (Dan, Webots R2025a). Old mount rowfit: 9.3 / 3.2 / 4.3 cm, 65.7 s. Pose came from the wbt fallback (supervisor read failed, fixed later in PR #7). Visible right turn on the last frame before parking (red finish bar fitted as lane; gated later in PR #7). |

## corner90: sharp 90° left (`butlerbot_corner90.wbt`, 8.0 m)

First waypoint-track world: 4 m straight, a true right-angle left, 4 m to the
finish (`tracks/corner90.json`). It has the box cameras and rowfit with the
sharp-corner state machine (PR #8): LANE → CORNER_APPROACH → PIVOT → REACQUIRE → LANE.

Scored with:

```
python scripts\drift_report.py "%USERPROFILE%\OneDrive\Desktop\Grok Workspace\lane-vision.csv" --track corner90
```

max / mean / rms = sideways distance off the corner90 centre line (GPS truth), metres.

| Date | run_id | Mode | Finished | Time s | Max | @x | Mean | RMS | Pivot | Notes |
|------|--------|------|----------|--------|-----|----|------|-----|-------|-------|
| 2026-10-01 | 20261001-224819 | rowfit + corner pivot (PR #8) | yes | 39.9 | **3.9 cm** | 3.96 m (left) | 0.8 cm | 1.3 cm | yaw +89.9° vs target +90.6° (err +0.7°), 3.1 s | Dan, Webots. One of two runs, both perfect. `CORNER seen left 1.80 m conf 1.00`, `REACQUIRE ok after 3 frames`. States `LANE > CORNER_APPROACH > PIVOT > REACQUIRE > LANE`, 1 pivot. One `Rowfit REJECTED fit (heading jump +29deg > 20deg …)` right after REACQUIRE → LANE, since fixed with the post-pivot settling window. Before the corner logic it braked at the corner (sim: x 4.29). |
