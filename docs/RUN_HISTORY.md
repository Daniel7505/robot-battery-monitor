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
