# Lane-keep — closed experiments (2026-08-17)

**Historical.** This is the picture-wins / LINE_CAM 128 freeze. It is **not** the live operator.

Live law (2026-09-02): shoulder nadir, fight 32/29 + ahead HOLD, cruise 5.5, v-scale 2.10. Champ **3.2 cm** on the red. See [`NORTH_STAR.md`](NORTH_STAR.md).

Product map: [`NORTH_STAR.md`](NORTH_STAR.md). Drive cards: [`LANE_KEEP_BASELINE_2026-08-17.md`](LANE_KEEP_BASELINE_2026-08-17.md).

Do **not** rerun these as “new ideas.” They were isolated, measured, and closed.

Driving policy is frozen at the **A-winner** plus two free loop throttles. Reopen only if Daniel or Web explicitly names a new experiment.

## Frozen policy

| Knob | Value | Do not |
|------|-------|--------|
| `PLANNER_KAHEAD` | **0.22** | Do not step to 0.15 / 0.45 / anything else |
| `DEFAULT_STEER_RELEASE_PER_S` | **4.0** | Do not raise to 6 or 8 (D already walked off) |
| `DEFAULT_CRUISE_RAD_S` | **5.5** (0.44 m/s) | Do not raise until first-lobe ≤ 14 cm **and** ≥ 60 % @ 5 cm on a scored S |
| Preview | **on** (`t_ahead` fallback 2.0 s) | Preview-off aborted the second lobe |
| Z/W | **lookout only** (fill veto) | Never vote left/right; meters are 12 cm of floor |
| LINE_CAM_L/R | identity, look −Z, **128×128** (frozen 2026-08-17) | Never remount / never look-at. 64 is the historical champion only — see `LANE_KEEP_BASELINE_2026-08-17.md` |
| Map (`s_track_path.csv`) | scorekeeper only | Not memory, not SLAM |

Presentation / clock (keep — free, not drive policy):

- **H:** `_paint_eye_huds` every **4** steps (32 ms)
- **H2:** Z/W `forecast_wall_hit` + full-frame fill/offset every **8** steps (~10 Hz). `lookout.step` still 8 ms on last fills. L/R yellow stays full rate.

## Closed this week — do not retry

| ID | What we tried | Result | Do not retry |
|----|---------------|--------|--------------|
| **A** | `PLANNER_KAHEAD` 0.45 → **0.22** | Second-lobe crest x≈15.3: 21 cm → **4.9 cm**. Kept. | Do not move kahead again |
| **D** | Release 4 → 6 | Walk-off **1.52 m**. Reverted. | Do not raise release |
| Preview-off | Far sliver silenced | Second lobe failed, abort x=18 y=1.72 | Do not turn preview off |
| Z/W meters as steer | `zYm`/`wYm` into planner | Wreck ct=−5.55 | Do not put forecast meters on the wheel |
| **H** | HUD paint every 4 steps | Integrated rt stayed **0.175**. Predicted ≥0.35–0.45 **falsified**. Overlay paint is not the 8 ms tax. | Do not re-tune HUD rate expecting realtime |
| **H2** | Z/W work every 8 steps | rt **0.175 → 0.210**. Missed ≥0.30. Speeds still 0.440. Keep the throttle; do not add another Z/W skip. | Do not re-throttle Z/W expecting a molasses kill |
| **I** | `NEAR_PREF_GAIN=1.0` when \|near\|<0.14 | IN_LANE ON_LINE. First-lobe **22.9 cm** (still 20–32). 5 cm share **35%** (worse). Crest **10.3 cm** (was 0.4). **Reverted.** | Do not reintroduce near-pref / near-vs-far gain |

Clock campaign **closed**. Single-knob accuracy campaign **closed**. No more scored S this week unless Daniel reopens.

## What is actually true about molasses

15 s straight (`scripts/measure_clocks.py`):

- `v_cmd = v_odo = v_gps = 0.440` — soft-grip delivers
- Integrated realtime factor **~0.17–0.21** (`--mode=realtime` cannot keep 8 ms + 6 cams)
- A 56 s physics lap is a ~5–6 minute wall movie
- HUD paint and Z/W scans are **not** the dominant cost

## Last honest accuracy (A + H + H2, I reverted)

Hygiene / A-winner band:

- Finish: **IN_LANE ON_LINE**
- Second-lobe crest x≈15.3: **0.4–5 cm**
- First-lobe entry: **20–32 cm** (accepted leftover; I did not move it)
- ~48 % inside 5 cm on hygiene; champion was **71 % / 11 cm** with Z/W mounted but unused

Remaining tightness is the L/R planner (far-preview, grab/release), **not** another camera.

## Hardware / eyes still locked

- LINE_CAM_L/R: `(0.30, ±0.65, 0.12)`, **128×128**, FOV 1.2, look −Z
- FORECAST_CAM_Z/W: `(0.16, ±0.06, 0.20)`, identity `0 0 1 0`, HUD stay
- Tape in-buffer is lit gold ~(0.91, 0.59, 0.22) score ~0.53 — same family as tile. Color tweak blinds. Fill ≥ `MIN_STRIPE_FILL` 0.03 is the discriminator.
- **Do not save the Webots world on exit.** PowerShell `;` not `&&`. No LLM in the drive loop.

## Measure

- Clocks: `python -u scripts/measure_clocks.py` (needs `control_diag.sim_time_s`)
- Scored S: `python -u scripts/measure_s_lane_keep.py` (timeout **900 s** wall at 128)
- Integrate rt as Δsim/Δwall. `rt=0` rows are stale twin POST, not stopped physics.
