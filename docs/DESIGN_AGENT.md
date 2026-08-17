# Design agent (first cut)

North-star item 3 — thin. Reads the SQLite catalog (`src/parts_db.py`) and
returns a few scored BOMs. **No Webots, no Postgres, no lane-keep.**

Scores are heuristics so we can later plug in a twin eval without changing
`propose(mission) -> {candidates: [...]}`.

## Interface

```python
from src.design_agent import Mission, propose, format_report

result = propose(
    Mission(
        name="mirrored S",
        track_length_m=25,
        max_cost_usd=800,
        max_mass_g=6000,
        max_minutes=5,
        prefer_low_compute=True,
    )
)
print(format_report(result))
```

Each candidate has `name`, `score`, `feasible`, `bom` (line items), `totals`
(cost, mass, energy_wh_est, pack_wh, energy_ok, speeds), `notes`, `rationale`.

## CLI

```powershell
python -m src.design_agent
python -m src.design_agent --length 25 --max-cost 800 --max-mass 6000 --minutes 5 --prefer-low-compute
```

## What it actually does

1. Loads motors × batteries × cameras from the catalog (wheels / compute / ESC / IMU are shared).
2. Qty: 2 drive motors, 2 wheels, 2 cameras (line-pair), one of everything else.
3. Energy (twin-backed): `src/twin_energy.py` uses the V1 wheeled baseline (~40 W total at 0.44 m/s → ~0.025 Wh/m, ×1.35 on an S). The old paper number (draw × `max_minutes`) is still returned as `energy_wh_paper` for comparison. Optional `data/twin_energy.json` (`wh_per_m`) overrides if present. **No Webots required.**
4. Speed: motor no-load RPM × wheel radius vs required `length / max_minutes`.
5. Voltage note if pack and motors differ by more than 2 V (48 V pack vs 12 V 37D).
6. Rank: feasible first, then cheaper / lighter / lower camera `compute_cost`.

Still not a live physics search. A 3S LiPo can look fine at 0.9 Wh vs 55 Wh and still be the wrong robot on the twin. Next plug-in is a real `measure_*` run writing `data/twin_energy.json`.
