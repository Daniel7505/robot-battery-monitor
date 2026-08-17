# Parts catalog (design-agent knowledge)

SQLite catalog of **purchasable hardware**. This is north-star item 1.

It is **not** the live telemetry store.

| Store | Engine | File / service | Purpose |
|:------|:-------|:---------------|:--------|
| Telemetry / power history | **PostgreSQL** | Docker `robot-battery-postgres`, `src/database.py` | Ticks, channel watts, forecasts |
| Hardware **runtime model** | YAML | `config/hardware_profiles/butlerbot_wheeled.yaml` | Power curves for the *current* twin |
| Parts **catalog** (this) | **SQLite** | `data/parts.db`, `src/parts_db.py` | SKUs the design agent will pick from |

Do not migrate or overwrite Postgres. Do not replace the YAML profile with this DB — they answer different questions (“what is this robot drawing?” vs “what can we buy?”).

## Schema

- `parts` — every SKU: name, category, vendor, mass_g, cost_usd, url
- `motors` / `batteries` / `cameras` — extra columns by category
- IMU, ESC, compute, wheels live as `parts` rows only in this first cut

Categories: `motor | battery | camera | wheel | structure | esc | sensor | compute | other`

## Use

```powershell
# Create data/parts.db if missing, seed examples, print demo queries
python -m src.parts_db
```

```python
from src.parts_db import connect, list_motors, get_part

conn = connect()  # default: <repo>/data/parts.db
for m in list_motors(conn, max_cost_usd=50, max_mass_g=250):
    print(m["sku"], m["cost_usd"], m["peak_torque_nm"])
```

Re-seed: delete `data/parts.db` and run `python -m src.parts_db`. Seed only fills an empty table.

## Seed source

Example rows are the real / catalog-class SKUs already named in `butlerbot_wheeled.yaml` (Pololu #4753, Adafruit BNO085 #4754, SparkFun TB6612, 48 V 10 Ah pack, 80 mm wheel, Jetson Orin Nano class) plus cheaper alternatives (19:1 37D, 3S LiPo, Pi Cam / USB cam) so the design agent has something to compare.

Energy numbers measured in Docker / Webots stay in Postgres and `docs/V1_WHEELED_ENERGY_BASELINE.md`. This catalog does not ingest those ticks.

## Next

Thin design agent is in [`DESIGN_AGENT.md`](DESIGN_AGENT.md) / `python -m src.design_agent`. Twin evaluation of those BOMs is still later. Grow this schema only when that agent needs a column.
