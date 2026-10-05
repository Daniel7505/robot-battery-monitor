# Multi-trip shopping list (fill trolley → ship → repeat)

**Status: next vertical slice after inventory/trolley.** A shopping list of
several SKUs across aisle groups is packed into **3–4 trolley trips** that fill
capacity before shipping. Route order is a simple aisle sweep (optional
nearest-neighbour). Reuses pretend pick + hitch trolley; clears the trolley
load between trips. BoM held. Ultrasound / path-around deferred. Dashboard
battery/time efficiency is a **light stub** (dict + track JSON), not a full UI.

## What you get

| piece | where |
|---|---|
| More shelf SKU groups | `tracks/warehouse.json` → `inventory` (7 groups across aisles 1–7) |
| Shopping plan | `tracks/warehouse.json` → `shopping_list` (lines, trips, routes, assumptions) |
| Pure planner + state machine | `src/shopping_list.py` |
| Per-trip missions | `shopping_list_trip_1` … `_4` (each has `RBM_ROUTE`) |
| Controller glue | `RBM_SHOPPING_LIST=demo` / `1`…`N` / `all` in `controller_inventory.py` |
| Efficiency stub | `shopping_efficiency_metrics()` → also under `shopping_list.efficiency` |

## Capacity / trip rules

* **Capacity = 2 cubes** — floor packing on the trolley bed
  (`0.55 × 0.40 / 0.28 × 0.28 ≈ 2`). No stacking, no mass limit. Derived from
  `TrolleyGeom` + cube footprint (`trolley_capacity_units()`).
* **Demo list = 7 lines** (one cube per seeded SKU) → **4 trips** at cap 2:
  `(2)+(2)+(2)+(1)`.
* **Packing:** visit-order first, then first-fit into capacity buckets; each
  full (or final) bucket ships.
* **Visit order (default `aisle`):** aisle ascending → block → SKU. Optional
  `nearest` from `start_xy` / last approach.
* **Per-trip route:** `RoadGraph.plan(to=shipping_1, via=approach points)` from
  receiving start. Static string — no live re-route around blocked aisles.

## Assumptions (also in `shopping_list.assumptions`)

1. Capacity is floor cubes on the bed, not weight.
2. Each list qty unit = one pretend-pick cube removal.
3. Fill-then-ship packing after visit ordering (not cluster TSP).
4. Aisle sweep by default; nearest-neighbour optional.
5. `RBM_ROUTE` planned once per trip from start → vias → SHP-1.
6. **Webots: one trip per launch** (finish at shipping ends the run). Unload /
   reset between trips is modelled in `MultiTripState` for offline / future
   continuous dock-return.
7. Ultrasound and path-around still deferred. BoM held.

## CMD (Windows)

```bat
git checkout feature/shopping-list-trips
git pull
pip install -r requirements.txt

REM dashboard twin must already be running: scripts\start.ps1

REM --- trip 1 smoke (two SKUs, fill trolley, ship) ---
set RBM_ROUTE=S,S,R,L,L,R,L,L,S,R,S,S,S,S,L,S
set RBM_SHOPPING_LIST=1
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World warehouse

REM --- trip 2 ---
set RBM_ROUTE=S,S,L,S,R,L,L,R,S,S,L,S
set RBM_SHOPPING_LIST=2
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World warehouse

REM trip 3 / 4: routes in tracks\warehouse.json missions shopping_list_trip_N
REM or: python scripts\warehouse_builder.py --summary

REM clear when done
set RBM_SHOPPING_LIST=
set RBM_ROUTE=
```

`RBM_SHOPPING_LIST=demo` is an alias for trip 1 (one-launch smoke).
`RBM_SHOPPING_LIST=all` runs the multi-trip state machine (unload between
trips); Webots still finishes at SHP after trip 1 until continuous dock-return
exists — use per-trip launches for real twin runs.

One-SKU pick still works: unset shopping list, set `RBM_PICK_SKU=demo`.

Unit tests (no Webots):

```bat
python -m pytest tests\test_shopping_list.py tests\test_inventory_pick.py tests\test_warehouse.py -q
```

Regenerate after changing `WarehouseSpec` / inventory seed:

```bat
python scripts\warehouse_builder.py
python scripts\warehouse_builder.py --summary
```

## Efficiency stub

`shopping_efficiency_metrics(plan)` returns `n_trips`, `total_units`, `path_m`
(sum of trip lengths when routes attached), `est_drive_wh` (placeholder
0.012 Wh/m), and a note. Written into `tracks/warehouse.json` →
`shopping_list.efficiency`. **Not** wired into the dashboard UI yet — Dan's
call whether to surface it on the battery card or a shopping panel.

## Design choices needing Dan

1. **Capacity = 2 floor cubes** — OK, or allow stacking / mass-based capacity?
2. **Webots one-trip-per-launch** vs continuous dock-return without finish —
   continuous needs finish-referee changes + return-to-aisles route.
3. **Aisle sweep vs nearest** as the default order for the demo list.
4. **Dashboard efficiency** — keep stub-only, or wire battery/time this PR?
5. Early corner clip on the flat squared warehouse still noted; unchanged.

## Deferred

* Continuous multi-trip in one Webots session (dock-return, no early finish)
* Avoidance / re-route around a blocked aisle
* Ultrasound
* Full dashboard battery/time efficiency panel
* BoM / real pick hardware
