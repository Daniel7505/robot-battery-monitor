# Inventory cubes + pull-behind trolley (one-SKU pretend pick)

**Status: first vertical slice after radar.** Placeholder inventory on shelves,
a hitch-following trolley prop, and a pretend pick for **one** demo SKU, then
drive to shipping. Multi-trip shopping lists, route optimisation that fills the
trolley before shipping, and dashboard battery/time efficiency metrics are
**deferred** (stubs only). BoM still held. Ultrasound deferred.

## What you get

| piece | where |
|---|---|
| Grouped placeholder cubes | `DEF WH_INVENTORY` in both warehouse worlds — teal = demo SKU, terracotta = extras |
| SKU catalog | `tracks/warehouse.json` → `inventory` (aisle / block / side / approach_xy / cube DEFs) |
| Pull-behind trolley | `DEF WH_TROLLEY` (+ hidden `WH_TROLLEY_LOAD` cargo) — Supervisor hitch each tick |
| One-SKU pretend pick | `RBM_PICK_SKU=demo` (or a concrete SKU) — stop at approach, remove one cube, raise cargo, resume |
| Mission | `pick_one_sku_to_shipping` (same `RBM_ROUTE` spine as `odd_aisle_to_shipping`) |
| Pure logic | `src/inventory_pick.py` (unit-tested) |
| Controller glue | `webots/controllers/butlerbot_controller/controller_inventory.py` |

Geometry (cube poses, approach points, trolley hitch) is **derived from the live
warehouse layout / chassis length**, not hard-coded aisle metres. Change
`WarehouseSpec` / `inv_*` / `trolley_*` and regenerate.

## Demo SKU

Default seed puts the demo group on the **odd aisle** used by
`odd_aisle_to_shipping` (aisle 5, rack block 0, left face):

* SKU id: `SKU-A05-B0-L`
* Approach: on the aisle centre line at the bay cluster (`approach_xy` in the catalog)
* Cubes: four 0.28 m placeholders on the lowest beam shelf, inset into the rack
  (off the taped lane)

Two extra groups (`SKU-A03-B0-L`, `SKU-A07-B1-L`) are visual placeholders for a
future shopping list — not picked in this PR.

## One-SKU demo (what happens)

1. Launch the **clean** warehouse (not `warehouse_obstacles` — that world still
   has the aisle-5 obstacle box on the same spine).
2. Set `RBM_ROUTE` to the pick mission string and `RBM_PICK_SKU=demo`.
3. Bot lane-keeps to aisle 5; when GPS is within ~1.1 m of the approach point it
   stops (`PICKING`), removes one `WH_INV_*` cube, raises `WH_TROLLEY_LOAD` onto
   the bed, then resumes (`DRIVING_TO_SHIP`) to SHP-1.
4. Console: `INVENTORY on — …`, `PICK … -> PICKING`, `PICK: removed …`,
   `PICK: WH_TROLLEY_LOAD raised onto bed`, then the usual finish line.

The trolley Solid is **not** a Robot child (Robot block stays byte-identical
across worlds). The controller teleports `WH_TROLLEY` behind the axle each tick
(hitch ≈ chassis/2 + clear + tongue + half bed ≈ 0.89 m). It is **visual-only**
(no `boundingObject` / no Physics): a static collision box on the hitch was
stalling the bot (not trolley mass — there is no physics mass).

## CMD (Windows)

```bat
git checkout feature/inventory-trolley
git pull
pip install -r requirements.txt

REM dashboard twin must already be running: scripts\start.ps1

set RBM_ROUTE=S,S,L,S,S,R,S,L,S,S,L,S,S,S
set RBM_PICK_SKU=demo
powershell -ExecutionPolicy Bypass -File scripts\launch_webots_twin.ps1 -World warehouse

REM clear when done
set RBM_PICK_SKU=
```

Or pin the SKU explicitly: `set RBM_PICK_SKU=SKU-A05-B0-L`.

Unit tests (no Webots):

```bat
python -m pytest tests\test_inventory_pick.py tests\test_warehouse.py -q
```

Regenerate after changing `WarehouseSpec`:

```bat
python scripts\warehouse_builder.py
python scripts\warehouse_builder.py --summary
```

## Design choices needing Dan

1. **Pretend pick only** — no arm IK / grasp; Supervisor deletes a cube and shows
   cargo on the trolley. Real pick hardware stays north-star / BoM later.
2. **Trolley is a world Solid**, hitch-followed — keeps the shared Robot block
   identical. A physics trailer hitch is a later option.
3. **One SKU / one trolley trip** this PR. Shopping-list route optimisation
   (fill trolley, then ship; 3–4 trips) is the next roadmap item.
4. **Dashboard battery/time as efficiency metrics** — stub OK; not wired here.
5. Early corner clip on the flat squared warehouse remains noted; unchanged.

## Deferred (next PR)

* Multi-SKU shopping list (3–4 trolley trips)
* Route optimisation that fills the trolley before shipping
* Efficiency metrics on the dashboard (battery / time)
* Avoidance / re-route around a blocked aisle (still open from obstacles)
* Ultrasound
