"""Inventory cubes + pull-behind trolley + pretend pick / shopping list (Webots glue).

Pure planning lives in ``src/inventory_pick.py`` and ``src/shopping_list.py``.
This module:

* follows ``DEF WH_TROLLEY`` behind ButlerBot each frame (Robot block stays
  byte-identical across worlds — the trolley is a warehouse Solid);
* when ``RBM_SHOPPING_LIST`` is set (``demo`` or trip index), runs the
  multi-trip fill-trolley-then-ship state machine for that trip's stops;
* else when ``RBM_PICK_SKU`` is set, runs the one-SKU pretend-pick machine;
* on pick: Supervisor-removes one ``WH_INV_*`` cube and raises
  ``WH_TROLLEY_LOAD`` onto the bed; on unload between trips, hides the load.

Shopping list off by default. One-SKU: ``RBM_PICK_SKU=demo``. Multi-trip:
``RBM_SHOPPING_LIST=demo`` (or ``1``..``N`` for a single trip) plus the matching
``RBM_ROUTE`` from ``tracks/warehouse.json`` missions ``shopping_list_trip_N``.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

try:
    from twin_publisher import project_env
except Exception:  # pragma: no cover
    def project_env(name, default=None):  # type: ignore[misc]
        return os.environ.get(name) or default

PICK_ENV = "RBM_PICK_SKU"
SHOP_ENV = "RBM_SHOPPING_LIST"
REPO_ROOT = Path(__file__).resolve().parents[3]



def shopping_list_from_env(env=None) -> str | None:
    """Return shopping-list mode raw value, or None when off.

    Values: ``demo`` / ``on`` / ``1`` = full demo plan (trip 1 for Webots CMD);
    ``2``..``N`` = that trip only; ``all`` = multi-trip state machine (unload
    between trips; continuous dock-return still deferred).
    """
    raw = (project_env(SHOP_ENV, "") if env is None else env.get(SHOP_ENV, "")) or ""
    raw = raw.strip()
    if not raw or raw.lower() in ("0", "false", "no", "off"):
        return None
    return raw


def load_shopping_plan(track_name: str = "warehouse") -> dict | None:
    path = REPO_ROOT / "tracks" / f"{track_name}.json"
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    plan = data.get("shopping_list")
    return plan if isinstance(plan, dict) else None


def resolve_shopping_trip(raw: str, plan: dict) -> tuple[int | None, str, bool]:
    """Map env → (0-based trip index or None for all, why, run_all).

    ``demo`` / ``on`` / ``true`` / ``yes`` → trip 0 (Webots one-launch smoke).
    ``all`` → every trip with unload between (state machine; Webots finish still
    ends the run after trip 1 unless Dan restarts).
    ``1``..``N`` → that trip only (1-based).
    """
    low = raw.strip().lower()
    trips = list(plan.get("trips") or [])
    if not trips:
        return None, f"{SHOP_ENV}={raw} but shopping_list has no trips", False
    if low in ("demo", "on", "true", "yes"):
        return 0, f"{SHOP_ENV}={raw} → trip 1/{len(trips)} (demo smoke)", False
    if low == "all":
        return 0, f"{SHOP_ENV}=all → {len(trips)} trips (unload between)", True
    try:
        n = int(low)
    except ValueError:
        return None, f"{SHOP_ENV}={raw} not demo/all/1..{len(trips)}", False
    if 1 <= n <= len(trips):
        return n - 1, f"{SHOP_ENV}={raw} → trip {n}/{len(trips)}", False
    return None, f"{SHOP_ENV}={raw} out of range 1..{len(trips)}", False


def _plan_from_json(data: dict):
    """Rebuild ShoppingPlan / MultiTripState inputs from tracks JSON."""
    from src.shopping_list import ShoppingLine, ShoppingPlan, TripPlan, TripStop

    lines = [ShoppingLine(sku=str(x["sku"]), qty=int(x.get("qty", 1))) for x in data.get("lines") or []]
    trips = []
    for t in data.get("trips") or []:
        stops = []
        for s in t.get("stops") or []:
            ax, ay = s["approach_xy"]
            stops.append(
                TripStop(
                    sku=str(s["sku"]),
                    qty=int(s.get("qty", 1)),
                    approach_xy=(float(ax), float(ay)),
                    aisle=int(s.get("aisle", 0)),
                    block=int(s.get("block", 0)),
                    side=str(s.get("side", "left")),
                )
            )
        trips.append(
            TripPlan(
                trip_index=int(t["trip_index"]),
                stops=stops,
                units=int(t.get("units", len(stops))),
                via=list(t.get("via") or []),
                route=t.get("route"),
                length_m=t.get("length_m"),
            )
        )
    return ShoppingPlan(
        lines=lines,
        trips=trips,
        capacity=int(data.get("capacity") or 2),
        order_mode=str(data.get("order_mode") or "aisle"),
        ship_exit=str(data.get("ship_exit") or "shipping_1"),
    )

def pick_sku_from_env(env=None) -> str | None:
    """Return the SKU to pick, or None when the feature is off."""
    raw = (project_env(PICK_ENV, "") if env is None else env.get(PICK_ENV, "")) or ""
    raw = raw.strip()
    if not raw or raw.lower() in ("0", "false", "no", "off"):
        return None
    return raw


def load_inventory_catalog(track_name: str = "warehouse") -> list[dict]:
    path = REPO_ROOT / "tracks" / f"{track_name}.json"
    if not path.is_file():
        return []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    return list(data.get("inventory") or [])


def resolve_sku(raw: str, catalog: list[dict]) -> tuple[str | None, str]:
    """Map env value → concrete SKU. ``demo`` / ``1`` / ``on`` → the demo group."""
    key = raw.strip()
    low = key.lower()
    if low in ("demo", "1", "true", "yes", "on"):
        for row in catalog:
            if row.get("demo"):
                return str(row["sku"]), f"{PICK_ENV}={raw} → demo {row['sku']}"
        if catalog:
            return str(catalog[0]["sku"]), f"{PICK_ENV}={raw} → first {catalog[0]['sku']}"
        return None, f"{PICK_ENV}={raw} but no inventory in track"
    want = key.upper()
    for row in catalog:
        if str(row.get("sku", "")).upper() == want:
            return str(row["sku"]), f"{PICK_ENV}={row['sku']}"
    return None, f"{PICK_ENV}={raw} not in inventory catalog"


class InventoryRuntime:
    """Trolley hitch + optional one-SKU pretend pick."""

    def __init__(self) -> None:
        from src.inventory_pick import TrolleyGeom

        self.robot = None
        self.trolley = None
        self.load_node = None
        self.geom = TrolleyGeom()
        self.catalog: list[dict] = []
        self.sku: str | None = None
        self.why = "off"
        self.pick = None  # PickState | MultiTripState | None
        self.multi = False  # True when running shopping-list MultiTripState
        self.shop_trip_i: int | None = None  # locked trip when not run_all
        self.cube_defs: list[str] = []
        self._sku_cubes: dict[str, list[str]] = {}
        self.removed: list[str] = []
        self.cargo_placed = False
        self.last_state = ""
        self.ok = False
        self._pick_holding = False  # True while we forced obstacle_v_cap=0 for pick
        self._pre_pick_cap = None  # obstacle_v_cap before we imposed the pick stop

    def enable(self, robot, track_name: str = "warehouse") -> list[str]:
        msgs: list[str] = []
        self.robot = robot
        self.catalog = load_inventory_catalog(track_name)
        self._sku_cubes = {
            str(r["sku"]): list(r.get("cube_defs") or []) for r in self.catalog if r.get("sku")
        }
        shop_raw = shopping_list_from_env()
        if shop_raw:
            plan_data = load_shopping_plan(track_name)
            if not plan_data:
                msgs.append(f"{SHOP_ENV}={shop_raw} but no shopping_list in track")
                self.why = msgs[-1]
            else:
                trip_i, why, run_all = resolve_shopping_trip(shop_raw, plan_data)
                self.why = why
                if trip_i is None:
                    msgs.append(why)
                else:
                    from src.shopping_list import MultiTripState

                    full = _plan_from_json(plan_data)
                    if not run_all:
                        # single-trip slice for Webots one-launch smoke / trip_N
                        t = full.trips[trip_i]
                        from src.shopping_list import ShoppingPlan

                        full = ShoppingPlan(
                            lines=full.lines,
                            trips=[t],
                            capacity=full.capacity,
                            order_mode=full.order_mode,
                            ship_exit=full.ship_exit,
                        )
                        # renumber to trip_index 1 for the slice
                        t.trip_index = 1
                        self.shop_trip_i = trip_i
                    self.pick = MultiTripState(plan=full)
                    self.multi = True
                    # cube defs for first stop
                    stop = self.pick.current_stop
                    if stop is not None:
                        self.sku = stop.sku
                        self.cube_defs = list(self._sku_cubes.get(stop.sku) or [])
        else:
            raw = pick_sku_from_env()
            if raw:
                sku, why = resolve_sku(raw, self.catalog)
                self.sku, self.why = sku, why
                if sku is None:
                    msgs.append(why)
                else:
                    row = next((r for r in self.catalog if r["sku"] == sku), None)
                    if row is None:
                        msgs.append(f"inventory row missing for {sku}")
                    else:
                        from src.inventory_pick import PickState

                        ax, ay = row["approach_xy"]
                        self.cube_defs = list(row.get("cube_defs") or [])
                        self.pick = PickState(sku=sku, approach_xy=(float(ax), float(ay)))
            else:
                self.why = (
                    f"{SHOP_ENV}/{PICK_ENV} unset: trolley follow only (no pretend pick)"
                )

        try:
            self.trolley = robot.getFromDef("WH_TROLLEY") if robot is not None else None
        except Exception as exc:
            self.trolley = None
            msgs.append(f"WH_TROLLEY: {type(exc).__name__}: {exc}")
        try:
            self.load_node = robot.getFromDef("WH_TROLLEY_LOAD") if robot is not None else None
        except Exception:
            self.load_node = None

        if self.trolley is None:
            msgs.append("WH_TROLLEY not in this world — trolley follow disabled")
            self.ok = False
        else:
            self.ok = True
            # TrolleyGeom defaults match WarehouseSpec (chassis 0.36 + tongue + half bed).
            # Re-derived here only if the catalog carries hitch_behind_m later.
        return msgs

    def describe(self) -> str:
        hitch = self.geom.hitch_behind_m
        if self.pick is not None and self.multi:
            stop = getattr(self.pick, "current_stop", None)
            ap = stop.approach_xy if stop else None
            return (f"INVENTORY shopping — trolley hitch {hitch:.2f} m; {self.why}; "
                    f"approach {ap}")
        if self.pick is not None:
            return (f"INVENTORY on — trolley hitch {hitch:.2f} m; pretend pick {self.why}; "
                    f"approach {self.pick.approach_xy}")
        return f"INVENTORY trolley follow — hitch {hitch:.2f} m ({self.why})"

    # ---- per-frame ----------------------------------------------------------
    def follow(self, gps_xy, yaw: float) -> None:
        """Hitch the trolley behind the robot (call every controller tick)."""
        if self.ok:
            self._follow_trolley(gps_xy, yaw)

    def update(self, gps_xy, yaw: float, dt: float, ctl=None) -> None:
        """Pretend-pick state machine; may set ctl.obstacle_v_cap = 0 while stopped.

        Call this *after* obstacle apply so a pick stop is not overwritten by CLEAR.
        On resume (cmd.v_cap is None) we **release** the stop via
        :func:`apply_pick_speed_cap` — leaving 0 sticky stranded the bot mid-aisle.
        """
        from src.inventory_pick import apply_pick_speed_cap

        if self.ok:
            self._follow_trolley(gps_xy, yaw)
        if self.pick is None or ctl is None:
            return
        cmd = self.pick.update(gps_xy, dt)
        if cmd.state != self.last_state:
            print(f"PICK {self.last_state or 'START'} -> {cmd.state}"
                  + (f" ({cmd.note})" if cmd.note else ""))
            self.last_state = cmd.state
        # shopping: refresh cube_defs when the active SKU changes
        if self.multi and cmd.sku and cmd.sku != self.sku:
            self.sku = cmd.sku
            self.cube_defs = list(self._sku_cubes.get(cmd.sku) or [])
        cur = getattr(ctl, "obstacle_v_cap", None)
        new_cap, self._pick_holding, self._pre_pick_cap = apply_pick_speed_cap(
            cmd.v_cap, cur, holding=self._pick_holding, saved_cap=self._pre_pick_cap
        )
        ctl.obstacle_v_cap = new_cap
        if cmd.pick_now:
            self._do_pick()
        if self.multi and getattr(self.pick, "clear_load", False):
            self._hide_cargo()

    def mark_shipped(self) -> None:
        if self.pick is None:
            return
        if self.multi:
            cmd = self.pick.mark_shipped()
            print(f"PICK -> {cmd.state} ({cmd.note})")
            self.last_state = cmd.state
            # one-shot unload hide; MultiTripState.clear_load fires on next ticks too
            self._hide_cargo()
            self._pick_holding = False
            self._pre_pick_cap = None
            return
        self.pick.mark_shipped()
        if self.last_state != "DONE":
            print("PICK -> DONE (shipping exit)")
            self.last_state = "DONE"
        self._pick_holding = False
        self._pre_pick_cap = None

    def fields(self) -> dict:
        """CSV / HUD extras (empty strings when idle)."""
        if self.pick is None:
            return {"pick_state": "", "pick_sku": "", "pick_dist_m": ""}
        if self.multi:
            stop = getattr(self.pick, "current_stop", None)
            return {
                "pick_state": self.pick.state,
                "pick_sku": stop.sku if stop else "",
                "pick_dist_m": "",
            }
        return {
            "pick_state": self.pick.state,
            "pick_sku": self.pick.sku,
            "pick_dist_m": "",
        }

    # ---- supervisor actions -------------------------------------------------
    def _follow_trolley(self, gps_xy, yaw: float) -> None:
        if self.trolley is None or gps_xy is None:
            return
        try:
            x, y, _ = self.geom.world_pose((float(gps_xy[0]), float(gps_xy[1])), float(yaw))
            z = self.geom.bed_h_m / 2.0
            field = self.trolley.getField("translation")
            if field is not None:
                field.setSFVec3f([x, y, z])
            rot = self.trolley.getField("rotation")
            if rot is not None:
                rot.setSFRotation([0.0, 0.0, 1.0, float(yaw)])
        except Exception:
            pass

    def _do_pick(self) -> None:
        """Remove one remaining cube and show cargo on the trolley bed."""
        name = None
        for def_name in self.cube_defs:
            if def_name in self.removed:
                continue
            try:
                node = self.robot.getFromDef(def_name) if self.robot is not None else None
            except Exception:
                node = None
            if node is None:
                continue
            try:
                node.remove()
                self.removed.append(def_name)
                name = def_name
                break
            except Exception as exc:
                print(f"PICK: could not remove {def_name} ({type(exc).__name__}: {exc})")
                return
        if name is None:
            print("PICK: no inventory cube left to remove")
        else:
            print(f"PICK: removed {name} (pretend pick onto trolley)")
        self._show_cargo()

    def _show_cargo(self) -> None:
        if self.cargo_placed or self.load_node is None:
            return
        try:
            # bed top relative to trolley solid centre (centre at bed_h/2)
            z = self.geom.bed_h_m / 2.0 + 0.14
            field = self.load_node.getField("translation")
            if field is not None:
                field.setSFVec3f([0.0, 0.0, z])
            self.cargo_placed = True
            print("PICK: WH_TROLLEY_LOAD raised onto bed")
        except Exception as exc:
            print(f"PICK: could not place cargo ({type(exc).__name__}: {exc})")

    def _hide_cargo(self) -> None:
        """Clear trolley load between trips (reset bed visual)."""
        if self.load_node is None:
            self.cargo_placed = False
            return
        try:
            field = self.load_node.getField("translation")
            if field is not None:
                # sink below bed (same hidden pose the world starts with: translation 0 0 -2)
                field.setSFVec3f([0.0, 0.0, -2.0])
            if self.cargo_placed:
                print("PICK: WH_TROLLEY_LOAD cleared (unload between trips)")
            self.cargo_placed = False
        except Exception as exc:
            print(f"PICK: could not clear cargo ({type(exc).__name__}: {exc})")
            self.cargo_placed = False
