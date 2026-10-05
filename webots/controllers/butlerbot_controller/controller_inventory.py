"""Inventory cubes + pull-behind trolley + one-SKU pretend pick (Webots glue).

Pure planning lives in ``src/inventory_pick.py``. This module:

* follows ``DEF WH_TROLLEY`` behind ButlerBot each frame (Robot block stays
  byte-identical across worlds — the trolley is a warehouse Solid);
* when ``RBM_PICK_SKU`` is set, runs the pretend-pick state machine and stops
  the bot at the SKU approach point (via ``obstacle_v_cap = 0``);
* on pick: Supervisor-removes one ``WH_INV_*`` cube and raises
  ``WH_TROLLEY_LOAD`` onto the bed.

Off by default. Enable with ``RBM_PICK_SKU=<sku>`` or ``RBM_PICK_SKU=demo``
(uses the demo group from ``tracks/warehouse.json`` inventory).
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
REPO_ROOT = Path(__file__).resolve().parents[3]


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
        self.pick = None  # PickState | None
        self.cube_defs: list[str] = []
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
            self.why = f"{PICK_ENV} unset: trolley follow only (no pretend pick)"

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
        cur = getattr(ctl, "obstacle_v_cap", None)
        new_cap, self._pick_holding, self._pre_pick_cap = apply_pick_speed_cap(
            cmd.v_cap, cur, holding=self._pick_holding, saved_cap=self._pre_pick_cap
        )
        ctl.obstacle_v_cap = new_cap
        if cmd.pick_now:
            self._do_pick()

    def mark_shipped(self) -> None:
        if self.pick is not None:
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
        dist = None
        # last command distance is not stored; leave blank in fields helper
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
