"""Placeholder inventory + one-SKU pretend-pick planning (pure, no Webots).

Vertical slice after radar: locate one shelf SKU group, stop beside it, pretend
to pick (caller removes / hides a cube and optionally shows trolley cargo), then
continue to shipping. Multi-trip shopping lists live in ``src/shopping_list.py``.

Geometry is derived from a warehouse layout (aisle centres, rack blocks, rack
depth, beam pitch) — nothing aisle- or bay-hardcoded beyond which demo groups
to seed. The controller / builder call into this module; unit tests cover the
math and the state machine without Webots.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# States for the one-SKU pretend-pick demo
ST_IDLE = "IDLE"
ST_TO_PICK = "DRIVING_TO_PICK"
ST_PICKING = "PICKING"
ST_TO_SHIP = "DRIVING_TO_SHIP"
ST_DONE = "DONE"

# Defaults derived from the wheeled chassis profile (chassis_length 0.36 m) +
# a short tongue; the controller may override hitch from the live robot.
DEFAULT_CHASSIS_LENGTH_M = 0.36
DEFAULT_TONGUE_M = 0.28
DEFAULT_HITCH_CLEAR_M = 0.15  # extra gap axle-rear → tongue tip so the cart never kisses the body
DEFAULT_TROLLEY_LENGTH_M = 0.55
DEFAULT_TROLLEY_WIDTH_M = 0.40
DEFAULT_TROLLEY_BED_H_M = 0.18
DEFAULT_TROLLEY_RAIL_H_M = 0.32
DEFAULT_CUBE_M = (0.28, 0.28, 0.28)  # placeholder tote / carton
DEFAULT_CUBES_PER_GROUP = 4
DEFAULT_PICK_RADIUS_M = 1.10  # stop when this close to the approach point on the lane
DEFAULT_PICK_DWELL_S = 2.0  # hold still while the pretend pick runs
DEFAULT_APPROACH_ALONG_M = 0.0  # pick stop at the group's lane x (centre of the bay cluster)


@dataclass(frozen=True)
class InventoryCube:
    """One placeholder cube on a rack face (world frame, metres)."""

    def_name: str  # Webots DEF, e.g. WH_INV_SKU_A05_B0_0
    sku: str
    cx: float
    cy: float
    cz: float  # centre z (floor = 0)
    sx: float
    sy: float
    sz: float
    index: int


@dataclass(frozen=True)
class InventoryGroup:
    """Cubes that share one SKU, grouped by aisle + rack block + face side."""

    sku: str
    aisle: int
    block: int
    side: str  # "left" (+y of the aisle) or "right" (-y)
    approach_xy: tuple[float, float]  # on the aisle centre line, where the bot stops to pick
    cubes: tuple[InventoryCube, ...]
    meta: dict = field(default_factory=dict)

    @property
    def n_cubes(self) -> int:
        return len(self.cubes)

    @property
    def centroid_xy(self) -> tuple[float, float]:
        if not self.cubes:
            return self.approach_xy
        return (
            sum(c.cx for c in self.cubes) / len(self.cubes),
            sum(c.cy for c in self.cubes) / len(self.cubes),
        )


@dataclass
class TrolleyGeom:
    """Pull-behind trolley dimensions (metres), derived from the chassis + tongue."""

    chassis_length_m: float = DEFAULT_CHASSIS_LENGTH_M
    tongue_m: float = DEFAULT_TONGUE_M
    hitch_clear_m: float = DEFAULT_HITCH_CLEAR_M
    length_m: float = DEFAULT_TROLLEY_LENGTH_M
    width_m: float = DEFAULT_TROLLEY_WIDTH_M
    bed_h_m: float = DEFAULT_TROLLEY_BED_H_M
    rail_h_m: float = DEFAULT_TROLLEY_RAIL_H_M

    @property
    def hitch_behind_m(self) -> float:
        """Axle → trolley bed centre, along −x in the robot frame."""
        return (self.chassis_length_m / 2.0 + self.hitch_clear_m + self.tongue_m + self.length_m / 2.0)

    def world_pose(self, robot_xy: tuple[float, float], yaw: float) -> tuple[float, float, float]:
        """Trolley bed centre (x, y, yaw) behind the robot, same heading."""
        hb = self.hitch_behind_m
        x = robot_xy[0] - hb * math.cos(yaw)
        y = robot_xy[1] - hb * math.sin(yaw)
        return (x, y, yaw)


def sku_id(aisle: int, block: int, side: str = "left") -> str:
    """Stable SKU label from aisle / block / face (source of truth for groups)."""
    side_c = "L" if side == "left" else "R"
    return f"SKU-A{aisle:02d}-B{block}-{side_c}"


def demo_group_specs(layout) -> list[dict]:
    """Which placeholder groups to seed. Derived from the live aisle / shipping layout.

    Primary demo SKU sits on the odd aisle used by ``odd_aisle_to_shipping`` so
    one existing RBM_ROUTE already drives past it. Extra groups span other taped
    inner aisles / rack blocks for the multi-trip shopping list.
    """
    n = layout.n_aisles - 1
    odds = [
        i
        for i in range(1, n)
        if (i - layout.sp.main_aisle) % 2 and i in layout.taped and i != layout.ship_aisle
    ]
    odd = odds[len(odds) // 2] if odds else layout.ship_aisle
    specs: list[dict] = [{"aisle": odd, "block": 0, "side": "left", "demo": True}]
    seen = {(odd, 0, "left")}
    candidates = [
        i
        for i in layout.taped
        if i not in (0, layout.n_aisles - 1, layout.sp.main_aisle, layout.ship_aisle)
    ]
    n_blocks = len(layout.rack_blocks)
    for i, aisle in enumerate(candidates):
        block = i % n_blocks
        key = (aisle, block, "left")
        if key in seen:
            block = (block + 1) % n_blocks
            key = (aisle, block, "left")
        if key in seen:
            continue
        seen.add(key)
        specs.append({"aisle": aisle, "block": block, "side": "left", "demo": False})
        if len(specs) >= 7:
            break
    return specs


def build_inventory_groups(
    layout,
    *,
    cube_m: tuple[float, float, float] = DEFAULT_CUBE_M,
    cubes_per_group: int = DEFAULT_CUBES_PER_GROUP,
    specs: list[dict] | None = None,
) -> list[InventoryGroup]:
    """Place placeholder cubes on rack faces from the layout (no hard-coded x/y).

    Cubes sit on the lowest beam level, inset from the aisle face into the rack
    so they stay off the taped lane (lane-clearance tests still pass).
    """
    sp = layout.sp
    sx, sy, sz = cube_m
    beam_z0 = sp.rack_h_m / sp.beam_levels  # top of first goods shelf ≈ first beam
    # sit the cube on that shelf (centre z)
    cz = beam_z0 + sz / 2.0
    face_inset = sy / 2.0 + 0.05  # cube centre this far into the rack from the face
    aisle_half = sp.aisle_w_m / 2.0
    groups: list[InventoryGroup] = []
    for spec in specs if specs is not None else demo_group_specs(layout):
        aisle = int(spec["aisle"])
        block = int(spec["block"])
        side = spec.get("side", "left")
        if not (0 <= aisle < layout.n_aisles) or not (0 <= block < len(layout.rack_blocks)):
            raise ValueError(f"bad inventory spec {spec}")
        xa, xb = layout.rack_blocks[block]
        ay = layout.aisle_y[aisle]
        # face y: left of aisle = +y face at ay + aisle_half; right = ay - aisle_half
        if side == "left":
            face_y = ay + aisle_half
            cy = face_y + face_inset
        else:
            face_y = ay - aisle_half
            cy = face_y - face_inset
        # cluster cubes along the bay (x) around the block centre, gap between them
        span = min(xb - xa - 0.6, cubes_per_group * (sx + 0.08))
        x0 = (xa + xb) / 2.0 - span / 2.0 + sx / 2.0
        step = span / max(1, cubes_per_group - 1) if cubes_per_group > 1 else 0.0
        sku = sku_id(aisle, block, side)
        cubes = []
        for k in range(cubes_per_group):
            cx = x0 + k * step
            cubes.append(
                InventoryCube(
                    def_name=f"WH_INV_{sku.replace('-', '_')}_{k}",
                    sku=sku,
                    cx=round(cx, 3),
                    cy=round(cy, 3),
                    cz=round(cz, 3),
                    sx=sx,
                    sy=sy,
                    sz=sz,
                    index=k,
                )
            )
        approach_x = round((xa + xb) / 2.0 + DEFAULT_APPROACH_ALONG_M, 3)
        groups.append(
            InventoryGroup(
                sku=sku,
                aisle=aisle,
                block=block,
                side=side,
                approach_xy=(approach_x, round(ay, 3)),
                cubes=tuple(cubes),
                meta={
                    "demo": bool(spec.get("demo")),
                    "face_y": round(face_y, 3),
                    "beam_z0": round(beam_z0, 3),
                    "mission": "pick_one_sku_to_shipping" if spec.get("demo") else None,
                },
            )
        )
    return groups


def inventory_to_json(groups: list[InventoryGroup]) -> list[dict]:
    """Serializable catalog written into tracks/warehouse.json."""
    out = []
    for g in groups:
        out.append(
            {
                "sku": g.sku,
                "aisle": g.aisle,
                "block": g.block,
                "side": g.side,
                "n_cubes": g.n_cubes,
                "approach_xy": [g.approach_xy[0], g.approach_xy[1]],
                "centroid_xy": [round(g.centroid_xy[0], 3), round(g.centroid_xy[1], 3)],
                "cube_defs": [c.def_name for c in g.cubes],
                "demo": bool(g.meta.get("demo")),
                "mission": g.meta.get("mission"),
            }
        )
    return out


def find_group(groups: list[InventoryGroup], sku: str) -> InventoryGroup | None:
    want = sku.strip().upper()
    for g in groups:
        if g.sku.upper() == want:
            return g
    return None


def demo_sku(groups: list[InventoryGroup]) -> str | None:
    for g in groups:
        if g.meta.get("demo"):
            return g.sku
    return groups[0].sku if groups else None


@dataclass
class PickCommand:
    """What the controller should do this frame for the pretend-pick demo."""

    state: str
    v_cap: float | None  # None = do not override; 0 = stop for pick
    pick_now: bool = False  # fire the remove / hide + place-on-trolley once
    sku: str = ""
    note: str = ""
    approach_xy: tuple[float, float] | None = None
    dist_to_approach_m: float | None = None


def apply_pick_speed_cap(
    cmd_v_cap: float | None,
    ctl_v_cap: float | None,
    *,
    holding: bool,
    saved_cap: float | None,
) -> tuple[float | None, bool, float | None]:
    """Apply / release a pretend-pick stop on ``obstacle_v_cap``.

    Returns ``(new_ctl_v_cap, holding, saved_cap)``.

    While ``cmd_v_cap`` is 0 we force a stop (remembering the prior cap). When
    the pick machine returns ``cmd_v_cap is None`` again we **restore** the
    saved cap — leaving 0 sticky was stranding the bot mid-aisle forever
    (obstacles off → nothing else clears ``obstacle_v_cap``).
    """
    if cmd_v_cap is not None:
        if not holding:
            saved_cap = ctl_v_cap
        if ctl_v_cap is None:
            return float(cmd_v_cap), True, saved_cap
        return min(float(ctl_v_cap), float(cmd_v_cap)), True, saved_cap
    if holding:
        return saved_cap, False, None
    return ctl_v_cap, False, saved_cap


@dataclass
class PickState:
    """One-SKU pretend-pick state machine (pure)."""

    sku: str
    approach_xy: tuple[float, float]
    pick_radius_m: float = DEFAULT_PICK_RADIUS_M
    pick_dwell_s: float = DEFAULT_PICK_DWELL_S
    state: str = ST_TO_PICK
    dwell_s: float = 0.0
    picked: bool = False
    pick_fired: bool = False

    def update(self, robot_xy: tuple[float, float] | None, dt: float) -> PickCommand:
        if robot_xy is None:
            return PickCommand(state=self.state, v_cap=None, sku=self.sku, note="no gps")
        dx = robot_xy[0] - self.approach_xy[0]
        dy = robot_xy[1] - self.approach_xy[1]
        dist = math.hypot(dx, dy)
        if self.state == ST_TO_PICK:
            if dist <= self.pick_radius_m:
                self.state = ST_PICKING
                self.dwell_s = 0.0
                return PickCommand(
                    state=self.state,
                    v_cap=0.0,
                    sku=self.sku,
                    note="arrived at pick",
                    approach_xy=self.approach_xy,
                    dist_to_approach_m=dist,
                )
            return PickCommand(
                state=self.state,
                v_cap=None,
                sku=self.sku,
                note="driving to pick",
                approach_xy=self.approach_xy,
                dist_to_approach_m=dist,
            )
        if self.state == ST_PICKING:
            self.dwell_s += max(0.0, float(dt))
            fire = False
            if not self.pick_fired and self.dwell_s >= self.pick_dwell_s * 0.4:
                # fire once mid-dwell so the stop is visible before / after
                fire = True
                self.pick_fired = True
                self.picked = True
            # Never hold longer than 2x dwell (resume even if a tick was missed)
            hard_deadline = max(self.pick_dwell_s * 2.0, self.pick_dwell_s + 1.0)
            if self.dwell_s >= self.pick_dwell_s or self.dwell_s >= hard_deadline:
                if not self.pick_fired:
                    fire = True
                    self.pick_fired = True
                    self.picked = True
                self.state = ST_TO_SHIP
                return PickCommand(
                    state=self.state,
                    v_cap=None,  # caller MUST release the sticky stop (see apply_pick_speed_cap)
                    pick_now=fire,
                    sku=self.sku,
                    note="pick done, resume to shipping",
                    approach_xy=self.approach_xy,
                    dist_to_approach_m=dist,
                )
            return PickCommand(
                state=self.state,
                v_cap=0.0,
                pick_now=fire,
                sku=self.sku,
                note="picking",
                approach_xy=self.approach_xy,
                dist_to_approach_m=dist,
            )
        if self.state == ST_TO_SHIP:

            return PickCommand(
                state=self.state,
                v_cap=None,
                sku=self.sku,
                note="driving to shipping",
                approach_xy=self.approach_xy,
                dist_to_approach_m=dist,
            )
        return PickCommand(state=self.state, v_cap=None, sku=self.sku, note=self.state)

    def mark_shipped(self) -> None:
        self.state = ST_DONE


def make_pick_state(group: InventoryGroup, **kwargs) -> PickState:
    return PickState(sku=group.sku, approach_xy=group.approach_xy, **kwargs)
