"""Multi-trip shopping list: fill trolley capacity, ship, repeat.

Builds on one-SKU pretend pick + hitch trolley (``src/inventory_pick.py``).
This module is pure (no Webots): pack a shopping list into 3–4 trips that fill
the trolley before shipping, order picks with a simple route heuristic, and
drive a multi-trip state machine that clears the trolley load between trips.

Webots glue lives in ``controller_inventory.py``. Continuous multi-trip in one
Webots session (dock-return without finish) is deferred — CMD runs one trip
per launch using the per-trip ``RBM_ROUTE`` from the plan.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.inventory_pick import (
    DEFAULT_CUBE_M,
    DEFAULT_PICK_DWELL_S,
    DEFAULT_PICK_RADIUS_M,
    DEFAULT_TROLLEY_LENGTH_M,
    DEFAULT_TROLLEY_WIDTH_M,
    InventoryGroup,
    PickCommand,
    ST_DONE,
    ST_PICKING,
    ST_TO_PICK,
    ST_TO_SHIP,
    TrolleyGeom,
    find_group,
)

# Multi-trip extras (one-SKU states reused for the active stop)
ST_UNLOADING = "UNLOADING"
ST_IDLE = "IDLE"

ORDER_AISLE = "aisle"  # sort by aisle, then block, then sku
ORDER_NEAREST = "nearest"  # nearest-neighbour from start / last pick

# Floor packing: bed 0.55×0.40 / cube 0.28×0.28 ≈ 2. Derived, not magic.
DEFAULT_TROLLEY_CAPACITY = max(
    1,
    int(
        (DEFAULT_TROLLEY_LENGTH_M * DEFAULT_TROLLEY_WIDTH_M)
        // (DEFAULT_CUBE_M[0] * DEFAULT_CUBE_M[1])
    ),
)

ASSUMPTIONS = (
    "Trolley capacity = floor cubes that fit on the bed (length×width / cube "
    "footprint), currently 2 — no stacking / no mass limit.",
    "Each shopping line is one pretend-pick cube removal (qty counts cubes).",
    "Trip packing is first-fit after visit ordering: fill to capacity, then ship.",
    "Visit order default = aisle ascending, then block, then SKU (warehouse "
    "sweep). Optional nearest-neighbour from start_xy / last approach.",
    "Per-trip RBM_ROUTE is planned once from receiving start → via approaches "
    "→ shipping_1 (RoadGraph). No live re-route around blocked aisles.",
    "Webots: one trip per launch (finish at SHP ends the run). Unload/reset "
    "between trips is modelled in the state machine for offline / future "
    "continuous dock-return.",
    "Ultrasound and path-around remain deferred. BoM held.",
)


def trolley_capacity_units(
    geom: TrolleyGeom | None = None,
    cube_m: tuple[float, float, float] = DEFAULT_CUBE_M,
) -> int:
    """Derive integer floor capacity from trolley bed × cube footprint."""
    g = geom or TrolleyGeom()
    bed = max(1e-9, g.length_m * g.width_m)
    foot = max(1e-9, cube_m[0] * cube_m[1])
    return max(1, int(bed // foot))


def shopping_group_specs(layout) -> list[dict]:
    """Same seed as ``demo_group_specs`` (several aisle groups for multi-trip)."""
    from src.inventory_pick import demo_group_specs

    return demo_group_specs(layout)


@dataclass(frozen=True)
class ShoppingLine:
    sku: str
    qty: int = 1


@dataclass(frozen=True)
class TripStop:
    sku: str
    qty: int
    approach_xy: tuple[float, float]
    aisle: int
    block: int
    side: str = "left"


@dataclass
class TripPlan:
    trip_index: int  # 1-based
    stops: list[TripStop]
    units: int
    ship_after: bool = True
    route: str | None = None
    length_m: float | None = None
    via: list[list[float]] = field(default_factory=list)

    @property
    def skus(self) -> list[str]:
        return [s.sku for s in self.stops]


@dataclass
class ShoppingPlan:
    lines: list[ShoppingLine]
    trips: list[TripPlan]
    capacity: int
    order_mode: str
    assumptions: tuple[str, ...] = ASSUMPTIONS
    ship_exit: str = "shipping_1"
    start_xy: tuple[float, float] | None = None

    @property
    def total_units(self) -> int:
        return sum(max(0, int(ln.qty)) for ln in self.lines)

    @property
    def n_trips(self) -> int:
        return len(self.trips)


def demo_shopping_lines(groups: list[InventoryGroup]) -> list[ShoppingLine]:
    """Default list: one cube from each seeded group (≈7 → 4 trips at cap 2)."""
    # demo SKU first, then aisle order for the rest
    demo = [g for g in groups if g.meta.get("demo")]
    rest = sorted(
        (g for g in groups if not g.meta.get("demo")),
        key=lambda g: (g.aisle, g.block, g.sku),
    )
    ordered = demo + rest
    return [ShoppingLine(sku=g.sku, qty=1) for g in ordered]


def _expand_units(lines: list[ShoppingLine], groups: list[InventoryGroup]) -> list[TripStop]:
    """Expand qty into unit stops; drop unknown SKUs."""
    out: list[TripStop] = []
    for ln in lines:
        g = find_group(groups, ln.sku)
        if g is None:
            raise ValueError(f"shopping list SKU not in inventory: {ln.sku}")
        q = max(0, int(ln.qty))
        for _ in range(q):
            out.append(
                TripStop(
                    sku=g.sku,
                    qty=1,
                    approach_xy=g.approach_xy,
                    aisle=g.aisle,
                    block=g.block,
                    side=g.side,
                )
            )
    return out


def _order_stops(
    stops: list[TripStop],
    mode: str,
    start_xy: tuple[float, float] | None,
) -> list[TripStop]:
    if not stops:
        return []
    if mode == ORDER_NEAREST:
        remaining = list(stops)
        cur = start_xy if start_xy is not None else remaining[0].approach_xy
        ordered: list[TripStop] = []
        while remaining:
            i = min(
                range(len(remaining)),
                key=lambda k: math.hypot(
                    remaining[k].approach_xy[0] - cur[0],
                    remaining[k].approach_xy[1] - cur[1],
                ),
            )
            nxt = remaining.pop(i)
            ordered.append(nxt)
            cur = nxt.approach_xy
        return ordered
    # aisle order (default)
    return sorted(stops, key=lambda s: (s.aisle, s.block, s.sku, s.approach_xy[0]))


def _pack_trips(stops: list[TripStop], capacity: int) -> list[TripPlan]:
    cap = max(1, int(capacity))
    trips: list[TripPlan] = []
    bucket: list[TripStop] = []
    for stop in stops:
        if len(bucket) >= cap:
            trips.append(
                TripPlan(
                    trip_index=len(trips) + 1,
                    stops=list(bucket),
                    units=len(bucket),
                    via=[[s.approach_xy[0], s.approach_xy[1]] for s in bucket],
                )
            )
            bucket = []
        bucket.append(stop)
    if bucket:
        trips.append(
            TripPlan(
                trip_index=len(trips) + 1,
                stops=list(bucket),
                units=len(bucket),
                via=[[s.approach_xy[0], s.approach_xy[1]] for s in bucket],
            )
        )
    return trips


def plan_shopping_trips(
    lines: list[ShoppingLine],
    groups: list[InventoryGroup],
    *,
    capacity: int | None = None,
    order: str = ORDER_AISLE,
    start_xy: tuple[float, float] | None = None,
    ship_exit: str = "shipping_1",
) -> ShoppingPlan:
    """Pack lines into capacity-limited trips with a simple visit order."""
    cap = trolley_capacity_units() if capacity is None else max(1, int(capacity))
    mode = order if order in (ORDER_AISLE, ORDER_NEAREST) else ORDER_AISLE
    units = _expand_units(lines, groups)
    ordered = _order_stops(units, mode, start_xy)
    trips = _pack_trips(ordered, cap)
    return ShoppingPlan(
        lines=list(lines),
        trips=trips,
        capacity=cap,
        order_mode=mode,
        ship_exit=ship_exit,
        start_xy=start_xy,
    )


def attach_trip_routes(
    plan: ShoppingPlan,
    road_graph,
    *,
    to_exit: str | None = None,
) -> ShoppingPlan:
    """Fill each trip's ``route`` / ``length_m`` via ``RoadGraph.plan`` (mutates trips)."""
    exit_name = to_exit or plan.ship_exit
    for trip in plan.trips:
        via = [tuple(v) for v in trip.via]
        p = road_graph.plan(to_exit=exit_name, via=via)
        if getattr(p, "ok", True) and p.route:
            trip.route = p.route
            trip.length_m = float(p.length_m)
        else:
            trip.route = None
            trip.length_m = None
    return plan


def shopping_efficiency_metrics(
    plan: ShoppingPlan,
    *,
    battery_wh_start: float | None = None,
    elapsed_s: float | None = None,
) -> dict:
    """Light efficiency stub: trip/unit counts + optional path / battery / time.

    Dashboard wiring can later surface these keys; for now callers / docs use
    the dict. ``battery_wh_per_m_stub`` is a placeholder constant, not measured.
    """
    path_m = sum(t.length_m or 0.0 for t in plan.trips)
    # stub: ~12 Wh/km wheeled cruise guess — not from twin telemetry
    wh_per_m = 0.012
    est_wh = round(path_m * wh_per_m, 3) if path_m else None
    return {
        "kind": "shopping_efficiency_stub",
        "n_trips": plan.n_trips,
        "total_units": plan.total_units,
        "capacity": plan.capacity,
        "order_mode": plan.order_mode,
        "path_m": round(path_m, 2) if path_m else None,
        "est_drive_wh": est_wh,
        "battery_wh_start": battery_wh_start,
        "elapsed_s": elapsed_s,
        "units_per_trip": [t.units for t in plan.trips],
        "note": (
            "Stub metrics only — wire dashboard battery/time when twin path "
            "lengths + SoC are logged per shopping trip. Ultrasound deferred."
        ),
    }


def plan_to_json(plan: ShoppingPlan) -> dict:
    return {
        "capacity": plan.capacity,
        "order_mode": plan.order_mode,
        "ship_exit": plan.ship_exit,
        "n_trips": plan.n_trips,
        "total_units": plan.total_units,
        "lines": [{"sku": ln.sku, "qty": ln.qty} for ln in plan.lines],
        "trips": [
            {
                "trip_index": t.trip_index,
                "units": t.units,
                "skus": t.skus,
                "via": t.via,
                "route": t.route,
                "length_m": t.length_m,
                "stops": [
                    {
                        "sku": s.sku,
                        "qty": s.qty,
                        "aisle": s.aisle,
                        "block": s.block,
                        "side": s.side,
                        "approach_xy": [s.approach_xy[0], s.approach_xy[1]],
                    }
                    for s in t.stops
                ],
            }
            for t in plan.trips
        ],
        "assumptions": list(plan.assumptions),
        "efficiency": shopping_efficiency_metrics(plan),
    }


@dataclass
class MultiTripState:
    """Fill trolley → ship → unload/reset → next trip (pure state machine).

    Reuses the one-SKU pick dwell geometry. ``load_units`` tracks cubes on the
    bed; ``clear_load`` is set when unloading so the controller can hide
    ``WH_TROLLEY_LOAD``.
    """

    plan: ShoppingPlan
    trip_i: int = 0  # 0-based
    stop_i: int = 0  # within trip, 0-based; counts completed picks via pick_fired
    load_units: int = 0
    state: str = ST_TO_PICK
    dwell_s: float = 0.0
    pick_fired: bool = False
    pick_radius_m: float = DEFAULT_PICK_RADIUS_M
    pick_dwell_s: float = DEFAULT_PICK_DWELL_S
    unload_dwell_s: float = 1.0
    units_picked_total: int = 0
    clear_load: bool = False  # one-shot: hide cargo between trips
    last_note: str = ""

    @property
    def current_trip(self) -> TripPlan | None:
        if 0 <= self.trip_i < len(self.plan.trips):
            return self.plan.trips[self.trip_i]
        return None

    @property
    def current_stop(self) -> TripStop | None:
        trip = self.current_trip
        if trip is None or not (0 <= self.stop_i < len(trip.stops)):
            return None
        return trip.stops[self.stop_i]

    def update(self, robot_xy: tuple[float, float] | None, dt: float) -> PickCommand:
        self.clear_load = False
        if self.state == ST_DONE or self.current_trip is None:
            self.state = ST_DONE
            return PickCommand(state=ST_DONE, v_cap=None, note="shopping done")

        if self.state == ST_UNLOADING:
            self.dwell_s += max(0.0, float(dt))
            if self.dwell_s >= self.unload_dwell_s:
                self.clear_load = True
                self.load_units = 0
                self.trip_i += 1
                self.stop_i = 0
                self.dwell_s = 0.0
                self.pick_fired = False
                if self.current_trip is None:
                    self.state = ST_DONE
                    return PickCommand(state=ST_DONE, v_cap=None, note="all trips shipped")
                self.state = ST_TO_PICK
                stop = self.current_stop
                return PickCommand(
                    state=self.state,
                    v_cap=None,
                    sku=stop.sku if stop else "",
                    note=f"trip {self.trip_i + 1} start",
                    approach_xy=stop.approach_xy if stop else None,
                )
            return PickCommand(
                state=ST_UNLOADING,
                v_cap=0.0,
                note="unloading at shipping",
            )

        stop = self.current_stop
        if stop is None:
            # trip stops exhausted → go ship
            if self.state != ST_TO_SHIP:
                self.state = ST_TO_SHIP
            return PickCommand(
                state=ST_TO_SHIP,
                v_cap=None,
                note=f"trip {self.trip_i + 1} to shipping (full)",
            )

        if robot_xy is None:
            return PickCommand(
                state=self.state,
                v_cap=None,
                sku=stop.sku,
                note="no gps",
                approach_xy=stop.approach_xy,
            )

        dx = robot_xy[0] - stop.approach_xy[0]
        dy = robot_xy[1] - stop.approach_xy[1]
        dist = math.hypot(dx, dy)

        if self.state == ST_TO_SHIP:
            return PickCommand(
                state=ST_TO_SHIP,
                v_cap=None,
                sku=stop.sku,
                note="driving to shipping",
                approach_xy=stop.approach_xy,
                dist_to_approach_m=dist,
            )

        if self.state == ST_TO_PICK:
            if dist <= self.pick_radius_m:
                self.state = ST_PICKING
                self.dwell_s = 0.0
                self.pick_fired = False
                return PickCommand(
                    state=ST_PICKING,
                    v_cap=0.0,
                    sku=stop.sku,
                    note=f"arrived trip {self.trip_i + 1} stop {self.stop_i + 1}",
                    approach_xy=stop.approach_xy,
                    dist_to_approach_m=dist,
                )
            return PickCommand(
                state=ST_TO_PICK,
                v_cap=None,
                sku=stop.sku,
                note=f"to pick {stop.sku}",
                approach_xy=stop.approach_xy,
                dist_to_approach_m=dist,
            )

        if self.state == ST_PICKING:
            self.dwell_s += max(0.0, float(dt))
            fire = False
            if not self.pick_fired and self.dwell_s >= self.pick_dwell_s * 0.4:
                fire = True
                self.pick_fired = True
                self.load_units += 1
                self.units_picked_total += 1
            hard = max(self.pick_dwell_s * 2.0, self.pick_dwell_s + 1.0)
            if self.dwell_s >= self.pick_dwell_s or self.dwell_s >= hard:
                if not self.pick_fired:
                    fire = True
                    self.pick_fired = True
                    self.load_units += 1
                    self.units_picked_total += 1
                # advance stop
                self.stop_i += 1
                self.dwell_s = 0.0
                self.pick_fired = False
                trip = self.current_trip
                if trip is not None and self.stop_i < len(trip.stops):
                    self.state = ST_TO_PICK
                    nxt = trip.stops[self.stop_i]
                    return PickCommand(
                        state=ST_TO_PICK,
                        v_cap=None,
                        pick_now=fire,
                        sku=nxt.sku,
                        note=f"picked → next {nxt.sku}",
                        approach_xy=nxt.approach_xy,
                        dist_to_approach_m=dist,
                    )
                self.state = ST_TO_SHIP
                return PickCommand(
                    state=ST_TO_SHIP,
                    v_cap=None,
                    pick_now=fire,
                    sku=stop.sku,
                    note="trip full → shipping",
                    approach_xy=stop.approach_xy,
                    dist_to_approach_m=dist,
                )
            return PickCommand(
                state=ST_PICKING,
                v_cap=0.0,
                pick_now=fire,
                sku=stop.sku,
                note="picking",
                approach_xy=stop.approach_xy,
                dist_to_approach_m=dist,
            )

        return PickCommand(state=self.state, v_cap=None, sku=stop.sku, note=self.state)

    def mark_shipped(self) -> PickCommand:
        """Call when the finish referee hits shipping for the active trip."""
        if self.state == ST_DONE:
            return PickCommand(state=ST_DONE, v_cap=None, note="already done")
        # begin unload / clear load; next update advances trip
        self.state = ST_UNLOADING
        self.dwell_s = 0.0
        self.clear_load = False
        return PickCommand(state=ST_UNLOADING, v_cap=0.0, note="shipped → unload")
