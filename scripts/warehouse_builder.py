#!/usr/bin/env python3
"""Prototype warehouse course: layout -> track JSON, Webots world, preview PNG.

A generic small-company warehouse (~70 x 65 m, under 50k sq ft): rack rows
with forklift aisles, front / middle / back cross aisles, and a shipping &
receiving area (dock doors, dock lane, staging). Taped yellow lanes (the
same 1.30 m lane / 6 cm stripe as every other course) follow the aisles and
form a road graph the existing controller already handles: lanes, sharp 90deg
corners, T's, plus crossings and a gap (a 3 m forklift crossing with no tape).

Everything is derived from :class:`WarehouseSpec` (sizes in metres); change
a number and regenerate. The robot start convention is kept: the robot
spawns at (0, 0) facing +x, which is just inside receiving door RCV-1, facing
into the building.

    python scripts/warehouse_builder.py                 # write all three
    python scripts/warehouse_builder.py --summary       # print the layout numbers only
    python scripts/warehouse_builder.py --preview out.png

Outputs:
    tracks/warehouse.json                    road graph (TrackNetwork format) + missions
    webots/worlds/butlerbot_warehouse.wbt    floor, walls, dock doors, racks, staging, track
    webots/worlds/butlerbot_warehouse_obstacles.wbt   the same + obstacle test props (WH_OBSTACLES)
    webots/worlds/textures/obstacle_speckle.png       grain texture of the props (stereo needs texture)
    /workspace/warehouse_preview.png         top-down preview (or --preview PATH)

Shelves, walls, doors and staging items are static Solids with a
boundingObject (no physics), so distance sensors / stereo / radar added
later see them. The clean world has an empty ``WH_OBSTACLES`` group; the
``_obstacles`` world fills it with three test props placed on known routes
(:meth:`Layout.obstacle_props`) and carries ``# OBSTACLES on`` so the
controller turns stereo obstacle sensing on there by default.

Placeholder inventory cubes (grouped by aisle / block SKU) live in
``DEF WH_INVENTORY``; a pull-behind trolley prop ``DEF WH_TROLLEY`` starts
behind the spawn and the controller keeps it hitched. See
``src/inventory_pick.py``, ``src/shopping_list.py``, ``docs/INVENTORY_TROLLEY.md``, and ``docs/SHOPPING_LIST.md``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

TRACK_JSON = os.path.join(ROOT, "tracks", "warehouse.json")
WORLD = os.path.join(ROOT, "webots", "worlds", "butlerbot_warehouse.wbt")
OBSTACLES_WORLD = os.path.join(ROOT, "webots", "worlds", "butlerbot_warehouse_obstacles.wbt")
PROP_TEXTURE = os.path.join(ROOT, "webots", "worlds", "textures", "obstacle_speckle.png")
PROP_TEXTURE_URL = "textures/obstacle_speckle.png"  # relative to the world file
PREVIEW = "/workspace/warehouse_preview.png"
SQFT_PER_M2 = 10.7639


@dataclass
class WarehouseSpec:
    # building (x = into the building from the dock wall, y = along the dock wall, + = left of the robot)
    width_m: float = 70.0  # along y (dock wall length)
    depth_m: float = 65.0  # along x
    dock_wall_x: float = -4.0  # robot starts 4 m inside RCV-1
    wall_h_m: float = 9.0
    wall_t_m: float = 0.3
    # racks: selective pallet racking, back-to-back doubles + one single row on each side wall
    rack_depth_m: float = 1.1
    flue_m: float = 0.2  # gap between back-to-back racks
    rack_h_m: float = 6.0
    beam_levels: int = 4  # beam levels above the floor (pitch rack_h / levels)
    bay_m: float = 2.7  # upright pitch
    aisle_w_m: float = 3.6  # clear between rack faces (reach / counterbalance forklift)
    min_wall_clear_m: float = 0.3  # side wall to the single rack (more if the width leaves room)
    cross_aisle_w_m: float = 4.0
    front_aisle_x: float = 18.0  # front cross aisle centre (staging area in front of it)
    n_mid_cross: int = 1  # middle cross aisles, evenly spaced
    back_wall_clear_m: float = 0.5  # back cross aisle edge to back wall
    # tape (same lane as the other courses)
    lane_w_m: float = 1.30
    stripe_w_m: float = 0.06
    main_aisle: int = 2  # aisle index (0 = rightmost) that holds the start lane at y = 0
    tape_every: int = 1  # tape every n-th aisle counted from the main aisle (+ both edge aisles); 1 = every aisle
    shipping_aisle: int | None = None  # taped aisle extended to shipping door SHP-1 (None: 2nd aisle in from the left wall)
    # shipping & receiving
    dock_lane_x: float = 4.0  # tape lane along the dock wall (dock apron in front of it)
    plaza_x: float = 10.0  # forklift crossing on the start lane (tape gap)
    plaza_w_m: float = 3.0
    plaza_len_m: float = 8.0
    door_w_m: float = 3.0
    door_h_m: float = 3.2
    door_pitch_m: float = 4.5
    receiving_doors: tuple = (-2, -1, 0, 1)  # door offsets (x pitch) from RCV-1 at y = 0
    shipping_doors: tuple = (-1, 0, 1, 2)  # offsets from SHP-1 (on the shipping aisle)
    exit_inset_m: float = 1.0  # exit bar this far inside the dock wall
    # main 3D view: top-down FOLLOW cam. Straight down over the robot, fixed world orientation (dock wall at the
    # bottom), Webots "Tracking Shot" (copies the robot's translation only). The building has no roof.
    follow_fov: float = 0.6  # Viewpoint fieldOfView (rad); Webots applies it to the wider side of the 3D window
    follow_span_m: float = 30.0  # floor shown across the wider side of the 3D window (a few aisles)
    follow_min_clear_m: float = 3.0  # eye at least this far above the tallest thing (the 9 m walls)
    # staging
    pallet_m: tuple = (1.2, 1.0)  # footprint x, y
    pallet_heights: tuple = (1.0, 1.2, 1.4, 1.1)
    table_m: tuple = (2.4, 0.9, 0.9)  # packing table x, y, height
    lane_clear_m: float = 0.9  # staging items keep this clear of a lane edge
    # obstacle test props (butlerbot_warehouse_obstacles.wbt only). Positions come from the layout:
    obs_box_m: tuple = (0.6, 0.5, 0.5)  # carton / tote stack x, y, height: on the odd-aisle route's aisle centre
    obs_table_intrude_m: float = 0.30  # packing-table edge this far from the dock-lane centre (inside the corridor)
    obs_table_at_m: float = 4.8  # ... starting this far along the dock lane past the start-lane crossing
    obs_shelf_m: tuple = (1.2, 0.6, 1.8)  # free-standing shelf unit x, y, height beside the start lane
    obs_shelf_gap_m: float = 0.10  # its corner this far OUTSIDE the lane edge (outside the corridor: pass)
    obs_shelf_x_m: float = 13.0  # its near corner x on the start lane (after the forklift crossing)
    # placeholder inventory (grouped cubes on rack faces) + pull-behind trolley prop
    inv_cube_m: tuple = (0.28, 0.28, 0.28)  # tote / carton placeholder
    inv_cubes_per_group: int = 4
    trolley_length_m: float = 0.55
    trolley_width_m: float = 0.40
    trolley_bed_h_m: float = 0.18
    trolley_rail_h_m: float = 0.32
    trolley_tongue_m: float = 0.28
    chassis_length_m: float = 0.36  # ButlerBot wheeled chassis (profile); hitch derives from this


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------


@dataclass
class Box:
    name: str
    kind: str  # rack / wall / door / pallet / table / header
    cx: float
    cy: float
    sx: float
    sy: float
    h: float
    z0: float = 0.0
    meta: dict = field(default_factory=dict)

    @property
    def rect(self):
        return (self.cx - self.sx / 2, self.cy - self.sy / 2, self.cx + self.sx / 2, self.cy + self.sy / 2)


class Layout:
    def __init__(self, sp: WarehouseSpec):
        self.sp = sp
        W, D = sp.width_m, sp.depth_m
        dbl = 2 * sp.rack_depth_m + sp.flue_m
        n_dbl = int(math.floor((W - 2 * sp.min_wall_clear_m - 2 * sp.rack_depth_m - sp.aisle_w_m) / (sp.aisle_w_m + dbl)))
        if n_dbl < 1:
            raise ValueError("building too narrow for one double rack row")
        self.n_aisles = n_dbl + 1
        used = 2 * sp.rack_depth_m + self.n_aisles * sp.aisle_w_m + n_dbl * dbl
        self.side_clear = (W - used) / 2.0
        pitch = sp.aisle_w_m + dbl
        y_local = [self.side_clear + sp.rack_depth_m + sp.aisle_w_m / 2 + i * pitch for i in range(self.n_aisles)]
        if not 0 <= sp.main_aisle < self.n_aisles:
            raise ValueError(f"main_aisle {sp.main_aisle} not in 0..{self.n_aisles - 1}")
        self.y0 = -y_local[sp.main_aisle]  # building right wall (min y)
        self.y1 = self.y0 + W
        self.aisle_y = [self.y0 + y for y in y_local]
        self.x0 = sp.dock_wall_x
        self.x1 = sp.dock_wall_x + D
        self.xF = sp.front_aisle_x
        self.xB = self.x1 - sp.back_wall_clear_m - sp.cross_aisle_w_m / 2
        rx0, rx1 = self.xF + sp.cross_aisle_w_m / 2, self.xB - sp.cross_aisle_w_m / 2
        blk = (rx1 - rx0 - sp.n_mid_cross * sp.cross_aisle_w_m) / (sp.n_mid_cross + 1)
        if blk < 2 * sp.bay_m:
            raise ValueError("building too shallow for the cross aisles")
        self.rack_blocks = [(rx0 + k * (blk + sp.cross_aisle_w_m), rx0 + k * (blk + sp.cross_aisle_w_m) + blk)
                            for k in range(sp.n_mid_cross + 1)]
        self.xM = [b[1] + sp.cross_aisle_w_m / 2 for b in self.rack_blocks[:-1]]
        n = self.n_aisles
        taped = {0, n - 1} | {i for i in range(n) if (i - sp.main_aisle) % sp.tape_every == 0}
        self.taped = sorted(taped)
        inner = [i for i in self.taped if i not in (0, n - 1, sp.main_aisle)]
        if sp.shipping_aisle is not None:
            self.ship_aisle = sp.shipping_aisle
        else:  # second aisle in from the left wall (SHP-1..4 stay put whatever tape_every is)
            self.ship_aisle = n - 3 if (n - 3) in inner else (inner[-1] if inner else n - 1)
        if self.ship_aisle in (0, n - 1, sp.main_aisle) or self.ship_aisle not in self.taped:
            raise ValueError("shipping_aisle must be a taped inner aisle (not the start or an edge aisle)")
        self.ship_y = self.aisle_y[self.ship_aisle]
        self.doors = ([("RCV-%d" % (k + 1), "receiving", 0.0 + o * sp.door_pitch_m)
                       for k, o in enumerate(sorted(sp.receiving_doors, key=lambda o: (abs(o), o)))] +
                      [("SHP-%d" % (k + 1), "shipping", self.ship_y + o * sp.door_pitch_m)
                       for k, o in enumerate(sorted(sp.shipping_doors, key=lambda o: (abs(o), -o)))])
        for name, _r, y in self.doors:
            if not (self.y0 + sp.door_w_m / 2 + 0.5 <= y <= self.y1 - sp.door_w_m / 2 - 0.5):
                raise ValueError(f"door {name} at y={y:.2f} is outside the dock wall")
        self.r2_y = 0.0 - sp.door_pitch_m  # RCV-2: start of the dock lane
        self.s2_y = self.ship_y + sp.door_pitch_m  # SHP-2: end of the dock lane

    # ---- rack rows (12 for the default spec) ---------------------------------
    def rack_rows(self):
        """[(row name, y_min, y_max, kind)] full-depth rows across the building (right to left)."""
        sp = self.sp
        a = sp.aisle_w_m / 2
        rows = [("R01", self.aisle_y[0] - a - sp.rack_depth_m, self.aisle_y[0] - a, "single")]
        for i in range(self.n_aisles - 1):
            rows.append((f"R{i + 2:02d}", self.aisle_y[i] + a, self.aisle_y[i + 1] - a, "double"))
        rows.append((f"R{self.n_aisles + 1:02d}", self.aisle_y[-1] + a, self.aisle_y[-1] + a + sp.rack_depth_m, "single"))
        return rows

    def racks(self) -> list:
        out = []
        for name, ya, yb, kind in self.rack_rows():
            for k, (xa, xb) in enumerate(self.rack_blocks):
                out.append(Box(f"{name}{chr(ord('A') + k)}", "rack", (xa + xb) / 2, (ya + yb) / 2, xb - xa, yb - ya,
                               self.sp.rack_h_m, meta={"row": name, "type": kind, "block": k}))
        return out

    # ---- track network -------------------------------------------------------
    def track_dict(self) -> dict:
        sp = self.sp
        xe = sp.dock_wall_x + sp.exit_inset_m
        ya, yz = self.aisle_y[0], self.aisle_y[-1]
        r = lambda v: round(float(v), 3)  # noqa: E731
        roads = [
            {"name": "dock_lane", "waypoints": [[r(xe), r(self.r2_y)], [r(sp.dock_lane_x), r(self.r2_y)],
                                                [r(sp.dock_lane_x), r(self.s2_y)], [r(xe), r(self.s2_y)]],
             "exit_start": "receiving_2", "exit_end": "shipping_2"},
            {"name": "forklift_crossing", "waypoints": [[r(sp.plaza_x), r(-sp.plaza_len_m / 2)], [r(sp.plaza_x), r(sp.plaza_len_m / 2)]],
             "lane_width_m": sp.plaza_w_m},
            {"name": "perimeter", "waypoints": [[r(self.xF), 0.0], [r(self.xF), r(ya)], [r(self.xB), r(ya)], [r(self.xB), r(yz)],
                                                [r(self.xF), r(yz)], [r(self.xF), 0.0]]},
        ]
        for i in self.taped:
            if i in (0, self.n_aisles - 1, sp.main_aisle):
                continue
            y = self.aisle_y[i]
            if i == self.ship_aisle:
                roads.append({"name": f"aisle_{i}", "waypoints": [[r(xe), r(y)], [r(self.xB), r(y)]], "exit_start": "shipping_1"})
            else:
                roads.append({"name": f"aisle_{i}", "waypoints": [[r(self.xF), r(y)], [r(self.xB), r(y)]]})
        for k, x in enumerate(self.xM):
            roads.append({"name": f"cross_{k + 1}", "waypoints": [[r(x), r(ya)], [r(x), r(yz)]]})
        return {
            "name": "warehouse",
            "description": (f"Prototype warehouse {sp.width_m:g} x {sp.depth_m:g} m: start just inside receiving door RCV-1, "
                            f"taped lanes through the dock area (dock lane, {sp.plaza_w_m:g} m forklift crossing = tape gap), "
                            f"a perimeter loop on the front / back cross aisles and the edge aisles, "
                            f"{len(self.taped) - 2} inner taped aisles, {sp.n_mid_cross} middle cross aisle(s). "
                            f"Generated by scripts/warehouse_builder.py - do not hand-edit."),
            "lane_width_m": sp.lane_w_m,
            "stripe_width_m": sp.stripe_w_m,
            "paint_before_start_m": 0.3,
            "paint_after_finish_m": 0.5,
            "waypoints": [[0, 0], [r(self.xB), 0]],
            "exit": None,
            "roads": roads,
        }

    def missions(self) -> list:
        """Named test drives: (name, description, to_exit, via points)."""
        n = self.n_aisles - 1
        far_block = self.rack_blocks[-1]
        # an aisle an odd number of aisles from the start aisle (untaped while tape_every was 2): the middle one
        odds = [i for i in range(1, n) if (i - self.sp.main_aisle) % 2 and i in self.taped and i != self.ship_aisle]
        odd = odds[len(odds) // 2] if odds else self.ship_aisle
        out = [
            {"name": "far_aisle_to_shipping",
             "description": f"receiving RCV-1 -> far aisle {n} (back block) -> shipping SHP-1",
             "to": "shipping_1", "via": [[round((far_block[0] + far_block[1]) / 2, 2), round(self.aisle_y[n], 3)]]},
            {"name": "back_loop_to_receiving",
             "description": "receiving RCV-1 -> up the start aisle -> back cross aisle -> aisle 0 -> front -> back through "
                            "the forklift crossing -> receiving RCV-2",
             "to": "receiving_2", "via": [[round(self.xB, 3), round(self.aisle_y[0] / 2, 3)],
                                          [round((self.rack_blocks[0][0] + self.rack_blocks[0][1]) / 2, 2), round(self.aisle_y[0], 3)]]},
            {"name": "dock_shuttle",
             "description": "receiving RCV-1 -> dock lane -> shipping SHP-2 (short run along the dock)",
             "to": "shipping_2", "via": []},
            {"name": "odd_aisle_to_shipping",
             "description": f"receiving RCV-1 -> front cross aisle -> down aisle {odd} (taped since every aisle is taped, "
                            f"front to back) -> back cross aisle -> down shipping aisle {self.ship_aisle} -> shipping SHP-1",
             "to": "shipping_1", "via": [[round((xa + xb) / 2, 2), round(self.aisle_y[odd], 3)] for xa, xb in self.rack_blocks] +
                                        [[round((far_block[0] + far_block[1]) / 2, 2), round(self.ship_y, 3)]]},
            # one-SKU pretend pick: same spine as odd_aisle; controller stops at the demo group on aisle {odd}
            {"name": "pick_one_sku_to_shipping",
             "description": f"receiving RCV-1 -> aisle {odd} demo SKU group (pretend pick) -> shipping aisle "
                            f"{self.ship_aisle} -> shipping SHP-1",
             "to": "shipping_1", "via": [[round((xa + xb) / 2, 2), round(self.aisle_y[odd], 3)] for xa, xb in self.rack_blocks] +
                                        [[round((far_block[0] + far_block[1]) / 2, 2), round(self.ship_y, 3)]]},
        ]
        # multi-trip shopping list: one mission per trolley trip (via = stop approach points)
        from src.shopping_list import demo_shopping_lines, plan_shopping_trips

        groups = self.inventory_groups()
        shop = plan_shopping_trips(demo_shopping_lines(groups), groups)
        for trip in shop.trips:
            out.append({
                "name": f"shopping_list_trip_{trip.trip_index}",
                "description": (
                    f"shopping trip {trip.trip_index}/{shop.n_trips}: pick "
                    + ", ".join(trip.skus)
                    + f" (trolley cap {shop.capacity}) -> shipping SHP-1"
                ),
                "to": "shipping_1",
                "via": [[round(float(v[0]), 3), round(float(v[1]), 3)] for v in trip.via],
            })
        return out

    # ---- walls / doors -------------------------------------------------------
    def walls_and_doors(self) -> list:
        sp = self.sp
        t, H = sp.wall_t_m, sp.wall_h_m
        out = []
        cx = (self.x0 + self.x1) / 2
        out.append(Box("WALL_RIGHT", "wall", cx, self.y0 - t / 2, sp.depth_m + 2 * t, t, H))
        out.append(Box("WALL_LEFT", "wall", cx, self.y1 + t / 2, sp.depth_m + 2 * t, t, H))
        out.append(Box("WALL_BACK", "wall", self.x1 + t / 2, (self.y0 + self.y1) / 2, t, sp.width_m, H))
        xw = self.x0 - t / 2
        edges = sorted((y - sp.door_w_m / 2, y + sp.door_w_m / 2, name) for name, _r, y in self.doors)
        y = self.y0
        k = 0
        for a, b, name in edges:
            if a - y > 1e-6:
                k += 1
                out.append(Box(f"WALL_DOCK_{k}", "wall", xw, (y + a) / 2, t, a - y, H))
            out.append(Box(f"HEADER_{name.replace('-', '')}", "header", xw, (a + b) / 2, t, b - a, H - sp.door_h_m, z0=sp.door_h_m))
            out.append(Box(f"DOOR_{name.replace('-', '')}", "door", xw - 0.05, (a + b) / 2, 0.08, b - a, sp.door_h_m,
                           meta={"door": name}))
            y = b
        if self.y1 - y > 1e-6:
            out.append(Box(f"WALL_DOCK_{k + 1}", "wall", xw, (y + self.y1) / 2, t, self.y1 - y, H))
        return out

    # ---- staging (pallets ~1 m, packing tables) -------------------------------
    def staging(self, net) -> list:
        sp = self.sp
        from src.track_geometry import _poly_dist

        def clear(b: Box) -> bool:
            x0, y0, x1, y1 = b.rect
            pts = [(x0 + (x1 - x0) * i / 4, y0 + (y1 - y0) * j / 4) for i in range(5) for j in range(5)]
            for g in net.roads:
                lim = max(g.spec.widths) / 2 + sp.lane_clear_m
                if any(_poly_dist(g.centerline, *p) < lim for p in pts):
                    return False
            return True

        px, py = sp.pallet_m
        xa = sp.dock_lane_x + sp.lane_w_m / 2 + sp.lane_clear_m
        xb = self.xF - sp.cross_aisle_w_m / 2 - 0.5
        n_x = int((xb - xa + 0.1) // (px + 0.1))
        out = []
        # staging lanes: 2 pallets wide, then a 2.4 m forklift gap, across the dock zone
        y = self.y0 + 1.0
        lane = 0
        r_hi = max(y for _n, r, y in self.doors if r == "receiving") + sp.door_pitch_m
        s_lo = min(y for _n, r, y in self.doors if r == "shipping") - sp.door_pitch_m
        k_p = k_t = 0
        while y + 2 * py + 0.2 <= self.y1 - 1.0:
            lane += 1
            zone = "receiving" if y < r_hi else ("shipping" if y > s_lo else "packing")
            for j in range(2):
                yc = y + py / 2 + j * (py + 0.2)
                if zone == "packing":
                    if j:
                        continue
                    for i in range(int((xb - xa + 0.3) // (sp.table_m[0] + 0.6))):
                        b = Box(f"TABLE_{k_t + 1:02d}", "table", xa + sp.table_m[0] / 2 + i * (sp.table_m[0] + 0.6),
                                y + sp.table_m[1] / 2 + 0.4, sp.table_m[0], sp.table_m[1], sp.table_m[2],
                                meta={"zone": zone})
                        if clear(b):
                            k_t += 1
                            out.append(b)
                    continue
                for i in range(n_x):
                    xc = xa + px / 2 + i * (px + 0.1)
                    h = sp.pallet_heights[(k_p + lane) % len(sp.pallet_heights)]
                    b = Box(f"PALLET_{k_p + 1:03d}", "pallet", xc, yc, px, py, h, meta={"zone": zone})
                    if clear(b):
                        k_p += 1
                        out.append(b)
            y += 2 * py + 0.2 + 2.4
        return out

    # ---- obstacle test props (obstacles world only) ----------------------------
    def obstacle_props(self, staging=()) -> list:
        """Three props on known routes, all derived from the layout. Each Box has
        meta: route (mission), expect (what the robot should do), lane (where)."""
        sp = self.sp
        n = self.n_aisles - 1
        odds = [i for i in range(1, n) if (i - sp.main_aisle) % 2 and i in self.taped and i != self.ship_aisle]
        odd = odds[len(odds) // 2] if odds else self.ship_aisle
        bx, by, bh = sp.obs_box_m
        blk = self.rack_blocks[0]
        out = [Box("OBS_BOX", "obs_box", round((blk[0] + blk[1]) / 2, 3), round(self.aisle_y[odd], 3), bx, by, bh,
                   meta={"route": "odd_aisle_to_shipping", "lane": f"aisle {odd} centre, rack block 0",
                         "expect": "STOP before it; delete it in Webots -> resume"})]
        tx, ty, th = sp.table_m  # packing table: long side along the dock lane (robot drives +y there)
        x_in = sp.dock_lane_x + sp.obs_table_intrude_m
        out.append(Box("OBS_TABLE", "obs_table", round(x_in + ty / 2, 3), round(sp.obs_table_at_m + tx / 2, 3), ty, tx, th,
                       meta={"route": "dock_shuttle", "lane": f"dock lane x={sp.dock_lane_x:g}, edge "
                                                              f"{sp.obs_table_intrude_m:.2f} m right of centre",
                             "expect": "STOP (waist band)"}))
        shx, shy, shh = sp.obs_shelf_m
        y_in = sp.lane_w_m / 2 + sp.obs_shelf_gap_m
        out.append(Box("OBS_SHELF", "obs_shelf", round(sp.obs_shelf_x_m + shx / 2, 3), round(y_in + shy / 2, 3), shx, shy,
                       shh, meta={"route": "every mission (start lane)", "lane": f"start lane y=0, corner "
                                                                                 f"{y_in:.2f} m left of centre",
                                  "expect": "PASS (outside the corridor), no slow / stop"}))
        for b in out:  # props must not sit on staging items
            for q in staging:
                a, c = b.rect, q.rect
                if a[0] < c[2] and c[0] < a[2] and a[1] < c[3] and c[1] < a[3]:
                    raise ValueError(f"{b.name} overlaps {q.name}")
        return out

    def inventory_groups(self):
        """Placeholder inventory cube groups (by aisle / block SKU). See src.inventory_pick."""
        from src.inventory_pick import build_inventory_groups

        return build_inventory_groups(
            self, cube_m=tuple(self.sp.inv_cube_m), cubes_per_group=int(self.sp.inv_cubes_per_group)
        )

    def summary(self, net=None, graph=None) -> dict:
        sp = self.sp
        rows = self.rack_rows()
        d = {
            "building_m": [sp.depth_m, sp.width_m],
            "area_m2": round(sp.width_m * sp.depth_m, 1),
            "area_sqft": round(sp.width_m * sp.depth_m * SQFT_PER_M2),
            "walls_x": [self.x0, self.x1], "walls_y": [round(self.y0, 3), round(self.y1, 3)],
            "shelf_rows": len(rows), "rows_single": sum(1 for r in rows if r[3] == "single"),
            "rows_double": sum(1 for r in rows if r[3] == "double"),
            "rack_runs_faces": sum(1 if r[3] == "single" else 2 for r in rows),
            "rack_blocks_x": [[round(a, 2), round(b, 2)] for a, b in self.rack_blocks],
            "rack_h_m": sp.rack_h_m, "rack_depth_m": sp.rack_depth_m, "double_row_depth_m": 2 * sp.rack_depth_m + sp.flue_m,
            "aisles": self.n_aisles, "aisle_w_m": sp.aisle_w_m, "aisle_y": [round(y, 3) for y in self.aisle_y],
            "taped_aisles": self.taped, "shipping_aisle": self.ship_aisle,
            "cross_aisles_x": [round(self.xF, 3)] + [round(x, 3) for x in self.xM] + [round(self.xB, 3)],
            "cross_aisle_w_m": sp.cross_aisle_w_m,
            "side_wall_clear_m": round(self.side_clear, 3),
            "doors": [[n, r, round(y, 3)] for n, r, y in self.doors],
            "staging_x": [round(sp.dock_lane_x + sp.lane_w_m / 2, 2), round(self.xF - sp.cross_aisle_w_m / 2, 2)],
        }
        if graph is not None:
            d["junctions"] = len(graph.junctions)
            d["corners"] = len(graph.corners())
            d["exits"] = [e.name for e in net.exits]
            d["dead_ends"] = sum(1 for n in graph.nodes if n.kind == "dead_end")  # the forklift crossing's two ends
            d["taped_lane_m"] = round(sum(g.length_m for g in net.roads), 1)
        return d


# ---------------------------------------------------------------------------
# Track JSON (+ missions -> RBM_ROUTE via src.route_planner)
# ---------------------------------------------------------------------------


def build_track(lay: Layout):
    from src.route_planner import RoadGraph
    from src.track_geometry import TrackNetwork

    d = lay.track_dict()
    net = TrackNetwork.from_dict(d, TRACK_JSON)
    g = RoadGraph(net)
    missions, routes = [], {}
    for m in lay.missions():
        p = g.plan(m["to"], via=m["via"])
        if not p.ok:
            raise ValueError(f"mission {m['name']}: {p.note}")
        f = g.follow(p.route)
        if f.exit != m["to"]:
            raise ValueError(f"mission {m['name']}: RBM_ROUTE={p.route} ends at {f.exit}, not {m['to']}")
        missions.append({**m, "route": p.route, "length_m": p.length_m, "junctions": p.junctions_passed,
                         "route_entries": len(p.entries),
                         "corners": sum(1 for e in p.edges for _ in _corners_on(net, e))})
        routes[p.route] = m["to"]
    dflt = g.follow(None)
    routes["default"] = dflt.exit  # None: the default (straight, else left) circles the perimeter
    d["routes"] = routes
    d["missions"] = missions
    from src.inventory_pick import inventory_to_json

    d["inventory"] = inventory_to_json(lay.inventory_groups())
    from src.shopping_list import (
        attach_trip_routes,
        demo_shopping_lines,
        plan_shopping_trips,
        plan_to_json,
    )

    shop = plan_shopping_trips(demo_shopping_lines(lay.inventory_groups()), lay.inventory_groups())
    attach_trip_routes(shop, g)
    # sync mission routes onto the shopping plan trips
    by_name = {m["name"]: m for m in missions}
    for trip in shop.trips:
        m = by_name.get(f"shopping_list_trip_{trip.trip_index}")
        if m:
            trip.route = m["route"]
            trip.length_m = m["length_m"]
    d["shopping_list"] = plan_to_json(shop)
    d["notes"] = [
        "RBM_ROUTE strings come from src/route_planner.py (python scripts/warehouse_builder.py --summary prints them).",
        f"Default route (no RBM_ROUTE): {dflt.note or ('ends at ' + str(dflt.exit))}.",
        "Obstacles: stereo + forward radar on ButlerBot (docs/OBSTACLES.md); test props in butlerbot_warehouse_obstacles.wbt; rolling-ball world butlerbot_radar_motion.wbt.",
        "Inventory: placeholder cubes in WH_INVENTORY (grouped by aisle/block SKU); pull-behind WH_TROLLEY; one-SKU pretend pick via RBM_PICK_SKU (docs/INVENTORY_TROLLEY.md).",
        "Shopping list: multi-trip fill-trolley-then-ship via RBM_SHOPPING_LIST=demo (docs/SHOPPING_LIST.md); per-trip missions shopping_list_trip_N.",
    ]
    return d, TrackNetwork.from_dict(d, TRACK_JSON), g, dflt


def _corners_on(net, e):
    g = net.roads[e.road]
    sp = g.spec
    lo, hi = min(e.s0, e.s1), max(e.s0, e.s1)
    s = 0.0
    for j in range(1, len(sp.waypoints)):
        s += math.dist(sp.waypoints[j - 1], sp.waypoints[j])
        if j < len(sp.waypoints) - 1 and lo + 1e-6 < s < hi - 1e-6 and abs(g._phi[j]) > math.radians(10):
            yield sp.waypoints[j]


def write_track_json(d: dict, path: str = TRACK_JSON) -> str:
    text = json.dumps(d, indent=2)
    # keep [x, y] pairs on one line (readable diffs)
    import re

    text = re.sub(r"\[\s+(-?[\d.]+),\s+(-?[\d.]+)\s+\]", r"[\1, \2]", text)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text + "\n")
    return path


# ---------------------------------------------------------------------------
# Webots world
# ---------------------------------------------------------------------------

_COLORS = {  # none of these pass the lane yellow test (min(R,G) - B >= 0.22)
    "wall": (0.78, 0.78, 0.76), "header": (0.78, 0.78, 0.76), "door": (0.45, 0.5, 0.56),
    "upright": (0.12, 0.22, 0.45), "beam": (0.15, 0.32, 0.62), "load": (0.58, 0.5, 0.42),
    "pallet_wood": (0.55, 0.45, 0.36), "pallet_load": (0.62, 0.58, 0.55), "table": (0.35, 0.37, 0.4),
    "inv_cube": (0.55, 0.28, 0.22),  # terracotta tote — fails yellow (min-B = 0.06)
    "inv_cube_demo": (0.22, 0.45, 0.55),  # teal demo SKU — fails yellow
    "trolley": (0.25, 0.28, 0.32), "trolley_bed": (0.40, 0.42, 0.38), "trolley_load": (0.55, 0.28, 0.22),
}
WH_RULE = "# -----------------------------------------------------------------------------\n"
WH_BEGIN = "# WAREHOUSE_BEGIN"
WH_END = "# WAREHOUSE_END"


def _f(v: float) -> str:
    t = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


class _Apps:
    def __init__(self):
        self.done = set()

    def __call__(self, key: str, ind: str) -> str:
        if key in self.done:
            return f"appearance USE WH_APP_{key.upper()}"
        self.done.add(key)
        r, g, b = _COLORS[key]
        return (f"appearance DEF WH_APP_{key.upper()} PBRAppearance {{\n{ind}  baseColor {_f(r)} {_f(g)} {_f(b)}\n"
                f"{ind}  roughness 0.85\n{ind}  metalness 0\n{ind}}}")


def _shape(app, key, size, at=(0, 0, 0), ind="      "):
    s = f"{ind}Pose {{\n{ind}  translation {' '.join(_f(v) for v in at)}\n{ind}  children [\n"
    s += f"{ind}    Shape {{\n{ind}      {app(key, ind + '      ')}\n"
    s += f"{ind}      geometry Box {{ size {' '.join(_f(v) for v in size)} }}\n{ind}    }}\n{ind}  ]\n{ind}}}\n"
    return s


def _solid(b: Box, app, children: str, comment: str = "") -> str:
    zc = b.z0 + b.h / 2
    s = f"    # {comment}\n" if comment else ""
    s += f"    DEF WH_{b.name} Solid {{\n      translation {_f(b.cx)} {_f(b.cy)} {_f(zc)}\n"
    # radarCrossSection > 0 so Webots Radar can see the solid (0 = invisible to radar).
    rcs = 2.0 if b.kind.startswith("obs_") else 1.0
    s += f"      name \"{b.name.lower()}\"\n      radarCrossSection {rcs:g}\n      children [\n{children}      ]\n"
    s += f"      boundingObject Box {{ size {_f(b.sx)} {_f(b.sy)} {_f(b.h)} }}\n    }}\n"
    return s


def _rack_children(b: Box, sp: WarehouseSpec, app) -> str:
    """Visual detail (shared boundingObject is the whole rack envelope):
    goods inset, blue beams per level on each aisle face, uprights per bay."""
    ch = ""
    h = b.h
    ch += _shape(app, "load", (b.sx - 0.1, b.sy - 0.15, h - 0.25), (0, 0, -0.05), "        ")
    faces = [-1, 1]
    for lv in range(1, sp.beam_levels + 1):
        z = -h / 2 + lv * h / sp.beam_levels - 0.06
        for f in faces:
            ch += _shape(app, "beam", (b.sx, 0.05, 0.12), (0, f * (b.sy / 2 - 0.025), z), "        ")
    n_bay = max(1, int(round(b.sx / sp.bay_m)))
    for k in range(n_bay + 1):
        x = -b.sx / 2 + 0.04 + k * (b.sx - 0.08) / n_bay
        for f in faces:
            ch += _shape(app, "upright", (0.08, 0.08, h), (x, f * (b.sy / 2 - 0.04), 0), "        ")
    return ch


def _tex_app(tint, scale=2.0, ind="          ") -> str:
    r, g, b = tint
    return (f"appearance PBRAppearance {{\n{ind}  baseColor {_f(r)} {_f(g)} {_f(b)}\n"
            f"{ind}  baseColorMap ImageTexture {{\n{ind}    url [\n{ind}      \"{PROP_TEXTURE_URL}\"\n{ind}    ]\n{ind}  }}\n"
            f"{ind}  roughness 0.85\n{ind}  metalness 0\n"
            f"{ind}  textureTransform TextureTransform {{\n{ind}    scale {_f(scale)} {_f(scale)}\n{ind}  }}\n"
            f"{ind}}}")


def _tex_shape(tint, size, at=(0, 0, 0), scale=2.0) -> str:
    ind = "        "
    return (f"{ind}Pose {{\n{ind}  translation {' '.join(_f(v) for v in at)}\n{ind}  children [\n"
            f"{ind}    Shape {{\n{ind}      {_tex_app(tint, scale, ind + '      ')}\n"
            f"{ind}      geometry Box {{ size {' '.join(_f(v) for v in size)} }}\n{ind}    }}\n{ind}  ]\n{ind}}}\n")


# tints (x the grey speckle texture): blue / grey, far from the lane-yellow test (min(R,G) - B >= 0.22)
_PROP_TINT = {"obs_box": (0.55, 0.62, 0.78), "obs_table": (0.62, 0.62, 0.66), "obs_shelf": (0.45, 0.55, 0.8),
              "obs_goods": (0.75, 0.75, 0.8)}


def obstacle_props_vrml(props: list) -> str:
    out = []
    for b in props:
        m = b.meta
        ch = ""
        if b.kind == "obs_box":
            ch = _tex_shape(_PROP_TINT["obs_box"], (b.sx, b.sy, b.h), scale=1.0)
        elif b.kind == "obs_table":
            ch = _tex_shape(_PROP_TINT["obs_table"], (b.sx, b.sy, 0.05), (0, 0, b.h / 2 - 0.025), scale=3.0)
            for dx in (-1, 1):
                for dy in (-1, 1):
                    ch += _tex_shape(_PROP_TINT["obs_table"], (0.05, 0.05, b.h - 0.05),
                                     (dx * (b.sx / 2 - 0.05), dy * (b.sy / 2 - 0.05), -0.025), scale=0.5)
        else:  # shelf unit: 4 uprights, 4 boards, goods on the boards
            for dx in (-1, 1):
                for dy in (-1, 1):
                    ch += _tex_shape(_PROP_TINT["obs_shelf"], (0.05, 0.05, b.h),
                                     (dx * (b.sx / 2 - 0.025), dy * (b.sy / 2 - 0.025), 0), scale=0.5)
            levels = (0.10, 0.65, 1.20, b.h - 0.03)
            for z in levels:
                ch += _tex_shape(_PROP_TINT["obs_shelf"], (b.sx, b.sy, 0.03), (0, 0, z - b.h / 2), scale=2.0)
            for z0, z1 in zip(levels[:-1], levels[1:]):
                gh = min(0.4, z1 - z0 - 0.08)
                ch += _tex_shape(_PROP_TINT["obs_goods"], (b.sx - 0.2, b.sy - 0.1, gh),
                                 (0, 0, z0 + 0.015 + gh / 2 - b.h / 2), scale=1.0)
        out.append(_solid(b, None, ch, f"OBSTACLE TEST PROP {b.name}: {m['lane']} — route {m['route']} — "
                                        f"expect {m['expect']}"))
    return "".join(out)


def write_prop_texture(path: str = PROP_TEXTURE) -> str:
    """Deterministic multi-scale grey speckle (256 px, tiles cleanly enough at 1-3 repeats)."""
    import numpy as np

    os.makedirs(os.path.dirname(path), exist_ok=True)
    rng = np.random.default_rng(20261002)
    img = np.zeros((256, 256))
    for cell, amp in ((4, 0.5), (16, 0.3), (64, 0.2)):
        g = rng.random((256 // cell + 1, 256 // cell + 1))
        img += amp * np.kron(g, np.ones((cell, cell)))[:256, :256]
    img = 0.35 + 0.65 * (img - img.min()) / (img.max() - img.min())
    from PIL import Image

    Image.fromarray((img * 255).astype("uint8"), mode="L").convert("RGB").save(path, optimize=True)
    return path


def inventory_props_vrml(groups: list, app) -> str:
    """Placeholder cubes in DEF WH_INVENTORY (static Solids, RCS for radar)."""
    out = []
    for g in groups:
        key = "inv_cube_demo" if g.meta.get("demo") else "inv_cube"
        for c in g.cubes:
            # DEF is WH_<name>; Box.name is without the WH_ prefix (_solid adds it)
            name = c.def_name[3:] if c.def_name.startswith("WH_") else c.def_name
            b = Box(name, "inv_cube", c.cx, c.cy, c.sx, c.sy, c.sz, z0=c.cz - c.sz / 2,
                    meta={"sku": g.sku, "aisle": g.aisle, "block": g.block, "side": g.side,
                          "demo": bool(g.meta.get("demo"))})
            ch = _shape(app, key, (c.sx, c.sy, c.sz), (0, 0, 0), "        ")
            comment = f"INVENTORY {g.sku} cube {c.index} aisle {g.aisle} block {g.block} {g.side}"
            out.append(_solid(b, app, ch, comment))
    return "".join(out)


def trolley_vrml(lay: Layout, app) -> str:
    """Pull-behind trolley prop at the spawn hitch pose (controller keeps it following).

    Not a Robot child (Robot block stays byte-identical across worlds). Supervisor
    teleports DEF WH_TROLLEY each frame. Cargo WH_TROLLEY_LOAD starts under the
    floor and is raised onto the bed after a pretend pick.

    Visual-only: **no boundingObject / no Physics**. A static Solid with a
    boundingObject behind the bot (clipping the floor) was stalling forward
    motion — Webots treated it as an immovable hitch / contact sink. There is no
    physics mass to lighten; collision geometry was the bug.
    """
    from src.inventory_pick import TrolleyGeom

    sp = lay.sp
    tg = TrolleyGeom(
        chassis_length_m=sp.chassis_length_m,
        tongue_m=sp.trolley_tongue_m,
        length_m=sp.trolley_length_m,
        width_m=sp.trolley_width_m,
        bed_h_m=sp.trolley_bed_h_m,
        rail_h_m=sp.trolley_rail_h_m,
    )
    # start pose: behind spawn facing +x (yaw 0)
    x, y, _yaw = tg.world_pose((0.0, 0.0), 0.0)
    L, W, bed_h, rail_h = tg.length_m, tg.width_m, tg.bed_h_m, tg.rail_h_m
    tongue = tg.tongue_m
    zc = bed_h / 2
    ch = ""
    ch += _shape(app, "trolley_bed", (L, W, 0.04), (0, 0, -zc + 0.02), "        ")
    for dy in (-1, 1):
        ch += _shape(app, "trolley", (L - 0.04, 0.03, rail_h - bed_h), (0, dy * (W / 2 - 0.02), -zc + bed_h + (rail_h - bed_h) / 2), "        ")
    ch += _shape(app, "trolley", (0.03, W - 0.04, rail_h - bed_h), (L / 2 - 0.02, 0, -zc + bed_h + (rail_h - bed_h) / 2), "        ")
    ch += _shape(app, "trolley", (tongue, 0.04, 0.04), (L / 2 + tongue / 2, 0, -zc + 0.06), "        ")
    for dx in (-1, 1):
        for dy in (-1, 1):
            ch += _shape(app, "trolley", (0.08, 0.04, 0.08), (dx * (L / 2 - 0.1), dy * (W / 2 - 0.05), -zc + 0.04), "        ")
    ch += (
        "        DEF WH_TROLLEY_LOAD Solid {\n"
        "          translation 0 0 -2\n"
        '          name "trolley_load"\n'
        "          radarCrossSection 0.5\n"
        "          children [\n"
    )
    ch += _shape(app, "trolley_load", (0.28, 0.28, 0.28), (0, 0, 0), "            ")
    ch += (
        "          ]\n"
        "        }\n"
    )
    s = (f"    # PULL-BEHIND TROLLEY prop (controller follows ButlerBot; hitch {tg.hitch_behind_m:.2f} m).\n"
         f"    # Visual only: no boundingObject / no Physics — a static BO here stalled the robot.\n")
    s += f"    DEF WH_TROLLEY Solid {{\n      translation {_f(x)} {_f(y)} {_f(zc)}\n"
    s += f'      name "trolley"\n      radarCrossSection 1.5\n      children [\n{ch}      ]\n'
    s += "    }\n"
    return s



def warehouse_vrml(lay: Layout, staging: list, props: list | None = None) -> str:
    sp = lay.sp
    app = _Apps()
    racks = lay.racks()
    wd = lay.walls_and_doors()
    out = [WH_RULE, f"{WH_BEGIN}\n",
           f"# Prototype warehouse {sp.width_m:g} x {sp.depth_m:g} m ({sp.width_m * sp.depth_m:.0f} m2, "
           f"{sp.width_m * sp.depth_m * SQFT_PER_M2:,.0f} sq ft). Generated by scripts/warehouse_builder.py - do not hand-edit.\n",
           f"# {len(lay.rack_rows())} shelf rows ({sp.rack_h_m:g} m racks), {lay.n_aisles} aisles {sp.aisle_w_m:g} m, "
           f"cross aisles {sp.cross_aisle_w_m:g} m at x = " + ", ".join(f"{x:g}" for x in [lay.xF] + lay.xM + [lay.xB]) + "\n",
           "# Every rack / wall / door / pallet / table is a static Solid with a boundingObject (no physics),\n",
           "# so later distance sensors, stereo depth and radar see them.\n",
           WH_RULE,
           "DEF WAREHOUSE Group {\n  children [\n"]
    out.append("    # ---- walls and dock doors (closed roll-up doors in the dock wall) ----\n")
    for b in wd:
        key = b.kind if b.kind != "header" else "header"
        ch = _shape(app, key, (b.sx, b.sy, b.h), (0, 0, 0), "        ")
        out.append(_solid(b, app, ch, f"dock door {b.meta['door']}" if b.kind == "door" else ""))
    out.append("    # ---- rack rows (selective pallet racking) ----\n")
    for b in racks:
        out.append(_solid(b, app, _rack_children(b, sp, app),
                          f"rack {b.meta['row']} block {b.meta['block']} ({b.meta['type']}) x {b.rect[0]:.2f}..{b.rect[2]:.2f} "
                          f"y {b.rect[1]:.2f}..{b.rect[3]:.2f} h {b.h:g}"))
    out.append("    # ---- staging: pallets (~1-1.4 m) and packing tables (0.9 m) ----\n")
    for b in staging:
        if b.kind == "pallet":
            ch = _shape(app, "pallet_wood", (b.sx, b.sy, 0.14), (0, 0, -b.h / 2 + 0.07), "        ")
            ch += _shape(app, "pallet_load", (b.sx - 0.06, b.sy - 0.06, b.h - 0.14), (0, 0, 0.07), "        ")
        else:
            ch = _shape(app, "table", (b.sx, b.sy, 0.05), (0, 0, b.h / 2 - 0.025), "        ")
            for dx in (-1, 1):
                for dy in (-1, 1):
                    ch += _shape(app, "table", (0.05, 0.05, b.h - 0.05),
                                 (dx * (b.sx / 2 - 0.05), dy * (b.sy / 2 - 0.05), -0.025), "        ")
        out.append(_solid(b, app, ch))
    # ---- placeholder inventory cubes (grouped by aisle/block SKU) + pull-behind trolley ----
    groups = lay.inventory_groups()
    out.append(
        "    # ---- INVENTORY: placeholder cubes on rack faces (src/inventory_pick.py). Demo SKU is teal.\n"
    )
    out.append("    DEF WH_INVENTORY Group {\n      children [\n")
    inv = inventory_props_vrml(groups, app)
    out.append("".join(("    " + ln + "\n") if ln else "\n" for ln in inv.rstrip("\n").split("\n")))
    out.append("      ]\n    }\n")
    out.append(trolley_vrml(lay, app))
    if props:
        out.append("    # ---- OBSTACLE TEST PROPS (scripts/warehouse_builder.py obstacle_props). Textured: block-matching\n"
                   "    #      stereo needs texture. Delete one in the scene tree while the robot waits = 'removed'.\n")
        out.append("    DEF WH_OBSTACLES Group {\n      children [\n")
        out.append("".join("    " + ln + "\n" if ln else "\n" for ln in obstacle_props_vrml(props).rstrip("\n").split("\n")))
        out.append("      ]\n    }\n")
    else:
        out.append("    # ---- obstacles: none in this world (butlerbot_warehouse_obstacles.wbt has the test props).\n"
                   "    #      Shelves above are already Solids with boundingObjects for the stereo pair / radar.\n")
        out.append("    DEF WH_OBSTACLES Group {\n      children [\n      ]\n    }\n")
    out.append("  ]\n}\n")
    out.append(f"{WH_END}\n")
    return "".join(out)


def follow_viewpoint(lay: Layout) -> dict:
    """Top-down FOLLOW Viewpoint over the robot, derived from the spec.

    Webots ENU: a Viewpoint with identity orientation looks along +x; axis-angle
    ``0 1 0 pi/2`` pitches it straight down (image top = +x = into the building,
    image left = +y, so the dock wall is at the bottom, same as the preview).
    ``followType "Tracking Shot"`` copies the robot's translation only, so the eye
    stays straight above the robot and the map never rotates ("Mounted Shot" would
    spin with the robot's heading, "Pan and Tilt Shot" would stay put and tilt).
    Height: ``follow_span_m`` of floor across the wider side of the window at
    ``follow_fov`` (Webots applies fieldOfView to the wider side), and never lower
    than ``follow_min_clear_m`` above the walls, the tallest thing in the building
    (racks 6 m, door headers up to the wall top). There is no roof or ceiling, and
    the eye only ever moves horizontally, so it cannot clip into anything.
    The eye starts above the spawn (0, 0); the controller re-applies this pose from
    the world's Viewpoint at start (``_apply_follow_camera``).
    """
    sp = lay.sp
    tallest = max([sp.wall_h_m, sp.rack_h_m, sp.door_h_m] + [b.z0 + b.h for b in lay.walls_and_doors() + lay.racks()])
    z = max(sp.follow_span_m / 2 / math.tan(sp.follow_fov / 2), tallest + sp.follow_min_clear_m)
    near = max(0.05, round(0.01 * z, 2))  # depth precision: the 13 mm tape must not z-fight the floor from z m up
    far = round(4 * z, 1)  # room to scroll out; finite keeps depth precision
    return {"position": (0.0, 0.0, z), "orientation": (0.0, 1.0, 0.0, math.pi / 2), "fov": sp.follow_fov,
            "near": near, "far": far, "span_m": 2 * z * math.tan(sp.follow_fov / 2), "tallest_m": tallest}


def follow_viewpoint_vrml(lay: Layout) -> str:
    v = follow_viewpoint(lay)
    sp = lay.sp
    z = v["position"][2]
    return ("DEF VIEWPOINT Viewpoint {\n"
            f"  # Warehouse TOP-DOWN FOLLOW cam: straight down over ButlerBot from {z:.1f} m "
            f"(walls {sp.wall_h_m:g} m, racks {sp.rack_h_m:g} m), ~{v['span_m']:.0f} m of floor across the window.\n"
            "  # Tracking Shot = follows the robot's translation only, so the map never rotates:\n"
            "  # dock wall at the bottom, +x (into the building) up, +y left, same as the preview.\n"
            "  # Derived from WarehouseSpec by scripts/warehouse_builder.py follow_viewpoint(); the controller\n"
            "  # keeps this pose (re-centred on the robot) instead of the 3.2 m chase. No roof / ceiling.\n"
            '  follow "ButlerBot"\n'
            '  followType "Tracking Shot"\n'
            "  followSmoothness 0.2\n"
            f"  orientation {' '.join(_f(c) for c in v['orientation'])}\n"
            f"  position {' '.join(_f(c) for c in v['position'])}\n"
            f"  fieldOfView {_f(v['fov'])}\n"
            f"  near {_f(v['near'])}\n"
            f"  far {_f(v['far'])}\n"
            "}\n")


def build_world(lay: Layout, net, staging: list, template: str | None = None, props: list | None = None) -> str:
    import track_builder as tb  # scripts/

    template = template or tb.TEMPLATE_WORLD
    with open(template, encoding="utf-8") as fh:
        text = fh.read()
    sp = lay.sp
    cx, cy = (lay.x0 + lay.x1) / 2, (lay.y0 + lay.y1) / 2
    fl0 = text.index("\nFloor {") + 1
    fl1 = text.index("\n}\n", fl0) + 3
    floor = (f"# Warehouse floor = building footprint (same appearance as the test worlds: the lane vision was\n"
             f"# tuned on it). Walls at x {lay.x0:g}..{lay.x1:g}, y {lay.y0:g}..{lay.y1:g}.\n"
             f"Floor {{\n  translation {_f(cx)} {_f(cy)} 0\n  name \"ground\"\n  size {_f(sp.depth_m)} {_f(sp.width_m)}\n"
             f"  tileSize 1 1\n  appearance Parquetry {{\n    type \"chequered\"\n    colorOverride 0.55 0.56 0.58\n  }}\n}}\n")
    text = text[:fl0] + floor + text[fl1:]
    vp0 = text.index("\nDEF VIEWPOINT Viewpoint {") + 1
    vp1 = text.index("\n}\n", vp0) + 3
    text = text[:vp0] + follow_viewpoint_vrml(lay) + text[vp1:]
    # Indoors: 6 m racks would throw the aisles into hard shadow; no shadows in this world.
    head, rest = text.split("\nRobot {", 1)
    head = head.replace("castShadows TRUE", "castShadows FALSE")
    k = head.index("\nDirectionalLight {") + 1
    head = head[:k] + "# Warehouse: no shadows (6 m racks would throw the aisles into hard shadow; indoor light is diffuse).\n" + head[k:]
    text = head + "\nRobot {" + rest
    pose = text.index("\nDEF TRACK Pose {") + 1
    rule2 = text.rindex(tb.BLOCK_RULE, 0, text.rindex(tb.BLOCK_RULE, 0, pose))
    text = text[:rule2] + warehouse_vrml(lay, staging, props) + "\n" + text[rule2:]
    text = tb.replace_track_block(text, tb.emit_track_vrml(net))
    lines = text.split("\n")
    for k, ln in enumerate(lines):
        if ln.lstrip().startswith('"TRACK (ENU):'):
            lines[k] = tb.info_line(net)
            lines.insert(k + 1, f'    "WAREHOUSE: {sp.width_m:g} x {sp.depth_m:g} m prototype, start inside receiving door RCV-1, '
                                f'RBM_ROUTE per junction (see tracks/warehouse.json missions)"')
            lines.insert(k + 2, '    "INVENTORY: placeholder cubes in WH_INVENTORY (aisle/block SKUs); '
                                   'pull-behind WH_TROLLEY; one-SKU pretend pick: set RBM_PICK_SKU '
                                   '(see tracks/warehouse.json inventory + docs/INVENTORY_TROLLEY.md)"')
            if props:
                lines.insert(k + 3, '    "OBSTACLES: test props in WH_OBSTACLES (box in the odd aisle, table edge on the dock lane, '
                                    'shelf beside the start lane); stereo obstacle sensing ON here (RBM_OBSTACLES=0 = off)"')
            break
    if props:
        lines.insert(1, "# OBSTACLES on   (controller: stereo obstacle sensing on by default in this world; "
                        "RBM_OBSTACLES=0 turns it off). Generated by scripts/warehouse_builder.py.")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Preview
# ---------------------------------------------------------------------------


def draw_preview(lay: Layout, net, graph, staging: list, missions: list, path: str = PREVIEW) -> str:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.patches as mp
    import matplotlib.pyplot as plt

    sp = lay.sp
    # dock wall at the bottom, world +x up the page; plot X = -world y so the robot's left (+y) is plot left
    def P(x, y):
        return (-y, x)

    fig, ax = plt.subplots(figsize=(17, 15), dpi=110)
    ax.set_facecolor("#e9e9e6")
    # building
    x0, y0 = P(lay.x0, lay.y1)
    ax.add_patch(mp.Rectangle((x0, y0), sp.width_m, sp.depth_m, fc="#d5d6d8", ec="none", zorder=0))
    for b in lay.walls_and_doors():
        (a, c), (bb, d) = P(b.rect[0], b.rect[3]), P(b.rect[2], b.rect[1])
        if b.kind == "door":
            ax.add_patch(mp.Rectangle((a, c - 0.5), bb - a, 1.2, fc="#5a7fa8", ec="k", lw=0.6, zorder=3))
            ax.text((a + bb) / 2, c - 1.6, b.meta["door"], ha="center", va="top", fontsize=7.5, weight="bold",
                    color="#1f4e79" if b.meta["door"].startswith("RCV") else "#7a2e00")
        elif b.kind == "wall":
            ax.add_patch(mp.Rectangle((a, c), bb - a, d - c, fc="#444", ec="none", zorder=3))
    # zones
    zx0, zx1 = lay.x0, lay.xF - sp.cross_aisle_w_m / 2
    rcv_hi = max(y for _n, r, y in lay.doors if r == "receiving") + sp.door_pitch_m
    shp_lo = min(y for _n, r, y in lay.doors if r == "shipping") - sp.door_pitch_m
    for (ya, yb), lab, col in (((lay.y0, rcv_hi), "RECEIVING", "#cfe2f3"), ((rcv_hi, shp_lo), "PACKING / STAGING", "#e4ecd8"),
                               ((shp_lo, lay.y1), "SHIPPING", "#f6dcc6")):
        (a, c), (bb, d) = P(zx0, yb), P(zx1, ya)
        ax.add_patch(mp.Rectangle((a, c), bb - a, d - c, fc=col, ec="none", alpha=0.8, zorder=0.5))
        ax.text((a + bb) / 2, zx1 - 0.6, lab, ha="center", va="top", fontsize=10, weight="bold", color="#333", zorder=6)
    # aisles (labels) and cross aisles
    for i, y in enumerate(lay.aisle_y):
        px, _ = P(0, y)
        ax.text(px, (lay.rack_blocks[0][0] + lay.rack_blocks[0][1]) / 2, f"aisle {i}\n{sp.aisle_w_m:g} m",
                ha="center", va="center", fontsize=6.5, color="#333", rotation=90, zorder=6,
                bbox=dict(fc="white", ec="none", alpha=0.6, pad=0.5))
    for x, lab in [(lay.xF, "FRONT CROSS AISLE")] + [(x, f"MID CROSS AISLE {k + 1}") for k, x in enumerate(lay.xM)] + \
            [(lay.xB, "BACK CROSS AISLE")]:
        ax.text(-lay.y0 + 1.0, x, f"{lab}\n{sp.cross_aisle_w_m:g} m, x = {x:g}", ha="left", va="center", fontsize=7.5,
                color="#333", zorder=6)
    ax.text(-lay.y0 + 1.0, sp.dock_lane_x, f"DOCK LANE\nx = {sp.dock_lane_x:g}", ha="left", va="center", fontsize=7.5,
            color="#333")
    ax.text(-lay.y0 + 1.0, (lay.x0 + sp.dock_lane_x) / 2 - 0.5, "DOCK APRON", ha="left", va="center", fontsize=7.5,
            color="#333")
    ax.text(-lay.y0 + 1.0, (sp.dock_lane_x + lay.xF) / 2, "STAGING", ha="left", va="center", fontsize=7.5, color="#333")
    # racks
    for b in lay.racks():
        (a, c), (bb, d) = P(b.rect[0], b.rect[3]), P(b.rect[2], b.rect[1])
        ax.add_patch(mp.Rectangle((a, c), bb - a, d - c, fc="#2f5597", ec="#1b2f57", lw=0.6, zorder=2))
        if b.meta["block"] == len(lay.rack_blocks) - 1:
            ax.text((a + bb) / 2, d + 0.5, b.meta["row"], ha="center", va="bottom", fontsize=7, weight="bold",
                    color="#1b2f57", zorder=6)
    # staging
    for b in staging:
        (a, c), (bb, d) = P(b.rect[0], b.rect[3]), P(b.rect[2], b.rect[1])
        ax.add_patch(mp.Rectangle((a, c), bb - a, d - c, fc="#a0805c" if b.kind == "pallet" else "#5d6168",
                                  ec="#5b4630", lw=0.4, zorder=2))
    # tape
    for c, le, re_ in net.paint_segments():
        xs, ys = zip(*[P(*p) for p in c])
        ax.plot(xs, ys, color="#e8c400", lw=1.6, solid_capstyle="butt", zorder=4)
    # plaza (gap)
    ax.text(*P(sp.plaza_x, -sp.plaza_len_m / 2 - 0.4), f"GAP: forklift crossing\n{sp.plaza_w_m:g} m, no tape",
            ha="left", va="center", fontsize=7, color="#7a5c00", zorder=6)
    # junctions
    for n in graph.junctions:
        ax.plot(*P(*n.xy), "o", ms=4, mfc="white", mec="#555", zorder=5)
    # missions
    cols = ["#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]
    for k, m in enumerate(missions):
        p = graph.plan(m["to"], via=m["via"])
        pts = [q for e in p.edges for q in e.poly]
        off = (k - 1) * 0.35
        xs, ys = zip(*[P(q[0] + off, q[1] + off) for q in pts])
        ax.plot(xs, ys, ls=(0, (4, 3)), color=cols[k % len(cols)], lw=1.3, zorder=4.5,
                label=f"{m['name']}: {m['length_m']:.0f} m, {m['junctions']} junctions  RBM_ROUTE={m['route']}")
    # start / exits
    ax.annotate("", xy=P(2.5, 0), xytext=P(0, 0), arrowprops=dict(arrowstyle="-|>", color="#00a03c", lw=2.5), zorder=7)
    ax.plot(*P(0, 0), "s", ms=9, color="#00c040", zorder=7)
    ax.text(*P(0.3, 1.0), "START (0,0)\nfacing +x", fontsize=8, weight="bold", color="#006b28", ha="right", va="bottom", zorder=7)
    for e in net.exits:
        ux, uy = math.cos(e.heading), math.sin(e.heading)
        h = 0.5 * (e.width_m + 0.10)
        a, b = P(e.xy[0] + h * uy, e.xy[1] - h * ux), P(e.xy[0] - h * uy, e.xy[1] + h * ux)
        ax.plot([a[0], b[0]], [a[1], b[1]], color="red", lw=3, zorder=7)
        ax.text(*P(e.xy[0] + 0.6, e.xy[1]), f"EXIT\n{e.name}", fontsize=7, color="red", weight="bold",
                ha="center", va="bottom", zorder=7, bbox=dict(fc="white", ec="none", alpha=0.7, pad=0.3))
    # axes, scale bar
    X0, Y0 = P(lay.x0, lay.y1)
    ax.set_xlim(X0 - 3, X0 + sp.width_m + 16)
    ax.set_ylim(lay.x0 - 7, lay.x1 + 3)
    ax.set_aspect("equal")
    sb_x, sb_y = X0 + 1.0, lay.x0 - 5.5
    for i in range(5):
        ax.add_patch(mp.Rectangle((sb_x + 2 * i, sb_y), 2, 0.6, fc="k" if i % 2 == 0 else "white", ec="k", lw=0.8, zorder=8))
    ax.text(sb_x, sb_y + 0.9, "0", ha="center", fontsize=8)
    ax.text(sb_x + 10, sb_y + 0.9, "10 m", ha="center", fontsize=8)
    ax.text(sb_x + 12, sb_y + 0.3, "scale bar (m)", va="center", fontsize=8)
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    area = sp.width_m * sp.depth_m
    ax.set_title(f"ButlerBot prototype warehouse: {sp.width_m:g} x {sp.depth_m:g} m = {area:.0f} m² "
                 f"(~{area * SQFT_PER_M2:,.0f} sq ft).  {len(lay.rack_rows())} shelf rows ({sp.rack_h_m:g} m), "
                 f"{lay.n_aisles} aisles {sp.aisle_w_m:g} m, cross aisles {sp.cross_aisle_w_m:g} m, "
                 f"{len(graph.junctions)} taped junctions\n"
                 "world frame: x up the page (robot's start heading), +y to the left; dock wall at the bottom",
                 fontsize=10)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.005), fontsize=7.5, frameon=False, ncol=1)
    handles = [mp.Patch(fc="#2f5597", label=f"rack ({sp.rack_h_m:g} m)"), mp.Patch(fc="#a0805c", label="pallet (1.0-1.4 m)"),
               mp.Patch(fc="#5d6168", label="packing table (0.9 m)"), mp.Patch(fc="#5a7fa8", label="dock door"),
               plt.Line2D([], [], color="#e8c400", lw=2, label="yellow tape lane (1.30 m)"),
               plt.Line2D([], [], color="red", lw=3, label="exit / finish (red)"),
               plt.Line2D([], [], marker="o", ls="", mfc="white", mec="#555", label="taped junction")]
    leg2 = ax.legend(handles=handles, loc="upper right", bbox_to_anchor=(1.0, 1.0), fontsize=7.5, framealpha=0.95)
    ax.add_artist(leg2)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.005), fontsize=7.5, frameon=False, ncol=1)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", pad_inches=0.3)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------


def build_all(sp: WarehouseSpec | None = None, *, track_path=TRACK_JSON, world_path=WORLD, preview_path=PREVIEW,
              write=True, obstacles_path=OBSTACLES_WORLD):
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    sp = sp or WarehouseSpec()
    lay = Layout(sp)
    d, net, g, dflt = build_track(lay)
    staging = lay.staging(net)
    props = lay.obstacle_props(staging)
    inventory = lay.inventory_groups()
    res = {"layout": lay, "track": d, "net": net, "graph": g, "staging": staging, "default": dflt,
           "props": props, "inventory": inventory}
    if write:
        if track_path:
            write_track_json(d, track_path)
            from src.track_geometry import load_track

            net = load_track(track_path)  # paint from the file as written (spec.path -> TRACK_FILE tag)
            res["net"] = net
        if world_path:
            with open(world_path, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(build_world(lay, net, staging))
        if obstacles_path:
            if not os.path.isfile(PROP_TEXTURE):
                write_prop_texture()
            with open(obstacles_path, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(build_world(lay, net, staging, props=props))
        if preview_path:
            draw_preview(lay, net, g, staging, d["missions"], preview_path)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--summary", action="store_true", help="print the layout and routes, write nothing")
    ap.add_argument("--preview", default=PREVIEW, help=f"preview PNG path (default {PREVIEW}; '' = none)")
    ap.add_argument("--no-world", action="store_true")
    a = ap.parse_args(argv)
    res = build_all(write=not a.summary, preview_path=a.preview or None, world_path=None if a.no_world else WORLD,
                    obstacles_path=None if a.no_world else OBSTACLES_WORLD)
    lay, d = res["layout"], res["track"]
    s = lay.summary(res["net"], res["graph"])
    s["staging_items"] = {"pallets": sum(1 for b in res["staging"] if b.kind == "pallet"),
                          "tables": sum(1 for b in res["staging"] if b.kind == "table")}
    print(json.dumps(s, indent=1))
    for m in d["missions"]:
        print(f"mission {m['name']}: {m['length_m']} m, {m['junctions']} junctions, {m['corners']} corners -> "
              f"{m['to']}\n  set RBM_ROUTE={m['route']}")
    print(f"default (no RBM_ROUTE): {res['default'].note or res['default'].exit}")
    for b in res["props"]:
        print(f"obstacle prop {b.name}: x {b.rect[0]:.2f}..{b.rect[2]:.2f} y {b.rect[1]:.2f}..{b.rect[3]:.2f} h {b.h:g} — "
              f"{b.meta['lane']}; route {b.meta['route']}: expect {b.meta['expect']}")
    for g in res["inventory"]:
        mark = " DEMO" if g.meta.get("demo") else ""
        print(f"inventory{mark} {g.sku}: aisle {g.aisle} block {g.block} {g.side}, {g.n_cubes} cubes, "
              f"approach ({g.approach_xy[0]:.2f}, {g.approach_xy[1]:.2f})")
    shop = d.get("shopping_list") or {}
    if shop:
        print(f"shopping_list: {shop.get('n_trips')} trips, capacity {shop.get('capacity')}, "
              f"units {shop.get('total_units')}, order {shop.get('order_mode')}")
        for t in shop.get("trips") or []:
            print(f"  trip {t['trip_index']}: {t['skus']}  set RBM_ROUTE={t.get('route')}")
    if not a.summary:
        print(f"wrote {TRACK_JSON}\nwrote {WORLD}" + ("" if a.no_world else f"\nwrote {OBSTACLES_WORLD}")
              + (f"\nwrote {a.preview}" if a.preview else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
