"""Plan RBM_ROUTE strings on a road-network track (offline only).

The robot never reads a map: at each junction it sees openings (L / S / R)
and ``src.route_policy.RoutePolicy`` takes the next ``RBM_ROUTE`` entry when
there are two or more ways on (corners and one-way junctions don't use an
entry). This module works out, from a :class:`src.track_geometry.TrackNetwork`,
which entries make the robot follow a given path, so a big course (the
warehouse) can be driven by route string without counting junctions by hand.

Generic, nothing course-specific:

* nodes = every place two road centre lines meet (crossing, T, branch start),
  plus road ends (an exit, or a dead end such as the end of a plaza);
* arms = the ways out of a node along each road through it;
* at a node the robot arriving with heading h may take any arm except the one
  back the way it came; an arm is S / L / R by its angle to h (within
  ``TURN_TOL_DEG`` of 0 / +90 / -90 deg). Other angles are not offered;
* a node with >= 2 ways on "uses a route entry" (same rule as RoutePolicy).

    from src.route_planner import RoadGraph
    g = RoadGraph(load_track("warehouse"))
    plan = g.plan(to_exit="shipping_1", via=[(40.0, 48.0)])
    plan.route   # "L,S,S,S,S,R,..."  -> set RBM_ROUTE=...
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

TURN_TOL_DEG = 30.0
NODE_MERGE_M = 0.5


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _seg_hit(p0, p1, q0, q1, tol=1e-6):
    """Parameters (t, u) where segment p0-p1 meets q0-q1 (endpoints included), else None."""
    rx, ry = p1[0] - p0[0], p1[1] - p0[1]
    sx, sy = q1[0] - q0[0], q1[1] - q0[1]
    den = rx * sy - ry * sx
    if abs(den) < 1e-12:
        return None  # parallel / collinear: roads overlapping along a leg are not a junction
    qpx, qpy = q0[0] - p0[0], q0[1] - p0[1]
    t = (qpx * sy - qpy * sx) / den
    u = (qpx * ry - qpy * rx) / den
    lt, lu = tol / max(1e-12, math.hypot(rx, ry)), tol / max(1e-12, math.hypot(sx, sy))
    if -lt <= t <= 1 + lt and -lu <= u <= 1 + lu:
        return max(0.0, min(1.0, t)), max(0.0, min(1.0, u))
    return None


def _cum(poly):
    out = [0.0]
    for a, b in zip(poly, poly[1:]):
        out.append(out[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return out


def _point_at(poly, cum, s):
    for i in range(len(poly) - 1):
        if s <= cum[i + 1] or i == len(poly) - 2:
            L = max(1e-12, cum[i + 1] - cum[i])
            t = max(0.0, min(1.0, (s - cum[i]) / L))
            return (poly[i][0] + t * (poly[i + 1][0] - poly[i][0]), poly[i][1] + t * (poly[i + 1][1] - poly[i][1]))
    return poly[-1]


def _heading_at(poly, cum, s, forward=True):
    """Driving heading leaving arc length s forward (or backward)."""
    n = len(poly) - 1
    if forward:
        for i in range(n):
            if cum[i + 1] > s + 1e-9 and cum[i + 1] - cum[i] > 1e-12:
                return math.atan2(poly[i + 1][1] - poly[i][1], poly[i + 1][0] - poly[i][0])
        i = n - 1
        return math.atan2(poly[i + 1][1] - poly[i][1], poly[i + 1][0] - poly[i][0])
    for i in range(n - 1, -1, -1):
        if cum[i] < s - 1e-9 and cum[i + 1] - cum[i] > 1e-12:
            return math.atan2(poly[i][1] - poly[i + 1][1], poly[i][0] - poly[i + 1][0])
    return math.atan2(poly[0][1] - poly[1][1], poly[0][0] - poly[1][0])


def _sub_poly(poly, cum, s0, s1):
    """Centre line between arc lengths s0 and s1 (reversed when s1 < s0)."""
    lo, hi = min(s0, s1), max(s0, s1)
    pts = [_point_at(poly, cum, lo)] + [p for p, c in zip(poly, cum) if lo < c < hi] + [_point_at(poly, cum, hi)]
    return pts if s1 >= s0 else pts[::-1]


@dataclass
class Node:
    id: int
    xy: tuple
    kind: str  # "junction" | "exit" | "dead_end" | "start" (main road's first waypoint)
    exit: str | None = None
    roads: set = field(default_factory=set)


@dataclass
class Edge:
    """Drive along road ``road`` from node ``a`` to node ``b``."""
    a: int
    b: int
    road: int
    s0: float
    s1: float
    h_out: float  # heading leaving a
    h_in: float  # heading arriving at b
    length: float
    poly: list


@dataclass
class Decision:
    node: int
    xy: tuple
    kind: str  # junction kind as the robot meets it: t_end / plus / side_L / side_R / corner / gap...
    options: str  # e.g. "L S R"
    choice: str
    uses_entry: bool


@dataclass
class Plan:
    route: str  # RBM_ROUTE value ("" = none needed)
    exit: str | None
    length_m: float
    decisions: list
    edges: list
    ok: bool = True
    note: str = ""

    @property
    def entries(self) -> list:
        return [d.choice for d in self.decisions if d.uses_entry]

    @property
    def junctions_passed(self) -> int:
        return sum(1 for d in self.decisions if d.kind != "corner")


class RoadGraph:
    def __init__(self, net, turn_tol_deg: float = TURN_TOL_DEG):
        self.net = net
        self.tol = math.radians(turn_tol_deg)
        self.polys = [list(g.centerline) for g in net.roads]
        self.cums = [_cum(p) for p in self.polys]
        self.nodes: list[Node] = []
        self.edges: list[Edge] = []
        self._build()

    # ------------------------------------------------------------ building
    def _node_at(self, xy, kind="junction", exit_name=None) -> int:
        for n in self.nodes:
            if n.kind == kind == "junction" and math.hypot(n.xy[0] - xy[0], n.xy[1] - xy[1]) < NODE_MERGE_M:
                return n.id
        self.nodes.append(Node(len(self.nodes), (float(xy[0]), float(xy[1])), kind, exit_name))
        return len(self.nodes) - 1

    def _build(self):
        stops = {k: [] for k in range(len(self.polys))}  # road -> [(s, node)]
        R = len(self.polys)
        for a in range(R):
            for b in range(a + 1, R):
                pa, pb, ca, cb = self.polys[a], self.polys[b], self.cums[a], self.cums[b]
                for i in range(len(pa) - 1):
                    for j in range(len(pb) - 1):
                        hit = _seg_hit(pa[i], pa[i + 1], pb[j], pb[j + 1])
                        if hit is None:
                            continue
                        t, u = hit
                        sa = ca[i] + t * (ca[i + 1] - ca[i])
                        sb = cb[j] + u * (cb[j + 1] - cb[j])
                        xy = _point_at(pa, ca, sa)
                        nid = self._node_at(xy)
                        self.nodes[nid].roads.update((a, b))
                        for road, s in ((a, sa), (b, sb)):
                            if not any(abs(s - s2) < 1e-6 and n2 == nid for s2, n2 in stops[road]):
                                stops[road].append((s, nid))
        exits = {}
        for e in getattr(self.net, "exits", []):
            exits[(e.road, "end" if math.hypot(e.xy[0] - self.polys[e.road][-1][0],
                                               e.xy[1] - self.polys[e.road][-1][1]) < 1e-6 else "start")] = e.name
        for k in range(R):
            L = self.cums[k][-1]
            have = sorted(stops[k])
            for end, s in (("start", 0.0), ("end", L)):
                if any(abs(s2 - s) < 1e-6 for s2, _ in have):
                    continue  # this end sits in a junction
                name = exits.get((k, end))
                xy = self.polys[k][0] if end == "start" else self.polys[k][-1]
                kind = "exit" if name else ("start" if (k == 0 and end == "start") else "dead_end")
                nid = self._node_at(xy, kind, name)
                self.nodes[nid].roads.add(k)
                stops[k].append((s, nid))
            st = sorted(stops[k])
            for (s0, n0), (s1, n1) in zip(st, st[1:]):
                if s1 - s0 < 1e-6:
                    continue
                pc, cc = self.polys[k], self.cums[k]
                fwd = Edge(n0, n1, k, s0, s1, _heading_at(pc, cc, s0, True), _wrap(_heading_at(pc, cc, s1, False) + math.pi),
                           s1 - s0, _sub_poly(pc, cc, s0, s1))
                back = Edge(n1, n0, k, s1, s0, _heading_at(pc, cc, s1, False), _wrap(_heading_at(pc, cc, s0, True) + math.pi),
                            s1 - s0, _sub_poly(pc, cc, s1, s0))
                self.edges += [fwd, back]
        self.out = {n.id: [e for e in self.edges if e.a == n.id] for n in self.nodes}

    # ------------------------------------------------------------ queries
    @property
    def junctions(self) -> list:
        return [n for n in self.nodes if n.kind == "junction"]

    def corners(self) -> list:
        """Interior sharp / rounded corners of every road (no route entry)."""
        out = []
        for k, g in enumerate(self.net.roads):
            sp = g.spec
            for j in range(1, len(sp.waypoints) - 1):
                if abs(g._phi[j]) > math.radians(10):
                    out.append((k, sp.waypoints[j], math.degrees(g._phi[j])))
        return out

    def options(self, node: int, h_in: float) -> dict:
        """{'L'|'S'|'R': Edge} the robot can take arriving at ``node`` with heading h_in."""
        opts = {}
        for e in self.out[node]:
            rel = _wrap(e.h_out - h_in)
            if abs(abs(rel) - math.pi) < self.tol:
                continue  # straight back
            for name, ang in (("S", 0.0), ("L", math.pi / 2), ("R", -math.pi / 2)):
                if abs(_wrap(rel - ang)) < self.tol and name not in opts:
                    opts[name] = e
        return opts

    @staticmethod
    def _kind(opts: dict) -> str:
        k = "".join(sorted(opts))
        return {"LRS": "plus", "LR": "t_end", "LS": "side_L", "RS": "side_R", "S": "straight",
                "L": "corner", "R": "corner"}.get(k, "dead_end" if not opts else k)

    def start_edge(self) -> Edge:
        sx, sy = self.net.start_xy
        cands = [e for e in self.edges if e.road == 0 and e.s0 < e.s1 and abs(e.s0) < 1e-6]
        if not cands:
            raise ValueError("main road has no edge from its start")
        return cands[0]

    def _passes(self, e: Edge, p, r: float) -> bool:
        for a, b in zip(e.poly, e.poly[1:]):
            vx, vy = b[0] - a[0], b[1] - a[1]
            L2 = vx * vx + vy * vy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
            if math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy) <= r:
                return True
        return False

    def plan(self, to_exit: str, via=(), via_radius_m: float | None = None) -> Plan:
        """Shortest drive from the start to exit ``to_exit`` passing the ``via``
        points (x, y) in order (each within ``via_radius_m`` of the path, default
        half a lane). No U-turns: only S / L / R at each node."""
        r = via_radius_m if via_radius_m is not None else 0.5 * float(self.net.spec.widths[0])
        via = [tuple(v) for v in via]
        e0 = self.start_edge()
        idx = {id(e): i for i, e in enumerate(self.edges)}

        def stage_after(e, st):
            while st < len(via) and self._passes(e, via[st], r):
                st += 1
            return st

        start = (idx[id(e0)], stage_after(e0, 0))
        dist = {start: e0.length}
        prev = {start: None}
        pq = [(e0.length, start)]
        goal = None
        while pq:
            d, (ei, st) = heapq.heappop(pq)
            if d > dist.get((ei, st), 1e18) + 1e-9:
                continue
            e = self.edges[ei]
            nb = self.nodes[e.b]
            if nb.kind == "exit":
                if nb.exit == to_exit and st == len(via):
                    goal = (ei, st)
                    break
                continue
            for _name, e2 in self.options(e.b, e.h_in).items():
                st2 = stage_after(e2, st)
                key = (idx[id(e2)], st2)
                nd = d + e2.length
                if nd < dist.get(key, 1e18) - 1e-9:
                    dist[key] = nd
                    prev[key] = (ei, st)
                    heapq.heappush(pq, (nd, key))
        if goal is None:
            return Plan("", None, 0.0, [], [], ok=False, note=f"no path to exit {to_exit!r} via {via}")
        chain = []
        k = goal
        while k is not None:
            chain.append(self.edges[k[0]])
            k = prev[k]
        chain.reverse()
        return self._describe(chain, dist[goal])

    def _describe(self, chain, length) -> Plan:
        dec = []
        for e, e2 in zip(chain, chain[1:]):
            opts = self.options(e.b, e.h_in)
            choice = next(n for n, x in opts.items() if x is e2)
            kind = self._kind(opts)
            dec.append(Decision(e.b, self.nodes[e.b].xy, kind, " ".join(n for n in "LSR" if n in opts), choice,
                                len(opts) >= 2))
            # corners inside the next edge never use an entry; listed for the report only
        last = self.nodes[chain[-1].b]
        route = ",".join(d.choice for d in dec if d.uses_entry)
        return Plan(route, last.exit, round(length, 2), dec, chain)

    def follow(self, route: str | None = None, max_steps: int = 500) -> Plan:
        """What RoutePolicy would do with ``route`` (then the default: straight,
        else left, else right). Stops at an exit, a dead end, or a loop."""
        from src.route_policy import RoutePolicy

        pol = RoutePolicy(route)
        e = self.start_edge()
        chain, seen, length = [e], set(), e.length
        for _ in range(max_steps):
            nb = self.nodes[e.b]
            if nb.kind != "junction":
                p = self._describe(chain, length)
                p.ok = nb.kind == "exit"
                p.note = "" if p.ok else "dead end"
                return p
            opts = self.options(e.b, e.h_in)
            c, _note = pol.choose("L" in opts, "S" in opts, "R" in opts)
            if c is None:
                p = self._describe(chain, length)
                p.ok, p.note = False, "no way on"
                return p
            e = opts[c]
            state = (id(e), pol.i)
            if state in seen and pol.i >= len(pol.route):
                p = self._describe(chain, length)
                p.ok, p.note, p.exit = False, "loops forever (default policy circles the network)", None
                return p
            seen.add(state)
            chain.append(e)
            length += e.length
        p = self._describe(chain, length)
        p.ok, p.note = False, "too many steps"
        return p
