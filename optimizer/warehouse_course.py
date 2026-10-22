"""Expected turn count for the butlerbot warehouse world.

The world file names its track (``# TRACK_FILE``). The lap Dan runs does
not set ``RBM_ROUTE``. The controller then goes straight at a junction
when it can, otherwise left. ``RBM_MISSION_LAPS=1`` stops that loop at
one perimeter-road length (201 m). On that path the robot makes three
left pivots and stops on the front cross aisle just past the main lane.

A ``ROUTE`` line that names a mission uses that mission's L/R count.
Those missions are other drives.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

DEFAULT_LAP_NAME = "default_perimeter_lap"
WORLD_REL = Path("webots") / "worlds" / "butlerbot_warehouse.wbt"

_TRACK_FILE = re.compile(r"^# TRACK_FILE\s+(\S+)", re.M)
_ROUTE_LOG = re.compile(
    r"ROUTE\s+RBM_ROUTE=(?P<route>unset|[SLR](?:\s*,\s*[SLR])*)",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class LapTurn:
    """One pivot on the driven path. ``at_m`` is centreline distance from the start."""

    direction: str  # left or right
    x: float
    y: float
    at_m: float
    source: str  # junction or tape


@dataclass(frozen=True)
class WarehouseCourse:
    name: str
    route: str
    turns: int
    corners: int
    length_m: float
    from_log: bool
    stop_xy: tuple[float, float] | None = None
    lap_turns: tuple[LapTurn, ...] = ()

    def describe(self) -> str:
        if self.name == DEFAULT_LAP_NAME:
            if self.lap_turns and all(turn.direction == "left" for turn in self.lap_turns):
                kind = f"{self.turns} left"
            else:
                kind = ", ".join(
                    f"{turn.direction} at ({turn.x:.1f}, {turn.y:.1f})"
                    for turn in self.lap_turns
                ) or str(self.turns)
            return (
                f"{kind} (RBM_ROUTE unset, straight-else-left, "
                f"stop at {self.length_m:.0f} m)"
            )
        return f"{self.turns} ({self.name}: {self.route})"

    def layout(self) -> str:
        """Plain centreline of the default lap. Empty for a named mission."""
        if self.name != DEFAULT_LAP_NAME or not self.lap_turns:
            return ""
        parts = ["Start (0, 0) facing +x on the main lane y=0"]
        prev = 0.0
        for turn in self.lap_turns:
            parts.append(
                f"{turn.at_m - prev:.1f} m, {turn.direction} ({turn.source}) "
                f"at ({turn.x:.1f}, {turn.y:.1f})"
            )
            prev = turn.at_m
        if self.stop_xy is not None:
            parts.append(
                f"{self.length_m - prev:.1f} m to stop "
                f"({self.stop_xy[0]:.1f}, {self.stop_xy[1]:.1f})"
            )
        return "; ".join(parts)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def track_relpath(repo: Path | None = None) -> str:
    """Path recorded on ``butlerbot_warehouse.wbt`` (``tracks/warehouse.json``)."""
    root = repo or repo_root()
    text = (root / WORLD_REL).read_text(encoding="utf-8", errors="replace")
    match = _TRACK_FILE.search(text)
    if match is None:
        raise FileNotFoundError(f"{WORLD_REL} has no TRACK_FILE line")
    return match.group(1)


def load_missions(repo: Path | None = None) -> list[dict]:
    root = repo or repo_root()
    path = root / track_relpath(root)
    data = json.loads(path.read_text(encoding="utf-8"))
    missions = data.get("missions")
    if not isinstance(missions, list):
        raise ValueError(f"{path} has no missions")
    return missions


def route_turn_count(route: str) -> int:
    """L and R entries. S is a straight-through junction, not a turn."""
    return sum(1 for part in str(route or "").split(",") if part.strip().upper() in {"L", "R"})


def parse_route_from_console(console_text: str) -> str | None:
    """RBM_ROUTE from a ``ROUTE`` console line, or None when unset / absent."""
    match = _ROUTE_LOG.search(console_text or "")
    if match is None:
        return None
    raw = match.group("route").strip()
    if raw.lower() == "unset":
        return None
    parts = [part.strip().upper() for part in raw.split(",") if part.strip()]
    return ",".join(parts)


def _norm(route: str) -> str:
    return ",".join(part.strip().upper() for part in str(route).split(",") if part.strip())


def _tape_turns(net, edge, at_m: float) -> list[LapTurn]:
    """Sharp tape bends interior to ``edge``. Travel against the waypoint order flips the sign."""
    road = net.roads[edge.road]
    spec = road.spec
    waypoints = spec.waypoints
    cum = [0.0]
    for j in range(1, len(waypoints)):
        cum.append(cum[-1] + math.dist(waypoints[j - 1], waypoints[j]))
    forward = edge.s1 >= edge.s0
    lo, hi = min(edge.s0, edge.s1), max(edge.s0, edge.s1)
    found: list[LapTurn] = []
    for j in range(1, len(waypoints) - 1):
        phi = road._phi[j]
        if abs(phi) <= math.radians(10):
            continue
        arc = cum[j]
        if not (lo + 1e-6 < arc < hi - 1e-6):
            continue
        along = (arc - edge.s0) if forward else (edge.s0 - arc)
        signed = phi if forward else -phi
        xy = waypoints[j]
        found.append(
            LapTurn(
                direction="left" if signed > 0 else "right",
                x=float(xy[0]),
                y=float(xy[1]),
                at_m=at_m + along,
                source="tape",
            )
        )
    return found


def _point_at(edges, cap_m: float) -> tuple[float, float]:
    remain = float(cap_m)
    for edge in edges:
        for start, end in zip(edge.poly, edge.poly[1:]):
            seg = math.hypot(end[0] - start[0], end[1] - start[1])
            if seg >= remain - 1e-9:
                scale = 0.0 if seg < 1e-9 else remain / seg
                return (
                    start[0] + scale * (end[0] - start[0]),
                    start[1] + scale * (end[1] - start[1]),
                )
            remain -= seg
    last = edges[-1].poly[-1]
    return (float(last[0]), float(last[1]))


def default_perimeter_lap(repo: Path | None = None) -> WarehouseCourse:
    """The unset-route lap: straight-else-left, one perimeter length.

    Junction L/R choices and tape corners sharper than 10° both pivot.
    A bend past the distance cap is not part of the lap.
    """
    from src.mission_park import warehouse_perimeter_m
    from src.route_planner import RoadGraph
    from src.track_geometry import load_track

    root = repo or repo_root()
    track = root / track_relpath(root)
    cap = warehouse_perimeter_m(track)
    if cap is None:
        raise ValueError(f"{track} has no perimeter length")
    net = load_track(str(track))
    plan = RoadGraph(net).follow(None)
    turns: list[LapTurn] = []
    entries: list[str] = []
    acc = 0.0
    for index, edge in enumerate(plan.edges):
        if index > 0:
            decision = plan.decisions[index - 1]
            if acc < cap - 1e-6 and decision.choice in {"L", "R"}:
                turns.append(
                    LapTurn(
                        direction="left" if decision.choice == "L" else "right",
                        x=float(decision.xy[0]),
                        y=float(decision.xy[1]),
                        at_m=acc,
                        source="junction",
                    )
                )
            if acc < cap - 1e-6 and decision.uses_entry:
                entries.append(decision.choice)
        for bend in _tape_turns(net, edge, acc):
            if bend.at_m < cap - 1e-6:
                turns.append(bend)
        acc += edge.length
        if acc >= cap - 1e-6:
            break
    turns.sort(key=lambda turn: turn.at_m)
    return WarehouseCourse(
        name=DEFAULT_LAP_NAME,
        route="unset",
        turns=len(turns),
        corners=sum(1 for turn in turns if turn.source == "tape"),
        length_m=float(cap),
        from_log=False,
        stop_xy=_point_at(plan.edges, cap),
        lap_turns=tuple(turns),
    )


def resolve_course(console_text: str = "", *, repo: Path | None = None) -> WarehouseCourse | None:
    """Printed mission route, or the default one-perimeter lap when the route is unset."""
    try:
        missions = load_missions(repo)
    except (OSError, ValueError):
        return None
    route = parse_route_from_console(console_text)
    if route:
        for mission in missions:
            if _norm(str(mission.get("route") or "")) == _norm(route):
                return _from_mission(mission, from_log=True)
        return WarehouseCourse(
            name="custom",
            route=route,
            turns=route_turn_count(route),
            corners=route_turn_count(route),
            length_m=0.0,
            from_log=True,
        )
    try:
        return default_perimeter_lap(repo)
    except (OSError, ValueError, ImportError):
        return None


def _from_mission(mission: dict, *, from_log: bool) -> WarehouseCourse:
    route = str(mission.get("route") or "")
    return WarehouseCourse(
        name=str(mission.get("name") or ""),
        route=route,
        turns=route_turn_count(route),
        corners=int(mission.get("corners") or 0),
        length_m=float(mission.get("length_m") or 0.0),
        from_log=from_log,
    )
