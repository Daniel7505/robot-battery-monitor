"""Expected turn count for the butlerbot warehouse world.

The world file names its track (``# TRACK_FILE``). Each mission in that
track has an ``RBM_ROUTE`` string. A turn is an L or R entry on that
route. Straight-through junctions are not turns. The baseline finished
course, ``back_loop_to_receiving``, has three of them.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

BASELINE_MISSION = "back_loop_to_receiving"
WORLD_REL = Path("webots") / "worlds" / "butlerbot_warehouse.wbt"

_TRACK_FILE = re.compile(r"^# TRACK_FILE\s+(\S+)", re.M)
_ROUTE_LOG = re.compile(
    r"ROUTE\s+RBM_ROUTE=(?P<route>unset|[SLR](?:\s*,\s*[SLR])*)",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class WarehouseCourse:
    name: str
    route: str
    turns: int
    corners: int
    length_m: float
    from_log: bool


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


def resolve_course(console_text: str = "", *, repo: Path | None = None) -> WarehouseCourse | None:
    """Mission the lap was on, else the baseline  back-loop course.

    A route printed by the controller wins. With no route, the warehouse
    world's baseline finished lap is ``back_loop_to_receiving``.
    """
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
    baseline = next((m for m in missions if m.get("name") == BASELINE_MISSION), None)
    if baseline is None:
        return None
    return _from_mission(baseline, from_log=False)


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
