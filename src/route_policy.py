"""Route policy at junctions: ``RBM_ROUTE`` (one choice per junction) or the default.

``RBM_ROUTE="S,L,R"`` = straight at the first junction, left at the second,
right at the third; after the list runs out (or with no list) the DEFAULT is
"straight if possible, else left, else right". A listed choice that the
junction does not offer falls back to the default (and says so in the log).
"Straight possible" includes *unknown* (None): a wide plaza whose far side
is beyond the camera reach is crossed blind (GAP_CROSS), not turned away from.
Corners and one-way junctions (a gap, the end of the paint) never consume a
route entry.
"""

from __future__ import annotations

_ALIASES = {"S": "S", "STRAIGHT": "S", "F": "S", "L": "L", "LEFT": "L", "R": "R", "RIGHT": "R"}


def parse_route(text: str | None) -> tuple[list[str], list[str]]:
    """'S,L,R' -> (['S', 'L', 'R'], warnings). Separators: comma, semicolon, space."""
    out: list[str] = []
    bad: list[str] = []
    for tok in (text or "").replace(";", ",").replace(" ", ",").split(","):
        t = tok.strip().upper()
        if not t:
            continue
        if t in _ALIASES:
            out.append(_ALIASES[t])
        else:
            bad.append(tok.strip())
    warn = [f"RBM_ROUTE: ignored {b!r} (use S, L or R)" for b in bad]
    return out, warn


def default_choice(left: bool | None, straight: bool | None, right: bool | None) -> str | None:
    if straight is not False:
        return "S"
    if left:
        return "L"
    if right:
        return "R"
    return None


class RoutePolicy:
    def __init__(self, route: list[str] | str | None = None) -> None:
        if isinstance(route, str):
            route, _w = parse_route(route)
        self.route = list(route or [])
        self.i = 0
        self.history: list[tuple[str, str]] = []  # (choice, note)

    def describe(self) -> str:
        return (",".join(self.route) + " then default" if self.route else "default") + \
            " (straight if possible, else left)"

    def choose(self, left: bool | None, straight: bool | None, right: bool | None) -> tuple[str | None, str]:
        avail = {"L": bool(left), "S": straight is not False, "R": bool(right)}
        dflt = default_choice(left, straight, right)
        if sum(avail.values()) < 2:  # one way (or none): not a choice, keep the route entry
            note = f"only way {dflt}" if dflt else "no way on"
            self.history.append((dflt or "-", note))
            return dflt, note
        if self.i < len(self.route):
            want = self.route[self.i]
            self.i += 1
            if avail[want]:
                note = f"route #{self.i} {want}"
                choice = want
            else:
                note = f"route #{self.i} wanted {want} — not offered, default {dflt}"
                choice = dflt
        else:
            choice = dflt
            note = f"default {dflt}"
        self.history.append((choice or "-", note))
        return choice, note
