"""Junction classifier on top of the per-row line tracer (no track knowledge).

The corner detector (:func:`src.lane_vision.corner_cues`) already follows each
lane line up the image and notices where it "widens, then disappears". At an
intersection the same row-by-row yellow presence/width tells more:

* a line whose crossbar smears AWAY from the lane centre (``BEND_OUT``) means
  a road joins on that side (an opening);
* a line whose crossbar smears TOWARDS the centre (``BEND_IN``) is the outer
  line of a corner (existing corner path);
* a line that just stops (``END``) is a gap;
* a line still running at the top of the picture ``CONT``-inues.

Then, from the free (unclaimed) wide yellow bars and from lines that resume
beyond the gap, the far side of the crossing:

==============  ===========================================================
kind            evidence
==============  ===========================================================
corner_L/R      any BEND_IN (the corner path drives it)
t_end           both sides open, a bar across the lane centre beyond
plus            both sides open, the lane resumes / far pieces with an empty
                centre (straight True) or nothing visible yet (straight None,
                a wide plaza whose far side is beyond the camera reach)
side_L/R        one side opens, the other line keeps going straight past
                the opening (a branch; straight True)
gap_straight    both lines END and resume farther on (no side roads)
lane_lost       both lines END and nothing follows (yet)
dead_end        both END and a bar blocks: nowhere to go
==============  ===========================================================

Output: :class:`Junction` (kind, distances, openings left/straight/right,
confidence); :class:`JunctionTracker` adds 2-frame persistence (distance
consistent within ``JUNCTION_PERSIST_TOL_M`` after odometry).

Large intersections (credit: Dan): the far side of a wide crossing can be
beyond the 1.96 m camera reach, so "straight" may be *unknown* (None) when the
robot has to decide. The behaviour layer treats None as "probably straight"
and crosses blind (GAP_CROSS), re-deciding when it sees more.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.lane_vision import STRIPE_W_M, CornerCue, _median  # noqa: F401  (re-exported for callers)

JUNCTION_PAIR_TOL_M = 0.35  # both sides' events this close in x = one crossing
JUNCTION_SIDE_MARGIN_M = 0.25  # the continuing line must run this far past the opening
JUNCTION_RESUME_MIN_M = 0.30  # a resumed line starts at least this far past the near edge
JUNCTION_RESUME_Y_TOL_M = 0.25  # ... and this close to the line's own lateral position
JUNCTION_BAR_MIN_M = 0.20  # free yellow run at least this wide = a bar
JUNCTION_BAR_CENTRE_M = 0.25  # a bar covering |y| < this blocks straight
JUNCTION_END_MAX_X_M = 1.70  # a line END farther than this may just be far-row thinning
JUNCTION_STRAIGHT_DEG = 10.0  # 'continues straight' = heading change below this
JUNCTION_PERSIST_FRAMES = 2
JUNCTION_PERSIST_TOL_M = 0.30
JUNCTION_MIN_CONF = 0.5

KINDS = ("corner_L", "corner_R", "t_end", "side_L", "side_R", "plus", "gap_straight", "lane_lost", "dead_end",
         "opening_L", "opening_R")  # opening_*: one side opens, not yet known if branch or corner
TURN_KINDS = ("t_end", "side_L", "side_R", "plus", "gap_straight", "lane_lost", "dead_end")  # acted on


@dataclass
class Junction:
    kind: str
    near_m: float  # distance to the near crossing line / start of the gap (robot frame x)
    center_m: float  # estimated junction centre = pivot point
    far_m: float | None  # far side of the crossing, when seen
    left: bool | None
    straight: bool | None
    right: bool | None
    confidence: float
    frames: int = 1
    seen: bool = False
    events: dict = field(default_factory=dict)  # side -> (event, x)

    def openings(self) -> str:
        """'L S? R' style: letter = open, '?' = unknown, '-' = closed."""
        out = []
        for ch, v in (("L", self.left), ("S", self.straight), ("R", self.right)):
            out.append(ch if v else (ch + "?" if v is None else "-"))
        return " ".join(out)

    def short(self) -> str:
        return f"{self.kind} {self.center_m:.2f}m [{self.openings()}] c{self.confidence:.2f}"


def _cam_side(cam) -> int:
    p = cam.pixel_to_ground(0.5 * cam.width, cam.height - 1)
    return 1 if (p is not None and p[1] > 0.0) else -1


def _straight(pts) -> bool:
    if len(pts) < 6:
        return True
    n = len(pts)
    a, b = pts[: n // 2], pts[n // 2:]

    def ang(seg):
        (x0, y0), (x1, y1) = seg[0], seg[-1]
        return math.atan2(y1 - y0, max(1e-6, x1 - x0))

    return abs(math.degrees(ang(b) - ang(a))) < JUNCTION_STRAIGHT_DEG


def _side_event(tracks, side: int):
    """(event, x, track) for the lane line on ``side`` (+1 left, -1 right)."""
    cand = [t for t in tracks if t["first_x"] < 0.6 and (t["pts"][0][1] > 0.05) == (side > 0)
            and len(t["pts"]) >= 4]
    if not cand:
        return "MISSING", None, None
    t = max(cand, key=lambda t: (len(t["pts"]), -abs(t["pts"][0][1])))
    cue: CornerCue | None = t.get("cue")
    if cue is not None:
        return ("BEND_IN" if cue.role == "outer" else "BEND_OUT"), cue.x_m, t
    if t["end"] == "wide":
        return "CURVE", t["last_x"], t
    if t["end"] == "gap":
        if t["last_x"] <= JUNCTION_END_MAX_X_M and _straight(t["pts"]):
            return "END", t["last_x"] + 0.5 * STRIPE_W_M, t
        return "CURVE", t["last_x"], t
    if t["end"] == "edge":
        return "EDGE", t["last_x"], t
    return ("CONT" if _straight(t["pts"]) else "CURVE"), t["last_x"], t


def classify_junction(per_cam, lane_w: float | None, stripe_w: float = STRIPE_W_M) -> Junction | None:
    """One frame. ``per_cam``: list of (cam, (rows, tracks, used), cues)."""
    lw = float(lane_w) if lane_w else 1.0
    side_cam = {}
    for cam, traced, _cues in per_cam:
        side_cam.setdefault(_cam_side(cam), (cam, traced))
    ev = {}
    for side in (1, -1):
        if side not in side_cam:
            return None
        _cam, (_rows, tracks, _used) = side_cam[side]
        ev[side] = _side_event(tracks, side)
    (eL, xL, tL), (eR, xR, tR) = ev[1], ev[-1]
    events = {"L": (eL, xL), "R": (eR, xR)}
    if "BEND_IN" in (eL, eR):
        side = 1 if eR == "BEND_IN" else -1  # right line crossing over = left turn
        x = xR if eR == "BEND_IN" else xL
        return Junction(kind="corner_L" if side > 0 else "corner_R", near_m=x, center_m=x - 0.5 * lw, far_m=None,
                        left=side > 0, straight=False, right=side < 0, confidence=0.7, events=events)
    open_ev = ("BEND_OUT", "END")

    # free bars (unclaimed wide runs) and resumed lines, from every camera
    bars = []  # (x, y_lo, y_hi)
    for cam, (rows, _tracks, used), _cues in per_cam:
        for r, runs in rows.items():
            for i, (a, b, _cm, gw, ya, yb, gx) in enumerate(runs):
                if i in used.get(r, ()) or gw < JUNCTION_BAR_MIN_M:
                    continue
                bars.append((gx, min(ya, yb), max(ya, yb)))

    def resumed(side, x_n, y_line):
        _cam, (_rows, tracks, _used) = side_cam[side]
        out = [t["first_x"] for t in tracks
               if t["first_x"] > x_n + JUNCTION_RESUME_MIN_M and len(t["pts"]) >= 3
               and abs(t["pts"][0][1] - y_line) < JUNCTION_RESUME_Y_TOL_M]
        return min(out) if out else None

    if eL in open_ev and eR in open_ev and abs(xL - xR) < JUNCTION_PAIR_TOL_M:
        x_n = min(xL, xR)
        beyond = [b for b in bars if b[0] > x_n + 0.25]
        block = [b for b in beyond if b[1] < JUNCTION_BAR_CENTRE_M and b[2] > -JUNCTION_BAR_CENTRE_M]
        pieces = [b for b in beyond if b not in block]
        rl = resumed(1, x_n, tL["last_y"])
        rr = resumed(-1, x_n, tR["last_y"])
        left, right = eL == "BEND_OUT", eR == "BEND_OUT"
        far = None
        conf = 0.6 + (0.1 if eL == eR else 0.0)
        if block:
            far = min(b[0] for b in block)
            straight = False
            kind = "t_end" if (left or right) else "dead_end"
            conf += 0.2
        elif rl is not None or rr is not None or pieces:
            far_c = [v for v in (rl, rr) if v is not None] + [b[0] for b in pieces]
            far = min(far_c)
            straight = True
            kind = "plus" if (left or right) else "gap_straight"
            conf += 0.2
        else:
            straight = None
            kind = "plus" if (left or right) else "lane_lost"
        w_cross = (far - x_n) if far is not None and far - x_n > 0.3 else lw
        return Junction(kind=kind, near_m=x_n, center_m=x_n + 0.5 * w_cross, far_m=far, left=left,
                        straight=straight, right=right, confidence=min(1.0, conf), events=events)

    for side, (e_o, x_o, t_o), (e_c, x_c, t_c) in ((1, ev[1], ev[-1]), (-1, ev[-1], ev[1])):
        if e_o != "BEND_OUT" or e_c not in ("CONT", "EDGE") or t_c is None:
            continue
        r_o = resumed(side, x_o, t_o["last_y"])
        need = x_o + (r_o - x_o if r_o is not None else lw) + JUNCTION_SIDE_MARGIN_M
        if x_c < need or not _straight(t_c["pts"]):
            continue
        w_cross = (r_o - x_o) if r_o is not None and r_o - x_o > 0.3 else lw
        return Junction(kind="side_L" if side > 0 else "side_R", near_m=x_o, center_m=x_o + 0.5 * w_cross,
                        far_m=r_o, left=side > 0, straight=True, right=side < 0,
                        confidence=0.7 + (0.2 if r_o is not None else 0.0), events=events)
    for side, (e_o, x_o, _t_o), (e_c, _x_c, _t_c) in ((1, ev[1], ev[-1]), (-1, ev[-1], ev[1])):
        if e_o == "BEND_OUT" and e_c in ("CONT", "EDGE", "CURVE"):
            # one side opens, the other not (yet) seen far enough: a branch or a
            # corner's inner line. Not acted on; the behaviour slows down for it.
            return Junction(kind="opening_L" if side > 0 else "opening_R", near_m=x_o, center_m=x_o + 0.5 * lw,
                            far_m=None, left=side > 0 or None, straight=None, right=side < 0 or None,
                            confidence=0.4, events=events)
    return None


def yellow_view(per_cam, centre_m: float = 0.30) -> dict:
    """Where is ANY yellow (scan used while crossing blind / looking around).

    ``ahead`` / ``left`` / ``right``: nearest x (m) of yellow in the centre band
    (|y| < centre_m), left of it, right of it; ``bar``: nearest x of a wide run
    covering the whole centre band (a line across the way ahead). None = none.
    """
    out = {"ahead": None, "left": None, "right": None, "bar": None}

    def put(k, x):
        if out[k] is None or x < out[k]:
            out[k] = x

    for _cam, (rows, _tracks, _used), _cues in per_cam:
        for runs in rows.values():
            for (_a, _b, _cm, gw, ya, yb, gx) in runs:
                lo, hi = min(ya, yb), max(ya, yb)
                if hi > -centre_m and lo < centre_m:
                    put("ahead", gx)
                if hi > centre_m:
                    put("left", gx)
                if lo < -centre_m:
                    put("right", gx)
                if gw >= JUNCTION_BAR_MIN_M and lo < -JUNCTION_BAR_CENTRE_M and hi > JUNCTION_BAR_CENTRE_M:
                    put("bar", gx)
    return out


class JunctionTracker:
    """2-frame persistence for :func:`classify_junction`."""

    def __init__(self) -> None:
        self.prev: Junction | None = None

    def reset(self) -> None:
        self.prev = None

    def update(self, j: Junction | None, dx_m: float) -> Junction | None:
        if j is None:
            self.prev = None
            return None
        p = self.prev
        if p is not None and _same_family(p.kind, j.kind) and abs(j.near_m - (p.near_m - abs(dx_m))) < JUNCTION_PERSIST_TOL_M:
            j.frames = p.frames + 1
        j.seen = (j.frames >= JUNCTION_PERSIST_FRAMES and j.confidence >= JUNCTION_MIN_CONF
                  and not j.kind.startswith("opening"))
        self.prev = j
        return j


def _same_family(a: str, b: str) -> bool:
    """plus / t_end / gap_straight / lane_lost refine as the far side comes into view."""
    both = {"plus", "t_end", "gap_straight", "lane_lost", "dead_end"}
    if a == b or (a in both and b in both):
        return True
    return {a, b} in ({"opening_L", "side_L"}, {"opening_R", "side_R"})
