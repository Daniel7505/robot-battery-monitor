"""Row traces, stripe edges, and the sharp-corner cue."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.lane_camera import STRIPE_W_M, CameraModel
from src.lane_mask import _runs

# --------------------------------------------------------------------------
# Sharp-corner cue (Dan's idea: "the line widens, then disappears").
#
# Walking a lane line up the image (near -> far), its row run stays one stripe
# wide. Where the line turns 90 deg the run in one row suddenly spans far more
# than a stripe (the line now runs ACROSS the row), smeared toward the turn,
# and the rows beyond it have no yellow where the line would have continued.
# A rounded corner widens gradually over many rows instead (ramp), so it is
# not flagged. No track knowledge: only the camera model and the mask.
# --------------------------------------------------------------------------

CORNER_WIDE_FACTOR = 3.0  # crossbar run >= this x measured stripe width (ground metres)
CORNER_WIDE_MIN_M = 0.15  # ... and at least this wide (2.5 stripes)
CORNER_NORMAL_FACTOR = 1.6  # run <= this x stripe = still a normal line row
CORNER_RAMP_MAX_M = 0.15  # last normal row -> crossbar row (x). Rounded corners ramp over 0.3+ m
CORNER_MAX_RAMP_ROWS = 1  # rows between normal and crossbar width (a bend has several)
CORNER_MIN_LINE_ROWS = 4  # normal rows tracked before the crossbar
CORNER_MIN_SMEAR_M = 0.12  # smear past the line on the turn side
CORNER_EMPTY_BAND_M = 0.10  # half-width of the "line would continue here" band
CORNER_EMPTY_MAX_PX = 2  # yellow pixels allowed in that band beyond the crossbar
CORNER_CLIP_MARGIN_M = 0.12  # fit points this close to / past the crossbar are dropped
CORNER_PERSIST_FRAMES = 2
CORNER_PERSIST_TOL_M = 0.25  # |d_now - (d_before - dx)|
CORNER_MIN_CONF = 0.6  # ``seen`` needs this (after persistence)


@dataclass
class CornerCue:
    """One camera's view of one line turning sharply."""

    cam: str
    direction: int  # +1 = smear toward robot left (+y) = left turn, -1 = right
    role: str  # "outer": line on the far side of the smear (crosses the lane), "inner"
    x_m: float  # ground x of the crossbar centre at the line (robot frame, ahead of the axle)
    line_y_m: float  # ground y of the line just before the crossbar
    wide_m: float  # crossbar run ground width
    stripe_m: float  # measured stripe width of this line (median of its normal rows)
    ramp_m: float  # x from the last normal row to the crossbar row
    empty_beyond: bool | None  # None = crossbar at the top of the view, can't tell
    row: int
    col: float


@dataclass
class CornerEstimate:
    """Fused corner cue (both cameras, persistence over frames)."""

    seen: bool  # persistent for CORNER_PERSIST_FRAMES and conf >= CORNER_MIN_CONF
    direction: int  # +1 left, -1 right
    distance_m: float  # ahead of the axle to the outer line's crossbar (stripe centre)
    confidence: float
    frames: int  # consecutive consistent frames
    cams: str  # e.g. "nadir_right outer + nadir_left inner"
    clip_x_m: float  # fit points with x >= this were dropped this frame
    cues: list = field(default_factory=list)

    @property
    def side(self) -> str:
        return "left" if self.direction > 0 else "right"


def _row_runs_ground(cam: CameraModel, mask, w: int, row: int):
    """[(a, b, col_mid, ground_width_m, y_a, y_b, x_m)] for one row's yellow runs."""
    out = []
    for a, b in _runs(mask[row * w: (row + 1) * w]):
        p0 = cam.pixel_to_ground(a - 0.5, row)
        p1 = cam.pixel_to_ground(b + 0.5, row)
        if p0 is None or p1 is None:
            continue
        out.append((a, b, 0.5 * (a + b), math.hypot(p1[0] - p0[0], p1[1] - p0[1]), p0[1], p1[1],
                    0.5 * (p0[0] + p1[0])))
    return out


def trace_lines(cam: CameraModel, mask, width: int | None = None, height: int | None = None,
                *, stripe_w: float = STRIPE_W_M):
    """Follow every lane line up the image (near -> far).

    Returns ``(rows, tracks, used)``: ``rows[r]`` = that row's yellow runs with
    ground widths (:func:`_row_runs_ground`); each track is a dict with its
    normal-width rows (``cols``, ``widths``, ground ``pts``), ``first_x`` /
    ``last_x`` / ``last_y`` and how it ended (``end``): ``"wide"`` (crossbar row
    in ``wide``), ``"gap"`` (no paint where it would continue), ``"edge"`` (left
    the picture sideways) or ``"top"`` (still going at the top row). ``used``
    = {row: set(run indexes claimed by a track)}.
    """
    w = int(width or cam.width)
    h = int(height or cam.height)
    rows = {r: _row_runs_ground(cam, mask, w, r) for r in range(h)}
    tracks: list[dict] = []
    used_all: dict[int, set] = {}
    for r in range(h - 1, -1, -1):
        runs = rows[r]
        used = used_all.setdefault(r, set())
        for t in tracks:
            if t["state"] != "on":
                continue
            cols = t["cols"]
            if len(cols) >= 2:
                (r1, c1), (r0, c0) = cols[-1], cols[max(0, len(cols) - 4)]
                slope = (c1 - c0) / (r1 - r0) if r1 != r0 else 0.0
                pred = c1 + slope * (r - r1)
            else:
                pred = cols[-1][1]
            t["pred"] = pred
            tol = 3.0 + 0.6 * t["px"]
            best = None
            for i, run in enumerate(runs):
                a, b = run[0], run[1]
                if a - tol <= pred <= b + tol:
                    d = 0.0 if a <= pred <= b else min(abs(pred - a), abs(pred - b))
                    if best is None or d < best[0]:
                        best = (d, i)
            if best is None:
                t["gap"] += 1
                if t["gap"] > 3:
                    t["state"] = "ended"
                    t["end"] = "edge" if (t["edge_touch"] or pred < 1.0 or pred > w - 2.0) else "gap"
                continue
            i = best[1]
            used.add(i)
            a, b, cm, gw, ya, yb, gx = runs[i]
            ref = _median(t["widths"][:6])  # the line's own stripe width, from its nearest rows
            t["gap"] = 0
            if gw <= CORNER_NORMAL_FACTOR * max(ref, 0.75 * stripe_w):
                t["widths"].append(gw)
                t["cols"].append((r, cm))
                t["px"] = b - a + 1
                t["last_x"] = gx
                t["last_y"] = 0.5 * (ya + yb)
                t["pts"].append((gx, 0.5 * (ya + yb)))
                t["edge_touch"] = a == 0 or b == w - 1
            elif gw >= max(CORNER_WIDE_FACTOR * ref, CORNER_WIDE_MIN_M):
                t["state"] = "wide"
                t["end"] = "wide"
                t["wide"] = (r, a, b, gw, ya, yb, pred)
            else:
                t["ramp_rows"] += 1  # widening, keep following its centre
                t["cols"].append((r, pred))
        for i, run in enumerate(runs):
            if i in used or len(tracks) >= 8:
                continue
            a, b, cm, gw, ya, yb, gx = run
            if gw <= CORNER_NORMAL_FACTOR * stripe_w and a > 0 and b < w - 1:
                used.add(i)
                tracks.append({"state": "on", "cols": [(r, cm)], "widths": [gw], "px": b - a + 1, "gap": 0,
                               "ramp_rows": 0, "last_x": gx, "last_y": 0.5 * (ya + yb), "first_x": gx,
                               "first_row": r, "pts": [(gx, 0.5 * (ya + yb))], "edge_touch": False,
                               "end": None, "cam": cam.name})
    for t in tracks:
        if t["state"] == "on":
            t["end"] = "top"
    return rows, tracks, used_all


BAR_FILTER_MIN_M = 0.20  # free row run this wide (ground) = a line across the view


def _drop_bar_points(pts, traced):
    """Drop fit points that lie on a free wide run (a road line running across).

    At a crossing the far side's lines run across the view; their row runs are
    wide and claimed by no lane line (:func:`trace_lines`), and their column
    runs give points all along them. Only used while a junction is near (the
    controller sets ``LaneTracker.bar_filter_frames``)."""
    rows, _tracks, used = traced
    bad: dict[int, list] = {}
    for r, runs in rows.items():
        for i, (a, b, _cm, gw, *_rest) in enumerate(runs):
            if gw >= BAR_FILTER_MIN_M and i not in used.get(r, ()):
                bad.setdefault(r, []).append((a - 1.0, b + 1.0))
    if not bad:
        return pts
    out = []
    for p in pts:
        spans = bad.get(int(round(p.row)))
        if spans and any(a <= p.col <= b for a, b in spans):
            continue
        out.append(p)
    return out


def corner_cues(cam: CameraModel, mask, width: int | None = None, height: int | None = None,
                *, stripe_w: float = STRIPE_W_M, traced=None) -> list[CornerCue]:
    """Lines in one camera that 'widen, then disappear' (see module notes).

    Each line is followed from the bottom row upward (nearest run to the
    column extrapolated from its last rows). The first row whose run is
    >= CORNER_WIDE_FACTOR x the line's measured stripe width, reached within
    CORNER_RAMP_MAX_M of a normal-width row, is the crossbar. The smear side
    gives the turn direction; the band where the line would continue is
    checked for yellow in the farther rows. ``traced``: a :func:`trace_lines`
    result to reuse.
    """
    w = int(width or cam.width)
    h = int(height or cam.height)
    if not mask or w < 4 or h < 4:
        return []
    rows, tracks, _used = traced if traced is not None else trace_lines(cam, mask, w, h, stripe_w=stripe_w)
    done = [t for t in tracks if t["end"] == "wide"]
    cues: list[CornerCue] = []
    for t in done:
        if len(t["widths"]) < CORNER_MIN_LINE_ROWS:
            continue
        r, a, b, gw, ya, yb, pred = t["wide"]
        if t["ramp_rows"] > CORNER_MAX_RAMP_ROWS:
            continue  # widened gradually over several rows: a rounded bend, not a crossbar
        ref = _median(t["widths"][:6])
        near_edge = cam.pixel_to_ground(pred, r + 0.5)
        line_pt = cam.pixel_to_ground(pred, r)
        if near_edge is None or line_pt is None:
            continue
        ramp = near_edge[0] - t["last_x"]
        if ramp > CORNER_RAMP_MAX_M:
            continue
        yl = t["last_y"]
        y_hi, y_lo = max(ya, yb), min(ya, yb)
        ext_left, ext_right = y_hi - yl, yl - y_lo
        if max(ext_left, ext_right) < CORNER_MIN_SMEAR_M:
            continue
        direction = 1 if ext_left > ext_right else -1
        role = "outer" if (yl < 0.0) == (direction > 0) else "inner"
        # rows beyond: is there yellow where the line would have continued?
        empty: bool | None = None
        rr = r - 1
        while rr >= 0 and any(ra <= pred <= rb and rg >= CORNER_WIDE_FACTOR * ref * 0.6
                              for ra, rb, _c, rg, *_ in rows[rr]):
            rr -= 1  # still inside the crossbar stripe
        if rr >= 1:
            band = 0
            for q in range(rr, -1, -1):
                pl = cam.pixel_to_ground(pred, q)
                if pl is None:
                    continue
                # band in pixels at this row: CORNER_EMPTY_BAND_M on the ground
                p_r = cam.pixel_to_ground(pred + 1.0, q)
                m_per_px = abs(p_r[1] - pl[1]) if p_r is not None else stripe_w
                half_px = CORNER_EMPTY_BAND_M / max(1e-3, m_per_px)
                c0, c1 = int(max(0, pred - half_px)), int(min(w - 1, pred + half_px))
                band += sum(mask[q * w + c0: q * w + c1 + 1])
            empty = band <= CORNER_EMPTY_MAX_PX
        cue = CornerCue(cam=cam.name, direction=direction, role=role,
                        x_m=near_edge[0] + 0.5 * stripe_w, line_y_m=yl, wide_m=gw, stripe_m=ref,
                        ramp_m=ramp, empty_beyond=empty, row=r, col=pred)
        t["cue"] = cue
        cues.append(cue)
    return cues


def _median(v):
    s = sorted(v)
    n = len(s)
    if not n:
        return 0.0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def fuse_corner_cues(cues: list[CornerCue]) -> tuple[int, float, float, str, list[CornerCue]] | None:
    """(direction, distance_m, confidence, cams, used cues) from one frame's cues, or None.

    Needs an OUTER cue (the line that crosses the lane: right line smearing
    left = left turn). An inner cue (the inner line bending away) of the same
    direction, nearer by about a lane width, adds confidence; on its own it
    never triggers (a rounded corner still has a mitred inner line).
    """
    outer = [c for c in cues if c.role == "outer"]
    if not outer:
        return None
    dirs = {c.direction for c in outer}
    if len(dirs) != 1:
        return None
    d = dirs.pop()
    best = min(outer, key=lambda c: c.x_m)
    conf = 0.5
    if best.empty_beyond:
        conf += 0.25
    elif best.empty_beyond is False:
        conf -= 0.2
    inner = [c for c in cues if c.role == "inner" and c.direction == d and c.x_m < best.x_m - 0.2]
    used = [best]
    if inner:
        conf += 0.2
        used.append(min(inner, key=lambda c: c.x_m))
    if any(c.role == "outer" and c is not best and abs(c.x_m - best.x_m) < 0.15 for c in outer):
        conf += 0.05  # both cameras see the same crossbar
    cams = " + ".join(f"{c.cam} {c.role}" for c in used)
    return d, best.x_m, max(0.0, min(1.0, conf)), cams, used
