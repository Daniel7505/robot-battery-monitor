"""Waypoint tracks: centreline, painted lane lines, sampler and GPS referee.

A track is a list of centreline waypoints (metres, world ENU, robot starts at
the first waypoint facing the second). Each interior waypoint may carry a
corner ``radius_m`` (0 = sharp, the default) and any waypoint may carry a
``lane_width_m`` (linear in arc length between the waypoints that set one).
The painted lines are the exact offset curves of that centreline:

* sharp corner: both lines get a true mitre (intersection of the offset legs),
* rounded corner of centre radius r: outer line radius r + w/2, inner line
  radius r - w/2; if that is <= 0 the inner line gets a sharp mitre instead
  (the trimmed offset curve, so it never folds back on itself).

Track files (``tracks/*.json``) are plain data so a Paint / Unity / CAD
importer can write the same format. The geometry here is used for painting
the world, the offline sim, drift scoring and the finish referee only; the
onboard lane keep never reads it.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRACKS_DIR = os.path.join(ROOT, "tracks")
DEFAULT_LANE_W_M = 1.30
DEFAULT_STRIPE_W_M = 0.06
ARC_STEP_M = 0.02  # fillet sampling, measured on the widest offset of the corner
_EPS = 1e-9


@dataclass
class TrackSpec:
    name: str
    waypoints: list  # [(x, y)]
    radii: list  # centre corner radius per waypoint (ends ignored), 0 = sharp
    widths: list  # lane width per waypoint (filled in)
    stripe_w_m: float = DEFAULT_STRIPE_W_M
    paint_before_start_m: float = 0.3
    paint_after_finish_m: float = 0.5
    start_bar: bool = True
    finish_bar: bool = True
    ticks: list = field(default_factory=list)  # [(x, y) or (x, y, heading_rad)] snapped to the centreline
    description: str = ""
    path: str | None = None


def _fill_widths(pts, given, default):
    """Linear in arc length between waypoints that set a width; ends hold."""
    s = [0.0]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        s.append(s[-1] + math.hypot(x1 - x0, y1 - y0))
    known = [i for i, w in enumerate(given) if w is not None]
    if not known:
        return [float(default)] * len(pts)
    out = []
    for i in range(len(pts)):
        if given[i] is not None:
            out.append(float(given[i]))
            continue
        lo = max((k for k in known if k < i), default=None)
        hi = min((k for k in known if k > i), default=None)
        if lo is None:
            out.append(float(given[hi]))
        elif hi is None:
            out.append(float(given[lo]))
        else:
            t = (s[i] - s[lo]) / max(_EPS, s[hi] - s[lo])
            out.append(float(given[lo]) + t * (float(given[hi]) - float(given[lo])))
    return out


def spec_from_dict(d: dict, path: str | None = None) -> TrackSpec:
    pts, radii, widths = [], [], []
    for w in d["waypoints"]:
        if isinstance(w, dict):
            pts.append((float(w["x"]), float(w["y"])))
            radii.append(float(w.get("radius_m", 0.0)))
            widths.append(w.get("lane_width_m"))
        else:
            pts.append((float(w[0]), float(w[1])))
            radii.append(0.0)
            widths.append(None)
    if len(pts) < 2:
        raise ValueError("a track needs at least 2 waypoints")
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if math.hypot(x1 - x0, y1 - y0) < 1e-6:
            raise ValueError(f"duplicate waypoint at ({x0}, {y0})")
    return TrackSpec(
        name=str(d.get("name") or (os.path.splitext(os.path.basename(path))[0] if path else "track")),
        waypoints=pts,
        radii=radii,
        widths=_fill_widths(pts, widths, d.get("lane_width_m", DEFAULT_LANE_W_M)),
        stripe_w_m=float(d.get("stripe_width_m", DEFAULT_STRIPE_W_M)),
        paint_before_start_m=float(d.get("paint_before_start_m", 0.3)),
        paint_after_finish_m=float(d.get("paint_after_finish_m", 0.5)),
        start_bar=bool(d.get("start_bar", True)),
        finish_bar=bool(d.get("finish_bar", True)),
        ticks=[tuple(float(v) for v in p[:3]) for p in d.get("ticks", [])],
        description=str(d.get("description", "")),
        path=path,
    )


def track_path(name_or_path: str) -> str:
    """'corner90' -> tracks/corner90.json; a path is returned as is."""
    p = os.path.expanduser(str(name_or_path))
    if os.path.isfile(p):
        return p
    for cand in (os.path.join(TRACKS_DIR, p), os.path.join(TRACKS_DIR, p + ".json")):
        if os.path.isfile(cand):
            return cand
    raise FileNotFoundError(f"track {name_or_path!r} not found (looked in {TRACKS_DIR})")


def load_track(name_or_path: str) -> "TrackGeometry":
    p = track_path(name_or_path)
    with open(p, encoding="utf-8") as fh:
        return TrackGeometry(spec_from_dict(json.load(fh), p))


# --------------------------------------------------------------------------
# Offset curves
# --------------------------------------------------------------------------


def _unit(dx, dy):
    L = math.hypot(dx, dy)
    return (dx / L, dy / L)


def _turn(u0, u1) -> float:
    return math.atan2(u0[0] * u1[1] - u0[1] * u1[0], u0[0] * u1[0] + u0[1] * u1[1])


def _intersect(a, va, b, vb):
    """Point where line a + t*va meets b + s*vb (None if parallel)."""
    den = va[0] * vb[1] - va[1] * vb[0]
    if abs(den) < 1e-12:
        return None
    t = ((b[0] - a[0]) * vb[1] - (b[1] - a[1]) * vb[0]) / den
    return (a[0] + t * va[0], a[1] + t * va[1])


class TrackGeometry:
    """Everything derived from a TrackSpec."""

    def __init__(self, spec: TrackSpec):
        self.spec = spec
        P = list(spec.waypoints)
        self._u = [_unit(b[0] - a[0], b[1] - a[1]) for a, b in zip(P, P[1:])]
        self._phi = [0.0] + [_turn(self._u[i - 1], self._u[i]) for i in range(1, len(P) - 1)] + [0.0]
        max_off = max(spec.widths) / 2.0 + spec.stripe_w_m / 2.0
        # Same number of samples per corner on every offset, so the two edges of a
        # stripe pair up one-to-one.
        self._k = [1] * len(P)
        for j in range(1, len(P) - 1):
            r = spec.radii[j]
            if r > 0 and abs(self._phi[j]) > 1e-9:
                self._k[j] = max(2, int(math.ceil(abs(self._phi[j]) * (r + max_off) / ARC_STEP_M)) + 1)
        self.centerline = self.offset_curve(0.0)
        self._cum = [0.0]
        for a, b in zip(self.centerline, self.centerline[1:]):
            self._cum.append(self._cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length_m = self._cum[-1]
        # Fail early (with the leg named) if any painted edge would fold back.
        h = spec.stripe_w_m / 2.0
        for side in (1.0, -1.0):
            for e in (h, -h):
                self.offset_curve(0.5 * side, e)

    # --- construction -----------------------------------------------------
    def offset_curve(self, frac: float, extra: float = 0.0, extend: bool = False) -> list:
        """Curve at signed left offset ``frac * lane_width + extra`` (frac = +-0.5 for the lines).

        ``extend`` adds the paint run-in before the start and run-out after the
        finish (straight, along the first / last leg).
        """
        sp = self.spec
        P = list(sp.waypoints)
        o = [frac * w + extra for w in sp.widths]
        n = len(P)
        legs = []  # (A, B, unit_dir) of each offset leg
        for i in range(n - 1):
            ux, uy = self._u[i]
            nx, ny = -uy, ux
            A = (P[i][0] + o[i] * nx, P[i][1] + o[i] * ny)
            B = (P[i + 1][0] + o[i + 1] * nx, P[i + 1][1] + o[i + 1] * ny)
            legs.append((A, B, _unit(B[0] - A[0], B[1] - A[1])))
        out = [legs[0][0]]
        for j in range(1, n - 1):
            A0, B0, v0 = legs[j - 1]
            A1, B1, v1 = legs[j]
            X = _intersect(A0, v0, A1, v1)
            if X is None:  # collinear legs
                X = ((B0[0] + A1[0]) / 2.0, (B0[1] + A1[1]) / 2.0)
            k = self._k[j]
            phi = self._phi[j]
            r = sp.radii[j]
            R = r - math.copysign(1.0, phi) * o[j] if r > 0 else 0.0
            if k == 1 or R <= 1e-9:
                out.extend([X] * k)
                continue
            psi = _turn(v0, v1)
            t = R * math.tan(abs(psi) / 2.0)
            T1 = (X[0] - v0[0] * t, X[1] - v0[1] * t)
            sg = math.copysign(1.0, psi)
            C = (T1[0] - sg * v0[1] * R, T1[1] + sg * v0[0] * R)
            a1 = math.atan2(T1[1] - C[1], T1[0] - C[0])
            for m in range(k):
                a = a1 + psi * m / (k - 1)
                out.append((C[0] + R * math.cos(a), C[1] + R * math.sin(a)))
        out.append(legs[-1][1])
        self._check_no_foldback(out, frac, extra)
        if extend:
            u0, u1 = legs[0][2], legs[-1][2]
            b, f = sp.paint_before_start_m, sp.paint_after_finish_m
            if b > 0:
                out.insert(0, (out[0][0] - u0[0] * b, out[0][1] - u0[1] * b))
            if f > 0:
                out.append((out[-1][0] + u1[0] * f, out[-1][1] + u1[1] * f))
        return out

    def _check_no_foldback(self, pts, frac, extra):
        """A leg too short for its corners makes the offset run backwards."""
        P = self.spec.waypoints
        # Map each output point back to its leg: points of vertex j sit between legs j-1 and j.
        idx = 0
        owners = [0]
        for j in range(1, len(P) - 1):
            owners.extend([j] * self._k[j])
        owners.append(len(P) - 1)
        for a in range(len(pts) - 1):
            p, q = pts[a], pts[a + 1]
            if math.hypot(q[0] - p[0], q[1] - p[1]) < 1e-9:
                continue
            j0, j1 = owners[a], owners[a + 1]
            if j0 == j1:
                continue  # inside a fillet
            u = self._u[j0]
            if (q[0] - p[0]) * u[0] + (q[1] - p[1]) * u[1] < -1e-9:
                raise ValueError(
                    f"track {self.spec.name}: leg {j0}->{j1} is too short for its corners at offset "
                    f"{frac:+.2f}*w{extra:+.3f} m (offset line folds back)"
                )
            idx += 1

    def lane_line(self, side: float, extend: bool = True) -> list:
        """Centre of the painted line, side +1 left / -1 right."""
        return self.offset_curve(0.5 * side, 0.0, extend)

    def stripe_edges(self, side: float):
        """(left_edge, right_edge) of one painted stripe, paired point for point."""
        h = self.spec.stripe_w_m / 2.0
        return (self.offset_curve(0.5 * side, +h, True), self.offset_curve(0.5 * side, -h, True))

    def dense_centerline(self, step: float = 0.04, extend_before: float = 0.0, extend_after: float = 0.0) -> list:
        """Centreline resampled every ~step m (for scoring)."""
        pts = []
        c = self.centerline
        if extend_before > 0:
            u = self._u[0]
            n = max(1, int(round(extend_before / step)))
            pts += [(c[0][0] - u[0] * extend_before * (1 - i / n), c[0][1] - u[1] * extend_before * (1 - i / n)) for i in range(n)]
        for a, b in zip(c, c[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            if L < 1e-12:
                continue
            n = max(1, int(math.ceil(L / step)))
            pts += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
        pts.append(c[-1])
        if extend_after > 0:
            u = self._u[-1]
            n = max(1, int(round(extend_after / step)))
            pts += [(c[-1][0] + u[0] * extend_after * i / n, c[-1][1] + u[1] * extend_after * i / n) for i in range(1, n + 1)]
        return pts

    # --- queries ------------------------------------------------------------
    def width_at_s(self, s: float) -> float:
        """Lane width at centreline arc length s (linear between waypoints)."""
        P, W = self.spec.waypoints, self.spec.widths
        acc = 0.0
        for i in range(len(P) - 1):
            L = math.hypot(P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1])
            if s <= acc + L or i == len(P) - 2:
                t = max(0.0, min(1.0, (s - acc) / L))
                return W[i] + t * (W[i + 1] - W[i])
            acc += L
        return W[-1]

    def heading_at_vertex(self, i: int) -> float:
        c = self.centerline
        a, b = c[max(0, i - 1)], c[min(len(c) - 1, i + 1)]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def project(self, x: float, y: float, hint: int | None = None, window: int = 200) -> dict:
        """Nearest centreline point: arc length s, signed lateral (left +), heading, index.

        Past the ends the first / last leg is extended, so lateral stays a true
        sideways offset on the run-in and run-out.
        """
        c = self.centerline
        n = len(c) - 1
        lo, hi = (0, n) if hint is None else (max(0, hint - window), min(n, hint + window))
        best = None
        for i in range(lo, hi):
            (x0, y0), (x1, y1) = c[i], c[i + 1]
            vx, vy = x1 - x0, y1 - y0
            L2 = vx * vx + vy * vy
            if L2 < 1e-18:
                continue
            t = ((x - x0) * vx + (y - y0) * vy) / L2
            tc = t
            if not (i == 0 and t < 0) and not (i == n - 1 and t > 1):
                tc = max(0.0, min(1.0, t))
            px, py = x0 + tc * vx, y0 + tc * vy
            d = math.hypot(x - px, y - py)
            if best is None or d < best[0] - 1e-12:
                L = math.sqrt(L2)
                lat = (vx * (y - py) - vy * (x - px)) / L
                best = (d, i, self._cum[i] + tc * L, lat, math.atan2(vy, vx))
        d, i, s, lat, hd = best
        return {"s_m": s, "lateral_m": lat, "dist_m": d, "heading_rad": hd, "index": i}

    # --- referee --------------------------------------------------------------
    @property
    def start_xy(self):
        return self.spec.waypoints[0]

    @property
    def start_heading(self) -> float:
        u = self._u[0]
        return math.atan2(u[1], u[0])

    @property
    def finish_xy(self):
        return self.spec.waypoints[-1]

    @property
    def finish_heading(self) -> float:
        u = self._u[-1]
        return math.atan2(u[1], u[0])

    def past_finish_m(self, x: float, y: float) -> tuple[float, float]:
        """(distance past the finish line along the last leg, sideways from its centre)."""
        fx, fy = self.finish_xy
        u = self._u[-1]
        dx, dy = x - fx, y - fy
        return (dx * u[0] + dy * u[1], -dx * u[1] + dy * u[0])

    def crossed_finish(self, x: float, y: float, tol_m: float = 0.0) -> bool:
        """GPS referee: across the finish line, within a lane width of its centre,
        and closer to the end of the course than its middle (so an earlier leg that
        passes behind the finish line cannot trigger it)."""
        along, side = self.past_finish_m(x, y)
        if along < -tol_m or abs(side) > self.spec.widths[-1]:
            return False
        return self.project(x, y)["s_m"] >= 0.5 * self.length_m


class FinishReferee:
    """Controller-side referee: S default (x >= 16.5) or a track file's finish."""

    def __init__(self, track: TrackGeometry | None = None, finish_x_m: float = 16.5, source: str = "default S"):
        self.track = track
        self.finish_x_m = float(finish_x_m)
        self.source = source

    @property
    def name(self) -> str:
        return self.track.spec.name if self.track is not None else "s"

    def crossed(self, x: float, y: float) -> bool:
        if self.track is None:
            return float(x) >= self.finish_x_m
        return self.track.crossed_finish(float(x), float(y))

    def describe(self) -> str:
        if self.track is None:
            return f"finish x>={self.finish_x_m:g} m ({self.source})"
        fx, fy = self.track.finish_xy
        return (f"track {self.name}: finish ({fx:g}, {fy:g}) heading "
                f"{math.degrees(self.track.finish_heading):+.0f}deg, {self.track.length_m:.2f} m ({self.source})")


TRACK_FILE_TAG = "# TRACK_FILE "


def track_file_from_world(world_path: str | None) -> str | None:
    """The ``# TRACK_FILE tracks/x.json`` tag written by track_builder into a world."""
    if not world_path or not os.path.isfile(world_path):
        return None
    with open(world_path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith(TRACK_FILE_TAG):
                return line[len(TRACK_FILE_TAG):].strip()
    return None


def referee_for(world_path: str | None = None, env: dict | None = None, finish_x_m: float = 16.5) -> FinishReferee:
    """RBM_TRACK (name or file) > TRACK_FILE tag in the loaded world > S default."""
    env = os.environ if env is None else env
    choice = (env.get("RBM_TRACK") or "").strip()
    source = "RBM_TRACK"
    if not choice:
        choice = track_file_from_world(world_path) or ""
        source = f"world {os.path.basename(world_path)}" if choice else "default S"
    if not choice:
        return FinishReferee(None, finish_x_m, source)
    p = choice if os.path.isabs(choice) else os.path.join(ROOT, choice)
    return FinishReferee(load_track(p if os.path.isfile(p) else choice), finish_x_m, source)
