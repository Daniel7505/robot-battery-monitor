"""Centre-arc fit and the per-frame lane tracker."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

from src.lane_camera import STRIPE_W_M, CameraModel, NADIR_CAMS, camera_coverage
from src.lane_lines import (
    CORNER_CLIP_MARGIN_M,
    CORNER_MIN_CONF,
    CORNER_PERSIST_FRAMES,
    CORNER_PERSIST_TOL_M,
    CornerCue,
    CornerEstimate,
    _drop_bar_points,
    fuse_corner_cues,
    trace_lines,
)
from src.lane_mask import YELLOW_THRESH, LinePoint, detect_points, yellow_mask

FALLBACK_LANE_W_M = 1.30  # one-line frames, and the first frames before both lines have been seen
# The near fit looks about this far ahead of the axle. Width logged against
# the paint is the track width there, not the width under the axle.
WIDTH_FIT_AHEAD_M = 0.30
MIN_SIDE_PTS = 4
# Fit windows (floor x ahead of the axle). The cameras no longer see behind
# the axle (bottom row ~0.06 m ahead since the box mount), so the near fit
# must stop before a bend that starts ~0.5 m ahead can bend the axle heading:
# 0.9 m was fine only while ~0.55 m of floor behind the axle anchored it.
# fit_windows() widens them if a camera's bottom row sits farther ahead.
NEAR_MAX_X_M = 0.6  # near-field fit (offset / heading at the axle)
NEAR_MIN_SPAN_M = 0.4  # near window is at least this long past the bottom row
FAR_MIN_X_M = 0.25  # far-field fit (curvature ahead / pursuit goal)
SPLIT_X_M = 0.6  # points nearer than this are judged by the near model

# Plausibility gate (generic: robot kinematics + paint physics, no track data).
# Lane heading relative to the robot can only change between frames by the
# robot's own yaw (measured by the IMU and already applied to the prior) plus
# lane curvature x distance driven.
GATE_MAX_HEADING_RAD = math.radians(45.0)  # lane-keep never runs this crossed
GATE_LANE_KAPPA_MAX = 2.0  # 1/m: tightest lane bend (0.5 m radius) for the jump limit
GATE_HEADING_JUMP_RAD = 0.15  # + kappa_max * |dx| per frame
GATE_OFFSET_JUMP_M = 0.06  # + |dx| * GATE_OFFSET_JUMP_PER_M per frame
GATE_OFFSET_JUMP_PER_M = 0.6
GATE_SINGLE_SIDE_MIN_CONF = 0.45  # a side seen last frame now empty AND conf below this -> reject
GATE_MAX_HELD_FRAMES = 3  # matches rowfit_control.LOST_FRAMES_STOP
# Lane visible for less than this ahead of the axle (lines ending, e.g. at a
# finish bar): the fit is too short to trust heading/curvature, so drive the
# held lane. Short-view holds don't count as lost frames until
# GATE_MAX_SHORT_HOLDS in a row (~0.8 m at cruise), then they do.
GATE_MIN_VIEW_M = 0.5
GATE_MAX_SHORT_HOLDS = 6
# Post-pivot settling: after the relaxed re-acquire window (and again when the
# controller returns to LANE) the jump checks stay widened for a few frames,
# because the first fits of the new leg still settle (only part of it in view).
GATE_SETTLE_FRAMES = 5
GATE_SETTLE_HEADING_RAD = math.radians(25.0)  # added to the heading-jump limit
GATE_SETTLE_OFFSET_M = 0.15  # added to the offset-jump limit

# --------------------------------------------------------------------------
# Lane geometry: centre arc through (0, a0), direction psi, curvature kappa
# --------------------------------------------------------------------------


def signed_lateral(px: float, py: float, a0: float, psi: float, kappa: float) -> float:
    """Signed distance of (px,py) from the centre arc, + = left of it."""
    ux, uy = px, py - a0
    c, s = math.cos(psi), math.sin(psi)
    along = ux * c + uy * s
    lat = -ux * s + uy * c
    A = 2.0 * lat - kappa * (along * along + lat * lat)
    q = 1.0 - kappa * A
    return A / (1.0 + math.sqrt(max(0.0, q)))


def arc_point(a0: float, psi: float, kappa: float, s: float, x0: float = 0.0) -> tuple[float, float]:
    """Point at arc length s from the anchor (x0, a0)."""
    if abs(kappa) < 1e-9:
        fx, fy = s, 0.0
    else:
        fx = math.sin(kappa * s) / kappa
        fy = (1.0 - math.cos(kappa * s)) / kappa
    c, sn = math.cos(psi), math.sin(psi)
    return (x0 + fx * c - fy * sn, a0 + fx * sn + fy * c)


def _solve(A: list[list[float]], b: list[float]) -> list[float] | None:
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-14:
            return None
        M[i], M[p] = M[p], M[i]
        for r in range(n):
            if r != i:
                f = M[r][i] / M[i][i]
                if f:
                    for k in range(i, n + 1):
                        M[r][k] -= f * M[i][k]
    return [M[i][n] / M[i][i] for i in range(n)]


@dataclass
class LaneEstimate:
    valid: bool
    offset_m: float = 0.0  # robot vs lane centre at the axle, + = robot LEFT of centre
    heading_rad: float = 0.0  # robot vs lane direction, + = robot pointed LEFT of lane
    curvature_1pm: float = 0.0  # + = lane bends LEFT
    lookahead_m: float = 0.0  # farthest forward inlier
    confidence: float = 0.0
    n_left: int = 0
    n_right: int = 0
    lane_width_m: float = 0.0
    width_measured: bool = False  # a both-line width has been seen this run
    width_live: bool = False  # this frame's width is the near-window fit, not the fallback
    rms_m: float = 0.0
    width_ok_frac: float = 0.0  # share of points whose floor width ~ 6 cm
    a0: float = 0.0
    psi: float = 0.0
    near_curvature_1pm: float = 0.0
    pixels: dict = field(default_factory=dict)  # cam -> [(col,row,side)] for the HUD
    corner: "CornerEstimate | None" = None  # sharp-corner cue this frame (LaneTracker)
    junction: "object | None" = None  # src.junction_vision.Junction this frame (LaneTracker)
    view: "dict | None" = None  # src.junction_vision.yellow_view: nearest yellow ahead / left / right / bar
    # Plausibility gate (LaneTracker): a rejected fit comes back valid=False,
    # held=True, carrying the last good lane moved by odometry/IMU, and the
    # reason. The rejected fit's own numbers are kept in ``rejected``.
    held: bool = False
    counts_as_lost: bool = True  # held: does it count toward the lost-lane brake
    reject_reason: str = ""
    rejected: dict = field(default_factory=dict)

    def goal_point(self, lookahead_m: float) -> tuple[float, float]:
        """Centre-line point at straight-line distance ~lookahead from the axle."""
        Ld = max(0.05, float(lookahead_m))
        s0 = -self.a0 * math.sin(self.psi)  # arc length to the point abreast
        lo, hi = s0, s0 + Ld + abs(self.a0) + 0.5
        best = arc_point(self.a0, self.psi, self.curvature_1pm, hi)
        if math.hypot(*best) < Ld:
            # tight curl: take the farthest point reachable on the arc
            far, fs = best, hi
            for i in range(1, 41):
                s = s0 + (hi - s0) * i / 40.0
                p = arc_point(self.a0, self.psi, self.curvature_1pm, s)
                if math.hypot(*p) > math.hypot(*far):
                    far, fs = p, s
            return far
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            p = arc_point(self.a0, self.psi, self.curvature_1pm, mid)
            if math.hypot(*p) < Ld:
                lo = mid
            else:
                hi = mid
        return arc_point(self.a0, self.psi, self.curvature_1pm, 0.5 * (lo + hi))


def fit_lane(
    points: list[LinePoint],
    *,
    init: tuple[float, float, float] = (0.0, 0.0, 0.0),
    lane_w: float = FALLBACK_LANE_W_M,
    fit_width: bool = True,
    iters: int = 15,
    huber_m: float = 0.02,
    gate_m: float = 0.12,
) -> tuple[list[float], list[float], float] | None:
    """Robust GN fit. Points must carry side +/-1. Returns (params, weights, rms)."""
    pts = [p for p in points if p.side in (1, -1)]
    if len(pts) < 3:
        return None
    p = [float(init[0]), float(init[1]), float(init[2]), float(lane_w)]
    n_par = 4 if fit_width else 3
    sig_pt = 0.02
    w_prior = float(lane_w)
    wts = [1.0] * len(pts)
    mu = 1e-3

    def _res(q, pt):
        return signed_lateral(pt.x, pt.y, q[0], q[1], q[2]) - pt.side * 0.5 * q[3]

    for it in range(iters):
        r = [_res(p, pt) for pt in pts]
        for i, ri in enumerate(r):
            a = abs(ri)
            if it >= 2 and a > gate_m:
                wts[i] = 0.0
            else:
                wts[i] = 1.0 if a <= huber_m else huber_m / a
        JTJ = [[0.0] * n_par for _ in range(n_par)]
        JTr = [0.0] * n_par
        eps = 1e-6
        for i, pt in enumerate(pts):
            wi = wts[i]
            if wi <= 0.0:
                continue
            J = []
            for k in range(n_par):
                q = p[:]
                q[k] += eps
                J.append((_res(q, pt) - r[i]) / eps)
            for a_ in range(n_par):
                JTr[a_] += wi * J[a_] * r[i]
                for b_ in range(n_par):
                    JTJ[a_][b_] += wi * J[a_] * J[b_]
        # weak priors: curvature toward 0 (sigma 3 1/m), width toward estimate
        reg = [(2, 0.0, 3.0)]
        if fit_width:
            reg.append((3, w_prior, 0.4))
        for k, target, sig in reg:
            g = (sig_pt / sig) ** 2
            JTJ[k][k] += g
            JTr[k] += g * (p[k] - target)
        for k in range(n_par):
            JTJ[k][k] += mu
        step = _solve(JTJ, [-v for v in JTr])
        if step is None:
            return None
        step[1] = max(-0.5, min(0.5, step[1]))
        step[2] = max(-2.0, min(2.0, step[2]))
        for k in range(n_par):
            p[k] += step[k]
        p[2] = max(-6.0, min(6.0, p[2]))
        if max(abs(v) for v in step) < 1e-6:
            break
    r = [_res(p, pt) for pt in pts]
    for i, ri in enumerate(r):
        wts[i] = 0.0 if abs(ri) > gate_m else (1.0 if abs(ri) <= huber_m else huber_m / abs(ri))
    used = [ri for ri, wi in zip(r, wts) if wi > 0]
    if len(used) < 3:
        return None
    rms = math.sqrt(sum(v * v for v in used) / len(used))
    return p, wts, rms


def _init_from_points(pts: list[LinePoint], lane_w: float) -> tuple[float, float, float]:
    """Shared-slope line fit of (y - side*w/2) vs x as a GN starting point."""
    xs = [p.x for p in pts if p.side]
    ys = [p.y - p.side * 0.5 * lane_w for p in pts if p.side]
    if not xs:
        return (0.0, 0.0, 0.0)
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = 0.0 if sxx < 1e-6 else sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = max(-1.5, min(1.5, b))
    a = my - b * mx
    return (a, math.atan(b), 0.0)


def _side_counts(pts) -> tuple[int, int]:
    return (sum(1 for p in pts if p.side == 1), sum(1 for p in pts if p.side == -1))


def _drop_thin_sides(pts) -> None:
    nl, nr = _side_counts(pts)
    for side, n in ((1, nl), (-1, nr)):
        if 0 < n < MIN_SIDE_PTS:
            for p in pts:
                if p.side == side:
                    p.side = 0


def predict_model(
    model: tuple[float, float, float], dx: float, dyaw: float
) -> tuple[float, float, float] | None:
    """Express a centre arc in the robot frame after moving dx and turning dyaw."""
    a0, psi, k = model
    tx = dx * math.cos(0.5 * dyaw)
    ty = dx * math.sin(0.5 * dyaw)
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    ax, ay = -tx, a0 - ty
    nax, nay = c * ax - s * ay, s * ax + c * ay
    npsi = psi - dyaw
    sarc = 0.0
    for _ in range(25):  # slide the anchor along the arc back to x = 0
        px, _py = arc_point(nay, npsi, k, sarc, x0=nax)
        ca = math.cos(npsi + k * sarc)
        if abs(ca) < 0.2:
            return None
        if abs(px) < 1e-7:
            break
        sarc -= px / ca
    px, py = arc_point(nay, npsi, k, sarc, x0=nax)
    return (py, npsi + k * sarc, k)


def fit_windows(cams) -> tuple[float, float, float]:
    """(near_max_x, split_x, far_min_x) for these cameras' floor coverage.

    Defaults NEAR_MAX_X_M / SPLIT_X_M / FAR_MIN_X_M, pushed out so the near
    window keeps >= NEAR_MIN_SPAN_M of floor past the farthest-forward bottom
    row and the far window starts at or after it.
    """
    bots = []
    for cam in cams:
        x = camera_coverage(cam)["bottom_row_x_m"]
        if x is not None:
            bots.append(x)
    bottom = max(bots) if bots else 0.0
    near_max = max(NEAR_MAX_X_M, bottom + NEAR_MIN_SPAN_M)
    split = min(near_max, max(SPLIT_X_M, bottom + NEAR_MIN_SPAN_M))
    far_min = max(FAR_MIN_X_M, bottom)
    return near_max, split, far_min


class LaneTracker:
    """Stateful per-frame lane fit (keeps prior + online lane width)."""

    def __init__(self, cams: dict[str, CameraModel] | None = None, *, yellow_mode: str | None = None) -> None:
        self.cams = dict(cams or NADIR_CAMS)
        self.yellow_mode = yellow_mode
        self.near_max_x, self.split_x, self.far_min_x = fit_windows(self.cams.values())
        self.prior_near: tuple[float, float, float] | None = None
        self.prior_far: tuple[float, float, float] | None = None
        self.lane_w: float | None = None
        self.frames_lost = 0
        self.last: LaneEstimate | None = None
        self._last_good: LaneEstimate | None = None
        self.rejects = 0
        self.short_holds = 0
        self.corner: CornerEstimate | None = None  # this frame's fused corner cue
        self._corner_track: tuple[int, float, int] | None = None  # (dir, distance, frames)
        self.relax_frames = 0  # gate relaxed (jump / vanish / short-view checks off) this many frames
        self.settle_frames = 0  # jump limits widened this many frames (post-pivot settling)
        from src.junction_vision import JunctionTracker

        self.junction = None  # this frame's junction classification (src.junction_vision.Junction)
        self.view: dict | None = None
        self.bar_filter_frames = 0  # drop points on lines running ACROSS the view (junction / crossing)
        self._junction_track = JunctionTracker()

    def reset(self) -> None:
        self.prior_near = None
        self.prior_far = None
        self.lane_w = None
        self.frames_lost = 0
        self.last = None
        self._last_good = None
        self.rejects = 0
        self.short_holds = 0
        self.corner = None
        self._corner_track = None
        self.relax_frames = 0
        self.settle_frames = 0
        self.junction = None
        self._junction_track.reset()

    def begin_settle(self, frames: int = GATE_SETTLE_FRAMES) -> None:
        """Widen the jump checks for ``frames`` frames (the new lane is settling)."""
        self.settle_frames = max(self.settle_frames, int(frames))

    def begin_reacquire(self, relax_frames: int = 6) -> None:
        """After a pivot: forget the old lane (keep the measured width), relax the gate.

        The next fits start like start-up (left camera + left of the robot =
        left line), and for ``relax_frames`` frames only the +/-45 deg check
        of the plausibility gate applies, so the new leg's lane is not
        rejected as a jump from the old one.
        """
        self.prior_near = self.prior_far = None
        self.frames_lost = 0
        self._last_good = None
        self.short_holds = 0
        self.corner = None
        self._corner_track = None
        self.junction = None
        self._junction_track.reset()
        self.relax_frames = int(relax_frames)

    def _update_corner(self, cues: list[CornerCue], dx_m: float) -> CornerEstimate | None:
        fused = fuse_corner_cues(cues)
        if fused is None:
            self._corner_track = None
            return None
        d, dist, conf, cams, used = fused
        frames = 1
        if self._corner_track is not None:
            pd, pdist, pf = self._corner_track
            if pd == d and abs(dist - (pdist - abs(dx_m))) < CORNER_PERSIST_TOL_M:
                frames = pf + 1
        self._corner_track = (d, dist, frames)
        return CornerEstimate(
            seen=frames >= CORNER_PERSIST_FRAMES and conf >= CORNER_MIN_CONF,
            direction=d, distance_m=dist, confidence=conf, frames=frames, cams=cams,
            clip_x_m=dist - 0.5 * STRIPE_W_M - CORNER_CLIP_MARGIN_M, cues=used,
        )

    def _update_junction(self, per_cam, dx_m: float):
        from src.junction_vision import classify_junction, yellow_view

        try:
            self.view = yellow_view(per_cam)
            j = classify_junction(per_cam, self.lane_w or FALLBACK_LANE_W_M)
        except Exception:  # never let the classifier take the lane fit down
            if os.environ.get("RBM_JUNCTION_STRICT"):
                raise
            j = None
        self.junction = self._junction_track.update(j, dx_m)
        return self.junction

    def _predict(self, dx: float, dyaw: float) -> None:
        """Move the priors into the new robot frame (odometry dx, IMU dyaw)."""
        if self.prior_near is None or self.prior_far is None:
            return
        self.prior_near = predict_model(self.prior_near, dx, dyaw)
        self.prior_far = predict_model(self.prior_far, dx, dyaw)
        if self.prior_near is None or self.prior_far is None:
            self.prior_near = self.prior_far = None

    def process(
        self,
        images: dict[str, tuple],
        *,
        dx_m: float = 0.0,
        dyaw_rad: float = 0.0,
    ) -> LaneEstimate:
        """images: cam name -> (bgra_bytes, width, height)."""
        self._predict(float(dx_m), float(dyaw_rad))
        pts: list[LinePoint] = []
        cues: list[CornerCue] = []
        per_cam = []
        # Callers replace src.lane_vision.corner_cues; read that binding.
        from src.lane_vision import corner_cues
        for name, cam in self.cams.items():
            item = images.get(name)
            if not item or not item[0]:
                continue
            w, h = int(item[1] or cam.width), int(item[2] or cam.height)
            mask = yellow_mask(item[0], w, h, YELLOW_THRESH, self.yellow_mode)
            cam_pts = detect_points(cam, None, w, h, yellow_mode=self.yellow_mode, mask=mask)
            if w >= 4 and h >= 4:
                traced = trace_lines(cam, mask, w, h)
                if self.bar_filter_frames > 0:
                    cam_pts = _drop_bar_points(cam_pts, traced)
            pts.extend(cam_pts)
            if w >= 4 and h >= 4:
                cc = corner_cues(cam, mask, w, h, traced=traced)
                cues.extend(cc)
                per_cam.append((cam, traced, cc))
        corner = self._update_corner(cues, float(dx_m))
        junction = self._update_junction(per_cam, float(dx_m))
        clipped = False
        if junction is not None and not junction.kind.startswith(("corner", "opening")):
            clipped = True
            lim = junction.near_m - 0.5 * STRIPE_W_M - CORNER_CLIP_MARGIN_M
            if junction.kind in ("side_L", "side_R"):
                left = junction.kind == "side_L"
                hi = (junction.far_m + CORNER_CLIP_MARGIN_M) if junction.far_m is not None else 1e9
                pts = [p for p in pts if not ((p.y > 0.0) == left and lim <= p.x <= hi)]
            else:
                pts = [p for p in pts if p.x < lim]
        if corner is not None:
            # The lane ends at the crossbar: fit only what is before it, so the
            # turning paint neither bends the fit nor trips the jump gate.
            clipped = True
            pts = [p for p in pts if p.x < corner.clip_x_m]
            for c in corner.cues:
                if c.role == "inner":  # the inner line bends away here: what is past it is the new leg
                    lim = c.x_m - 0.5 * STRIPE_W_M - CORNER_CLIP_MARGIN_M
                    pts = [p for p in pts if not (p.x >= lim and (p.y > 0.0) == (c.line_y_m > 0.0))]
        self.corner = corner
        est = self.fit_points_gated(pts, dx_m=float(dx_m), allow_short=clipped)
        est.corner = corner
        est.junction = junction
        est.view = self.view
        if self.bar_filter_frames > 0:
            self.bar_filter_frames -= 1
        if self.relax_frames > 0:
            self.relax_frames -= 1
            if self.relax_frames == 0:
                self.begin_settle()  # relaxed window over: settle before the full gate
        elif self.settle_frames > 0:
            self.settle_frames -= 1
        self.last = est
        return est

    def gate_reason(self, est: LaneEstimate, pred_near, dx_m: float, allow_short: bool = False) -> str:
        """Why a fitted lane is physically implausible ("" = accept).

        ``allow_short`` is set when a corner or junction clip already cut the
        fit at the crossbar. A lookahead under ``GATE_MIN_VIEW_M`` is then the
        paint ending, not a lost lane, and must not reject the fit.
        """
        if not est.valid:
            return ""
        if abs(est.heading_rad) > GATE_MAX_HEADING_RAD:
            return f"heading {math.degrees(est.heading_rad):+.0f}deg beyond +/-{math.degrees(GATE_MAX_HEADING_RAD):.0f}"
        if self.relax_frames > 0:
            return ""  # re-acquiring after a pivot: the old lane is no reference
        last = self._last_good if pred_near is not None else None
        if last is not None and est.confidence < GATE_SINGLE_SIDE_MIN_CONF:
            for side, n_now, n_before in (("left", est.n_left, last.n_left), ("right", est.n_right, last.n_right)):
                if n_now == 0 and n_before >= MIN_SIDE_PTS:
                    return (
                        f"{side} side vanished (L{est.n_left} R{est.n_right}, was "
                        f"L{last.n_left} R{last.n_right}) at conf {est.confidence:.2f}"
                    )
        if (
            not allow_short
            and pred_near is not None
            and est.lookahead_m < GATE_MIN_VIEW_M
        ):
            return f"short view: lane seen only {est.lookahead_m:.2f} m ahead (< {GATE_MIN_VIEW_M:.2f})"
        if pred_near is not None:
            a0, psi, k = pred_near
            p_off = signed_lateral(0.0, 0.0, a0, psi, k)
            p_hd = -(psi + k * (-a0 * math.sin(psi)))
            lim_h = GATE_HEADING_JUMP_RAD + GATE_LANE_KAPPA_MAX * abs(dx_m)
            settling = self.settle_frames > 0
            if settling:
                lim_h += GATE_SETTLE_HEADING_RAD
            if abs(est.heading_rad - p_hd) > lim_h:
                return (
                    f"heading jump {math.degrees(est.heading_rad - p_hd):+.0f}deg > "
                    f"{math.degrees(lim_h):.0f}deg for {abs(dx_m):.2f} m driven (IMU yaw applied"
                    f"{', settling' if settling else ''})"
                )
            lim_o = GATE_OFFSET_JUMP_M + GATE_OFFSET_JUMP_PER_M * abs(dx_m)
            if settling:
                lim_o += GATE_SETTLE_OFFSET_M
            if abs(est.offset_m - p_off) > lim_o:
                return f"offset jump {100 * (est.offset_m - p_off):+.0f}cm > {100 * lim_o:.0f}cm for {abs(dx_m):.2f} m driven"
        return ""

    def held_estimate(self, reason: str, rejected: LaneEstimate | None = None) -> LaneEstimate:
        """Last good lane (priors already moved by odometry/IMU), flagged held."""
        rej = {}
        if rejected is not None and rejected.valid:
            rej = {
                "offset_m": rejected.offset_m, "heading_rad": rejected.heading_rad,
                "curvature_1pm": rejected.curvature_1pm, "confidence": rejected.confidence,
                "n_left": rejected.n_left, "n_right": rejected.n_right,
            }
        width = self.lane_w if self.lane_w is not None else FALLBACK_LANE_W_M
        if self.prior_near is None or self.prior_far is None:
            return LaneEstimate(valid=False, held=False, reject_reason=reason, rejected=rej, lane_width_m=width)
        a0n, psin, kn = self.prior_near
        a0, psi, k = self.prior_far
        last = self._last_good
        return LaneEstimate(
            valid=False,
            offset_m=signed_lateral(0.0, 0.0, a0n, psin, kn),
            heading_rad=-(psin + kn * (-a0n * math.sin(psin))),
            curvature_1pm=k,
            lookahead_m=0.0 if last is None else last.lookahead_m,
            confidence=0.0 if last is None else last.confidence,
            lane_width_m=width,
            width_measured=self.lane_w is not None,
            a0=a0,
            psi=psi,
            near_curvature_1pm=kn,
            held=True,
            reject_reason=reason,
            rejected=rej,
        )

    def fit_points_gated(
        self, pts: list[LinePoint], *, dx_m: float = 0.0, allow_short: bool = False
    ) -> LaneEstimate:
        """:meth:`fit_points` + plausibility gate. A rejected fit does not touch
        the priors / lane width; it counts as a lost frame (priors dropped after
        GATE_MAX_HELD_FRAMES in a row, as for a missing lane).

        ``allow_short`` skips the short-view reject. The corner/junction clip
        passes it so the lane ending at a crossbar is still a fit.
        """
        pred_near, pred_far = self.prior_near, self.prior_far
        lane_w, lost = self.lane_w, self.frames_lost
        est = self.fit_points(pts)
        reason = self.gate_reason(est, pred_near, dx_m, allow_short=allow_short)
        if not reason:
            if est.valid:
                self._last_good = est
                self.short_holds = 0
            return est
        self.prior_near, self.prior_far, self.lane_w = pred_near, pred_far, lane_w
        short = reason.startswith("short view")
        self.short_holds = self.short_holds + 1 if short else 0
        if short and self.short_holds <= GATE_MAX_SHORT_HOLDS:
            self.frames_lost = lost  # a short view is not a lost lane (yet)
        else:
            self.frames_lost = lost + 1
        held = self.held_estimate(reason, est)
        held.counts_as_lost = self.frames_lost > lost
        if self.frames_lost >= GATE_MAX_HELD_FRAMES:
            self.prior_near = self.prior_far = None
        self.rejects += 1
        return held

    def fit_points(self, pts: list[LinePoint]) -> LaneEstimate:
        lane_w = self.lane_w if self.lane_w is not None else FALLBACK_LANE_W_M
        width = lane_w
        fitted_width = False  # both near lines produced a width this frame
        if self.prior_near is not None and self.prior_far is not None:
            near_m, far_m = self.prior_near, self.prior_far
            self._assign_piecewise(pts, near_m, far_m, width, gate=0.25)
        else:
            # Start-up: left camera + left of the robot = left line, etc.
            for p in pts:
                if p.cam.endswith("left") and p.y > 0.0:
                    p.side = 1
                elif p.cam.endswith("right") and p.y < 0.0:
                    p.side = -1
                else:
                    p.side = 0
            _drop_thin_sides(pts)
            both = _side_counts(pts) >= (MIN_SIDE_PTS, MIN_SIDE_PTS)
            res = fit_lane(pts, init=_init_from_points(pts, width), lane_w=width, fit_width=both)
            if res is None:
                return self._lost(width)
            near_m = far_m = (res[0][0], res[0][1], res[0][2])
            if both and 0.4 < res[0][3] < 3.0:
                width = res[0][3]
                fitted_width = True
            self._assign_piecewise(pts, near_m, far_m, width, gate=0.15)
        got = False
        for it in range(3):
            _drop_thin_sides(pts)
            nl, nr = _side_counts(pts)
            if nl == 0 and nr == 0:
                break
            near_pts = [p for p in pts if p.side and p.x <= self.near_max_x]
            nb = _side_counts(near_pts)
            near_both = nb[0] >= MIN_SIDE_PTS and nb[1] >= MIN_SIDE_PTS
            near = self._refit(near_pts, near_m, width, fit_width=near_both)
            far = self._refit([p for p in pts if p.side and p.x >= self.far_min_x], far_m, width, False)
            if near is None and far is None:
                res = fit_lane(pts, init=near_m, lane_w=width, fit_width=False)
                if res is None:
                    break
                near = far = ((res[0][0], res[0][1], res[0][2]), res[0][3], res[2])
            near = near or far
            far = far or near
            near_m, far_m = near[0], far[0]
            if near_both and 0.4 < near[1] < 3.0:
                width = near[1]
                fitted_width = True
            got = True
            self._assign_piecewise(pts, near_m, far_m, width, gate=0.15 if it < 2 else 0.10)
        if not got:
            return self._lost(width)
        inl = [p for p in pts if p.side in (1, -1)]
        n_left = sum(1 for p in inl if p.side == 1)
        n_right = sum(1 for p in inl if p.side == -1)
        if n_left + n_right < MIN_SIDE_PTS:
            return self._lost(width)
        both = n_left >= MIN_SIDE_PTS and n_right >= MIN_SIDE_PTS
        if both and 0.4 < width < 3.0 and self.relax_frames <= 0:
            # (frozen while re-acquiring after a pivot: the corner's paint is still in view)
            self.lane_w = width if self.lane_w is None else 0.8 * self.lane_w + 0.2 * width
        measured = self.lane_w is not None
        a0n, psin, kn = near_m
        a0, psi, kappa = far_m
        res_all = []
        ok = 0
        for p in inl:
            m = near_m if p.x <= self.split_x else far_m
            res_all.append(signed_lateral(p.x, p.y, *m) - p.side * 0.5 * width)
            # 6 cm sanity check (validation only, not the scale)
            e = 1e-4
            gx = (signed_lateral(p.x + e, p.y, *m) - signed_lateral(p.x - e, p.y, *m)) / (2 * e)
            gy = (signed_lateral(p.x, p.y + e, *m) - signed_lateral(p.x, p.y - e, *m)) / (2 * e)
            gn = math.hypot(gx, gy) or 1.0
            wperp = abs(p.seg_dx * gx + p.seg_dy * gy) / gn
            if 0.5 * STRIPE_W_M <= wperp <= 1.8 * STRIPE_W_M:
                ok += 1
        rms = math.sqrt(sum(v * v for v in res_all) / len(res_all))
        width_ok = ok / len(inl)
        offset = signed_lateral(0.0, 0.0, a0n, psin, kn)
        s_abreast = -a0n * math.sin(psin)
        heading = -(psin + kn * s_abreast)
        look = max(p.x for p in inl)
        n_tot = n_left + n_right
        conf = min(1.0, n_tot / 40.0)
        conf *= 1.0 / (1.0 + (rms / 0.03) ** 2)
        conf *= 1.0 if both else 0.6
        conf *= 0.5 + 0.5 * width_ok
        if not measured:
            conf *= 0.7
        pixels: dict = {}
        for p in inl:
            pixels.setdefault(p.cam, []).append((p.col, p.row, p.side))
        self.frames_lost = 0
        self.prior_near = near_m
        self.prior_far = far_m
        return LaneEstimate(
            valid=True,
            offset_m=offset,
            heading_rad=heading,
            curvature_1pm=kappa,
            lookahead_m=look,
            confidence=max(0.0, min(1.0, conf)),
            n_left=n_left,
            n_right=n_right,
            lane_width_m=width if both else lane_w,
            width_measured=measured,
            width_live=bool(fitted_width and both),
            rms_m=rms,
            width_ok_frac=width_ok,
            a0=a0,
            psi=psi,
            near_curvature_1pm=kn,
            pixels=pixels,
        )

    def _lost(self, width: float) -> LaneEstimate:
        self.frames_lost += 1
        if self.frames_lost >= 3:
            self.prior_near = self.prior_far = None
        return LaneEstimate(valid=False, lane_width_m=width)

    @staticmethod
    def _refit(pts, init, width, fit_width):
        sub = [p for p in pts if p.side in (1, -1)]
        if len(sub) < 2 * MIN_SIDE_PTS:
            return None
        xs = [p.x for p in sub]
        ys = [p.y for p in sub]
        if max(max(xs) - min(xs), max(ys) - min(ys)) < 0.3:
            return None
        res = fit_lane(sub, init=init, lane_w=width, fit_width=fit_width, iters=12)
        if res is None:
            return None
        params, _w, rms = res
        return (params[0], params[1], params[2]), params[3], rms

    def _assign_piecewise(self, pts, near_m, far_m, lane_w, gate):
        hw = 0.5 * lane_w
        for p in pts:
            m = near_m if p.x <= self.split_x else far_m
            d = signed_lateral(p.x, p.y, *m)
            side = 1 if d > 0.0 else -1
            g = gate + 0.04 * max(0.0, p.x)
            p.side = side if abs(d - side * hw) < g else 0
