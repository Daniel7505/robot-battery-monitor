"""Obstacle reports, sensor fusion hook and the slow / stop / resume gate.

Layering (see docs/OBSTACLES.md)::

    stereo (src.stereo_depth)  ─┐
    radar  (src.radar_sense)   ─┼─> ObstacleFusion ──> ObstacleGate ──> RowfitController.obstacle_v_cap
    any other ObstacleSource   ─┘      (nearest           (CLEAR / SLOW_FOR_OBSTACLE /
                                        confirmed)          STOPPED_FOR_OBSTACLE /
                                                            WAITING_FOR_MOVING)

* :class:`ObstacleReport` is the ONE thing every sensor produces: nearest
  obstacle in the robot's lane corridor, in the bot frame (x forward from the
  axle, y left, z up from the floor). Radar fills ``velocity_m_s`` (closing
  speed, + = approaching) and ``notes["moving"]``; stereo leaves velocity
  ``None``.
* :class:`ObstacleFusion` keeps the latest report per source and hands the
  gate the nearest one (radar via ``fusion.add_source`` / ``add_report``).
* :class:`ObstacleGate` turns reports + current speed into a state and a
  speed cap. A moving hit (``notes["moving"]`` or |target radial| above the
  threshold) yields ``WAITING_FOR_MOVING`` (hold). When the object is still,
  the existing slow / stop / resume path applies. Nothing about obstacle
  positions is known in advance: every number comes from the sensor, the
  robot's live speed and the deceleration limit of the speed governor.

The lane / junction state machine is NOT replaced: the gate only caps the
forward speed, so a stop in the middle of a corner approach or a gap
crossing resumes exactly where it was (dead-reckoned distances keep
counting the real speed). No avoidance or dodging.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

STATE_CLEAR = "CLEAR"
STATE_SLOW = "SLOW_FOR_OBSTACLE"
STATE_STOPPED = "STOPPED_FOR_OBSTACLE"
STATE_WAITING = "WAITING_FOR_MOVING"  # object in lane is moving — hold until still

BAND_LOW = "low"  # pallet / box / tote: top below the waist
BAND_WAIST = "waist"  # table / cart / loaded pallet
BAND_TALL = "tall"  # rack / shelf / person-height and up
BAND_LABEL = {BAND_LOW: "low/pallet", BAND_WAIST: "waist/table", BAND_TALL: "tall/rack"}
WAIST_MIN_M = 0.6  # top of the obstacle at or above this -> waist band
TALL_MIN_M = 1.3  # ... at or above this -> tall band

# --- gate tuning (distances are CLEARANCE: obstacle x minus the robot's front)
STANDOFF_M = 0.35  # where the robot comes to rest in front of an obstacle
SLOW_EXTRA_M = 1.0  # slow zone = stop distance at cruise + this
V_SLOW_M_S = 0.20  # speed cap inside the slow zone (= rowfit V_MIN)
SOFT_DECEL_FRACTION = 0.5  # creep profile uses half the governor's decel (gentle)
CONFIRM_FRAMES = 2  # frames in a row (consistent distance) before acting
TRACK_MATCH_M = 0.40  # |new - predicted| inside this = same obstacle
CLEAR_FRAMES = 3  # visible-and-empty frames in a row before resuming (~1 s at 320 ms)
RESUME_HYST_M = 0.25  # an obstacle that moved this far past the stop distance also releases
MIN_CONF = 0.35  # reports below this confidence are ignored
MOVING_SPEED_M_S = 0.12  # |target radial| (m/s) to treat a hit as moving
MOVING_CLEAR_FRAMES = 2  # still frames in a row before leaving WAITING


@dataclass
class ObstacleReport:
    """Nearest obstacle in the lane corridor (bot frame), from one sensor frame.

    ``detected`` False = the sensor looked and the corridor is empty out to
    ``range_m``. Distances are along +x from the axle; ``clearance_m`` is
    from the robot's front. ``y_left_m`` / ``y_right_m``: lateral extent
    (+y = left). ``z_bottom_m`` / ``z_top_m``: height band of the points;
    ``top_clipped`` = the top is at the top of the view (it may be taller).
    """

    source: str = "stereo"
    detected: bool = False
    distance_m: float | None = None
    clearance_m: float | None = None
    y_left_m: float | None = None
    y_right_m: float | None = None
    z_bottom_m: float | None = None
    z_top_m: float | None = None
    top_clipped: bool = False
    band: str = ""
    confidence: float = 0.0
    n_points: int = 0
    range_m: float = 0.0
    velocity_m_s: float | None = None  # radar (closing speed, + = approaching); stereo None
    # lowest ray of the view: (lens x, lens z, tan of its depression). An
    # obstacle whose top is under that ray is in the blind zone right in front
    # of the robot; the gate will not 'clear' an obstacle it predicts is there.
    view_bottom: tuple | None = None
    notes: dict = field(default_factory=dict)

    def sees(self, x_m: float, z_top_m: float, margin_m: float = 0.03) -> bool:
        """Would the top of an obstacle at (x, z_top) be inside this view?"""
        if self.view_bottom is None:
            return True
        lx, lz, tan_down = self.view_bottom
        if x_m <= lx:
            return False
        return float(z_top_m) > lz - (float(x_m) - lx) * tan_down + margin_m

    @property
    def lateral_extent_m(self) -> float | None:
        if self.y_left_m is None or self.y_right_m is None:
            return None
        return self.y_left_m - self.y_right_m

    @property
    def band_label(self) -> str:
        return BAND_LABEL.get(self.band, self.band)

    def describe(self) -> str:
        if not self.detected:
            return f"{self.source}: corridor clear to {self.range_m:.1f} m"
        top = f"{self.z_top_m:.2f}{'+' if self.top_clipped else ''}"
        return (f"{self.source}: {self.band_label} obstacle at {self.distance_m:.2f} m "
                f"(clear {self.clearance_m:.2f} m), y {self.y_right_m:+.2f}..{self.y_left_m:+.2f} m, "
                f"z {self.z_bottom_m:.2f}..{top} m, conf {self.confidence:.2f}, {self.n_points} pts")


_BAND_RANK = {"": 0, "low": 1, "waist": 2, "tall": 3}


def higher_band(a: str, b: str) -> str:
    """The taller of two bands. A track keeps the tallest band it has seen: from
    close up the cameras' top ray clips a tall object (``top_clipped``), so a
    rack first seen as 'tall' at 3.5 m must not turn 'waist' at 1.5 m."""
    return a if _BAND_RANK.get(a, 0) >= _BAND_RANK.get(b, 0) else b


def height_band(z_top: float) -> str:
    if z_top >= TALL_MIN_M:
        return BAND_TALL
    if z_top >= WAIST_MIN_M:
        return BAND_WAIST
    return BAND_LOW



def _report_is_moving(rep: ObstacleReport) -> bool:
    """True when the sensor marked the hit as moving (radar) or closing hard."""
    notes = rep.notes or {}
    if "moving" in notes:
        return bool(notes["moving"])
    radial = notes.get("target_radial_m_s")
    if radial is not None:
        return abs(float(radial)) >= MOVING_SPEED_M_S
    return False

class ObstacleSource:
    """Interface for anything that sees obstacles (stereo now, radar later).

    ``latest()`` returns the newest :class:`ObstacleReport` or ``None`` if
    the source produced nothing new since the last call.
    """

    name = "source"

    def latest(self) -> ObstacleReport | None:  # pragma: no cover - interface
        raise NotImplementedError


class ObstacleFusion:
    """Latest report per source -> one report for the gate (nearest detection).

    First pass rule: a detection from any source wins over 'clear' (safe
    side); among detections the nearest wins. A moving radar hit (same
    distance band) is preferred over a static one so the gate can wait.
    """

    def __init__(self) -> None:
        self.reports: dict[str, ObstacleReport] = {}

    def add_report(self, rep: ObstacleReport | None) -> None:
        if rep is not None:
            self.reports[rep.source] = rep

    def add_source(self, src: ObstacleSource) -> None:
        self.add_report(src.latest())

    def fused(self) -> ObstacleReport | None:
        if not self.reports:
            return None
        hits = [r for r in self.reports.values() if r.detected and r.confidence >= MIN_CONF]
        if hits:
            def _key(r: ObstacleReport):
                notes = r.notes or {}
                moving = bool(notes.get("moving")) or abs(float(notes.get("target_radial_m_s", 0.0) or 0.0)) >= MOVING_SPEED_M_S
                # moving first, then nearest
                return (0 if moving else 1, float(r.distance_m))
            return min(hits, key=_key)
        return max(self.reports.values(), key=lambda r: r.range_m)


def stop_distance_m(v_m_s: float, decel_m_s2: float, latency_s: float, standoff_m: float = STANDOFF_M) -> float:
    """Clearance at which braking must start: standoff + reaction + v^2 / 2a."""
    v = max(0.0, float(v_m_s))
    return float(standoff_m) + v * max(0.0, float(latency_s)) + v * v / (2.0 * max(1e-3, float(decel_m_s2)))


class ObstacleGate:
    """CLEAR / SLOW / STOPPED / WAITING_FOR_MOVING with hysteresis.

    Call :meth:`on_frame` once per sensor frame (with the distance driven
    since the previous frame), read :attr:`v_cap` (``None`` = no cap) every
    tick. ``decel_m_s2``: the governor's slow-in limit
    (``rowfit_control.DECEL_M_S2``); ``v_cruise``: its cruise speed (sizes
    the slow zone). ``latency_s`` is updated from the live frame period.
    """

    def __init__(self, *, decel_m_s2: float, v_cruise_m_s: float, latency_s: float = 0.32,
                 standoff_m: float = STANDOFF_M, slow_extra_m: float = SLOW_EXTRA_M,
                 v_slow_m_s: float = V_SLOW_M_S) -> None:
        self.decel = float(decel_m_s2)
        self.v_cruise = float(v_cruise_m_s)
        self.latency_s = float(latency_s)
        self.standoff_m = float(standoff_m)
        self.slow_extra_m = float(slow_extra_m)
        self.v_slow = float(v_slow_m_s)
        self.state = STATE_CLEAR
        self.v_cap: float | None = None
        self.emergency = False
        self.track: dict | None = None  # {"clear", "hits", "misses", "band", "rep"}
        self.report: ObstacleReport | None = None
        self.events: list[str] = []
        self.history: list[dict] = []  # one entry per state change (sim / tests)
        self.frames = 0
        self.stops = 0
        self.blind_hold = False

    # ---- derived distances ------------------------------------------------
    def stop_distance(self, v: float) -> float:
        return stop_distance_m(v, self.decel, self.latency_s, self.standoff_m)

    @property
    def slow_distance(self) -> float:
        return self.stop_distance(self.v_cruise) + self.slow_extra_m

    # ---- per frame --------------------------------------------------------
    def on_frame(self, rep: ObstacleReport | None, *, v_m_s: float, dx_m: float = 0.0,
                 frame_dt_s: float | None = None) -> str:
        self.frames += 1
        if frame_dt_s is not None and 0.01 < frame_dt_s < 2.0:
            self.latency_s = float(frame_dt_s)
        self.report = rep
        hit = rep is not None and rep.detected and rep.confidence >= MIN_CONF and rep.clearance_m is not None
        tr = self.track
        if tr is not None:
            tr["clear"] -= max(0.0, float(dx_m))  # dead-reckon the remembered obstacle
        if hit:
            moving = _report_is_moving(rep)
            if tr is not None and abs(rep.clearance_m - tr["clear"]) <= TRACK_MATCH_M + abs(dx_m):
                still_n = 0 if moving else tr.get("still", 0) + 1
                tr.update(clear=float(rep.clearance_m), hits=tr["hits"] + 1, misses=0,
                          band=higher_band(tr["band"], rep.band),
                          rep=rep, z_top=max(tr["z_top"], float(rep.z_top_m or 0.0)),
                          moving=moving if moving else (tr.get("moving") and still_n < MOVING_CLEAR_FRAMES),
                          still=still_n)
            else:
                nearer = tr is None or rep.clearance_m < tr["clear"]
                if nearer or tr["hits"] < CONFIRM_FRAMES:
                    self.track = tr = {"clear": float(rep.clearance_m), "hits": 1, "misses": 0, "band": rep.band,
                                       "rep": rep, "z_top": float(rep.z_top_m or 0.0),
                                       "moving": moving, "still": 0 if moving else 1}
                else:  # a farther, different detection while a confirmed one is held: keep the near one
                    tr["misses"] = 0
            self.blind_hold = False
        elif tr is not None:
            # 'clear' only counts when the remembered obstacle should still be in view
            front = tr["rep"].distance_m - tr["rep"].clearance_m
            blind = rep is None or not rep.sees(tr["clear"] + front, tr["z_top"])
            self.blind_hold = bool(blind and tr["hits"] >= CONFIRM_FRAMES)
            if not self.blind_hold:
                tr["misses"] += 1
            if tr["misses"] >= CLEAR_FRAMES or (tr["hits"] < CONFIRM_FRAMES and tr["misses"] >= 1):
                self.track = tr = None
        self._update_state(float(v_m_s))
        return self.state

    def _update_state(self, v: float) -> None:
        tr = self.track
        confirmed = tr is not None and tr["hits"] >= CONFIRM_FRAMES
        clear_m = tr["clear"] if tr is not None else None
        moving = bool(tr and tr.get("moving"))
        prev = self.state
        self.emergency = False
        d_stop = self.stop_distance(v)
        if not confirmed:
            if self.state != STATE_CLEAR and tr is None:
                why = "corridor clear"
                if prev in (STATE_STOPPED, STATE_WAITING):
                    why += " (obstacle gone)"
                self._set(STATE_CLEAR, why)
        else:
            # Moving object in the lane: hold (wait) until it is still.
            if moving:
                self._set(STATE_WAITING, f"wait: moving object at clearance {clear_m:.2f} m")
            elif self.state == STATE_WAITING:
                # Just went still — drop into the usual slow/stop path below.
                if clear_m <= d_stop + 0.02 or clear_m <= self.standoff_m + 0.02:
                    self._set(STATE_STOPPED, f"stop: moving object now still at {clear_m:.2f} m")
                elif clear_m <= self.slow_distance:
                    self._set(STATE_SLOW, f"slow: moving object now still at {clear_m:.2f} m")
                else:
                    self._set(STATE_CLEAR, f"moving object now still and beyond slow zone ({clear_m:.2f} m)")
            elif self.state == STATE_STOPPED:
                if clear_m > self.stop_distance(0.0) + RESUME_HYST_M:
                    self._set(STATE_SLOW, f"obstacle moved off to {clear_m:.2f} m — resuming slow")
            elif clear_m <= d_stop + 0.02 or clear_m <= self.standoff_m + 0.02:
                self._set(STATE_STOPPED, f"stop: clearance {clear_m:.2f} m <= stop distance {d_stop:.2f} m "
                                         f"(v {v:.2f} m/s)")
                if clear_m < v * v / (2.0 * self.decel) + 0.05:
                    self.emergency = True
            elif self.state == STATE_CLEAR and clear_m <= self.slow_distance:
                self._set(STATE_SLOW, f"slow: clearance {clear_m:.2f} m <= slow zone {self.slow_distance:.2f} m")
            elif self.state == STATE_SLOW and clear_m > self.slow_distance + RESUME_HYST_M:
                self._set(STATE_CLEAR, f"obstacle beyond the slow zone ({clear_m:.2f} m)")
        # speed cap
        if self.state in (STATE_STOPPED, STATE_WAITING):
            self.v_cap = 0.0
        elif self.state == STATE_SLOW and clear_m is not None:
            room = max(0.0, clear_m - self.standoff_m)
            soft = math.sqrt(2.0 * SOFT_DECEL_FRACTION * self.decel * room)
            self.v_cap = min(self.v_slow, soft)
        elif self.state == STATE_SLOW:
            self.v_cap = self.v_slow
        else:
            self.v_cap = None

    def _set(self, state: str, why: str) -> None:
        if state == self.state:
            return
        rep = self.track["rep"] if self.track is not None else self.report
        self.history.append({"frame": self.frames, "from": self.state, "to": state, "why": why,
                             "clear_m": None if self.track is None else round(self.track["clear"], 3),
                             "band": self._band()})
        if state == STATE_STOPPED:
            self.stops += 1
        self.events.append(f"OBSTACLE {self.state} -> {state}: {why}"
                           + (f" [{rep.describe()}]" if rep is not None and rep.detected else ""))
        self.state = state

    def _band(self) -> str:
        if self.track is not None:
            return self.track["band"]
        return "" if self.report is None else self.report.band

    def drain_events(self) -> list[str]:
        out, self.events = self.events, []
        return out

    # ---- logs / HUD -------------------------------------------------------
    def fields(self) -> dict:
        tr = self.track
        rep = tr["rep"] if tr is not None else None
        return {
            "obstacle_state": self.state,
            "obstacle_m": None if tr is None else round(tr["clear"] + (rep.distance_m - rep.clearance_m), 3),
            "obstacle_clear_m": None if tr is None else round(tr["clear"], 3),
            "obstacle_band": "" if tr is None else tr["band"],
            "obstacle_conf": None if rep is None else round(rep.confidence, 2),
            "obstacle_moving": bool(tr.get("moving")) if tr is not None else False,
        }

    def hud_line(self) -> str:
        f = self.fields()
        short = {STATE_CLEAR: "OBS clear", STATE_SLOW: "OBS SLOW", STATE_STOPPED: "OBS STOP",
                 STATE_WAITING: "OBS WAIT"}[self.state]
        if f["obstacle_m"] is None:
            rep = self.report
            return short + ("" if rep is None else f" to {rep.range_m:.1f}m")
        hold = " blind" if self.blind_hold else ""
        moving = " mov" if (self.track and self.track.get("moving")) else ""
        return f"{short} {f['obstacle_m']:.2f}m {f['obstacle_band']} c{f['obstacle_conf'] or 0:.1f}{moving}{hold}"
