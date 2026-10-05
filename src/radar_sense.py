"""Forward radar → ObstacleReport (lane corridor, static vs moving).

Webots ``Radar`` returns targets with radial distance, relative radial speed
and azimuth (see docs/OBSTACLES.md). This module is pure Python: the controller
feeds target tuples; unit tests use the same path without Webots.

Coordinate notes (bot frame: x forward from the axle, y left, z up):

* The radar sits at ``radar_x_m`` on the front centre line and looks along +x.
* Azimuth is left-positive (Webots Radar: positive azimuth = left of the beam).
* Webots ``speed`` is the target's radial speed along the radar→target LOS
  (positive = receding). Closing speed relative to the robot is therefore
  ``-speed``. An approaching static wall while the robot drives at ``v_ego``
  yields closing ≈ ``v_ego``; subtracting ego gives ~0 (still). A ball rolling
  toward the robot yields closing > ``v_ego`` → positive world-relative
  approach → classified **moving**.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from src.obstacle_gate import (
    BAND_LOW,
    ObstacleReport,
    ObstacleSource,
    height_band,
)

RADAR_NAME = "radar_fwd"
RADAR_X_M = 0.16  # lens / antenna x in the bot frame (matches the .wbt mount)
RADAR_Z_M = 0.45
ROBOT_FRONT_X_M = 0.15  # axle → front face (same as stereo)
DEFAULT_HALF_WIDTH_M = 0.54  # stereo corridor default; overridden from live robot
DEFAULT_MAX_RANGE_M = 8.0
# |world-relative radial m/s| above this ⇒ moving (rolling ball / forklift / person)
MOVING_SPEED_M_S = 0.12
MIN_CONF = 0.4


@dataclass(frozen=True)
class RadarTarget:
    """One Webots radar return, already in SI units."""

    distance_m: float
    speed_m_s: float  # Webots sign: + = receding along LOS
    azimuth_rad: float  # + = left of the beam
    received_power_dbm: float = 0.0


@dataclass
class RadarParams:
    radar_x_m: float = RADAR_X_M
    front_x_m: float = ROBOT_FRONT_X_M
    half_width_m: float = DEFAULT_HALF_WIDTH_M
    max_range_m: float = DEFAULT_MAX_RANGE_M
    moving_speed_m_s: float = MOVING_SPEED_M_S
    # Assumed target height band when radar has no elevation (point return).
    assume_z_top_m: float = 0.5
    assume_z_bottom_m: float = 0.0


def target_in_corridor(t: RadarTarget, prm: RadarParams) -> bool:
    """True when the return sits inside the lane corridor half-width."""
    if t.distance_m <= 0.0 or t.distance_m > prm.max_range_m:
        return False
    y = t.distance_m * math.sin(t.azimuth_rad)
    return abs(y) <= prm.half_width_m + 0.05  # tiny margin for azimuth noise


def classify_motion(t: RadarTarget, *, v_ego_m_s: float = 0.0,
                    moving_speed_m_s: float = MOVING_SPEED_M_S) -> tuple[float, float, bool]:
    """Return (closing_rel_m_s, target_radial_m_s, is_moving).

    ``closing_rel``: + = target approaching the radar (relative).
    ``target_radial``: world-ish radial of the *target* along the LOS after
    removing the robot's forward contribution (cos azimuth). Positive = the
    target itself is closing on the robot.
    """
    closing_rel = -float(t.speed_m_s)
    ego_along = float(v_ego_m_s) * math.cos(t.azimuth_rad)
    target_radial = closing_rel - ego_along
    return closing_rel, target_radial, abs(target_radial) >= float(moving_speed_m_s)


def report_from_targets(targets: list[RadarTarget], *, v_ego_m_s: float = 0.0,
                        prm: RadarParams | None = None) -> ObstacleReport:
    """Nearest corridor hit → ObstacleReport; empty list → clear corridor."""
    prm = prm or RadarParams()
    hits = [t for t in targets if target_in_corridor(t, prm)]
    if not hits:
        return ObstacleReport(
            source="radar",
            detected=False,
            range_m=prm.max_range_m,
            confidence=1.0,
            notes={"n_targets": len(targets), "n_corridor": 0},
        )
    # Prefer a moving hit; else the nearest. Moving objects are the radar's job.
    scored = []
    for t in hits:
        closing, radial, moving = classify_motion(t, v_ego_m_s=v_ego_m_s,
                                                   moving_speed_m_s=prm.moving_speed_m_s)
        scored.append((0 if moving else 1, t.distance_m, t, closing, radial, moving))
    scored.sort()
    _pref, _d, t, closing, radial, moving = scored[0]
    # Bot-frame x of the face ≈ radar_x + range * cos(az) (small-angle ≈ radar_x + range)
    x_m = prm.radar_x_m + t.distance_m * math.cos(t.azimuth_rad)
    y_m = t.distance_m * math.sin(t.azimuth_rad)
    # Point return: give a small lateral extent so the gate's describe() is happy.
    half = 0.15
    clear = x_m - prm.front_x_m
    band = height_band(prm.assume_z_top_m)
    conf = max(MIN_CONF, min(1.0, 0.55 + 0.45 * max(0.0, 1.0 - abs(t.azimuth_rad))))
    return ObstacleReport(
        source="radar",
        detected=True,
        distance_m=round(x_m, 4),
        clearance_m=round(clear, 4),
        y_left_m=round(y_m + half, 4),
        y_right_m=round(y_m - half, 4),
        z_bottom_m=prm.assume_z_bottom_m,
        z_top_m=prm.assume_z_top_m,
        band=band if band else BAND_LOW,
        confidence=conf,
        n_points=1,
        range_m=prm.max_range_m,
        velocity_m_s=round(closing, 4),
        notes={
            "moving": moving,
            "target_radial_m_s": round(radial, 4),
            "azimuth_rad": round(t.azimuth_rad, 4),
            "radar_range_m": round(t.distance_m, 4),
            "n_targets": len(targets),
            "n_corridor": len(hits),
        },
    )


class RadarObstacleSensor(ObstacleSource):
    """``ObstacleSource`` wrapper: feed targets each frame, read ``latest()``."""

    name = "radar"

    def __init__(self, prm: RadarParams | None = None) -> None:
        self.prm = prm or RadarParams()
        self._latest: ObstacleReport | None = None
        self.frames = 0

    def process(self, targets: list[RadarTarget], *, v_ego_m_s: float = 0.0) -> ObstacleReport:
        rep = report_from_targets(targets, v_ego_m_s=v_ego_m_s, prm=self.prm)
        self._latest = rep
        self.frames += 1
        return rep

    def latest(self) -> ObstacleReport | None:
        out, self._latest = self._latest, None
        return out


def targets_from_webots_device(dev, *, log=None) -> list[RadarTarget]:
    """Pull ``RadarTarget`` list from a Webots Radar device (or None)."""
    if dev is None:
        return []
    try:
        n = int(dev.getNumberOfTargets())
    except Exception:
        try:
            n = int(getattr(dev, "number_of_targets", 0) or 0)
        except Exception:
            return []
    if n <= 0:
        return []
    out: list[RadarTarget] = []
    try:
        raw = dev.getTargets()
    except Exception as exc:
        if log is not None:
            log(f"WARNING radar getTargets failed: {type(exc).__name__}: {exc}")
        return []
    # Webots Python may return RadarTarget objects or a flat buffer; accept both.
    if raw is None:
        return []
    if hasattr(raw, "__len__") and raw and not hasattr(raw[0], "distance"):
        # Flat doubles: distance, received_power, speed, azimuth per target.
        flat = list(raw)
        for i in range(min(n, len(flat) // 4)):
            base = 4 * i
            out.append(RadarTarget(float(flat[base]), float(flat[base + 2]),
                                   float(flat[base + 3]), float(flat[base + 1])))
        return out
    for i, t in enumerate(raw):
        if i >= n:
            break
        try:
            out.append(RadarTarget(
                float(t.distance),
                float(t.speed),
                float(t.azimuth),
                float(getattr(t, "received_power", 0.0) or 0.0),
            ))
        except Exception:
            continue
    return out
