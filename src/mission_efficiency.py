"""Mission efficiency metrics — elapsed time, energy, distance from twin feed.

Light but real: the dashboard Live Monitor shows running totals for a Webots
(or other twin) mission without rewriting the PMS. Pure Python; no Webots.

Sources
-------
* **Elapsed** — wall/sim timestamps on successive ``TwinTelemetry`` samples
  after the tracker arms (first external sample, or after ``reset()``).
* **Energy (SoC)** — ``capacity_wh * (start_pct − current_pct) / 100``.
  Short runs may show ~0 Wh if the pack model barely moves; that is honest.
* **Energy (∫P dt)** — sum of channel draws × Δt / 3600. Useful when SoC is
  flat; labelled separately so operators do not confuse the two.
* **Distance** — cumulative chord length of GPS pose (x_m, y_m) when present.
  Missing pose → distance / avg speed stay ``None``.
* **Avg speed** — distance_m / elapsed_s when both are positive.

Mission arming: first accepted sample after construct/reset. Battery reset
from the twin command path should call ``reset()`` so the next sample is a
fresh baseline. Stale twin (no feed) does not clear totals — the last
snapshot stays visible until reset or a new session.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


def _as_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _pose_xy(pose: dict | None) -> tuple[float, float] | None:
    if not isinstance(pose, dict):
        return None
    x = _as_float(pose.get("x_m"))
    y = _as_float(pose.get("y_m"))
    if x is None or y is None:
        return None
    return (x, y)


def format_elapsed(seconds: float | None) -> str:
    """Human elapsed for the dashboard (m:ss or h:mm:ss)."""
    if seconds is None or seconds < 0:
        return "—"
    s = int(round(float(seconds)))
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    if h:
        return f"{h}:{m:02d}:{sec:02d}"
    return f"{m}:{sec:02d}"


@dataclass
class MissionEfficiencyTracker:
    """Accumulate twin-mission efficiency from successive telemetry samples."""

    capacity_wh: float = 480.0
    # Drop absurd dt spikes (paused Webots, network stall) from ∫P and distance.
    max_dt_s: float = 15.0
    # Ignore sub-millimetre GPS jitter between frames.
    min_step_m: float = 0.001

    _armed: bool = field(default=False, init=False)
    _started_at: datetime | None = field(default=None, init=False)
    _last_ts: datetime | None = field(default=None, init=False)
    _battery_start_pct: float | None = field(default=None, init=False)
    _battery_pct: float | None = field(default=None, init=False)
    _energy_wh_integrated: float = field(default=0.0, init=False)
    _distance_m: float = field(default=0.0, init=False)
    _last_xy: tuple[float, float] | None = field(default=None, init=False)
    _samples: int = field(default=0, init=False)
    _has_pose: bool = field(default=False, init=False)

    def reset(self, *, capacity_wh: float | None = None) -> None:
        """Clear totals; next ``update`` becomes the new mission start."""
        if capacity_wh is not None:
            self.capacity_wh = float(capacity_wh)
        self._armed = False
        self._started_at = None
        self._last_ts = None
        self._battery_start_pct = None
        self._battery_pct = None
        self._energy_wh_integrated = 0.0
        self._distance_m = 0.0
        self._last_xy = None
        self._samples = 0
        self._has_pose = False

    def update(
        self,
        *,
        timestamp: datetime | None = None,
        battery_pct: float | None = None,
        channel_draws: dict | None = None,
        pose: dict | None = None,
        capacity_wh: float | None = None,
    ) -> dict:
        """Ingest one telemetry sample; return the current snapshot dict."""
        if capacity_wh is not None and capacity_wh > 0:
            self.capacity_wh = float(capacity_wh)

        ts = timestamp or datetime.now(timezone.utc)
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)

        batt = _as_float(battery_pct)
        xy = _pose_xy(pose)

        if not self._armed:
            self._armed = True
            self._started_at = ts
            self._last_ts = ts
            self._battery_start_pct = batt
            self._battery_pct = batt
            self._last_xy = xy
            self._has_pose = xy is not None
            self._samples = 1
            return self.snapshot()

        dt = (ts - self._last_ts).total_seconds() if self._last_ts else 0.0
        if dt < 0:
            # Clock skew / replay — re-anchor time, skip integrals this tick.
            dt = 0.0
        self._last_ts = ts
        self._samples += 1

        if batt is not None:
            self._battery_pct = batt
            if self._battery_start_pct is None:
                self._battery_start_pct = batt

        # ∫P: skip absurd stalls (paused Webots / network gap) so we do not
        # bill the last wattage across a long silent gap.
        if 0.0 < dt <= self.max_dt_s and channel_draws:
            total_w = 0.0
            for v in channel_draws.values():
                fv = _as_float(v)
                if fv is not None and fv > 0:
                    total_w += fv
            if total_w > 0:
                self._energy_wh_integrated += total_w * dt / 3600.0

        # Distance always follows pose chords (GPS truth even after a stall).
        if xy is not None:
            self._has_pose = True
            if self._last_xy is not None and dt >= 0.0:
                dx = xy[0] - self._last_xy[0]
                dy = xy[1] - self._last_xy[1]
                step = math.hypot(dx, dy)
                if step >= self.min_step_m:
                    self._distance_m += step
            self._last_xy = xy

        return self.snapshot()

    def update_from_telemetry(self, tel: Any) -> dict:
        """Convenience: pull fields from a ``TwinTelemetry`` (or duck-type)."""
        if tel is None:
            return self.snapshot()
        capacity = _as_float(getattr(tel, "battery_capacity_wh", None))
        return self.update(
            timestamp=getattr(tel, "timestamp", None),
            battery_pct=getattr(tel, "battery_pct", None),
            channel_draws=getattr(tel, "channel_draws", None) or {},
            pose=getattr(tel, "pose", None) or {},
            capacity_wh=capacity,
        )

    @property
    def elapsed_s(self) -> float | None:
        if not self._armed or self._started_at is None or self._last_ts is None:
            return None
        return max(0.0, (self._last_ts - self._started_at).total_seconds())

    @property
    def battery_delta_pct(self) -> float | None:
        if self._battery_start_pct is None or self._battery_pct is None:
            return None
        return round(self._battery_start_pct - self._battery_pct, 3)

    @property
    def energy_wh_from_soc(self) -> float | None:
        delta = self.battery_delta_pct
        if delta is None or self.capacity_wh <= 0:
            return None
        return round(max(0.0, delta) * self.capacity_wh / 100.0, 3)

    def snapshot(self) -> dict:
        """JSON-friendly dict for twin_control / SocketIO / tests."""
        elapsed = self.elapsed_s
        distance = round(self._distance_m, 3) if self._has_pose else None
        avg_speed = None
        if distance is not None and elapsed is not None and elapsed > 0.05:
            avg_speed = round(distance / elapsed, 3)

        soc_wh = self.energy_wh_from_soc
        integ_wh = round(self._energy_wh_integrated, 3) if self._armed else None
        # Prefer SoC when it moved; else show integrated draw as the energy used.
        if soc_wh is not None and soc_wh > 0.005:
            energy_used = soc_wh
            energy_source = "soc_delta"
        elif integ_wh is not None and integ_wh > 0:
            energy_used = integ_wh
            energy_source = "integrated_draw"
        else:
            energy_used = soc_wh if soc_wh is not None else integ_wh
            energy_source = "soc_delta" if soc_wh is not None else (
                "integrated_draw" if integ_wh is not None else None
            )

        return {
            "kind": "mission_efficiency",
            "active": self._armed,
            "elapsed_s": round(elapsed, 2) if elapsed is not None else None,
            "elapsed_label": format_elapsed(elapsed),
            "battery_start_pct": (
                round(self._battery_start_pct, 2)
                if self._battery_start_pct is not None
                else None
            ),
            "battery_pct": (
                round(self._battery_pct, 2) if self._battery_pct is not None else None
            ),
            "battery_delta_pct": self.battery_delta_pct,
            "capacity_wh": round(float(self.capacity_wh), 1),
            "energy_wh_used": energy_used,
            "energy_wh_soc": soc_wh,
            "energy_wh_integrated": integ_wh,
            "energy_source": energy_source,
            "distance_m": distance,
            "avg_speed_m_s": avg_speed,
            "samples": self._samples,
            "started_at": self._started_at.isoformat() if self._started_at else None,
        }


def compute_efficiency_delta(
    *,
    elapsed_s: float,
    battery_start_pct: float,
    battery_end_pct: float,
    capacity_wh: float,
    distance_m: float | None = None,
    energy_wh_integrated: float | None = None,
) -> dict:
    """Pure one-shot helper for tests / offline run reports (no state)."""
    tracker = MissionEfficiencyTracker(capacity_wh=capacity_wh)
    t0 = datetime(2026, 10, 5, 12, 0, 0, tzinfo=timezone.utc)
    from datetime import timedelta

    tracker.update(
        timestamp=t0,
        battery_pct=battery_start_pct,
        channel_draws={},
        pose={"x_m": 0.0, "y_m": 0.0} if distance_m is not None else None,
    )
    # Second sample at elapsed_s with end battery / optional distance chord.
    pose_end = None
    if distance_m is not None:
        pose_end = {"x_m": float(distance_m), "y_m": 0.0}
    draws = {}
    if energy_wh_integrated and elapsed_s > 0:
        # Constant draw that integrates to the requested Wh.
        draws = {"Legs": float(energy_wh_integrated) * 3600.0 / float(elapsed_s)}
    return tracker.update(
        timestamp=t0 + timedelta(seconds=float(elapsed_s)),
        battery_pct=battery_end_pct,
        channel_draws=draws,
        pose=pose_end,
    )
