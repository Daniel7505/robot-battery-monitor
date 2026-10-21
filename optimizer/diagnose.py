"""Read MISSION_SUMMARY / SEGMENT_ENERGY lines out of dashboard logs."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

# A corner occupies part of a 10 ft segment. 20° keeps gentle weave on
# the straight side and warehouse pivots on the turn side.
TURN_HEADING_DEG = 20.0

_KV = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(-?(?:\d+(?:\.\d*)?|\.\d+)|na|[^\s]+)")


def _num(text: str | None) -> float | None:
    if text is None:
        return None
    cleaned = text.strip().replace("−", "-").replace("–", "-")
    if cleaned.lower() in {"na", "none", "null", ""}:
        return None
    try:
        return float(cleaned)
    except ValueError:
        return None


def _kv(line: str) -> dict[str, str]:
    return {key: value for key, value in _KV.findall(line)}


@dataclass
class MissionSummary:
    build: str | None
    duration_s: float | None
    dist_m: float | None
    turns: float | None
    wh_model: float | None
    wh_model_idle: float | None
    wh_model_drive: float | None
    wh_model_roll: float | None
    wh_model_visc: float | None
    wh_model_turn: float | None
    wh_model_accel: float | None
    wh_measured_drive_shaft: float | None
    wh_measured_drive_elec: float | None
    wh_measured: float | None
    wh_measured_soc: float | None
    err_pct: float | None
    eta_avg: float | None
    ctrl_cruise_m_s: float | None = None
    ctrl_straight_m_s: float | None = None
    ctrl_pivot_w_max_rad_s: float | None = None
    ctrl_k_steer: float | None = None
    ctrl_bom_build: str | None = None
    body_mass_kg: float | None = None
    opt_run_id: str | None = None
    opt_trial_id: str | None = None
    raw: str = ""

    def wh_per_m(self) -> float | None:
        if self.dist_m is None or self.dist_m <= 0:
            return None
        if self.wh_measured_drive_elec is None:
            return None
        return self.wh_measured_drive_elec / self.dist_m

    def shares(self) -> dict[str, float]:
        drive = self.wh_model_drive or 0.0
        if drive <= 1e-12:
            return {}
        return {
            "roll": (self.wh_model_roll or 0.0) / drive,
            "visc": (self.wh_model_visc or 0.0) / drive,
            "turn": (self.wh_model_turn or 0.0) / drive,
            "accel": (self.wh_model_accel or 0.0) / drive,
        }


@dataclass
class Segment:
    build: str | None
    avg_speed: float | None
    heading_change_deg: float | None
    err_pct: float | None
    wh_model_drive: float | None
    wh_model_roll: float | None
    wh_model_visc: float | None
    wh_model_turn: float | None
    dist_m: float | None
    raw: str = ""

    @property
    def is_turn(self) -> bool:
        if self.heading_change_deg is None:
            return False
        return abs(self.heading_change_deg) >= TURN_HEADING_DEG


@dataclass
class Diagnosis:
    summary: MissionSummary | None
    segments: list[Segment] = field(default_factory=list)
    power: dict[str, float | None] = field(default_factory=dict)
    straight_err: float | None = None
    turn_err: float | None = None
    speed_err_corr: float | None = None
    notes: list[str] = field(default_factory=list)
    per_build: dict[str, MissionSummary] = field(default_factory=dict)

    def share(self, name: str) -> float:
        if self.summary is None:
            return 0.0
        return self.summary.shares().get(name, 0.0)


def _summary_from(line: str) -> MissionSummary:
    kv = _kv(line)
    build = kv.get("build")
    return MissionSummary(
        build=None if not build or build == "na" else build,
        duration_s=_num(kv.get("duration_s")),
        dist_m=_num(kv.get("dist_m")),
        turns=_num(kv.get("turns")),
        wh_model=_num(kv.get("wh_model")),
        wh_model_idle=_num(kv.get("wh_model_idle")),
        wh_model_drive=_num(kv.get("wh_model_drive")),
        wh_model_roll=_num(kv.get("wh_model_roll")),
        wh_model_visc=_num(kv.get("wh_model_visc")),
        wh_model_turn=_num(kv.get("wh_model_turn")),
        wh_model_accel=_num(kv.get("wh_model_accel")),
        wh_measured_drive_shaft=_num(kv.get("wh_measured_drive_shaft")),
        wh_measured_drive_elec=_num(kv.get("wh_measured_drive_elec")),
        wh_measured=_num(kv.get("wh_measured")),
        wh_measured_soc=_num(kv.get("wh_measured_soc")),
        err_pct=_num(kv.get("err_pct")),
        eta_avg=_num(kv.get("eta_avg")),
        ctrl_cruise_m_s=_num(kv.get("ctrl_cruise_m_s")),
        ctrl_straight_m_s=_num(kv.get("ctrl_straight_m_s")),
        ctrl_pivot_w_max_rad_s=_num(kv.get("ctrl_pivot_w_max_rad_s")),
        ctrl_k_steer=_num(kv.get("ctrl_k_steer")),
        ctrl_bom_build=None
        if not kv.get("ctrl_bom_build") or kv.get("ctrl_bom_build") == "na"
        else kv.get("ctrl_bom_build"),
        body_mass_kg=_num(kv.get("body_mass_kg")),
        opt_run_id=None
        if not kv.get("opt_run_id") or kv.get("opt_run_id") == "na"
        else kv.get("opt_run_id"),
        opt_trial_id=None
        if not kv.get("opt_trial_id") or kv.get("opt_trial_id") == "na"
        else kv.get("opt_trial_id"),
        raw=line.strip(),
    )


def _segment_from(line: str) -> Segment:
    kv = _kv(line)
    build = kv.get("build")
    return Segment(
        build=None if not build or build == "na" else build,
        avg_speed=_num(kv.get("avg_speed")),
        heading_change_deg=_num(kv.get("heading_change_deg")),
        err_pct=_num(kv.get("err_pct")),
        wh_model_drive=_num(kv.get("wh_model_drive")),
        wh_model_roll=_num(kv.get("wh_model_roll")),
        wh_model_visc=_num(kv.get("wh_model_visc")),
        wh_model_turn=_num(kv.get("wh_model_turn")),
        dist_m=_num(kv.get("dist_m")),
        raw=line.strip(),
    )


def _mean(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def _corr(xs: list[float], ys: list[float]) -> float | None:
    n = len(xs)
    if n < 4 or n != len(ys):
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if dx < 1e-12 or dy < 1e-12:
        return None
    return num / (dx * dy)


def real_summary_line(line: str) -> bool:
    """A scored summary, not a prose mention of the token.

    Controller logs include ``dashboard will log MISSION_SUMMARY`` and
    ``MISSION_SUMMARY written → path`` before any lap exists. Those lines
    have no ``duration_s`` / ``dist_m`` and must not end a trial.
    """
    if "MISSION_SUMMARY" not in (line or ""):
        return False
    return "duration_s=" in line or "dist_m=" in line


def _lap_body(text: str) -> str:
    """Text of the mission that ends at the last real MISSION_SUMMARY."""
    lines = (text or "").splitlines()
    idxs = [i for i, line in enumerate(lines) if real_summary_line(line)]
    if not idxs:
        return text or ""
    start = idxs[-2] + 1 if len(idxs) >= 2 else 0
    return "\n".join(lines[start : idxs[-1] + 1])


def diagnose_lap(text: str) -> Diagnosis:
    """Diagnose only when this capture includes a real ``MISSION_SUMMARY`` line."""
    if not any(real_summary_line(line) for line in (text or "").splitlines()):
        empty = Diagnosis(
            summary=None,
            segments=[],
            straight_err=None,
            turn_err=None,
            notes=["No MISSION_SUMMARY in the log. The lap did not finish a scored mission."],
            per_build={},
        )
        return empty
    return diagnose(text)


def diagnose(text: str) -> Diagnosis:
    """Last mission in ``text``, plus the last summary seen for each build."""
    summaries: list[MissionSummary] = []
    power: dict[str, float | None] = {}
    for line in text.splitlines():
        if real_summary_line(line):
            summaries.append(_summary_from(line))
        elif "POWER_BREAKDOWN" in line:
            raw = _kv(line)
            power = {key: _num(value) for key, value in raw.items() if key != "build"}
    per_build: dict[str, MissionSummary] = {}
    for summary in summaries:
        if summary.build:
            per_build[summary.build] = summary
    body = _lap_body(text)
    segments = [_segment_from(line) for line in body.splitlines() if "SEGMENT_ENERGY" in line]
    summary = summaries[-1] if summaries else None
    straight = [seg.err_pct for seg in segments if not seg.is_turn and seg.err_pct is not None]
    turns = [seg.err_pct for seg in segments if seg.is_turn and seg.err_pct is not None]
    straight_pts = [
        (seg.avg_speed, seg.err_pct)
        for seg in segments
        if not seg.is_turn and seg.avg_speed is not None and seg.err_pct is not None
    ]
    notes: list[str] = []
    if summary is None:
        notes.append("No MISSION_SUMMARY in the log. The lap did not finish a scored mission.")
    else:
        shares = summary.shares()
        if shares:
            notes.append(
                "Model drive {drive:.3f} Wh on {build}: visc {visc:.0%}, roll {roll:.0%}, "
                "turn {turn:.0%}, accel {accel:.0%}.".format(
                    drive=summary.wh_model_drive or 0.0,
                    build=summary.build or "unset",
                    **shares,
                )
            )
        notes.append(
            "Measured electrical drive {elec} Wh, shaft {shaft} Wh, err_pct {err}. "
            "Negative err_pct means the model is above the measured electrical drive.".format(
                elec=summary.wh_measured_drive_elec,
                shaft=summary.wh_measured_drive_shaft,
                err=summary.err_pct,
            )
        )
    if straight:
        notes.append(f"Straight segments mean err_pct { _mean(straight):.1f} ({len(straight)} segments).")
    if turns:
        notes.append(f"Turn segments mean err_pct {_mean(turns):.1f} ({len(turns)} segments).")
    return Diagnosis(
        summary=summary,
        segments=segments,
        power=power,
        straight_err=_mean(straight),
        turn_err=_mean(turns),
        speed_err_corr=_corr([p[0] for p in straight_pts], [p[1] for p in straight_pts]),
        notes=notes,
        per_build=per_build,
    )
