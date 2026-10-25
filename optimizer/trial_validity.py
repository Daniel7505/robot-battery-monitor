"""Decide whether a finished lap is safe to score and learn from."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

from optimizer.body_mass import mass_matches_run
from optimizer.diagnose import MissionSummary
from optimizer.lap_launch import MIN_MISSION_DURATION_S, LapLaunch

LIMIT_TOLERANCE = 0.015
RUN_BODY_DEFAULT = "light"

# Keys the controller must echo on MISSION_SUMMARY for the lap to count.
_LIMIT_KEYS = (
    "controller.cruise_m_s",
    "controller.straight_m_s",
    "controller.pivot_w_max_rad_s",
    "controller.k_steer",
)


def run_body_from_params(params: dict | None) -> str:
    text = "" if params is None else str(params.get("bom_build") or "").strip()
    return text or RUN_BODY_DEFAULT


def _summary_limits(summary: MissionSummary | None) -> dict[str, float | str | None]:
    if summary is None:
        return {}
    return {
        "controller.cruise_m_s": summary.ctrl_cruise_m_s,
        "controller.straight_m_s": summary.ctrl_straight_m_s,
        "controller.pivot_w_max_rad_s": summary.ctrl_pivot_w_max_rad_s,
        "controller.k_steer": summary.ctrl_k_steer,
        "bom_build": summary.ctrl_bom_build or summary.build,
        "body_mass_kg": summary.body_mass_kg,
    }


def lap_body_name(summary: MissionSummary | None) -> str | None:
    if summary is None:
        return None
    if summary.ctrl_bom_build not in (None, ""):
        return str(summary.ctrl_bom_build).strip()
    build = summary.build
    if build in (None, "", "na"):
        return None
    return str(build).strip()


def body_matches_run(summary: MissionSummary | None, run_body: str) -> bool:
    """True when the lap's body matches the fixed run body (not default vs light)."""
    name = lap_body_name(summary)
    if name in (None, "", "default"):
        return False
    return name == run_body


def limits_match_intended(
    summary: MissionSummary | None,
    intended: dict | None,
) -> str | None:
    """Return a reason when the summary limits differ from the config that was written."""
    if summary is None:
        return "no MISSION_SUMMARY"
    if intended is None:
        return None
    observed = _summary_limits(summary)
    for key in _LIMIT_KEYS:
        want = intended.get(key)
        if want is None:
            continue
        got = observed.get(key)
        if got is None:
            return (
                f"lap log is missing {key} from the controller "
                "(expected it on MISSION_SUMMARY)"
            )
        if abs(float(want) - float(got)) > LIMIT_TOLERANCE:
            return (
                f"controller reported {key}={float(got):.3g} but config had {float(want):.3g}"
            )
    want_build = run_body_from_params(intended)
    got_build = lap_body_name(summary)
    if got_build not in (None, "", "default") and got_build != want_build:
        return f"lap body build={got_build} but config had bom_build={want_build}"
    return None


def summary_fresh_reason(
    summary: MissionSummary | None,
    *,
    launch: LapLaunch | None,
    file_mtime: float | None = None,
) -> str | None:
    """Stale or missing stamp vs this trial launch; None when the summary belongs to this lap."""
    if launch is None:
        return None
    if summary is None:
        return "no MISSION_SUMMARY"
    if file_mtime is not None:
        mtime_utc = datetime.fromtimestamp(file_mtime, tz=timezone.utc)
        if mtime_utc < launch.launched_utc - timedelta(seconds=1):
            return (
                "mission summary file mtime is before this trial launch "
                "(reused stale JSON from an earlier lap)"
            )
    if not summary.opt_run_id or not summary.opt_trial_id:
        return "MISSION_SUMMARY missing opt_run_id/opt_trial_id stamps for this optimizer trial"
    if summary.opt_run_id != launch.run_id:
        return (
            f"summary opt_run_id={summary.opt_run_id!r} does not match trial run {launch.run_id!r}"
        )
    if summary.opt_trial_id != launch.trial_id:
        return (
            f"summary opt_trial_id={summary.opt_trial_id!r} "
            f"does not match trial id {launch.trial_id!r}"
        )
    if launch.lap_start_wallclock:
        got = summary.lap_start_wallclock
        if not got:
            return "MISSION_SUMMARY missing lap_start_wallclock for this optimizer lap"
        if got != launch.lap_start_wallclock:
            return (
                f"summary lap_start_wallclock={got!r} does not match "
                f"this lap {launch.lap_start_wallclock!r} (stale or copied summary)"
            )
    if summary.duration_s is not None and summary.duration_s < MIN_MISSION_DURATION_S:
        return (
            f"mission duration_s={summary.duration_s:.1f} is too short for a warehouse lap "
            "(likely cached or instant return)"
        )
    return None


def trial_invalid_reason(
    summary: MissionSummary | None,
    intended: dict | None,
    run_body: str,
    *,
    launch: LapLaunch | None = None,
    file_mtime: float | None = None,
) -> str | None:
    """One reason this lap must not be scored as keep/refuted, or None if it is valid."""
    stale = summary_fresh_reason(summary, launch=launch, file_mtime=file_mtime)
    if stale:
        return stale
    if summary is None:
        return "no MISSION_SUMMARY"
    if summary.err_pct is None:
        return "err_pct is na"
    if not body_matches_run(summary, run_body):
        name = lap_body_name(summary) or summary.build or "unknown"
        return (
            f"lap build {name} does not match the fixed run body {run_body}; "
            "the reference lap must be on the same body"
        )
    mismatch = limits_match_intended(summary, intended)
    if mismatch:
        return mismatch
    ok_mass, mass_reason = mass_matches_run(summary, run_body)
    if not ok_mass and mass_reason:
        return mass_reason
    return None
