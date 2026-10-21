"""Apply optimizer trial params to config.yaml before a Webots launch."""

from __future__ import annotations

from pathlib import Path

from optimizer.config_store import ConfigStore
from src.controller_limits import KNOBS, load_controller_limits


def apply_trial_params(repo: Path, params: dict | None) -> str:
    """Write allowlisted keys from ``params`` into ``config/config.yaml``.

    Returns one log line with the effective controller limits the rowfit
    runtime will read on startup.
    """
    params = params or {}
    config_path = Path(repo) / "config" / "config.yaml"
    store = ConfigStore(config_path)
    merged = store.read()
    for key in KNOBS:
        if key in params:
            merged[key] = params[key]
    store.write_params(merged)
    lim = load_controller_limits(config_path)
    eff_c = lim.effective_cruise_m_s if lim.effective_cruise_m_s is not None else lim.cruise_m_s
    try:
        import yaml

        with open(config_path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        ctrl = data.get("controller") if isinstance(data.get("controller"), dict) else {}
        coast_on = ctrl.get("coast_enabled", True)
        coast_margin = ctrl.get("coast_margin_m", 0.0)
    except Exception:
        coast_on = params.get("controller.coast_enabled", True)
        coast_margin = params.get("controller.coast_margin_m", 0.0)
    return (
        "optimizer: applied trial config — "
        f"cruise={lim.cruise_m_s:.2f} m/s straight={lim.straight_m_s:.2f} m/s "
        f"effective_cruise={eff_c:.2f} m/s pivot_w_max={lim.pivot_w_max_rad_s:.2f} rad/s "
        f"k_steer={lim.k_steer:.2f} coast_enabled={coast_on} coast_margin_m={float(coast_margin):.2f} m"
        + (
            f" v_max={lim.v_max_m_s:.2f} m/s"
            if lim.v_max_m_s is not None
            else ""
        )
    )
