"""Apply optimizer trial params to a per-lap config copy before Webots launch."""

from __future__ import annotations

import shutil
from pathlib import Path

from optimizer.config_store import ConfigStore
from src.controller_limits import KNOBS, load_controller_limits

_LAP_CONFIG_NAME = "config.yaml"


def lap_config_path(lap_dir: Path) -> Path:
    return Path(lap_dir) / _LAP_CONFIG_NAME


def prepare_lap_config(repo: Path, lap_dir: Path, params: dict | None) -> tuple[Path, str]:
    """Copy shipped ``config.yaml`` into the lap folder and patch allowlisted keys.

    The repository ``config/config.yaml`` is never modified (comments preserved on
    the lap copy via ``ConfigStore.write_params``).
    """
    params = params or {}
    shipped = Path(repo) / "config" / "config.yaml"
    target = lap_config_path(lap_dir)
    if not shipped.is_file():
        raise FileNotFoundError(f"shipped config missing: {shipped}")
    shutil.copy2(shipped, target)
    store = ConfigStore(target)
    merged = store.read()
    for key in KNOBS:
        if key in params:
            merged[key] = params[key]
    store.write_params(merged)
    return target, _format_note(target, params)


def apply_trial_params(
    repo: Path,
    params: dict | None,
    *,
    lap_dir: Path | None = None,
) -> str:
    """Write trial knobs to a lap-local config (preferred) or legacy repo path."""
    if lap_dir is not None:
        _path, note = prepare_lap_config(repo, lap_dir, params)
        return note
    # Non-lap callers (tests): still avoid touching shipped config when possible.
    from optimizer.config_store import ConfigStore as _Legacy

    config_path = Path(repo) / "config" / "config.yaml"
    store = _Legacy(config_path)
    merged = store.read()
    for key in KNOBS:
        if key in (params or {}):
            merged[key] = (params or {})[key]
    store.write_params(merged)
    return _format_note(config_path, params or {})


def _format_note(config_path: Path, params: dict) -> str:
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
        f"file={config_path} "
        f"cruise={lim.cruise_m_s:.2f} m/s straight={lim.straight_m_s:.2f} m/s "
        f"effective_cruise={eff_c:.2f} m/s pivot_w_max={lim.pivot_w_max_rad_s:.2f} rad/s "
        f"k_steer={lim.k_steer:.2f} coast_enabled={coast_on} coast_margin_m={float(coast_margin):.2f} m"
        + (
            f" v_max={lim.v_max_m_s:.2f} m/s"
            if lim.v_max_m_s is not None
            else ""
        )
    )
