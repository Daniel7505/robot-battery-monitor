"""Optimizer lap config apply and coast pivot gate."""

from __future__ import annotations

import pytest

from src.controller_limits import AllowlistError, check_param
from src.rowfit_control import RowfitController, STATE_APPROACH


def test_check_param_accepts_bool_coast_enabled():
    assert check_param("controller.coast_enabled", True) == "true"
    assert check_param("controller.coast_enabled", False) == "false"


def test_check_param_rejects_invalid_coast_enabled():
    with pytest.raises(AllowlistError):
        check_param("controller.coast_enabled", "maybe")


def test_pivot_waits_for_entry_speed_at_gate(tmp_path):
    from src.physics_speed_limit import CoastIntoTurnParams, TurnBrakeParams

    coast = CoastIntoTurnParams(
        enabled=True,
        margin_m=0.0,
        crr=0.015,
        drag_linear_n_s_per_m=0.0,
        total_mass_kg=6.142,
        v_pivot_m_s=0.04,
        a_brake_m_s2=7.848,
    )
    tb = TurnBrakeParams(7.848, 0.32, 0.8, 0.04)
    ctl = RowfitController(v_cruise=0.44, turn_brake=tb, coast_into_turn=coast, ld_max=1.6)
    ctl.state = STATE_APPROACH
    ctl.remaining_m = 0.01
    ctl.gov.v = 0.25
    cmd = ctl._approach_tick(None, False, 0.008, 0.32, 0.0)
    assert ctl.state == STATE_APPROACH
    assert "brake-to-pivot" in cmd["reason"]


def test_apply_trial_params_writes_cruise(tmp_path):
    from optimizer.lap_config import apply_trial_params

    cfg_dir = tmp_path / "config"
    cfg_dir.mkdir()
    (cfg_dir / "config.yaml").write_text(
        """
bom_build: light
controller:
  coast_enabled: true
  coast_margin_m: 0.00
  cruise_m_s: 0.44
  straight_m_s: 0.50
  pivot_w_max_rad_s: 0.6
  k_steer: 2.0
""".strip()
        + "\n",
        encoding="utf-8",
    )
    repo = tmp_path
    lap_dir = tmp_path / "lap"
    lap_dir.mkdir()
    shipped = (cfg_dir / "config.yaml").read_text(encoding="utf-8")
    line = apply_trial_params(
        repo,
        {
            "bom_build": "light",
            "controller.cruise_m_s": 0.50,
            "controller.straight_m_s": 0.56,
            "controller.pivot_w_max_rad_s": 0.6,
            "controller.k_steer": 2.0,
            "controller.coast_enabled": True,
            "controller.coast_margin_m": 0.0,
        },
        lap_dir=lap_dir,
    )
    assert "cruise=0.50" in line
    assert (cfg_dir / "config.yaml").read_text(encoding="utf-8") == shipped
    text = (lap_dir / "config.yaml").read_text(encoding="utf-8")
    assert "cruise_m_s: 0.50" in text
