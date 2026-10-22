"""Per-lap config must not mutate the shipped repository config.yaml."""

from __future__ import annotations

from optimizer.lap_config import apply_trial_params, lap_config_path


def test_apply_trial_params_writes_lap_copy_not_shipped(tmp_path):
    repo = tmp_path / "repo"
    cfg_dir = repo / "config"
    cfg_dir.mkdir(parents=True)
    shipped = cfg_dir / "config.yaml"
    shipped.write_text(
        "# keep this comment\n"
        "bom_build: light\n"
        "controller:\n"
        "  cruise_m_s: 0.44\n"
        "  straight_m_s: 0.50\n"
        "  pivot_w_max_rad_s: 0.6\n"
        "  k_steer: 2.0\n"
        "  coast_enabled: true\n"
        "  coast_margin_m: 0.0\n",
        encoding="utf-8",
    )
    lap_dir = tmp_path / "lap"
    lap_dir.mkdir()
    note = apply_trial_params(
        repo,
        {"controller.cruise_m_s": 0.50, "controller.coast_enabled": "true"},
        lap_dir=lap_dir,
    )
    assert "coast_enabled" in note
    assert "# keep this comment" in shipped.read_text(encoding="utf-8")
    lap_text = lap_config_path(lap_dir).read_text(encoding="utf-8")
    assert "# keep this comment" in lap_text
    assert "cruise_m_s: 0.50" in lap_text
