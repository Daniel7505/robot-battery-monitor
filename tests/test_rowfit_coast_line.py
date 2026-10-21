"""Rowfit runtime coast config startup line."""

from __future__ import annotations


def test_coast_line_reports_enabled_config():
    from webots.controllers.butlerbot_controller import controller_rowfit as cr

    rt = cr.RowfitRuntime()
    line = rt.coast_line()
    assert line.startswith("COAST CONFIG")
    assert "enabled=" in line
    coast = rt.ctl.coast_into_turn
    assert coast is not None
    assert coast.enabled is True
