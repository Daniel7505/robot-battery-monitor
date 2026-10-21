"""Gyro read and pivot heading integration (no Webots)."""

from __future__ import annotations

import math

import pytest

from src.gyro_pivot import (
    FakeGyro,
    integrate_heading_rad,
    pivot_turn_progress_rad,
    rate_tracking_adjustment,
    read_yaw_rate_rad_s,
)
from src.rowfit_control import RowfitController, PIVOT_ANGLE_RAD, STATE_PIVOT


def test_fake_gyro_read():
    g = FakeGyro(yaw_rate_rad_s=0.5)
    r = read_yaw_rate_rad_s(g)
    assert r.yaw_rate_rad_s == pytest.approx(0.5)
    assert r.source == "gyro"


def test_imu_delta_fallback():
    r = read_yaw_rate_rad_s(
        None,
        imu_yaw_rad=0.1,
        prev_imu_yaw_rad=0.0,
        dt_s=0.1,
    )
    assert r.yaw_rate_rad_s == pytest.approx(1.0)
    assert r.source == "imu_delta"


def test_integrated_heading_reaches_90deg():
    integrated = 0.0
    w = PIVOT_ANGLE_RAD / 2.0  # rad/s
    dt = 1.0
    for _ in range(2):
        integrated = integrate_heading_rad(integrated, w, dt)
    assert pivot_turn_progress_rad(integrated, 1) >= PIVOT_ANGLE_RAD - 1e-6


def test_rate_tracking_nudges_command():
    adj = rate_tracking_adjustment(0.4, 0.1, gain=0.5)
    assert adj == pytest.approx(0.15)


def test_pivot_tick_uses_measured_yaw_rate():
    ctl = RowfitController(pivot_w_max=0.9)
    ctl.state = STATE_PIVOT
    ctl.corner_dir = 1
    ctl.yaw_target = 0.0
    ctl.pivot_start_yaw = -PIVOT_ANGLE_RAD
    yaw = -PIVOT_ANGLE_RAD + 0.05
    cmd = ctl._pivot_tick(None, False, 0.008, yaw, yaw_rate_rad_s=0.2)
    assert cmd.get("yaw_rate_rad_s") == pytest.approx(0.2, abs=1e-4)
    assert abs(ctl.pivot_integrated_rad) > 0.0
