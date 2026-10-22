from pathlib import Path

import pytest

from src.design_agent import Mission, propose
from src.hardware_profile import clear_profile_cache
from src.twin_energy import (
    TEN_FT_M,
    accel_electrical_wh,
    bom_drive_terms,
    bom_run_energy_wh,
    brushed_drive_point,
    estimate_mission_energy,
    format_mission_energy_line,
    geometry_scrub_wh,
    paper_energy,
    shift_energy_wh,
    straight_wh_per_m,
)


def test_straight_wh_per_m_is_about_a_quarter_watt_hour():
    # 40 W / 0.44 m/s / 3600 ≈ 0.025 Wh/m
    assert 0.020 < straight_wh_per_m() < 0.030


def test_s_mission_uses_twin_not_five_minute_crawl():
    twin = estimate_mission_energy(25.0, track="s")
    paper = paper_energy(25.0, draw_w=34.0, time_s=300.0)
    assert twin["source"] == "v1_wheeled_energy_baseline"
    assert twin["energy_wh"] < paper["energy_wh"]
    assert twin["energy_wh"] < 2.0
    assert twin["energy_wh"] > 0.3


def test_override_json_wins(tmp_path: Path):
    p = tmp_path / "twin_energy.json"
    p.write_text('{"wh_per_m": 0.1, "source": "measure_track_run"}', encoding="utf-8")
    out = estimate_mission_energy(10.0, track="s", override_path=p)
    assert out["source"] == "measure_track_run"
    assert out["energy_wh"] == 1.0


def test_payload_is_ignored_until_a_bom_build_is_selected():
    # No bom_build: the V1 straight figure, even if a payload is passed in.
    assert straight_wh_per_m(0.44, 10) == pytest.approx(straight_wh_per_m(), rel=1e-9)


def test_copper_loss_is_zero_at_reference_speed_and_rises_at_065():
    from src.bom_profile import load_bom
    from src.hardware_profile import build_profile_from_bom
    from src.twin_energy import MOTOR_COPPER_REF_SPEED_M_S, bom_drive_terms

    prof = build_profile_from_bom("light")
    ref = bom_drive_terms(prof, MOTOR_COPPER_REF_SPEED_M_S)
    fast = bom_drive_terms(prof, 0.65)
    assert fast["drive_w"] > ref["drive_w"] + 0.08


def test_brushed_model_matches_the_light_table_and_the_shift_total():
    # Published light build at 0.44 m/s: drive ~3.43 W, 0.0066 Wh/10 ft.
    point = brushed_drive_point(
        mass_kg=6.142,
        wheel_radius_m=0.04,
        speed_m_s=0.44,
        voltage_v=24,
        no_load_rpm=200,
        no_load_current_a=0.1,
        stall_current_a=3.0,
        stall_torque_nm=2.256,
    )
    assert point["torque_nm"] == pytest.approx(0.0181, abs=0.0002)
    assert point["duty"] == pytest.approx(0.533, abs=0.01)
    assert point["current_a"] == pytest.approx(0.123, abs=0.005)
    assert point["drive_w"] == pytest.approx(3.43, abs=0.05)
    assert point["drive_wh_per_10ft"] == pytest.approx(0.0066, abs=0.00015)
    assert point["extra_wh_per_10ft_per_kg_payload"] == pytest.approx(0.000219, abs=2e-5)
    energy = shift_energy_wh(
        idle_w=9.96,
        shift_h=6,
        distance_m=3000,
        drive_wh_per_10ft=point["drive_wh_per_10ft"],
        extra_wh_per_10ft_per_kg=point["extra_wh_per_10ft_per_kg_payload"],
        payload_kg=10,
    )
    # idle 9.96×6 plus 3 km of drive and 10 kg payload, rounded to 0.01 Wh.
    assert energy == 68.41


def test_shift_energy_rounds_to_the_hundredth_before_a_compare():
    raw = 9.96 * 6 + (3000 / TEN_FT_M) * (0.006597 + 10 * 0.000219)
    assert raw != round(raw, 2)
    assert shift_energy_wh(
        idle_w=9.96,
        shift_h=6,
        distance_m=3000,
        drive_wh_per_10ft=0.006597,
        extra_wh_per_10ft_per_kg=0.000219,
        payload_kg=10,
    ) == round(raw, 2)


def _use_bom(build):
    from src.config import config

    old = config._config.get("bom_build", None)
    config._config["bom_build"] = build
    clear_profile_cache()
    return old


def _restore_bom(old):
    from src.config import config

    if old is None:
        config._config.pop("bom_build", None)
    else:
        config._config["bom_build"] = old
    clear_profile_cache()


def test_bom_wh_per_m_grows_with_payload():
    old = _use_bom("light")
    try:
        empty = straight_wh_per_m(0.44, 0)
        ten = straight_wh_per_m(0.44, 10)
        twenty = straight_wh_per_m(0.44, 20)
        assert empty < ten < twenty
        # Table row, not the V1 0.025 figure and not the 1.35 turn factor.
        assert empty == pytest.approx(0.02576 / TEN_FT_M, rel=1e-4)
        assert ten == pytest.approx(empty + 10 * (0.000219 / TEN_FT_M), rel=1e-3)
        one_ten_ft = estimate_mission_energy(TEN_FT_M, track="s", speed_m_s=0.44, payload_kg=0)
        assert one_ten_ft["source"] == "bom_light"
        assert one_ten_ft["energy_wh"] == pytest.approx(0.02576, abs=0.001)
    finally:
        _restore_bom(old)


def test_measured_override_beats_the_bom_table(tmp_path: Path):
    old = _use_bom("light")
    try:
        path = tmp_path / "twin_energy.json"
        path.write_text('{"wh_per_m": 0.1, "source": "measure_track_run"}', encoding="utf-8")
        out = estimate_mission_energy(10.0, track="s", override_path=path, payload_kg=10)
        assert out["source"] == "measure_track_run"
        assert out["energy_wh"] == 1.0
        # The model line ignores the file so measured Wh can sit beside it.
        line = format_mission_energy_line(10.0, payload_kg=0, wh_measured=1.0)
        assert line.startswith("MISSION_ENERGY build=light ")
        assert "wh_measured=1.00" in line
        assert "wh_model=" in line
        assert "wh_model=1.0" not in line
    finally:
        _restore_bom(old)


def test_mission_energy_over_a_duration_includes_idle():
    # 9.96 W for 450 s is 1.245 Wh of idle, even if the robot did not move.
    old = _use_bom("light")
    try:
        parked = bom_run_energy_wh(0.0, 450.0, 0.0)
        assert parked["wh_model_idle"] == pytest.approx(1.245, abs=0.001)
        assert parked["wh_model_drive"] == 0.0
        assert parked["wh_model"] == pytest.approx(1.245, abs=0.001)
        line = format_mission_energy_line(0.0, payload_kg=0, wh_measured=0.5, duration_s=450)
        assert "wh_model=1.245" in line
        assert "wh_measured=0.50" in line
        assert "err_pct=na" not in line
        assert "err_pct=" in line
        moving = bom_run_energy_wh(TEN_FT_M, 450.0, 0.0)
        assert moving["wh_model"] > moving["wh_model_drive"] > 0
        assert moving["wh_model_idle"] == pytest.approx(1.245, abs=0.001)
    finally:
        _restore_bom(old)


def test_default_build_energy_error_is_na():
    """V1's blended cruise lump is not measured drive plus channel idle."""
    old = _use_bom(None)
    try:
        line = format_mission_energy_line(
            19.82, payload_kg=0.0, wh_measured=0.72, duration_s=60.5
        )
        assert line.startswith("MISSION_ENERGY build=default ")
        assert "wh_measured=0.72" in line
        assert "err_pct=na" in line
        model = line.split("wh_model=", 1)[1].split()[0]
        assert model not in ("", "na")
        assert float(model) == pytest.approx(19.82 * 40.0 / 0.44 / 3600.0, abs=0.002)
    finally:
        _restore_bom(old)


def test_drive_wh_per_metre_depends_on_speed_and_yaw_is_separate():
    """Rolling Wh per 10 ft is the motor line. Yaw is a separate turn term."""
    from src.hardware_profile import get_active_profile, load_hardware_profile

    old = _use_bom("light")
    try:
        prof = get_active_profile()
        cruise = bom_drive_terms(prof, 0.44)
        crawl = bom_drive_terms(prof, 0.12)
        assert cruise["drive_wh_per_10ft"] == pytest.approx(0.0066, abs=0.00015)
        # Same 10 ft takes longer at 0.12 m/s, so the rolling-torque watts
        # add a little more energy. Still the same order, not ~0.11 Wh.
        assert crawl["drive_wh_per_10ft"] > cruise["drive_wh_per_10ft"]
        assert crawl["drive_wh_per_10ft"] < 0.02

        dt = TEN_FT_M / 0.44
        straight = bom_run_energy_wh(TEN_FT_M, dt, 0.0)
        assert straight["wh_model_turn"] == 0.0
        assert straight["wh_model_accel"] == 0.0
        # Logged to 0.001 Wh, so 0.0066 becomes 0.007. Viscous hinge
        # damping is extra on top of that rolling bill.
        assert straight["wh_model_roll"] == pytest.approx(round(cruise["drive_wh_per_10ft"], 3), abs=1e-6)
        assert straight["wh_model_visc"] > straight["wh_model_roll"]
        assert straight["wh_model_drive"] == pytest.approx(
            straight["wh_model_roll"] + straight["wh_model_visc"], abs=0.002
        )
        assert straight["turn_source"] == "none"

        curved = bom_run_energy_wh(TEN_FT_M, dt, 0.0, heading_change_deg=61.6)
        assert curved["wh_model_turn"] > 0.0
        assert curved["turn_source"] == "world_fds_track_scrub"
        assert curved["wh_model_drive"] == pytest.approx(
            curved["wh_model_roll"] + curved["wh_model_visc"] + curved["wh_model_turn"],
            abs=0.002,
        )
        assert curved["wh_model_roll"] == straight["wh_model_roll"]
        assert curved["wh_model_visc"] == straight["wh_model_visc"]

        track = float(load_hardware_profile("butlerbot_wheeled")["geometry"]["track_width_m"])
        assert track == pytest.approx(0.34)
        scrub = geometry_scrub_wh(
            6.142, 61.6, crr=0.015, track_width_m=track, eta_driver=0.92
        )
        # The Crr arc is real and tiny. It is not the billed turn term.
        assert 0 < scrub < 0.001
        assert curved["wh_model_turn"] > 50.0 * scrub

        sped_up = accel_electrical_wh(prof, 0.0, 0.44, 1.0, 0.0)
        assert 0 < sped_up < 0.01
        assert accel_electrical_wh(prof, 0.44, 0.0, 1.0, 0.0) == 0.0
    finally:
        _restore_bom(old)


def test_warehouse_run_checks_drive_shape_without_a_fitted_bill():
    """Full warehouse run, stopped by hand because that world loops.

    121 segments, 369 m, 2145 s. Mean locomotion 0.120 Wh per 10 ft
    (sd 0.023). Correlation with heading −0.11, with speed +0.82.
    Bins, Wh per segment: <0.15 m/s 0.096; 0.15–0.25 0.103;
    0.25–0.35 0.126; >0.35 0.153. Totals: drive 14.5 Wh, idle 5.9 Wh.
    A reported line ``0.248*v + 0.056`` describes that log. It is not
    an input to the model.

    Check speeds sit inside those bins. Rolling-only falls as speed
    rises, so it is the wrong shape. Published hinge damping rises
    with speed and stays under every bin. The gap is sim tire contact
    and is left unfitted. Idle matches the 9.96 W BoM figure.
    """
    from src.twin.webots_power import SIM_WHEEL_RADIUS_M as drawn_radius
    from src.twin_energy import (
        SIM_HINGE_DAMPING_NM_S_RAD,
        SIM_WHEEL_RADIUS_M,
        sim_drive_terms,
    )

    old = _use_bom("light")
    try:
        from src.hardware_profile import get_active_profile

        prof = get_active_profile()
        assert drawn_radius == SIM_WHEEL_RADIUS_M == 0.08
        assert SIM_HINGE_DAMPING_NM_S_RAD == 0.15

        # Representative speeds inside each reported bin, not a regression.
        bins = (
            (0.10, 0.096),
            (0.20, 0.103),
            (0.30, 0.126),
            (0.40, 0.153),
        )
        roll_wh = []
        sim_wh = []
        for speed, observed in bins:
            roll = bom_drive_terms(prof, speed)["drive_wh_per_10ft"]
            sim = sim_drive_terms(prof, speed)["drive_wh_per_10ft"]
            roll_wh.append(roll)
            sim_wh.append(sim)
            assert sim > roll
            assert sim < observed
            assert observed - sim > 0.02
        assert roll_wh[0] > roll_wh[-1]
        assert sim_wh == sorted(sim_wh)
        assert sim_wh[-1] > sim_wh[0] * 2

        mean_v = 369.0 / 2145.0
        n_10ft = 369.0 / TEN_FT_M
        roll_total = bom_drive_terms(prof, mean_v)["drive_wh_per_10ft"] * n_10ft
        sim_total = sim_drive_terms(prof, mean_v)["drive_wh_per_10ft"] * n_10ft
        assert roll_total < 2.0
        assert sim_total > roll_total * 3.0
        assert sim_total < 14.5

        idle = 9.96 * 2145.0 / 3600.0
        assert idle == pytest.approx(5.9, abs=0.1)
        plain = bom_run_energy_wh(369.0, 2145.0, 0.0, heading_change_deg=0.0)
        # 400° spread over 2145 s is a tiny average yaw. A short corner is not.
        spread = bom_run_energy_wh(369.0, 2145.0, 0.0, heading_change_deg=400.0)
        corner_dt = TEN_FT_M / 0.22
        corner = bom_run_energy_wh(TEN_FT_M, corner_dt, 0.0, heading_change_deg=90.0)
        straight_corner = bom_run_energy_wh(TEN_FT_M, corner_dt, 0.0, heading_change_deg=0.0)
        assert plain["wh_model_turn"] == 0.0
        assert spread["wh_model_roll"] == plain["wh_model_roll"]
        assert spread["wh_model_visc"] == plain["wh_model_visc"]
        assert corner["wh_model_turn"] > straight_corner["wh_model_turn"]
        assert corner["wh_model_roll"] == straight_corner["wh_model_roll"]
        assert plain["wh_model_idle"] == pytest.approx(idle, abs=0.01)
        assert plain["wh_model_visc"] > plain["wh_model_roll"]
    finally:
        _restore_bom(old)


def test_turning_scrub_uses_world_contact_and_the_4693_curve():
    """107.6° corner: force from track, μ cap, and FDS k, then the motor line.

    The hand formula below is the bill. It is not a regression to run #8.
    """
    import math

    from src.hardware_profile import get_active_profile
    from src.twin_energy import (
        SIM_HINGE_DAMPING_NM_S_RAD,
        SIM_WHEEL_RADIUS_M,
        WORLD_DRIVE_COULOMB_FRICTION,
        WORLD_DRIVE_FDS_M_S_PER_N,
        electrical_from_shaft_w,
        sim_drive_terms,
        turning_scrub_point,
        turning_scrub_wh,
    )

    old = _use_bom("light")
    try:
        prof = get_active_profile()
        spec = prof["motors"]["left_wheel"]
        assert WORLD_DRIVE_COULOMB_FRICTION == 8.0
        assert WORLD_DRIVE_FDS_M_S_PER_N == 0.01
        assert SIM_WHEEL_RADIUS_M == 0.08
        mass = 6.142
        g = 9.81
        track = 0.34
        r_bom = 0.04
        crr = 0.015
        v = 0.22
        heading = 107.6
        dist = TEN_FT_M
        dt = dist / v
        yaw = math.radians(heading) / dt
        v_slip = (track / 2.0) * yaw
        normal = mass * g / 2.0
        force = min(WORLD_DRIVE_COULOMB_FRICTION * normal, v_slip / WORLD_DRIVE_FDS_M_S_PER_N)
        # Below the cap. Mobilizing μ=8 would be a different, larger force.
        assert force < WORLD_DRIVE_COULOMB_FRICTION * normal * 0.5
        tau_scrub = force * SIM_WHEEL_RADIUS_M
        assert tau_scrub < float(spec["stall_torque_nm"])

        def one(speed: float, scrub: float, radius: float) -> float:
            tau = crr * mass * g * r_bom / 2.0 + SIM_HINGE_DAMPING_NM_S_RAD * speed / SIM_WHEEL_RADIUS_M + scrub
            omega = speed / radius
            omega0 = 200.0 * 2.0 * math.pi / 60.0
            duty = omega / omega0 + tau / 2.256
            current = 0.1 + (3.0 - 0.1) * (tau / 2.256)
            assert current < 3.0
            return duty * 24.0 * current / 0.92

        # Turn delta uses the contact hinge rate, same as measured electrical
        # and the straight viscous bill. Catalog radius stays in the rolling
        # torque only. v/0.04 doubled light corner watts.
        straight = 2.0 * one(v, 0.0, SIM_WHEEL_RADIUS_M)
        turned = one(v - v_slip, tau_scrub, SIM_WHEEL_RADIUS_M) + one(v + v_slip, tau_scrub, SIM_WHEEL_RADIUS_M)
        catalog = (
            one(v - v_slip, tau_scrub, r_bom) + one(v + v_slip, tau_scrub, r_bom)
            - 2.0 * one(v, 0.0, r_bom)
        )
        hinge = straight
        assert sim_drive_terms(prof, v)["drive_w"] == pytest.approx(hinge, rel=1e-9)
        assert turned - straight < catalog
        point = electrical_from_shaft_w(
            crr * mass * g * r_bom / 2.0 + SIM_HINGE_DAMPING_NM_S_RAD * v / SIM_WHEEL_RADIUS_M,
            v / SIM_WHEEL_RADIUS_M,
            voltage_v=24.0,
            no_load_rpm=200.0,
            no_load_current_a=0.1,
            stall_current_a=3.0,
            stall_torque_nm=2.256,
            eta_driver=0.92,
        )
        assert 2.0 * point["elec_w"] == pytest.approx(straight, rel=1e-9)
        assert point["eta"] != pytest.approx(0.5, abs=0.02)
        assert point["eta"] != pytest.approx(0.92, abs=0.02)

        scrub = turning_scrub_point(prof, v, yaw)
        assert scrub["scrub_force_n"] == pytest.approx(force, rel=1e-9)
        assert scrub["scrub_torque_nm"] == pytest.approx(tau_scrub, rel=1e-9)
        assert scrub["straight_w"] == pytest.approx(straight, rel=1e-9)
        assert scrub["turned_w"] == pytest.approx(turned, rel=1e-9)
        assert scrub["delta_w"] == pytest.approx(turned - straight, rel=1e-9)
        assert scrub["clamped"] is False
        assert scrub["turn_source"] == "world_fds_track_scrub"
        assert turning_scrub_wh(prof, v, heading, dt) == pytest.approx(
            (turned - straight) * dt / 3600.0, rel=1e-9
        )
        assert turning_scrub_wh(prof, v, 0.0, dt) == 0.0
        # Design bill is untouched: no scrub torque, BoM radius, rolling only.
        cruise = bom_drive_terms(prof, 0.44)
        assert cruise["drive_w"] == pytest.approx(3.43, abs=0.05)
        assert cruise["drive_wh_per_10ft"] == pytest.approx(0.0066, abs=0.00015)
    finally:
        _restore_bom(old)


def test_straight_bill_stays_rolling_plus_hinge_damping():
    """Run #9 straights were about 10-25% low at 0.20 m/s and 30-40% at 0.29 m/s.

    The miss grows with speed. A constant extra torque makes the percent
    fall as speed rises. Extra viscous damping stays nearly flat. A
    0.2 rad/s wheel split is under 1% and also falls with speed. None of
    those is billed. The straight line stays Crr plus dampingConstant 0.15.
    """
    from src.hardware_profile import get_active_profile
    from src.twin_energy import (
        GRAVITY_M_S2,
        SIM_HINGE_DAMPING_NM_S_RAD,
        SIM_WHEEL_RADIUS_M,
        _hub_electrical_w,
        _profile_motor_args,
        electrical_from_shaft_w,
        sim_drive_terms,
        sim_hinge_torque_nm,
    )

    old = _use_bom("light")
    try:
        prof = get_active_profile()
        args = _profile_motor_args(prof)

        def at(v, extra_tau):
            tau = (
                args["crr"] * args["mass_kg"] * GRAVITY_M_S2 * args["wheel_radius_m"] / 2.0
                + SIM_HINGE_DAMPING_NM_S_RAD * v / SIM_WHEEL_RADIUS_M
                + extra_tau
            )
            point = electrical_from_shaft_w(
                tau,
                v / SIM_WHEEL_RADIUS_M,
                voltage_v=args["voltage_v"],
                no_load_rpm=args["no_load_rpm"],
                no_load_current_a=args["no_load_current_a"],
                stall_current_a=args["stall_current_a"],
                stall_torque_nm=args["stall_torque_nm"],
                eta_driver=args["eta_driver"],
            )
            return 2.0 * point["elec_w"]

        for v in (0.20, 0.29):
            assert sim_drive_terms(prof, v)["drive_w"] == pytest.approx(at(v, 0.0))
            assert sim_hinge_torque_nm(v) == pytest.approx(0.15 * v / 0.08)

        const_20 = (at(0.20, 0.05) - at(0.20, 0.0)) / at(0.20, 0.0)
        const_29 = (at(0.29, 0.05) - at(0.29, 0.0)) / at(0.29, 0.0)
        assert const_20 > const_29

        visc_20 = (at(0.20, 0.04 * 0.20 / 0.08) - at(0.20, 0.0)) / at(0.20, 0.0)
        visc_29 = (at(0.29, 0.04 * 0.29 / 0.08) - at(0.29, 0.0)) / at(0.29, 0.0)
        assert abs(visc_20 - visc_29) < 0.03

        def split_premium(v, half_split_rad_s):
            base = 2.0 * _hub_electrical_w(args, args["mass_kg"], v, 0.0)[0]
            dv = half_split_rad_s * SIM_WHEEL_RADIUS_M
            left = _hub_electrical_w(args, args["mass_kg"], v + dv, 0.0)[0]
            right = _hub_electrical_w(args, args["mass_kg"], max(0.01, v - dv), 0.0)[0]
            return (left + right - base) / base

        assert split_premium(0.20, 0.2) < 0.01
        assert split_premium(0.20, 0.2) > split_premium(0.29, 0.2)
        cruise = bom_drive_terms(prof, 0.44)
        assert cruise["drive_w"] == pytest.approx(3.43, abs=0.05)
        assert cruise["drive_wh_per_10ft"] == pytest.approx(0.0066, abs=0.00015)
    finally:
        _restore_bom(old)


def test_propose_exposes_paper_and_twin(tmp_path: Path):
    from src.parts_db import connect

    db = tmp_path / "parts.db"
    connect(db).close()
    result = propose(Mission(track_length_m=25, max_minutes=5), db_path=db)
    top = result["candidates"][0]
    t = top["totals"]
    assert t["energy_source"] == "v1_wheeled_energy_baseline"
    assert t["energy_wh_paper"] is not None
    assert t["energy_wh_est"] < t["energy_wh_paper"]
