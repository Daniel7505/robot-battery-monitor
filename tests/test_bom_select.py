"""Deterministic build-out chooser: cheapest first, no guessed watts."""

from src.design_agent import BOM_BUILD_ORDER, BuildRequest, select_build
from src.twin_energy import shift_energy_wh


def _worked() -> BuildRequest:
    return BuildRequest(
        payload_kg_max=10,
        payload_kg_avg=10,
        shift_h=6,
        distance_m=3000,
        speed_m_s=0.44,
    )


def test_worked_example_picks_light_at_68_41_wh():
    first = select_build(_worked())
    second = select_build(_worked())
    assert first["log"] == second["log"]
    assert first["chosen"] == "light"
    assert first["upgraded"] is False
    assert first["battery"] == "BLF-2410A"
    assert first["E_wh"] == 68.41
    assert first["reserve_wh"] == 192.0
    assert first["log"] == (
        "BUILD_SELECT chosen=light gates=budget:fail,light:pass,heavy:fail "
        "E_wh=68.41 reserve_wh=192.00"
    )
    light = first["gates"]["light"]
    assert light["pass"] is True
    assert light["failed"] == []
    # Radar and the driver have no published idle. They are omitted, not invented,
    # and they do not fail the energy gate.
    assert not any(flag.startswith("unknown_draw=") for flag in light["flags"])
    assert any(flag.startswith("price_missing=") for flag in light["flags"])

    budget = first["gates"]["budget"]
    assert budget["pass"] is False
    assert "energy" in budget["failed"]
    assert budget["E_wh"] is None
    unknown = next(flag for flag in budget["flags"] if flag.startswith("unknown_draw="))
    assert "cmp_rpi5" in unknown
    assert "cam_rpi_cm3" in unknown

    heavy = first["gates"]["heavy"]
    assert "energy" in heavy["failed"]
    assert heavy["E_wh"] is None
    assert any("cam_realsense_d435i" in flag for flag in heavy["flags"])


def test_long_shift_upgrades_the_light_pack():
    # 20 h of the same 3 km / 10 kg job no longer fits in 80% of 240 Wh.
    request = BuildRequest(
        payload_kg_max=10,
        payload_kg_avg=10,
        shift_h=20,
        distance_m=3000,
        speed_m_s=0.44,
    )
    picked = select_build(request)
    assert picked["chosen"] == "light"
    assert picked["upgraded"] is True
    assert picked["battery"] == "BLF-2420A"
    assert picked["reserve_wh"] == 384.0
    assert picked["E_wh"] is not None
    assert 192 < picked["E_wh"] <= picked["reserve_wh"]
    assert picked["E_wh"] == round(picked["E_wh"], 2)
    assert "light:pass" in picked["log"]
    assert picked["gates"]["budget"]["E_wh"] is None
    assert picked["gates"]["heavy"]["E_wh"] is None


def test_speed_or_torque_with_no_survivor_is_explicit():
    too_fast = select_build(BuildRequest(speed_m_s=5.0, distance_m=10, shift_h=0.1))
    assert too_fast["status"] == "NO_FEASIBLE_BUILD"
    assert too_fast["chosen"] is None
    assert too_fast["E_wh"] is None
    assert all("speed" in gate["failed"] for gate in too_fast["gates"].values())
    assert "NO_FEASIBLE_BUILD" in too_fast["log"]

    too_heavy = select_build(
        BuildRequest(payload_kg_max=10000, payload_kg_avg=0, distance_m=10, shift_h=0.1)
    )
    assert too_heavy["chosen"] is None
    assert all("torque" in gate["failed"] for gate in too_heavy["gates"].values())


def test_a_tight_budget_rejects_a_build_that_otherwise_fits():
    roomy = select_build(
        BuildRequest(
            payload_kg_max=10,
            payload_kg_avg=10,
            shift_h=6,
            distance_m=3000,
            speed_m_s=0.44,
            budget_usd=2000,
        )
    )
    assert roomy["chosen"] == "light"
    tight = select_build(
        BuildRequest(
            payload_kg_max=10,
            payload_kg_avg=10,
            shift_h=6,
            distance_m=3000,
            speed_m_s=0.44,
            budget_usd=100,
        )
    )
    assert tight["chosen"] is None
    assert "budget" in tight["gates"]["light"]["failed"]


def test_shift_compare_uses_the_rounded_wh():
    assert shift_energy_wh(
        idle_w=1.005,
        shift_h=1,
        distance_m=0,
        drive_wh_per_10ft=0,
        extra_wh_per_10ft_per_kg=0,
        payload_kg=0,
    ) == 1.0


def _toy_part(**fields):
    base = {
        "price_usd": 10,
        "mass_g": 100,
        "idle_w": 1.0,
        "voltage_v": 24,
        "no_load_rpm": 200,
        "no_load_current_a": 0.1,
        "stall_current_a": 3.0,
        "stall_torque_nm": 2.0,
        "diameter_mm": 80,
        "continuous_current_a_per_ch": 14,
        "peak_discharge_a": 20,
        "capacity_wh": 240,
        "max_continuous_discharge_a": 10,
        "part_number": "TOY",
    }
    base.update(fields)
    return base


def _toy_build(price, *, driver_id="d", free=1.0):
    return {
        "lines": [
            {"part_id": "m", "qty": 2},
            {"part_id": driver_id, "qty": 1},
            {"part_id": "b", "qty": 1},
            {"part_id": "w", "qty": 2},
        ],
        "totals": {
            "mass_kg_known": 6.0,
            "price_usd_known": price,
            "idle_w_known": 5.0,
            "battery_wh": 240,
            "price_missing_parts": [],
        },
        "energy": [{"motor_free_speed_m_s": free, "speed_m_s": 0.44}],
        "torque_headroom": {"max_total_rolling_mass_kg_steady": 300},
    }


def _toy_bom(*, weak_budget_driver=False):
    parts = [
        _toy_part(id="m", category="drive_motor", part_number="M"),
        _toy_part(id="d", category="motor_driver", part_number="D"),
        _toy_part(
            id="d_weak",
            category="motor_driver",
            part_number="DWEAK",
            continuous_current_a_per_ch=0.1,
        ),
        _toy_part(id="b", category="battery", part_number="PACK-A", capacity_wh=240),
        _toy_part(id="w", category="wheel", part_number="W", diameter_mm=80),
    ]
    driver = "d_weak" if weak_budget_driver else "d"
    return {
        "assumptions": {"crr": 0.015, "driver_eff": 0.92},
        "parts": parts,
        "builds": {
            "budget": _toy_build(50, driver_id=driver),
            "light": _toy_build(100),
            "heavy": _toy_build(200),
        },
    }


def test_cheapest_build_wins_when_every_gate_passes():
    picked = select_build(BuildRequest(distance_m=10, shift_h=1, payload_kg_avg=0), bom=_toy_bom())
    assert picked["chosen"] == BOM_BUILD_ORDER[0]
    assert picked["chosen"] == "budget"
    assert picked["E_wh"] is not None
    assert all(gate["pass"] for gate in picked["gates"].values())


def test_skips_a_cheaper_build_that_fails_current():
    picked = select_build(
        BuildRequest(distance_m=10, shift_h=1, payload_kg_avg=0),
        bom=_toy_bom(weak_budget_driver=True),
    )
    assert picked["chosen"] == "light"
    assert "current" in picked["gates"]["budget"]["failed"]
    assert picked["gates"]["light"]["pass"] is True
