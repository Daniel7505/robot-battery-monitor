"""BoM mass on the Webots body, without a Webots GUI.

The shipped worlds stay at 3.2 kg and a 0.08 m wheel. A selected build
must change the robot solid's mass and inertia, and must leave both the
drawn Shape and the contact cylinder at 0.08 m. The catalog wheel
diameter is recorded on the physics object and is not written into the
world: hinge-damper shaft energy scales as 1/r^2.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from src.butlerbot_proto import expand_world_text
from src.hardware_profile import build_profile_from_bom
from src.webots_body import (
    BODY_INERTIA_DIAG,
    BODY_MASS_KG,
    DRAWN_WHEEL_RADIUS_M,
    WHEEL_INERTIA_DIAG,
    apply_active_bom_body,
    apply_bom_body_vrml,
    body_matches,
    bom_body_physics,
    format_bom_body_log_line,
    install_command_radius,
    plan_bom_world,
    read_robot_body,
)

ROOT = Path(__file__).resolve().parent.parent
WORLDS = (
    ROOT / "webots" / "worlds" / "butlerbot.wbt",
    ROOT / "webots" / "worlds" / "butlerbot_warehouse.wbt",
)
BUILDS = ("light", "heavy", "budget")


@pytest.fixture
def select_build():
    from src.config import config

    had = "bom_build" in config._config
    old = config._config.get("bom_build")

    def select(build):
        if build is None:
            config._config["bom_build"] = None
        else:
            config._config["bom_build"] = build

    yield select
    if had:
        config._config["bom_build"] = old
    else:
        config._config.pop("bom_build", None)


def _prefix(text: str) -> str:
    return text.split("\nButlerBot {", 1)[0]


def _expanded_robot(text: str) -> str:
    expanded = expand_world_text(text)
    return expanded[expanded.index("Robot {"):]


@pytest.mark.parametrize("build", BUILDS)
@pytest.mark.parametrize("world", WORLDS, ids=lambda path: path.name)
def test_each_build_sets_mass_and_keeps_the_contact_radius(build, world):
    physics = bom_body_physics(build)
    profile = build_profile_from_bom(build)
    geom = profile["geometry"]
    assert physics is not None
    assert physics.mass_kg == pytest.approx(geom["mass_kg_known"])
    assert physics.wheel_radius_m == pytest.approx(geom["wheel_radius_m"])
    assert physics.contact_radius_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
    assert physics.drawn_radius_m == DRAWN_WHEEL_RADIUS_M
    assert physics.wheel_radius_m != pytest.approx(physics.contact_radius_m)
    mass_scale = physics.mass_kg / BODY_MASS_KG
    assert physics.body_inertia == pytest.approx(tuple(v * mass_scale for v in BODY_INERTIA_DIAG))
    assert physics.wheel_inertia == pytest.approx(WHEEL_INERTIA_DIAG)

    original = world.read_text(encoding="utf-8")
    patched = apply_bom_body_vrml(original, physics)
    assert _prefix(patched) == _prefix(original)  # floor, racks, and the rest of the world stay put
    robot = _expanded_robot(original)
    robot_after = _expanded_robot(patched)
    assert "    mass 3.2\n" not in robot_after
    assert robot_after.count("mass 0.085") == robot.count("mass 0.085")
    # Casters stay the shipped spheres. A 0.04 m BoM wheel is a drive-wheel
    # contact cylinder, not a resized caster.
    for caster in (
        "translation 0.11 0.13 0.044",
        "translation 0.11 -0.13 0.044",
        "translation -0.11 0.13 0.044",
        "translation -0.11 -0.13 0.044",
        "boundingObject Sphere {",
    ):
        assert robot_after.count(caster) == robot.count(caster)

    body = read_robot_body(patched)
    assert body.mass_kg == pytest.approx(physics.mass_kg)
    assert body.inertia == pytest.approx(physics.body_inertia)
    assert body.left.translation_y_m > 0
    assert body.right.translation_y_m < 0
    for wheel in (body.left, body.right):
        assert wheel.drawn_radius_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
        assert wheel.physics_radius_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
        assert wheel.anchor_z_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
        assert wheel.translation_z_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
        assert wheel.inertia == pytest.approx(WHEEL_INERTIA_DIAG)
    # The BoM note is a field comment on the ButlerBot instance. Expansion
    # keeps the mass and inertia, and drops that comment.
    assert f"contact cylinder stays {DRAWN_WHEEL_RADIUS_M:g} m" in patched
    assert f"Catalog wheel {physics.wheel_radius_m:g} m" in patched
    assert f"BoM {build}:" in patched
    assert body_matches(patched, physics)


def test_light_and_heavy_do_not_share_mass_or_catalog_radius():
    light = bom_body_physics("light")
    heavy = bom_body_physics("heavy")
    budget = bom_body_physics("budget")
    assert light.mass_kg < heavy.mass_kg
    assert light.wheel_radius_m < heavy.wheel_radius_m
    assert budget.wheel_radius_m != pytest.approx(light.wheel_radius_m)
    assert budget.mass_kg != pytest.approx(heavy.mass_kg)
    assert light.contact_radius_m == heavy.contact_radius_m == DRAWN_WHEEL_RADIUS_M


def test_switching_builds_does_not_scale_inertia_twice():
    original = (ROOT / "webots" / "worlds" / "butlerbot.wbt").read_text(encoding="utf-8")
    heavy = bom_body_physics("heavy")
    light = bom_body_physics("light")
    once = apply_bom_body_vrml(apply_bom_body_vrml(original, heavy), light)
    direct = apply_bom_body_vrml(original, light)
    assert read_robot_body(once).mass_kg == pytest.approx(light.mass_kg)
    assert read_robot_body(once).left.physics_radius_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
    assert read_robot_body(once).left.drawn_radius_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
    assert read_robot_body(once).inertia == pytest.approx(read_robot_body(direct).inertia)
    assert read_robot_body(once).left.inertia == pytest.approx(WHEEL_INERTIA_DIAG)


def test_unset_build_does_not_rewrite_world_text(select_build):
    select_build(None)
    sample = "    mass 3.2\n        radius 0.08\n"
    assert apply_active_bom_body(sample) is sample
    original = (ROOT / "webots" / "worlds" / "butlerbot.wbt").read_text(encoding="utf-8")
    assert apply_active_bom_body(original) is original
    assert bom_body_physics() is None


def test_prepare_writes_a_sibling_and_leaves_the_shipped_file(tmp_path, select_build):
    select_build("heavy")
    src = tmp_path / "butlerbot_warehouse.wbt"
    src.write_bytes((ROOT / "webots" / "worlds" / "butlerbot.wbt").read_bytes())
    before = src.read_bytes()
    plan = plan_bom_world(src)
    assert plan.reload is True
    assert Path(plan.path).name == "butlerbot_warehouse.bom-heavy.wbt"
    assert Path(plan.path).parent == tmp_path
    assert src.read_bytes() == before
    body = read_robot_body(Path(plan.path).read_text(encoding="utf-8"))
    heavy = bom_body_physics("heavy")
    assert body.mass_kg == pytest.approx(heavy.mass_kg)
    assert body.left.physics_radius_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)
    assert body.left.drawn_radius_m == pytest.approx(DRAWN_WHEEL_RADIUS_M)

    again = plan_bom_world(plan.path)
    assert again.reload is False
    assert again.path == plan.path
    assert Path(plan.path).read_bytes() == Path(again.path).read_bytes()


def test_command_radius_stays_on_the_contact_wheel():
    import src.lane_keep as lane_keep
    import src.rowfit_control as rowfit
    import src.teleop_agent as teleop
    from src.twin.webots_power import _hinge_rad_s

    saved = (lane_keep.WHEEL_RADIUS_M, rowfit.WHEEL_RADIUS_M, teleop.WHEEL_RADIUS_M)
    try:
        heavy = bom_body_physics("heavy")
        install_command_radius(heavy.contact_radius_m)
        assert lane_keep.WHEEL_RADIUS_M == DRAWN_WHEEL_RADIUS_M
        assert rowfit.WHEEL_RADIUS_M == DRAWN_WHEEL_RADIUS_M
        assert teleop.WHEEL_RADIUS_M == DRAWN_WHEEL_RADIUS_M
        assert _hinge_rad_s({}, 0.2) == pytest.approx(0.2 / DRAWN_WHEEL_RADIUS_M)
        # The installer can still be pointed elsewhere; the BoM path does not.
        install_command_radius(0.1)
        assert lane_keep.WHEEL_RADIUS_M == 0.1
        assert _hinge_rad_s({}, 0.2) == pytest.approx(2.0)
        assert teleop.abs_brake_wheel_velocity_latched(1.0, 0.4) == pytest.approx(
            -0.4 / 0.1 * teleop.BRAKE_OPPOSE_FACTOR
        )
    finally:
        lane_keep.WHEEL_RADIUS_M, rowfit.WHEEL_RADIUS_M, teleop.WHEEL_RADIUS_M = saved


def test_bom_body_log_line_names_contact_and_catalog_radius():
    light = format_bom_body_log_line("light")
    heavy = format_bom_body_log_line("heavy")
    assert light is not None and heavy is not None
    assert light.startswith("BOM BODY build=light ")
    assert "mass_kg=6.142" in light
    assert "catalog_wheel_radius_m=0.04" in light
    assert "contact_radius_m=0.08" in light
    assert "mass_kg=9.812" in heavy
    assert "catalog_wheel_radius_m=0.1" in heavy
    assert "contact_radius_m=0.08" in heavy
    default = format_bom_body_log_line("")
    assert default is not None
    assert default.startswith("BOM BODY build=default ")
    assert "mass_kg=3.2" in default
    assert "contact_radius_m=0.08" in default
    assert "no BoM build selected" in default
