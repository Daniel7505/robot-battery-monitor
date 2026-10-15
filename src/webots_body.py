"""Put the selected BoM build's mass on the Webots robot.

The shipped ``butlerbot*.wbt`` files keep a 3.2 kg body and a 0.08 m drive
wheel. That 0.08 m is both the drawn mesh and the contact cylinder. The
controller, odometry (``v = ω · r``), and the calibrated viscous energy bill
all assume that same 0.08 m.

When ``config.yaml`` sets ``bom_build`` to ``light``, ``heavy``, or
``budget``:

* The robot solid's mass becomes ``geometry.mass_kg_known`` from
  ``config/bom/bom.json`` (via the same profile the twin already loads).
* That solid's inertia diagonal scales by ``mass / 3.2`` from the shipped
  diagonal. The shape of the body box does not change.
* The contact cylinder stays 0.08 m on purpose. A warehouse check with the
  catalog radius on the contact cylinder (light 0.04 m, heavy 0.10 m) made
  shaft energy follow the hinge damper, which scales as ``1/r²``: the light
  lap's shaft rose from ~0.866 Wh to 3.096 Wh, and the heavy lap used *less*
  shaft energy per metre than the light one. The viscous model still bills
  0.08 m, so the comparison error left the ~5% band. Catalog diameter stays
  on the profile for motor math; it is not written onto the tire.
* Wheel solid mass stays 0.085 kg and wheel inertia stays the shipped
  diagonal. The listing is already inside ``mass_kg_known``, and the
  light/budget pages do not say whether ``mass_g`` is per wheel or per pair.
* The drawn ``Shape`` cylinder stays 0.08 m. Cameras, lane paint, and the
  rest of the world are not resized.

Unset ``bom_build`` leaves the world text untouched. A selected build is
written to a sibling ``<world>.bom-<build>.wbt`` (gitignored). The shipped
file stays the 3.2 kg / 0.08 m baseline.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

# Shipped butlerbot*.wbt body. These are the file's numbers, not new specs.
DRAWN_WHEEL_RADIUS_M = 0.08
BODY_MASS_KG = 3.2
BODY_INERTIA_DIAG = (0.055, 0.062, 0.078)
WHEEL_INERTIA_DIAG = (1.4e-4, 2.7e-4, 1.4e-4)
WHEEL_SOLID_MASS_KG = 0.085

_BODY_PHYSICS = "\n  physics Physics {"
_INERTIA_LINE = re.compile(
    r"(inertiaMatrix \[\n\s+)([0-9.eE+-]+) ([0-9.eE+-]+) ([0-9.eE+-]+)"
)
_MASS_LINE = re.compile(r"(    mass )([0-9.eE+-]+)")
_COMMENT_BEFORE_MASS = re.compile(r"    # [^\n]*\n(?=    mass )")
_RADIUS_LINE = re.compile(r"(radius\s+)([0-9.eE+-]+)")
_ANCHOR_Z = re.compile(r"(anchor 0 -?0\.17 )([0-9.eE+-]+)")
_TRANSLATION_Y = re.compile(r"translation 0 (-?[0-9.eE+-]+) ([0-9.eE+-]+)")
_BOM_SUFFIX = re.compile(r"^(?P<stem>.+)\.bom-[a-z0-9_-]+$")


@dataclass(frozen=True)
class BomBodyPhysics:
    """Mass to write onto the robot. Contact radius stays 0.08 m.

    ``wheel_radius_m`` is the catalog radius (``diameter_mm / 2000``). It is
    logged and used by motor math. ``contact_radius_m`` is what the physics
    cylinder, the controller, and the viscous bill share.
    """

    build: str
    mass_kg: float
    wheel_radius_m: float
    contact_radius_m: float
    drawn_radius_m: float
    body_inertia: tuple[float, float, float]
    wheel_inertia: tuple[float, float, float]


@dataclass(frozen=True)
class WheelBody:
    drawn_radius_m: float
    physics_radius_m: float
    anchor_z_m: float
    translation_y_m: float
    translation_z_m: float
    inertia: tuple[float, float, float]


@dataclass(frozen=True)
class RobotBody:
    mass_kg: float
    inertia: tuple[float, float, float]
    left: WheelBody
    right: WheelBody


def _fmt(value: float) -> str:
    return f"{float(value):.10g}"


def bom_body_physics(build_id: str | None = None) -> BomBodyPhysics | None:
    """Physics for one BoM build, or None when no build is selected.

    Pass ``build_id`` to ignore config. With no argument, this follows
    ``bom_build`` the same way the twin does.
    """
    if build_id is None:
        from src.hardware_profile import active_bom_build

        build_id = active_bom_build()
    if not build_id:
        return None
    from src.hardware_profile import build_profile_from_bom

    profile = build_profile_from_bom(build_id)
    geom = profile.get("geometry") or {}
    mass = geom.get("mass_kg_known")
    radius = geom.get("wheel_radius_m")
    if mass is None or radius is None:
        return None
    mass_f = float(mass)
    radius_f = float(radius)
    if mass_f <= 0 or radius_f <= 0:
        return None
    mass_scale = mass_f / BODY_MASS_KG
    return BomBodyPhysics(
        build=str(profile.get("bom_build") or build_id),
        mass_kg=mass_f,
        wheel_radius_m=radius_f,
        contact_radius_m=DRAWN_WHEEL_RADIUS_M,
        drawn_radius_m=DRAWN_WHEEL_RADIUS_M,
        body_inertia=tuple(v * mass_scale for v in BODY_INERTIA_DIAG),
        wheel_inertia=WHEEL_INERTIA_DIAG,
    )


def format_bom_body_log_line(build_id: str | None = None) -> str | None:
    """The physics line the Webots console prints, for the dashboard log.

    ``None`` when no BoM build is selected. Contact radius stays 0.08 m;
    the catalog diameter is named and not applied to the tire.
    """
    physics = bom_body_physics(build_id)
    if physics is None:
        return None
    return (
        f"BOM BODY build={physics.build} mass_kg={physics.mass_kg:g} "
        f"catalog_wheel_radius_m={physics.wheel_radius_m:g} "
        f"contact_radius_m={physics.contact_radius_m:g} "
        f"drawn_wheel_radius_m={physics.drawn_radius_m:g} "
        f"(contact cylinder stays {physics.contact_radius_m:g} m; "
        f"v=ω·r and the viscous bill use that, not the catalog diameter)"
    )


def apply_active_bom_body(text: str) -> str:
    """Patch robot VRML from the active ``bom_build``.

    Returns ``text`` itself when no build is selected, so a generated world
    stays byte-identical.
    """
    physics = bom_body_physics()
    if physics is None:
        return text
    return apply_bom_body_vrml(text, physics)


def apply_bom_body_vrml(text: str, physics: BomBodyPhysics) -> str:
    """Set body mass and body inertia. The contact cylinder stays 0.08 m."""
    return _patch_body_solid(text, physics)


def read_robot_body(text: str) -> RobotBody:
    """Read the robot solid and the two drive wheels back out of VRML."""
    start, end = _body_span(text)
    if start < 0:
        raise ValueError("robot physics block not found")
    block = text[start:end]
    mass = _MASS_LINE.search(block)
    inertia = _INERTIA_LINE.search(block)
    if mass is None or inertia is None:
        raise ValueError("robot mass or inertia not found")
    return RobotBody(
        mass_kg=float(mass.group(2)),
        inertia=(float(inertia.group(2)), float(inertia.group(3)), float(inertia.group(4))),
        left=_read_wheel(text, "LEFT_WHEEL_JOINT"),
        right=_read_wheel(text, "RIGHT_WHEEL_JOINT"),
    )


def body_matches(text: str, physics: BomBodyPhysics, tol: float = 1e-6) -> bool:
    """True when mass and inertia match and both contact wheels are still 0.08 m."""
    try:
        body = read_robot_body(text)
    except ValueError:
        return False

    def close(left: float, right: float) -> bool:
        return abs(left - right) <= tol

    if not close(body.mass_kg, physics.mass_kg):
        return False
    if any(not close(a, b) for a, b in zip(body.inertia, physics.body_inertia)):
        return False
    for wheel in (body.left, body.right):
        if not close(wheel.physics_radius_m, physics.contact_radius_m):
            return False
        if not close(wheel.anchor_z_m, physics.contact_radius_m):
            return False
        if not close(wheel.translation_z_m, physics.contact_radius_m):
            return False
        if not close(wheel.drawn_radius_m, physics.drawn_radius_m):
            return False
        if any(not close(a, b) for a, b in zip(wheel.inertia, physics.wheel_inertia)):
            return False
    return True


def shipped_world_path(path: Path) -> Path:
    """``butlerbot.bom-light.wbt`` → ``butlerbot.wbt``. Other names pass through."""
    match = _BOM_SUFFIX.match(path.stem)
    if not match:
        return path
    return path.with_name(match.group("stem") + path.suffix)


def patched_world_path(path: Path, build: str) -> Path:
    shipped = shipped_world_path(path)
    safe = re.sub(r"[^a-z0-9_-]+", "", str(build).strip().lower()) or "build"
    return shipped.with_name(f"{shipped.stem}.bom-{safe}{shipped.suffix}")


@dataclass(frozen=True)
class WorldPlan:
    """Where Webots should run, and whether an already-open world must reload."""

    path: str
    reload: bool


def plan_bom_world(world_path: str | Path) -> WorldPlan:
    """Decide which world file to run.

    No ``bom_build``: the given path, no write. A file that already has this
    build's mass (and the 0.08 m contact wheel): that path, no reload.
    Otherwise write ``<name>.bom-<build>.wbt`` beside the shipped world (the
    shipped file is not modified) and ask for a reload so a running
    controller picks up the new mass. The contact cylinder is not resized.
    """
    path = Path(world_path)
    physics = bom_body_physics()
    if physics is None or not path.is_file():
        return WorldPlan(str(path), False)
    current = path.read_text(encoding="utf-8")
    if body_matches(current, physics):
        return WorldPlan(str(path), False)
    shipped = shipped_world_path(path)
    source = shipped if shipped.is_file() else path
    patched = apply_bom_body_vrml(source.read_text(encoding="utf-8"), physics)
    dest = patched_world_path(path, physics.build)
    dest.write_text(patched, encoding="utf-8", newline="\n")
    return WorldPlan(str(dest), True)


def prepare_bom_world(world_path: str | Path) -> str:
    """Path from :func:`plan_bom_world`. Launchers open this file fresh."""
    return plan_bom_world(world_path).path


def install_command_radius(radius_m: float) -> None:
    """Point m/s ↔ rad/s conversions at ``radius_m``.

    The BoM body path passes ``contact_radius_m`` (0.08 m), the same radius
    as the contact cylinder and the viscous bill. Passing the catalog radius
    (0.04 m / 0.10 m) makes ground speed and shaft power disagree with the
    model. Lane geometry does not move.
    """
    import sys

    radius = float(radius_m)
    for mod_name in ("src.lane_keep", "src.rowfit_control", "src.teleop_agent"):
        module = sys.modules.get(mod_name)
        if module is None:
            try:
                module = __import__(mod_name, fromlist=["WHEEL_RADIUS_M"])
            except Exception:
                continue
        if hasattr(module, "WHEEL_RADIUS_M"):
            setattr(module, "WHEEL_RADIUS_M", radius)


def _patch_body_solid(text: str, physics: BomBodyPhysics) -> str:
    start, end = _body_span(text)
    if start < 0:
        return text
    block = text[start:end]
    comment = (
        f"    # BoM {physics.build}: body mass is mass_kg_known {_fmt(physics.mass_kg)} kg "
        f"(shipped placeholder was {_fmt(BODY_MASS_KG)} kg). "
        f"Catalog wheel {_fmt(physics.wheel_radius_m)} m; contact cylinder stays "
        f"{_fmt(physics.contact_radius_m)} m so odometry and the viscous bill match. "
        f"Body inertia scales with mass/{_fmt(BODY_MASS_KG)}. "
        f"Wheel solid mass stays {_fmt(WHEEL_SOLID_MASS_KG)} kg.\n"
    )
    if _COMMENT_BEFORE_MASS.search(block):
        block = _COMMENT_BEFORE_MASS.sub(comment, block, count=1)
    else:
        block = block.replace("    mass ", comment + "    mass ", 1)
    block = _MASS_LINE.sub(lambda m: m.group(1) + _fmt(physics.mass_kg), block, count=1)
    block = _set_inertia(block, physics.body_inertia)
    return text[:start] + block + text[end:]


def _set_inertia(block: str, diag: tuple[float, float, float]) -> str:
    def repl(match: re.Match[str]) -> str:
        return f"{match.group(1)}{_fmt(diag[0])} {_fmt(diag[1])} {_fmt(diag[2])}"

    return _INERTIA_LINE.sub(repl, block, count=1)


def _read_wheel(text: str, name: str) -> WheelBody:
    start, end = _joint_span(text, name)
    if start < 0:
        raise ValueError(f"{name} not found")
    block = text[start:end]
    drawn = _drawn_radius(block)
    physics = _bounding_radius(block)
    anchor = _ANCHOR_Z.search(block)
    translation = _TRANSLATION_Y.search(block)
    inertia = _INERTIA_LINE.search(block)
    if drawn is None or physics is None or anchor is None or translation is None or inertia is None:
        raise ValueError(f"{name} is missing radius, anchor, or inertia")
    return WheelBody(
        drawn_radius_m=drawn,
        physics_radius_m=physics,
        anchor_z_m=float(anchor.group(2)),
        translation_y_m=float(translation.group(1)),
        translation_z_m=float(translation.group(2)),
        inertia=(float(inertia.group(2)), float(inertia.group(3)), float(inertia.group(4))),
    )


def _drawn_radius(block: str) -> float | None:
    idx = block.find("boundingObject")
    head = block if idx < 0 else block[:idx]
    match = re.search(r"geometry Cylinder \{[^}]*?radius\s+([0-9.eE+-]+)", head, re.S)
    if match is None:
        return None
    return float(match.group(1))


def _bounding_radius(block: str) -> float | None:
    idx = block.find("boundingObject")
    if idx < 0:
        return None
    match = _RADIUS_LINE.search(block[idx:])
    if match is None:
        return None
    return float(match.group(2))


def _body_span(text: str) -> tuple[int, int]:
    start = text.find(_BODY_PHYSICS)
    if start < 0:
        return -1, -1
    end = text.find("\n  }", start)
    if end < 0:
        return -1, -1
    return start, end + len("\n  }")


def _joint_span(text: str, name: str) -> tuple[int, int]:
    token = f"DEF {name}"
    start = text.find(token)
    if start < 0:
        return -1, -1
    nxt = text.find("\n    DEF ", start + len(token))
    if nxt < 0:
        nxt = len(text)
    return start, nxt
