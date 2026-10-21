"""Allowlisted controller knobs shared by Webots and the overnight optimizer.

The optimizer may write only these keys in ``config/config.yaml``. It must
not edit the energy model, the physics source, or ``config/bom/bom.json``.
BoM *selection* is the top-level ``bom_build`` key (light / heavy / budget).

Defaults match ``src/rowfit_control.py`` and ``lane_keep.DEFAULT_K_STEER``.
A missing section leaves the robot on those constants.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "config.yaml"

# Files the overnight loop is not allowed to open for writing.
PROTECTED_PATHS = (
    "src/twin_energy.py",
    "src/mission_energy_log.py",
    "src/twin/webots_power.py",
    "src/energy_predictor.py",
    "src/webots_body.py",
    "config/bom/bom.json",
    "webots/worlds/butlerbot_warehouse.wbt",
)


@dataclass(frozen=True)
class Knob:
    """One tunable config entry."""

    key: str
    kind: str  # "float" or "choice"
    default: float | str | None
    lo: float | None = None
    hi: float | None = None
    step: float | None = None
    choices: tuple[str, ...] = ()


KNOBS: dict[str, Knob] = {
    "bom_build": Knob(
        "bom_build",
        "choice",
        None,
        choices=("", "light", "heavy", "budget"),
    ),
    "controller.cruise_m_s": Knob(
        "controller.cruise_m_s", "float", 0.44, lo=0.30, hi=0.50, step=0.06
    ),
    "controller.straight_m_s": Knob(
        "controller.straight_m_s", "float", 0.50, lo=0.36, hi=0.56, step=0.02
    ),
    "controller.pivot_w_max_rad_s": Knob(
        "controller.pivot_w_max_rad_s", "float", 0.9, lo=0.40, hi=1.20, step=0.30
    ),
    "controller.k_steer": Knob(
        "controller.k_steer", "float", 2.0, lo=1.4, hi=3.0, step=0.5
    ),
}

# Straight cap stays above cruise, and not so far above it that the
# straightaway bump outruns the steer headroom the 0.50 m/s tune left.
MAX_STRAIGHT_GAP_M_S = 0.12

ALLOWLIST = tuple(KNOBS)

# The overnight loop may tune only these keys. ``bom_build`` stays fixed for
# the run (default light). ``straight_m_s`` is still read for the controller
# but is not tuned: warehouse laps top out at ``cruise_m_s``.
OPTIMIZER_TUNING_KEYS = (
    "controller.cruise_m_s",
    "controller.pivot_w_max_rad_s",
    "controller.k_steer",
)


class AllowlistError(ValueError):
    """A proposed change is outside the knobs the loop may touch."""


@dataclass(frozen=True)
class ControllerLimits:
    cruise_m_s: float
    straight_m_s: float
    pivot_w_max_rad_s: float
    k_steer: float
    bom_build: str | None
    warnings: tuple[str, ...] = ()
    # Physics cap (see docs/PHYSICS_SPEED_LIMIT.md). ``cruise_m_s`` stays the
    # optimizer preference; ``effective_*`` is what the controller runs.
    v_max_m_s: float | None = None
    effective_cruise_m_s: float | None = None
    effective_straight_m_s: float | None = None
    speed_limit_active: bool = False

    def as_params(self) -> dict[str, float | str | None]:
        return {
            "bom_build": self.bom_build or "",
            "controller.cruise_m_s": self.cruise_m_s,
            "controller.straight_m_s": self.straight_m_s,
            "controller.pivot_w_max_rad_s": self.pivot_w_max_rad_s,
            "controller.k_steer": self.k_steer,
        }

    @property
    def gate_top_speed_m_s(self) -> float:
        """Largest forward speed the rowfit + obstacle layers may target."""
        ec = self.effective_cruise_m_s if self.effective_cruise_m_s is not None else self.cruise_m_s
        es = self.effective_straight_m_s if self.effective_straight_m_s is not None else self.straight_m_s
        return max(float(ec), float(es))


def _blank(value) -> bool:
    if value is None:
        return True
    if isinstance(value, str) and value.strip().lower() in {"", "null", "none", "~"}:
        return True
    return False


def check_param(key: str, value) -> float | str | None:
    """Return a normalised value or raise ``AllowlistError``."""
    if key not in KNOBS:
        raise AllowlistError(
            f"{key!r} is not on the optimizer allowlist ({', '.join(ALLOWLIST)})"
        )
    knob = KNOBS[key]
    if knob.kind == "choice":
        text = "" if _blank(value) else str(value).strip()
        if text not in knob.choices:
            raise AllowlistError(
                f"{key} must be one of {', '.join(knob.choices) or '(blank)'}, not {value!r}"
            )
        return text or None
    try:
        number = round(float(value), 4)
    except (TypeError, ValueError) as exc:
        raise AllowlistError(f"{key} must be a number, not {value!r}") from exc
    if knob.lo is not None and number < float(knob.lo) - 1e-9:
        raise AllowlistError(f"{key}={number} is below {knob.lo}")
    if knob.hi is not None and number > float(knob.hi) + 1e-9:
        raise AllowlistError(f"{key}={number} is above {knob.hi}")
    return number


def check_set(params: dict) -> None:
    """Reject a combo whose straight cap sits under cruise or too far above it."""
    cruise = params.get("controller.cruise_m_s")
    straight = params.get("controller.straight_m_s")
    if cruise is None or straight is None:
        return
    cruise_f = float(cruise)
    straight_f = float(straight)
    if straight_f + 1e-9 < cruise_f:
        raise AllowlistError(
            f"straight_m_s {straight_f} is below cruise_m_s {cruise_f}"
        )
    if straight_f - cruise_f > MAX_STRAIGHT_GAP_M_S + 1e-9:
        raise AllowlistError(
            f"straight_m_s - cruise_m_s = {straight_f - cruise_f:.3f} "
            f"exceeds {MAX_STRAIGHT_GAP_M_S} m/s"
        )


def _dig(data: dict, key: str):
    if "." not in key:
        return data.get(key)
    section, name = key.split(".", 1)
    block = data.get(section)
    if not isinstance(block, dict):
        return None
    return block.get(name)


def limits_from_mapping(data: dict | None) -> ControllerLimits:
    """Resolve knobs from a loaded YAML mapping. Invalid keys fall back."""
    data = data or {}
    warnings: list[str] = []
    resolved: dict[str, float | str | None] = {}
    for key, knob in KNOBS.items():
        raw = _dig(data, key)
        if _blank(raw):
            resolved[key] = knob.default if knob.kind == "float" else None
            continue
        try:
            resolved[key] = check_param(key, raw)
        except AllowlistError as exc:
            warnings.append(str(exc))
            resolved[key] = knob.default if knob.kind == "float" else None
    try:
        check_set(resolved)
    except AllowlistError as exc:
        warnings.append(str(exc) + " — using speed defaults")
        resolved["controller.cruise_m_s"] = KNOBS["controller.cruise_m_s"].default
        resolved["controller.straight_m_s"] = KNOBS["controller.straight_m_s"].default
    bom = resolved.get("bom_build")
    limits = ControllerLimits(
        cruise_m_s=float(resolved["controller.cruise_m_s"]),
        straight_m_s=float(resolved["controller.straight_m_s"]),
        pivot_w_max_rad_s=float(resolved["controller.pivot_w_max_rad_s"]),
        k_steer=float(resolved["controller.k_steer"]),
        bom_build=None if _blank(bom) else str(bom),
        warnings=tuple(warnings),
    )
    return _apply_physics_speed_limit(limits, data)


def _apply_physics_speed_limit(limits: ControllerLimits, data: dict | None) -> ControllerLimits:
    try:
        from src.physics_speed_limit import (
            effective_cruise_m_s,
            effective_straight_m_s,
            physics_speed_limit_for_active_robot,
        )

        physics = physics_speed_limit_for_active_robot(data, bom_build=limits.bom_build)
        v_max = float(physics.v_max_m_s)
        eff_c = effective_cruise_m_s(limits.cruise_m_s, v_max)
        eff_s = effective_straight_m_s(limits.straight_m_s, v_max)
        active = v_max + 1e-6 < float(limits.cruise_m_s)
        return ControllerLimits(
            cruise_m_s=limits.cruise_m_s,
            straight_m_s=limits.straight_m_s,
            pivot_w_max_rad_s=limits.pivot_w_max_rad_s,
            k_steer=limits.k_steer,
            bom_build=limits.bom_build,
            warnings=limits.warnings,
            v_max_m_s=v_max,
            effective_cruise_m_s=eff_c,
            effective_straight_m_s=eff_s,
            speed_limit_active=active,
        )
    except Exception as exc:
        warn = limits.warnings + (f"physics speed limit skipped ({exc})",)
        return ControllerLimits(
            cruise_m_s=limits.cruise_m_s,
            straight_m_s=limits.straight_m_s,
            pivot_w_max_rad_s=limits.pivot_w_max_rad_s,
            k_steer=limits.k_steer,
            bom_build=limits.bom_build,
            warnings=warn,
            v_max_m_s=None,
            effective_cruise_m_s=limits.cruise_m_s,
            effective_straight_m_s=limits.straight_m_s,
            speed_limit_active=False,
        )


def load_controller_limits(path: Path | str | None = None) -> ControllerLimits:
    """Read ``config.yaml`` from disk (not the process-wide singleton)."""
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    if not config_path.is_file():
        return limits_from_mapping({})
    try:
        with open(config_path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except Exception as exc:
        limits = limits_from_mapping({})
        return ControllerLimits(
            cruise_m_s=limits.cruise_m_s,
            straight_m_s=limits.straight_m_s,
            pivot_w_max_rad_s=limits.pivot_w_max_rad_s,
            k_steer=limits.k_steer,
            bom_build=limits.bom_build,
            warnings=(f"could not read {config_path}: {exc}",),
        )
    if not isinstance(data, dict):
        return limits_from_mapping({})
    return limits_from_mapping(data)
