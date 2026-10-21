"""Physics-based forward speed cap from stopping distance and braking limits.

All lengths in metres (m), times in seconds (s), speeds in m/s,
accelerations in m/s², mass in kilograms (kg), torque in newton-metres (N·m).

The Webots contact wheel stays at 0.08 m (``src.webots_body.DRAWN_WHEEL_RADIUS_M``).
Braking force and motor torque use the catalog BoM wheel radius
(``geometry.wheel_radius_m``, 0.04 m for 80 mm tires).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.twin_energy import GRAVITY_M_S2

# Longitudinal peak friction on dry sealed concrete (not Webots ODE coulombFriction=8).
# See config.yaml ``controller.brake_mu`` and docs/PHYSICS_SPEED_LIMIT.md.
DEFAULT_BRAKE_MU = 0.8
# bom.json assumptions.turning_scrub: both drive wheels carry the weight.
DEFAULT_DRIVEN_WEIGHT_FRACTION = 1.0
DEFAULT_REACTION_LATENCY_S = 0.32  # s — one camera frame; matches obstacle gate default
DEFAULT_SAFETY_MARGIN = 0.8  # unitless factor on computed v_max
# Nadir top-row reach on repo poses (``lane_vision`` / ``aim_camera.py``).
DEFAULT_NADIR_TOP_ROW_X_M = 1.96


@dataclass(frozen=True)
class PhysicsSpeedLimitResult:
    """Computed cap and the deceleration that produced it."""

    v_max_m_s: float
    a_brake_m_s2: float
    a_friction_m_s2: float
    a_motor_m_s2: float
    total_mass_kg: float
    wheel_radius_m: float
    brake_mu: float
    driven_weight_fraction: float
    d_available_m: float
    reaction_latency_s: float
    safety_margin: float
    torque_limited: bool
    friction_limited: bool

    @property
    def speed_limit_active(self) -> bool:
        return self.v_max_m_s < float("inf")


def friction_limited_decel_m_s2(
    *,
    brake_mu: float,
    driven_weight_fraction: float,
) -> float:
    """Peak longitudinal deceleration from tyre friction (m/s²)."""
    mu = max(0.0, float(brake_mu))
    frac = max(0.0, min(1.0, float(driven_weight_fraction)))
    return mu * frac * GRAVITY_M_S2


def motor_limited_decel_m_s2(
    *,
    total_mass_kg: float,
    wheel_radius_m: float,
    brake_torque_nm_per_motor: float,
    motor_qty: float = 2.0,
) -> float:
    """Peak longitudinal deceleration from driven-wheel torque (m/s²)."""
    mass = max(1e-9, float(total_mass_kg))
    radius = max(1e-9, float(wheel_radius_m))
    torque = max(0.0, float(brake_torque_nm_per_motor))
    qty = max(0.0, float(motor_qty))
    force_n = qty * torque / radius
    return force_n / mass


def combine_brake_decel_m_s2(a_friction: float, a_motor: float) -> float:
    return min(max(0.0, float(a_friction)), max(0.0, float(a_motor)))


def braking_distance_m(
    v_m_s: float,
    *,
    a_brake_m_s2: float,
    reaction_latency_s: float,
    v_target_m_s: float = 0.0,
) -> float:
    """Path length to decelerate from ``v_m_s`` to ``v_target_m_s`` with latency (m).

    ``d = v·t + (v² − v_target²) / (2·a)`` (constant ``a_brake`` after reaction time).
    """
    v = max(0.0, float(v_m_s))
    vt = max(0.0, float(v_target_m_s))
    a = max(1e-6, float(a_brake_m_s2))
    t = max(0.0, float(reaction_latency_s))
    if v <= vt + 1e-9:
        return 0.0
    return v * t + (v * v - vt * vt) / (2.0 * a)


def max_speed_for_distance_m(
    d_available_m: float,
    *,
    a_brake_m_s2: float,
    reaction_latency_s: float,
    safety_margin: float = 1.0,
    v_target_m_s: float = 0.0,
) -> float:
    """Largest entry speed allowed ``d_available_m`` before the pivot (m/s).

    From ``d = v·t + (v² − v_target²) / (2·a)``:

    ``v = −a·t + sqrt(a²·t² + 2·a·d + v_target²)``, then × ``safety_margin``.
    """
    d = max(0.0, float(d_available_m))
    a = max(1e-6, float(a_brake_m_s2))
    t = max(0.0, float(reaction_latency_s))
    vt = max(0.0, float(v_target_m_s))
    margin = max(0.0, float(safety_margin))
    if d <= 0.0:
        return vt * margin
    under = a * a * t * t + 2.0 * a * d + vt * vt
    if under <= 0.0:
        return vt * margin
    v = -a * t + math.sqrt(under)
    return max(vt, v * margin)


def max_speed_from_stop_distance_m(
    *,
    d_available_m: float,
    a_brake_m_s2: float,
    reaction_latency_s: float,
    safety_margin: float = 1.0,
) -> float:
    """Largest speed that can stop within ``d_available_m`` including reaction travel.

    Uses the consistent quadratic from constant deceleration after latency::

        d = v * t_latency + v² / (2 * a)

    Solve for v (positive root)::

        v = -a * t + sqrt(a² * t² + 2 * a * d)

    Then multiply by ``safety_margin`` (0–1 typical).
    """
    return max_speed_for_distance_m(
        d_available_m,
        a_brake_m_s2=a_brake_m_s2,
        reaction_latency_s=reaction_latency_s,
        safety_margin=safety_margin,
        v_target_m_s=0.0,
    )


@dataclass(frozen=True)
class TurnBrakeParams:
    """Mass/payload-aware turn-approach braking (shared with cruise cap)."""

    a_brake_m_s2: float
    reaction_latency_s: float
    safety_margin: float
    approach_target_m_s: float  # m/s — speed at pivot entry (often V_CREEP)


DEFAULT_APPROACH_TARGET_M_S = 0.04  # matches rowfit V_CREEP_M_S

# Turn/pivot perception (nadir lane cameras — see docs/PHYSICS_SPEED_LIMIT.md).
# ``lane_vision.MAX_RANGE_M``: floor projection discard beyond 3.5 m.
# ``rowfit_control.lookahead_bounds_from_coverage``: ld_max = min(top_row_x_m) − 0.36 m.
NADIR_FLOOR_PROJECTION_MAX_M = 3.5
NADIR_LD_FAR_MARGIN_M = 0.36  # rowfit_control.LD_FAR_MARGIN_M


def nadir_lookahead_max_m(coverage_top_row_x_m: float) -> float:
    """Row-fit pursuit / turn planning horizon from nadir top-row reach (m)."""
    return max(0.0, float(coverage_top_row_x_m) - NADIR_LD_FAR_MARGIN_M)


def default_nadir_ld_max_m(config_data: dict | None = None) -> float:
    """``ld_max`` from config override or repo nadir coverage (~1.60 m)."""
    sl = _speed_limit_config(config_data)
    top = sl.get("nadir_top_row_x_m")
    top_x = DEFAULT_NADIR_TOP_ROW_X_M if top is None else float(top)
    return nadir_lookahead_max_m(top_x)


def obstacle_gate_detection_range_m(config_data: dict | None = None) -> float:
    """Stereo corridor depth and radar range used by the obstacle stack (m)."""
    sl = _speed_limit_config(config_data)
    raw = sl.get("obstacle_detection_range_m")
    if raw is not None:
        return max(0.0, float(raw))
    from src.radar_sense import DEFAULT_MAX_RANGE_M
    from src.stereo_depth import CorridorParams

    return min(float(CorridorParams().max_range_m), float(DEFAULT_MAX_RANGE_M))


def resolve_d_available_m(
    config_data: dict | None = None,
    *,
    obstacles_enabled: bool = True,
) -> float:
    """Stop budget for the global cruise cap: nadir look-ahead, optionally capped by obstacle range."""
    ld = default_nadir_ld_max_m(config_data)
    if not obstacles_enabled:
        return ld
    return min(ld, obstacle_gate_detection_range_m(config_data))


def _config_override_m(sl: dict, key: str) -> float | None:
    raw = sl.get(key)
    if raw is None:
        return None
    if isinstance(raw, str) and raw.strip().lower() in {"", "null", "none", "~"}:
        return None
    return float(raw)


def max_speed_for_obstacle_slow_zone_m(
    detection_range_m: float,
    *,
    a_brake_m_s2: float,
    reaction_latency_s: float,
    safety_margin: float = 1.0,
    standoff_m: float = 0.35,
    slow_extra_m: float = 1.0,
) -> float:
    """Largest speed whose slow zone (d_stop + slow_extra) fits in ``detection_range_m``."""
    from src.obstacle_gate import SLOW_EXTRA_M, STANDOFF_M, stop_distance_m

    det = max(0.0, float(detection_range_m))
    standoff = float(standoff_m if standoff_m is not None else STANDOFF_M)
    slow_extra = float(slow_extra_m if slow_extra_m is not None else SLOW_EXTRA_M)
    if det <= standoff + slow_extra + 1e-6:
        return 0.0
    margin = max(0.0, float(safety_margin))
    a = max(1e-6, float(a_brake_m_s2))
    t = max(0.0, float(reaction_latency_s))
    budget = det - slow_extra

    def slow_clearance(v: float) -> float:
        return stop_distance_m(v, a, t, standoff) + slow_extra

    if slow_clearance(0.0) > det + 1e-9:
        return 0.0
    lo, hi = 0.0, 20.0
    while slow_clearance(hi) <= det + 1e-9 and hi < 200.0:
        hi *= 2.0
    for _ in range(64):
        mid = (lo + hi) / 2.0
        if slow_clearance(mid) <= det + 1e-9:
            lo = mid
        else:
            hi = mid
    return lo * margin


def effective_turn_distance_m(
    path_remaining_m: float,
    *,
    perception_horizon_m: float,
) -> float:
    """Distance used for turn-approach braking: min(path, what nadir can see ahead)."""
    path = max(0.0, float(path_remaining_m))
    horizon = max(0.0, float(perception_horizon_m))
    return min(path, horizon)


def v_max_cruise_from_lookahead_m(
    d_lookahead_m: float,
    *,
    a_brake_m_s2: float,
    reaction_latency_s: float = DEFAULT_REACTION_LATENCY_S,
    safety_margin: float = DEFAULT_SAFETY_MARGIN,
    v_target_m_s: float = 0.0,
) -> float:
    """Max cruise speed if the only stop budget is look-ahead distance ``d`` (m/s)."""
    return max_speed_for_distance_m(
        d_lookahead_m,
        a_brake_m_s2=a_brake_m_s2,
        reaction_latency_s=reaction_latency_s,
        safety_margin=safety_margin,
        v_target_m_s=v_target_m_s,
    )


def turn_brake_params_from_physics(
    physics: PhysicsSpeedLimitResult,
    *,
    approach_target_m_s: float = DEFAULT_APPROACH_TARGET_M_S,
) -> TurnBrakeParams:
    return TurnBrakeParams(
        a_brake_m_s2=float(physics.a_brake_m_s2),
        reaction_latency_s=float(physics.reaction_latency_s),
        safety_margin=float(physics.safety_margin),
        approach_target_m_s=max(0.0, float(approach_target_m_s)),
    )


def turn_brake_params_for_robot(
    config_data: dict | None = None,
    *,
    bom_build: str | None = None,
) -> TurnBrakeParams:
    sl = _speed_limit_config(config_data)
    target = sl.get("approach_target_m_s")
    approach = DEFAULT_APPROACH_TARGET_M_S if target is None else float(target)
    physics = physics_speed_limit_for_active_robot(config_data, bom_build=bom_build)
    return turn_brake_params_from_physics(physics, approach_target_m_s=approach)


def compute_physics_speed_limit(
    *,
    total_mass_kg: float,
    wheel_radius_m: float,
    brake_torque_nm_per_motor: float,
    motor_qty: float = 2.0,
    brake_mu: float = DEFAULT_BRAKE_MU,
    driven_weight_fraction: float = DEFAULT_DRIVEN_WEIGHT_FRACTION,
    d_available_m: float,
    reaction_latency_s: float = DEFAULT_REACTION_LATENCY_S,
    safety_margin: float = DEFAULT_SAFETY_MARGIN,
    clamp_v_max_for_obstacle_perception: bool = False,
    obstacle_detection_range_m: float | None = None,
) -> PhysicsSpeedLimitResult:
    """Return v_max from braking physics (before comparing to config cruise)."""
    a_f = friction_limited_decel_m_s2(
        brake_mu=brake_mu, driven_weight_fraction=driven_weight_fraction
    )
    a_m = motor_limited_decel_m_s2(
        total_mass_kg=total_mass_kg,
        wheel_radius_m=wheel_radius_m,
        brake_torque_nm_per_motor=brake_torque_nm_per_motor,
        motor_qty=motor_qty,
    )
    a_b = combine_brake_decel_m_s2(a_f, a_m)
    v_max = max_speed_from_stop_distance_m(
        d_available_m=d_available_m,
        a_brake_m_s2=a_b,
        reaction_latency_s=reaction_latency_s,
        safety_margin=safety_margin,
    )
    if clamp_v_max_for_obstacle_perception and obstacle_detection_range_m is not None:
        v_obs = max_speed_for_obstacle_slow_zone_m(
            obstacle_detection_range_m,
            a_brake_m_s2=a_b,
            reaction_latency_s=reaction_latency_s,
            safety_margin=safety_margin,
        )
        v_max = min(v_max, v_obs)
    return PhysicsSpeedLimitResult(
        v_max_m_s=v_max,
        a_brake_m_s2=a_b,
        a_friction_m_s2=a_f,
        a_motor_m_s2=a_m,
        total_mass_kg=float(total_mass_kg),
        wheel_radius_m=float(wheel_radius_m),
        brake_mu=float(brake_mu),
        driven_weight_fraction=float(driven_weight_fraction),
        d_available_m=float(d_available_m),
        reaction_latency_s=float(reaction_latency_s),
        safety_margin=float(safety_margin),
        torque_limited=a_m + 1e-9 < a_f,
        friction_limited=a_f <= a_m + 1e-9,
    )


def _dig(data: dict, key: str):
    if "." not in key:
        return data.get(key)
    section, name = key.split(".", 1)
    block = data.get(section)
    if not isinstance(block, dict):
        return None
    return block.get(name)


def _speed_limit_config(data: dict | None) -> dict:
    data = data or {}
    block = data.get("controller")
    if not isinstance(block, dict):
        block = {}
    sl = block.get("speed_limit")
    if not isinstance(sl, dict):
        sl = {}
    return sl


def _brake_torque_from_profile(profile: dict) -> tuple[float, str]:
    """Torque per motor used for braking decel (N·m) and a short source tag."""
    motors = profile.get("motors") or {}
    left = motors.get("left_wheel") or {}
    if left.get("continuous_torque_nm") is not None:
        return float(left["continuous_torque_nm"]), "BoM rec_continuous_torque_nm"
    if left.get("stall_torque_nm") is not None:
        return float(left["stall_torque_nm"]), "profile stall_torque_nm"
    build = profile.get("bom_build")
    if build:
        try:
            from src.bom_profile import load_bom

            builds = (load_bom().get("builds") or {}).get(str(build).lower()) or {}
            head = builds.get("torque_headroom") or {}
            if head.get("limit_nm_per_motor") is not None:
                return float(head["limit_nm_per_motor"]), str(
                    head.get("limit_source") or "bom torque_headroom"
                )
        except Exception:
            pass
    return 0.981, "default 0.981 N·m (Pololu 10 kg·cm class)"


def physics_speed_limit_for_active_robot(
    config_data: dict | None = None,
    *,
    bom_build: str | None = None,
) -> PhysicsSpeedLimitResult:
    """Load profile + config and compute the physics v_max."""
    from src.bom_profile import rolling_mass_kg
    from src.hardware_profile import build_profile_from_bom, get_active_profile, wheel_radius_m

    if bom_build:
        profile = build_profile_from_bom(str(bom_build).strip())
    else:
        profile = get_active_profile()
    sl = _speed_limit_config(config_data)

    mass = rolling_mass_kg(profile)
    radius = wheel_radius_m(profile)
    torque, _src = _brake_torque_from_profile(profile)
    if sl.get("brake_torque_nm_per_motor") is not None:
        torque = float(sl["brake_torque_nm_per_motor"])

    left = (profile.get("motors") or {}).get("left_wheel") or {}
    motor_qty = float(left.get("qty") or 2.0)
    if motor_qty <= 0:
        motor_qty = 2.0

    def _f(key: str, default: float) -> float:
        raw = sl.get(key)
        if raw is None:
            return default
        return float(raw)

    d_override = _config_override_m(sl, "stop_available_m")
    obs_raw = sl.get("obstacles_in_d_available")
    obstacles_in = True if obs_raw is None else bool(obs_raw)
    if d_override is not None:
        d_available = d_override
        obstacles_in = False  # explicit distance budget wins
    else:
        d_available = resolve_d_available_m(config_data, obstacles_enabled=obstacles_in)

    det_range = obstacle_gate_detection_range_m(config_data) if obstacles_in else None

    return compute_physics_speed_limit(
        total_mass_kg=mass,
        wheel_radius_m=radius,
        brake_torque_nm_per_motor=torque,
        motor_qty=motor_qty,
        brake_mu=_f("brake_mu", DEFAULT_BRAKE_MU),
        driven_weight_fraction=_f("driven_weight_fraction", DEFAULT_DRIVEN_WEIGHT_FRACTION),
        d_available_m=d_available,
        reaction_latency_s=_f("reaction_latency_s", DEFAULT_REACTION_LATENCY_S),
        safety_margin=_f("safety_margin", DEFAULT_SAFETY_MARGIN),
        clamp_v_max_for_obstacle_perception=obstacles_in,
        obstacle_detection_range_m=det_range,
    )


def effective_cruise_m_s(config_cruise_m_s: float, v_max_m_s: float) -> float:
    """Apply the physics cap to the optimizer cruise preference."""
    return min(float(config_cruise_m_s), max(0.0, float(v_max_m_s)))


def effective_straight_m_s(config_straight_m_s: float, v_max_m_s: float) -> float:
    return min(float(config_straight_m_s), max(0.0, float(v_max_m_s)))
