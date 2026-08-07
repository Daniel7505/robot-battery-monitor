"""
ButlerBot Webots controller — digital twin loop for the Robot Battery Monitor.

Runs inside the Webots process (``butlerbot.wbt``). Each simulation step:

1. Read sensors (GPS, IMU, wheel encoders) and keyboard / dashboard teleop.
2. Drive wheels (manual, API, or auto mission phases) with local + remote throttle.
3. Apply ABS-style stop when Space is pressed or the dashboard issues ``drive_stop``.
4. Estimate joint power, drain a virtual battery, and POST telemetry to the PMS
   via ``twin_publisher`` (``POST /api/twin/telemetry``).
5. Poll ``GET /api/twin/state`` for agent throttle, API drive cmds, ``stop_epoch``,
   and one-shot battery replenish.

Teleop keys (focus the 3D view first — see note below):

* **I / J / K / L** (or W/A/S/D, arrows) — drive / turn
* **Space** — ABS stop sequence (coast / hard-zero / yaw oppose)
* **R** — toggle auto mission phase loop (standby → transit → patrol → …)

Why click the **floor**, not the robot:
  Webots routes keyboard focus to the selected node. Selecting the robot body
  often steals focus from the world view so keys never reach this controller.
  Click empty floor (or the view background) so Keyboard events fire.

Residual spin / ABS design notes (read before changing brake code):

* GPS reports translation only — pure in-place yaw shows ~0 m/s while a hub
  still spins on camera. Stops therefore require **finite wheel encoders**,
  **dual-hub hard-zero**, and **IMU yaw-rate** quiescence — not GPS alone.
* ``setPosition(NaN)`` freewheels a hub; locks use only finite encoder angles.
* ``stop_epoch`` from the dashboard ensures a stop is not missed if wheel
  commands already read as zero on the next poll.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import traceback

from controller import Robot, Motor, PositionSensor, GPS, InertialUnit, Keyboard, Display

from twin_publisher import (
    battery_from_twin_state,
    build_payload,
    estimate_motor_power_w,
    fetch_twin_state,
    parse_controller_args,
    publish_telemetry,
    remote_throttle_factor,
    teleop_from_twin_state,
)

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def _load_teleop_agent():
    """Optionally load ``src/teleop_agent.py`` for shared brake/throttle math.

    Webots controllers are not always started with the repo on ``sys.path``,
    so we load by file path. Falls back to local helpers if import fails.
    """
    module_path = os.path.join(_PROJECT_ROOT, "src", "teleop_agent.py")
    if not os.path.isfile(module_path):
        return None
    spec = importlib.util.spec_from_file_location("teleop_agent", module_path)
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception:
        return None
    return module


_teleop = _load_teleop_agent()

# ---------------------------------------------------------------------------
# Constants — mission phases, motors, stop thresholds, key codes
# ---------------------------------------------------------------------------
# Mission phases — optional auto loop (press R to toggle)
PHASES = [
    {"name": "standby", "gait": "stand", "duration_s": 6, "drive_speed": 0.0, "turn_amp": 0.0, "arm_amp": 0.0, "torso_amp": 0.0},
    {"name": "drive_transit", "gait": "drive", "duration_s": 14, "drive_speed": 5.5, "turn_amp": 0.0, "arm_amp": 0.0, "torso_amp": 0.0},
    {"name": "patrol", "gait": "patrol", "duration_s": 12, "drive_speed": 3.8, "turn_amp": 0.28, "arm_amp": 0.0, "torso_amp": 0.0},
    {"name": "manipulate", "gait": "manipulate", "duration_s": 10, "drive_speed": 0.0, "turn_amp": 0.0, "arm_amp": 0.95, "torso_amp": 0.35},
    {"name": "return_idle", "gait": "stand", "duration_s": 10, "drive_speed": -4.5, "turn_amp": 0.0, "arm_amp": 0.0, "torso_amp": 0.0},
]

_NEUTRAL_TORSO = 0.0
_NEUTRAL_LEFT_ARM = 0.14
_NEUTRAL_RIGHT_ARM = -0.14

MOTOR_NAMES = ["left_wheel", "right_wheel", "torso_joint", "left_arm", "right_arm"]
SENSOR_NAMES = [
    "left_wheel_sensor", "right_wheel_sensor", "torso_sensor",
    "left_arm_sensor", "right_arm_sensor",
]

MAX_WHEEL_V = 10.0
MAX_JOINT_V = 1.8
WHEEL_RADIUS_M = 0.08
MAX_BRAKE_WHEEL_V = 12.0
MIN_BRAKE_WHEEL_V = 1.5
STOP_SPEED_M_S = 0.02
STOP_WHEEL_RAD_S = 0.08
BRAKE_COAST_PHASE_S = 0.15
MAX_BRAKE_DURATION_S = 12.0
# Residual body yaw (rad/s) that must die before we report idle — pure spin
# has GPS≈0 while the robot still rotates on camera.
STOP_YAW_RATE_RAD_S = 0.12
WHEEL_HOLD_VEL = 12.0  # position-mode approach rate for hub lock (not MAX_JOINT_V)

KEY_W = ord("W")
KEY_A = ord("A")
KEY_S = ord("S")
KEY_D = ord("D")
KEY_R = ord("R")
KEY_SPACE = ord(" ")
KEY_I = ord("I")
KEY_J = ord("J")
KEY_K = ord("K")
KEY_L = ord("L")

# Arrow keys (Webots keyboard constants)
try:
    KEY_UP = Keyboard.KEY_UP
    KEY_LEFT = Keyboard.KEY_LEFT
    KEY_DOWN = Keyboard.KEY_DOWN
    KEY_RIGHT = Keyboard.KEY_RIGHT
except AttributeError:
    KEY_UP, KEY_LEFT, KEY_DOWN, KEY_RIGHT = 315, 314, 317, 316

HUD_W = 320
HUD_H = 180


# ---------------------------------------------------------------------------
# Motion estimation & ABS brake helpers
# ---------------------------------------------------------------------------


class SpeedEstimator:
    """Finite-difference body speed from GPS samples.

    Returns translation speed only — yaw-in-place is invisible here and must
    be caught via wheel encoders / IMU in the stop path.
    """

    def __init__(self) -> None:
        self._prev_pos: list[float] | None = None

    def estimate_motion(self, gps: GPS, dt: float) -> tuple[float, float]:
        """Return (scalar speed m/s, signed forward speed along +X)."""
        try:
            pos = list(gps.getValues())
        except Exception:
            return 0.0, 0.0
        if self._prev_pos is not None and dt > 0:
            dx = pos[0] - self._prev_pos[0]
            dy = pos[1] - self._prev_pos[1]
            inv_dt = 1.0 / max(dt, 0.001)
            forward = dx * inv_dt
            speed = math.sqrt(dx * dx + dy * dy) * inv_dt
            self._prev_pos = pos
            if _teleop is not None:
                return _teleop.sanitize_motion(speed, forward)
            speed = min(speed, 0.85)
            forward = max(-0.85, min(0.85, forward))
            return round(speed, 3), round(forward, 3)
        self._prev_pos = pos
        return 0.0, 0.0

    def estimate(self, gps: GPS, dt: float) -> float:
        speed, _ = self.estimate_motion(gps, dt)
        return speed


def _wheel_rad_s(
    sensors: dict[str, PositionSensor],
    sensor_name: str,
    prev_pos: dict[str, float],
    dt: float,
) -> float:
    """Encoder angular rate (rad/s) via finite difference of position sensor."""
    sensor = sensors[sensor_name]
    pos = float(sensor.getValue())
    prev = prev_pos.get(sensor_name, pos)
    prev_pos[sensor_name] = pos
    if dt <= 0:
        return 0.0
    return (pos - prev) / dt


def _symmetric_brake_cmd(motion_sign: float, speed_m_s: float) -> float:
    """Latched-direction gentle brake — same cmd on both wheels.

    Direction is latched at ABS request time so GPS jitter cannot reverse
    the oppose command mid-stop.
    """
    if _teleop is not None:
        return _teleop.abs_brake_wheel_velocity_latched(motion_sign, speed_m_s)
    if motion_sign == 0.0 or speed_m_s < STOP_SPEED_M_S:
        return 0.0
    equiv = speed_m_s / WHEEL_RADIUS_M
    cap = MAX_BRAKE_WHEEL_V if speed_m_s >= 0.45 else 9.0
    mag = max(MIN_BRAKE_WHEEL_V, min(cap, equiv * 1.05))
    return -motion_sign * mag


class AbsBrakeController:
    """Coast-first ABS stop sequence for linear cruise and in-place spin.

    Sequence (simplified):

    1. ``request()`` latches motion sign and spin vs linear mode.
    2. Soft oppose while both hubs agree on forward cruise.
    3. Hard dual-hub zero once slow, spinning, or unequal.
    4. Complete only after **both** hubs and body yaw stay quiet for a hold.

    Spin/turn stops deliberately ignore GPS-only settle so residual circling ends.
    """

    def __init__(self) -> None:
        self.active = False
        self._prev_wheel_pos: dict[str, float] = {}
        self._elapsed_s = 0.0
        self._motion_sign = 0.0
        self._spin_mode = False
        self._calm_hold_s = 0.0

    def request(
        self,
        forward_m_s: float = 0.0,
        speed_m_s: float = 0.0,
        *,
        last_left_v: float = 0.0,
        last_right_v: float = 0.0,
        left_wv: float = 0.0,
        right_wv: float = 0.0,
    ) -> None:
        """Arm ABS using last drive cmd + current wheel rates (for spin detect)."""
        self.active = True
        self._elapsed_s = 0.0
        self._calm_hold_s = 0.0
        # Keep existing lock targets if any; do not clear mid-spin (that chases).
        if _teleop is not None:
            self._spin_mode = _teleop.is_spin_brake(
                last_left_v=last_left_v,
                last_right_v=last_right_v,
                left_wheel_rad_s=left_wv,
                right_wheel_rad_s=right_wv,
                speed_m_s=speed_m_s,
            )
            self._motion_sign = _teleop.latch_brake_motion_sign(
                forward_m_s,
                speed_m_s,
                last_left_v=last_left_v,
                last_right_v=last_right_v,
            )
        elif abs(forward_m_s) >= STOP_SPEED_M_S:
            self._spin_mode = False
            self._motion_sign = math.copysign(1.0, forward_m_s)
        elif speed_m_s >= STOP_SPEED_M_S:
            self._spin_mode = False
            self._motion_sign = 1.0
        else:
            self._spin_mode = abs(left_wv) > 0.12 and abs(right_wv) > 0.12 and left_wv * right_wv < 0
            self._motion_sign = 0.0
        # If wheels disagree strongly, always use spin halt (kills residual yaw)
        if abs(left_wv - right_wv) > 0.5 and left_wv * right_wv < 0:
            self._spin_mode = True
        mode = "spin-halt" if self._spin_mode else f"linear sign={self._motion_sign}"
        print(
            f"ABS brake request ({mode}) last_cmd L={last_left_v:.1f} R={last_right_v:.1f} "
            f"wv L={left_wv:.2f} R={right_wv:.2f} speed={speed_m_s:.3f}"
        )

    def clear(self) -> None:
        self.active = False
        self._elapsed_s = 0.0
        self._motion_sign = 0.0
        self._spin_mode = False
        self._calm_hold_s = 0.0

    def wheel_rad_s(self, sensors: dict[str, PositionSensor], sensor_name: str, dt: float) -> float:
        return _wheel_rad_s(sensors, sensor_name, self._prev_wheel_pos, dt)

    def _wheels_locked(self, left_wv: float, right_wv: float) -> bool:
        """Both hubs truly still — any single wheel spinning fails (user: left residual)."""
        return (
            abs(left_wv) < STOP_WHEEL_RAD_S
            and abs(right_wv) < STOP_WHEEL_RAD_S
            and abs(left_wv - right_wv) < 0.12
        )

    def apply(
        self,
        motors: dict[str, Motor],
        sensors: dict[str, PositionSensor] | None = None,
        *,
        speed_m_s: float,
        forward_m_s: float,
        left_wv: float,
        right_wv: float,
        dt: float,
        yaw_rate: float = 0.0,
    ) -> bool:
        """Apply ABS braking. Returns True when fully stopped.

        Must kill BOTH hub rates and residual body yaw. GPS speed is NOT sufficient
        (pure in-place spin reads ~0 on GPS while the left wheel still turns on camera).
        """
        if not self.active:
            return False

        self._elapsed_s += dt
        if not self._spin_mode and left_wv * right_wv < 0 and abs(left_wv - right_wv) > 0.45:
            self._spin_mode = True
            self._calm_hold_s = 0.0
            print("ABS → spin-halt (detected opposing wheels)")

        wheels_ok = self._wheels_locked(left_wv, right_wv)
        yaw_ok = abs(yaw_rate) < STOP_YAW_RATE_RAD_S
        if wheels_ok and yaw_ok:
            self._calm_hold_s += dt
        else:
            self._calm_hold_s = 0.0

        # Complete only when BOTH hubs quiet AND body yaw quiet for a hold
        if self._calm_hold_s >= 0.45:
            self.active = False
            _hard_zero_wheels(
                motors, sensors, left_wv=left_wv, right_wv=right_wv
            )
            print(
                f"ABS complete @ speed={speed_m_s:.3f} "
                f"wv L={left_wv:.2f} R={right_wv:.2f} yaw_rate={yaw_rate:.2f} "
                f"hold={self._calm_hold_s:.2f}s"
            )
            return True

        # Never force-complete while a hub or body yaw is still moving
        if self._elapsed_s >= MAX_BRAKE_DURATION_S and wheels_ok and yaw_ok:
            self.active = False
            _hard_zero_wheels(
                motors, sensors, left_wv=left_wv, right_wv=right_wv
            )
            print("ABS timeout complete (wheels+yaw quiet)")
            return True

        # Residual body yaw with quiet encoders: oppose yaw (left freewheel case)
        if abs(yaw_rate) >= STOP_YAW_RATE_RAD_S and abs(left_wv) < 0.5 and abs(right_wv) < 0.5:
            _oppose_body_yaw(motors, yaw_rate)
            return False

        # Spin / low speed / unequal hubs: hard-zero BOTH wheels every tick
        use_hard = (
            self._spin_mode
            or speed_m_s < 0.25
            or abs(left_wv - right_wv) > 0.35
            or abs(left_wv) > 0.5
            or abs(right_wv) > 0.5
            or self._elapsed_s >= 0.35
        )
        if use_hard:
            _hard_zero_wheels(
                motors, sensors, left_wv=left_wv, right_wv=right_wv
            )
            return False

        # Short linear soft-oppose only for agreed forward cruise
        if abs(forward_m_s) >= STOP_SPEED_M_S:
            self._motion_sign = math.copysign(1.0, forward_m_s)
        elif self._motion_sign == 0.0:
            self._motion_sign = 1.0
        cmd = _symmetric_brake_cmd(self._motion_sign, speed_m_s) * 0.7
        for wheel in ("left_wheel", "right_wheel"):
            motor = motors[wheel]
            _enable_full_wheel_torque(motor)
            motor.setPosition(float("inf"))
            motor.setVelocity(_clamp(cmd, MAX_BRAKE_WHEEL_V))
        return False


_DRIVE_KEY_CODES = frozenset({
    KEY_W, KEY_A, KEY_S, KEY_D,
    KEY_I, KEY_J, KEY_K, KEY_L,
    KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT,
})


class KeyTracker:
    """Track held keys from Webots keyboard events (press/release stream)."""

    def __init__(self) -> None:
        self._active: set[int] = set()

    def poll(self, keyboard: Keyboard) -> tuple[set[int], set[int]]:
        """Return (currently held keys, keys newly pressed this step)."""
        pressed: set[int] = set()
        key = keyboard.getKey()
        while key != -1:
            if key > 0:
                self._active.add(key)
                pressed.add(key)
            else:
                self._active.discard(-key)
            key = keyboard.getKey()
        return set(self._active), pressed

    def active_keys(self) -> set[int]:
        return set(self._active)

    def cancel_drive_keys(self) -> None:
        """Drop held drive keys — Space stop should not fight still-held I/J/K/L."""
        self._active -= _DRIVE_KEY_CODES


# ---------------------------------------------------------------------------
# Low-level motor / IMU utilities
# ---------------------------------------------------------------------------


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _safe_imu_roll(imu: InertialUnit | None) -> float:
    if imu is None:
        return 0.0
    try:
        return float(imu.getRollPitchYaw()[0])
    except Exception:
        return 0.0


# Fixed encoder targets for position-lock stop (must NOT chase the freewheel).
_WHEEL_LOCK_POS: dict[str, float] = {}


def _clear_wheel_locks() -> None:
    """Drop latched encoder holds so the next drive can free-spin the hubs."""
    _WHEEL_LOCK_POS.clear()


def _enable_full_wheel_torque(motor: Motor) -> None:
    """Raise torque/force caps so Stop can fight residual hub spin (both sides)."""
    for attr, value in (
        ("setAvailableTorque", 50.0),
        ("setAvailableForce", 50.0),
    ):
        if hasattr(motor, attr):
            try:
                getattr(motor, attr)(value)
            except Exception:
                pass


def _hard_zero_wheels(
    motors: dict[str, Motor],
    sensors: dict[str, PositionSensor] | None = None,
    *,
    left_wv: float | None = None,
    right_wv: float | None = None,
) -> None:
    """Zero BOTH hubs every tick — no freewheel, no single-wheel residual.

    Live bug: GPS speed≈0 + phase standby while left wheel still spins on camera.
    GPS only sees translation; pure yaw / one freewheeling hub is invisible to it.

    CRITICAL: never call setPosition(NaN). Sensor values are NaN before the first
    valid step — that invalid position leaves a hub freewheeling (user left spin).

    While a hub is still spinning fast: velocity-0 + full torque only.
    Once slow + finite encoder: hold fixed encoder angle.
    """
    rates = {"left_wheel": left_wv, "right_wheel": right_wv}
    for wheel in ("left_wheel", "right_wheel"):
        motor = motors[wheel]
        _enable_full_wheel_torque(motor)
        wv = rates.get(wheel)
        spinning = wv is not None and abs(wv) >= 0.25
        # Always assert velocity zero in velocity mode first (safe even if sensors NaN)
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        if spinning or sensors is None:
            continue
        sensor = sensors.get(f"{wheel}_sensor")
        if sensor is None:
            continue
        try:
            if wheel not in _WHEEL_LOCK_POS:
                pos = float(sensor.getValue())
                if not math.isfinite(pos):
                    continue  # keep velocity-0 only until encoder is valid
                _WHEEL_LOCK_POS[wheel] = pos
            hold = _WHEEL_LOCK_POS.get(wheel)
            if hold is None or not math.isfinite(hold):
                continue
            motor.setPosition(hold)
            motor.setVelocity(WHEEL_HOLD_VEL)
        except Exception:
            pass


def _halt_wheels(
    motors: dict[str, Motor],
    sensors: dict[str, PositionSensor] | None = None,
    *,
    lock: bool = False,
    left_wv: float | None = None,
    right_wv: float | None = None,
) -> None:
    """Stop drive wheels — both always. lock=True uses hard dual-hub freeze."""
    if lock:
        _hard_zero_wheels(motors, sensors, left_wv=left_wv, right_wv=right_wv)
        return
    for wheel in ("left_wheel", "right_wheel"):
        motor = motors[wheel]
        _enable_full_wheel_torque(motor)
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)


def _imu_yaw(imu: InertialUnit | None) -> float:
    if imu is None:
        return 0.0
    try:
        return float(imu.getRollPitchYaw()[2])
    except Exception:
        return 0.0


def _yaw_rate(imu: InertialUnit | None, prev_yaw: float, dt: float) -> tuple[float, float]:
    """Return (yaw_rate_rad_s, yaw_now) with pi wrapping."""
    yaw = _imu_yaw(imu)
    if dt <= 1e-6:
        return 0.0, yaw
    dy = yaw - prev_yaw
    # wrap to [-pi, pi]
    while dy > math.pi:
        dy -= 2.0 * math.pi
    while dy < -math.pi:
        dy += 2.0 * math.pi
    return dy / dt, yaw


def _oppose_body_yaw(
    motors: dict[str, Motor],
    yaw_rate: float,
    *,
    gain: float = 2.5,
) -> None:
    """Brief differential command opposing residual body spin (GPS may be ~0)."""
    if abs(yaw_rate) < STOP_YAW_RATE_RAD_S:
        return
    # Positive yaw_rate ≈ CCW: oppose with left+, right- (and reverse for CW)
    mag = min(5.0, abs(yaw_rate) * gain)
    sign = 1.0 if yaw_rate > 0.0 else -1.0
    left_cmd = sign * mag
    right_cmd = -sign * mag
    for wheel, cmd in (("left_wheel", left_cmd), ("right_wheel", right_cmd)):
        motor = motors[wheel]
        _enable_full_wheel_torque(motor)
        motor.setPosition(float("inf"))
        motor.setVelocity(_clamp(cmd, MAX_BRAKE_WHEEL_V))


def _set_drive(motors: dict[str, Motor], left_v: float, right_v: float, throttle: float) -> None:
    """Velocity-mode wheel command with agent throttle scale (0–1)."""
    scale = max(0.0, min(1.0, throttle))
    for side, cmd in (("left_wheel", left_v * scale), ("right_wheel", right_v * scale)):
        motor = motors[side]
        _enable_full_wheel_torque(motor)
        motor.setPosition(float("inf"))
        motor.setVelocity(_clamp(cmd, MAX_WHEEL_V))


def _apply_wheel_command(
    motors: dict[str, Motor],
    sensors: dict[str, PositionSensor],
    left_v: float,
    right_v: float,
    throttle: float,
) -> None:
    """Drive or hard-stop when the throttled command is essentially zero."""
    scale = max(0.0, min(1.0, throttle))
    left_cmd = _clamp(left_v * scale, MAX_WHEEL_V)
    right_cmd = _clamp(right_v * scale, MAX_WHEEL_V)
    if abs(left_cmd) < 0.01 and abs(right_cmd) < 0.01:
        _halt_wheels(motors, sensors, lock=True)
    else:
        _set_drive(motors, left_v, right_v, throttle)


def _hold_neutral_upper_body(motors: dict[str, Motor]) -> None:
    for name, pos in (
        ("torso_joint", _NEUTRAL_TORSO),
        ("left_arm", _NEUTRAL_LEFT_ARM),
        ("right_arm", _NEUTRAL_RIGHT_ARM),
    ):
        motor = motors[name]
        motor.setPosition(_clamp(pos, MAX_JOINT_V))
        motor.setVelocity(0.0)


def _set_joint_position(motors: dict[str, Motor], name: str, position: float) -> None:
    motors[name].setPosition(_clamp(position, MAX_JOINT_V))


def _apply_phase_motion(
    motors: dict[str, Motor],
    *,
    gait: str,
    phase: dict,
    t: float,
    throttle: float,
) -> tuple[float, float]:
    """Execute one auto-mission phase pose/drive; return wheel cmds (L, R)."""
    drive = phase["drive_speed"]
    turn = phase["turn_amp"]
    arm_amp = phase["arm_amp"]
    torso_amp = phase["torso_amp"]

    if gait in ("drive", "walk"):
        left_v = right_v = drive
        _set_drive(motors, left_v, right_v, throttle)
        _hold_neutral_upper_body(motors)
        return left_v, right_v

    if gait == "patrol":
        turn_cmd = turn * math.sin(t * 0.22)
        left_v = drive - turn_cmd
        right_v = drive + turn_cmd
        _set_drive(motors, left_v, right_v, throttle)
        _hold_neutral_upper_body(motors)
        return left_v, right_v

    if gait == "manipulate":
        _set_drive(motors, 0.0, 0.0, throttle)
        _set_joint_position(motors, "torso_joint", torso_amp * math.sin(t * 0.55))
        _set_joint_position(motors, "left_arm", arm_amp * math.sin(t * 1.15))
        _set_joint_position(motors, "right_arm", arm_amp * math.sin(t * 1.15 + math.pi * 0.85))
        return 0.0, 0.0

    if abs(drive) > 0.05:
        left_v = right_v = drive
        _set_drive(motors, left_v, right_v, throttle)
    else:
        left_v = right_v = 0.0
        _set_drive(motors, 0.0, 0.0, throttle)
    _hold_neutral_upper_body(motors)
    return left_v, right_v


# ---------------------------------------------------------------------------
# Teleop key mapping & local power agent
# ---------------------------------------------------------------------------


def _expand_teleop_keys(keys: set[int]) -> set[int]:
    """Map IJKL / arrows onto WASD semantics for a single drive helper."""
    expanded = set(keys)
    if _teleop is not None:
        expanded = _teleop.normalize_key_set(keys)
    else:
        expanded = {k - 32 if ord("a") <= k <= ord("z") else k for k in keys}
    if KEY_UP in keys:
        expanded.add(KEY_W)
    if KEY_DOWN in keys:
        expanded.add(KEY_S)
    if KEY_LEFT in keys:
        expanded.add(KEY_A)
    if KEY_RIGHT in keys:
        expanded.add(KEY_D)
    if KEY_I in expanded:
        expanded.add(KEY_W)
    if KEY_K in expanded:
        expanded.add(KEY_S)
    if KEY_J in expanded:
        expanded.add(KEY_A)
    if KEY_L in expanded:
        expanded.add(KEY_D)
    return expanded


def _teleop_drive(keys: set[int]) -> tuple[float, float]:
    """Keyboard → (left_wheel_v, right_wheel_v) in rad/s."""
    expanded = _expand_teleop_keys(keys)
    if _teleop is not None:
        return _teleop.drive_from_key_set(
            expanded, key_w=KEY_W, key_a=KEY_A, key_s=KEY_S, key_d=KEY_D
        )
    left = right = 0.0
    if KEY_W in expanded:
        left += 5.5
        right += 5.5
    if KEY_S in expanded:
        left -= 4.2
        right -= 4.2
    if KEY_A in expanded:
        left -= 2.6
        right += 2.6
    if KEY_D in expanded:
        left += 2.6
        right -= 2.6
    return left, right


def _local_throttle(battery_pct: float, thermal_c: float) -> tuple[float, str | None]:
    """Onboard cap from battery/heat (before merging dashboard intervention)."""
    if _teleop is not None:
        return _teleop.local_agent_throttle(battery_pct, thermal_c)
    return 1.0, None


def _update_thermal(thermal_c: float, draw_w: float, dt: float, motion_factor: float) -> float:
    if _teleop is not None:
        return _teleop.update_thermal_c(thermal_c, draw_w, dt, motion_factor=motion_factor)
    return thermal_c


def _merge_throttle(local: float, remote: float | None) -> float:
    """Take the stricter of local HUD agent and dashboard intervention factor."""
    if _teleop is not None:
        return _teleop.merge_throttle(local, remote)
    if remote is None:
        return local
    return min(local, float(remote))


# ---------------------------------------------------------------------------
# On-screen HUD (Webots Display device)
# ---------------------------------------------------------------------------


def _draw_gauge(display: Display, x: int, y: int, w: int, h: int, ratio: float, color: int, label: str) -> None:
    display.setColor(0x1A2230)
    display.fillRectangle(x, y, w, h)
    display.setColor(0x3A4A5A)
    display.drawRectangle(x, y, w, h)
    fill_w = max(0, int((w - 4) * ratio))
    if fill_w > 0:
        display.setColor(color)
        display.fillRectangle(x + 2, y + 2, fill_w, h - 4)
    display.setColor(0xCCDDEE)
    display.drawText(label, x + 6, y + h - 18)


def _draw_hud(
    display: Display,
    *,
    battery_pct: float,
    thermal_c: float,
    throttle: float,
    message: str | None,
    teleop_active: bool,
    auto_loop: bool,
    api_source: str = "",
    speed_m_s: float = 0.0,
    braking: bool = False,
) -> None:
    """Paint battery/heat gauges and teleop mode strip on the robot HUD."""
    display.setAlpha(0.92)
    display.setColor(0x080C12)
    display.fillRectangle(0, 0, HUD_W, HUD_H)

    batt_ratio = battery_pct / 100.0
    if _teleop is not None:
        batt_ratio = _teleop.battery_gauge_ratio(battery_pct)
        heat_ratio = _teleop.thermal_gauge_ratio(thermal_c)
        batt_color = _teleop.gauge_color_hex(1.0 - batt_ratio)
        heat_color = _teleop.gauge_color_hex(heat_ratio)
    else:
        heat_ratio = max(0.0, min(1.0, (thermal_c - 22.0) / 46.0))
        batt_color = 0x33DD66 if batt_ratio > 0.2 else 0xFF4444
        heat_color = 0x33DD66 if heat_ratio < 0.55 else 0xFFAA22

    _draw_gauge(display, 12, 18, 130, 22, batt_ratio, batt_color, f"BATT {battery_pct:.0f}%")
    _draw_gauge(display, 12, 52, 130, 22, heat_ratio, heat_color, f"HEAT {thermal_c:.0f}C")

    display.setColor(0x8899AA)
    if api_source:
        mode = f"API:{api_source[:8]}"
    else:
        mode = "TELEOP" if teleop_active else ("AUTO LOOP" if auto_loop else "STANDBY")
    display.drawText(mode, 12, 88)
    display.drawText(f"Agent cap {throttle * 100:.0f}%", 12, 106)
    speed_kmh = speed_m_s * 3.6
    speed_mph = speed_m_s * 2.237
    display.setColor(0x66EEFF if not braking else 0xFFAA44)
    display.drawText(f"{speed_m_s:.2f} m/s", 168, 22)
    display.setColor(0xAABBCC)
    display.drawText(f"{speed_kmh:.1f} km/h", 168, 42)
    display.drawText(f"{speed_mph:.1f} mph", 168, 58)
    if braking:
        display.setColor(0xFF8844)
        display.drawText("BRAKING", 168, 78)
    display.setColor(0x8899AA)
    display.drawText("I/J/K/L drive · Space stop", 12, 124)

    if message:
        display.setColor(0x3A1808)
        display.fillRectangle(0, HUD_H - 44, HUD_W, 44)
        display.setColor(0xFFAA33)
        display.drawRectangle(0, HUD_H - 44, HUD_W, 44)
        display.setColor(0xFFEECC)
        display.drawText(message[:42], 8, HUD_H - 28)


def _init_devices(
    robot: Robot, timestep: int
) -> tuple[dict[str, Motor], dict[str, PositionSensor], GPS, InertialUnit, Keyboard, Display | None]:
    """Enable motors, encoders, GPS, IMU, keyboard, and optional HUD display."""
    motors: dict[str, Motor] = {}
    sensors: dict[str, PositionSensor] = {}
    for name in MOTOR_NAMES:
        motor = robot.getDevice(name)
        motors[name] = motor
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        if hasattr(motor, "enableTorqueFeedback"):
            try:
                motor.enableTorqueFeedback(timestep)
            except Exception:
                pass
    for name in SENSOR_NAMES:
        sensor = robot.getDevice(name)
        sensors[name] = sensor
        sensor.enable(timestep)

    gps: GPS = robot.getDevice("gps")
    gps.enable(timestep)
    imu: InertialUnit = robot.getDevice("imu")
    imu.enable(timestep)

    keyboard: Keyboard | None = None
    if hasattr(robot, "getKeyboard"):
        keyboard = robot.getKeyboard()
    else:
        try:
            keyboard = robot.getDevice("keyboard")
        except Exception:
            keyboard = None
    if keyboard is not None:
        keyboard.enable(timestep)
    else:
        print("WARNING: Keyboard device unavailable — WASD teleop disabled")

    hud: Display | None = None
    try:
        hud = robot.getDevice("hud")
    except Exception:
        hud = None

    return motors, sensors, gps, imu, keyboard, hud


def _format_pose(gps: GPS) -> str:
    try:
        pos = gps.getValues()
        return f"({pos[0]:.2f}, {pos[1]:.2f})"
    except Exception:
        return "(?, ?)"


def _run_loop(robot: Robot, opts: dict) -> None:
    """Main digital-twin control loop (one Webots step per iteration).

    Priority each step: ABS stop → dashboard API drive → keyboard teleop →
    auto mission phases → residual spin kill / hard-zero idle.

    Also: thermal + battery models, HUD paint, and throttled twin publish.
    """
    timestep = int(robot.getBasicTimeStep())
    publish_every = max(1, int(opts["interval_s"] * 1000 / timestep))
    # Poll teleop/stop often — 1s latency made dashboard Stop feel like a slow fade.
    state_poll_every = max(3, min(publish_every, int(0.1 * 1000 / timestep)))
    motors, sensors, gps, imu, keyboard, hud = _init_devices(robot, timestep)
    speed_estimator = SpeedEstimator()
    key_tracker = KeyTracker()
    abs_brake = AbsBrakeController()

    phase_idx = 0
    phase_elapsed = 0.0
    dashboard = opts["dashboard_url"]
    startup_state = fetch_twin_state(dashboard)
    battery_pct = battery_from_twin_state(startup_state, default=100.0)
    thermal_c = 22.0
    tick = 0
    publish_fail_streak = 0
    auto_loop = False
    remote_throttle: float | None = None
    agent_message: str | None = None
    throttle_factor = 1.0
    keys_logged = False
    cached_api_left = 0.0
    cached_api_right = 0.0
    cached_api_source = ""
    last_api_sig = ""
    last_stop_epoch = 0.0  # last seen bridge stop_epoch (monotonic stop signal)
    drive_log_elapsed = 0.0
    last_teleop_left = 0.0
    last_teleop_right = 0.0
    prev_yaw = _imu_yaw(imu)
    yaw_rate = 0.0

    print(f"ButlerBot controller started — twin → {dashboard}/api/twin/telemetry")
    print(f"Battery synced from dashboard: {battery_pct:.1f}%")
    print("Teleop: Arrow keys or I/J/K/L — Space = stop. Click the FLOOR (not the robot)")
    print("Or use Dashboard: Drive Forward (API) under the twin panel")
    # Velocity-zero only at init — sensors are NaN until after the first steps
    for _w in ("left_wheel", "right_wheel"):
        _enable_full_wheel_torque(motors[_w])
        motors[_w].setPosition(float("inf"))
        motors[_w].setVelocity(0.0)

    while robot.step(timestep) != -1:
        try:
            tick += 1
            dt = timestep / 1000.0
            # --- Input: keyboard + R (auto loop) / Space (ABS) ---
            if keyboard is not None:
                keys, pressed = key_tracker.poll(keyboard)
            else:
                keys, pressed = set(), set()

            if KEY_R in pressed:
                auto_loop = not auto_loop
                print(f"Auto mission loop: {'ON' if auto_loop else 'OFF'}")

            # --- Sense: GPS translation, encoder rates, body yaw rate ---
            speed_m_s, forward_m_s = speed_estimator.estimate_motion(gps, dt)
            left_wv_early = abs_brake.wheel_rad_s(sensors, "left_wheel_sensor", dt)
            right_wv_early = abs_brake.wheel_rad_s(sensors, "right_wheel_sensor", dt)
            yaw_rate, prev_yaw = _yaw_rate(imu, prev_yaw, dt)

            hubs_locked = (
                abs(left_wv_early) < STOP_WHEEL_RAD_S
                and abs(right_wv_early) < STOP_WHEEL_RAD_S
                and abs(left_wv_early - right_wv_early) < 0.12
            )
            yaw_quiet = abs(yaw_rate) < STOP_YAW_RATE_RAD_S
            no_drive_cmd = (
                abs(last_teleop_left) < 0.05
                and abs(last_teleop_right) < 0.05
                and abs(cached_api_left) < 0.05
                and abs(cached_api_right) < 0.05
            )
            # ONLY zero GPS for mission/power when hubs AND body yaw are quiet.
            # Never mask a left-wheel / in-place spin as "idle" (camera can still spin).
            if hubs_locked and yaw_quiet and no_drive_cmd and not abs_brake.active:
                speed_m_s = 0.0
                forward_m_s = 0.0

            settled = (
                hubs_locked
                and yaw_quiet
                and (
                    _teleop.motion_settled(speed_m_s, left_wv_early, right_wv_early)
                    if _teleop is not None
                    else speed_m_s < 0.08
                )
            )

            if KEY_SPACE in pressed:
                abs_brake.request(
                    forward_m_s,
                    speed_m_s,
                    last_left_v=last_teleop_left,
                    last_right_v=last_teleop_right,
                    left_wv=left_wv_early,
                    right_wv=right_wv_early,
                )
                # Keep last_teleop_* until brake completes so spin mode stays correct
                key_tracker.cancel_drive_keys()
                keys = key_tracker.active_keys()
            if abs_brake.active:
                key_tracker.cancel_drive_keys()
                keys = key_tracker.active_keys()
            teleop_keys = keys - {KEY_R, KEY_SPACE}
            stop_pressed = KEY_SPACE in pressed
            if teleop_keys and not settled and not abs_brake.active:
                # New drive while still moving — re-latch ABS using prior cmd
                abs_brake.request(
                    forward_m_s,
                    speed_m_s,
                    last_left_v=last_teleop_left,
                    last_right_v=last_teleop_right,
                    left_wv=left_wv_early,
                    right_wv=right_wv_early,
                )
                key_tracker.cancel_drive_keys()
                teleop_keys = set()
            user_driving = bool(teleop_keys) and not abs_brake.active

            phase = PHASES[phase_idx]
            phase_name = phase["name"]
            gait = phase["gait"]

            if not user_driving and auto_loop:
                phase_elapsed += dt
                if phase_elapsed >= phase["duration_s"]:
                    phase_idx = (phase_idx + 1) % len(PHASES)
                    phase_elapsed = 0.0
                    phase = PHASES[phase_idx]
                    gait = phase["gait"]
                    phase_name = phase["name"]
                    print(f"Phase → {phase_name} ({gait}) @ {_format_pose(gps)}")

            t = phase_elapsed

            if not keys_logged and keys:
                keys_logged = True
                print(f"Keyboard active — keys detected (sample code {next(iter(keys))})")

            # --- Poll dashboard: throttle, API drive, stop_epoch, battery_reset ---
            if tick % state_poll_every == 0:
                twin_state = fetch_twin_state(dashboard)
                remote_throttle = remote_throttle_factor(twin_state)
                api_cmd = teleop_from_twin_state(twin_state)
                if api_cmd.get("battery_pct") is not None:
                    battery_pct = float(api_cmd["battery_pct"])
                    if api_cmd.get("reset_thermal"):
                        thermal_c = 22.0
                    print(f"Battery replenished → {battery_pct:.0f}% (dashboard command)")
                if api_cmd.get("active"):
                    # New external drive cancels any incomplete stop
                    if abs_brake.active:
                        abs_brake.clear()
                    _clear_wheel_locks()
                    cached_api_left = float(api_cmd.get("left_v") or 0.0)
                    cached_api_right = float(api_cmd.get("right_v") or 0.0)
                    cached_api_source = str(api_cmd.get("source") or "api")
                    # Track last command for stop/spin detection (API used to skip this)
                    last_teleop_left = cached_api_left
                    last_teleop_right = cached_api_right
                    sig = f"{cached_api_left}:{cached_api_right}:{cached_api_source}"
                    if sig != last_api_sig:
                        last_api_sig = sig
                        print(
                            f"External drive from {cached_api_source}: "
                            f"L={cached_api_left} R={cached_api_right}"
                        )
                else:
                    api_source = str(api_cmd.get("source") or "")
                    stop_epoch = float(api_cmd.get("stop_epoch") or 0.0)
                    was_api_driving = (
                        abs(cached_api_left) > 0.01 or abs(cached_api_right) > 0.01
                    )
                    need_abs = False
                    if stop_epoch > last_stop_epoch:
                        last_stop_epoch = stop_epoch
                        need_abs = True
                        print("External drive stop — ABS braking")
                    elif api_source == "stop" and cached_api_source != "stop":
                        need_abs = True
                        print("External drive stop — ABS braking")
                    elif was_api_driving and not abs_brake.active:
                        # duration_s expired: bridge zeros cmd without stop_epoch —
                        # still ABS so residual yaw/coast is killed.
                        need_abs = True
                        print("External drive expired — ABS braking")
                    if need_abs and not abs_brake.active:
                        abs_brake.request(
                            forward_m_s,
                            speed_m_s,
                            last_left_v=last_teleop_left,
                            last_right_v=last_teleop_right,
                            left_wv=left_wv_early,
                            right_wv=right_wv_early,
                        )
                    cached_api_left = 0.0
                    cached_api_right = 0.0
                    cached_api_source = api_source
                    if api_source != "stop":
                        last_api_sig = ""

            if stop_pressed:
                print("Keyboard stop — ABS braking")

            # Do not clear ABS while a stop is in progress

            left_wv = left_wv_early
            right_wv = right_wv_early
            wheel_vels = {"left_wheel": left_wv, "right_wheel": right_wv}

            # --- Actuate: ABS > API drive > keyboard > auto phases > residual kill ---
            api_driving = abs(cached_api_left) > 0.01 or abs(cached_api_right) > 0.01
            # Never clear ABS just because api_driving was true last frame
            if api_driving and not abs_brake.active:
                pass  # will drive below
            if abs_brake.active:
                done = abs_brake.apply(
                    motors,
                    sensors,
                    speed_m_s=speed_m_s,
                    forward_m_s=forward_m_s,
                    left_wv=left_wv,
                    right_wv=right_wv,
                    dt=dt,
                    yaw_rate=yaw_rate,
                )
                left_v = right_v = 0.0
                user_driving = False
                gait = "stand"
                phase_name = "standby"
                _hold_neutral_upper_body(motors)
                if done:
                    last_teleop_left = 0.0
                    last_teleop_right = 0.0
                    cached_api_left = 0.0
                    cached_api_right = 0.0
                    _hard_zero_wheels(
                        motors, sensors, left_wv=left_wv, right_wv=right_wv
                    )
            elif api_driving:
                user_driving = True
                left_v, right_v = cached_api_left, cached_api_right
                last_teleop_left, last_teleop_right = left_v, right_v
                _clear_wheel_locks()
                _set_drive(motors, left_v, right_v, 1.0)
                _hold_neutral_upper_body(motors)
            elif user_driving:
                left_v, right_v = _teleop_drive(teleop_keys)
                last_teleop_left, last_teleop_right = left_v, right_v
                _clear_wheel_locks()
                _set_drive(motors, left_v, right_v, 1.0)
                _hold_neutral_upper_body(motors)
            elif auto_loop:
                left_v, right_v = _apply_phase_motion(
                    motors, gait=gait, phase=phase, t=t, throttle=1.0
                )
            else:
                left_v = right_v = 0.0
                residual_hub = abs(left_wv) > 0.10 or abs(right_wv) > 0.10
                residual_yaw = abs(yaw_rate) >= STOP_YAW_RATE_RAD_S
                if residual_hub or residual_yaw:
                    # Still spinning on camera while GPS≈0 — kill it, do not report idle
                    if residual_yaw and not residual_hub:
                        _oppose_body_yaw(motors, yaw_rate)
                    else:
                        _hard_zero_wheels(
                            motors, sensors, left_wv=left_wv, right_wv=right_wv
                        )
                    if not abs_brake.active:
                        print(
                            f"Residual spin — re-ABS L={left_wv:.2f} R={right_wv:.2f} "
                            f"yaw_rate={yaw_rate:.2f}"
                        )
                        abs_brake.request(
                            forward_m_s,
                            speed_m_s,
                            last_left_v=last_teleop_left,
                            last_right_v=last_teleop_right,
                            left_wv=left_wv,
                            right_wv=right_wv,
                        )
                else:
                    _hard_zero_wheels(
                        motors, sensors, left_wv=left_wv, right_wv=right_wv
                    )
                _hold_neutral_upper_body(motors)

            # --- Power / mission labeling / battery + thermal models ---
            # Detect turn/spin before power + mission (GPS may be ~0 while wheels yaw)
            if _teleop is not None:
                turning = _teleop.is_turning_motion(
                    left_cmd=left_v,
                    right_cmd=right_v,
                    left_wheel_rad_s=left_wv,
                    right_wheel_rad_s=right_wv,
                    speed_m_s=speed_m_s,
                )
            else:
                turning = (
                    abs(left_v - right_v) > 1.0 and abs(left_v + right_v) < 1.5
                ) or (left_wv * right_wv < 0 and abs(left_wv - right_wv) > 0.4)

            # Pure spin: body GPS ~0 — proxy speed from wheel |ω| / cmd for power curve
            power_speed = speed_m_s
            if turning and speed_m_s < 0.12:
                power_speed = max(
                    speed_m_s,
                    0.5 * (abs(left_wv) + abs(right_wv)) * WHEEL_RADIUS_M,
                    0.5 * (abs(left_v) + abs(right_v)) * WHEEL_RADIUS_M,
                )
            joints = _read_joints(
                motors,
                sensors,
                wheel_vels=wheel_vels,
                cmd_wheel_v={"left_wheel": left_v, "right_wheel": right_v},
                speed_m_s=power_speed,
            )
            total_draw = sum(j.get("power_w", 0) for j in joints)
            wheel_motion = max(
                abs(j.get("velocity", 0.0))
                for j in joints
                if "wheel" in j.get("name", "")
            )
            residual_spin = (
                abs(left_wv) > 0.10
                or abs(right_wv) > 0.10
                or abs(yaw_rate) >= STOP_YAW_RATE_RAD_S
            )
            moving = (
                speed_m_s > 0.04
                or wheel_motion > 0.15
                or residual_spin
                or abs_brake.active
            )
            cmd_drive = abs(left_v) > 0.05 or abs(right_v) > 0.05
            # Keep mission/phase on drive/turn while commanding or still moving.
            # Residual hub/yaw spin must NOT look like standby idle on the dashboard.
            if abs_brake.active:
                gait = "stand"
                phase_name = "standby"
            elif residual_spin and not (user_driving or api_driving or cmd_drive):
                gait = "turn"
                phase_name = "teleop_turn"
            elif turning and not abs_brake.active:
                gait = "turn"
                phase_name = "teleop_turn"
            elif (user_driving or api_driving or cmd_drive) and not abs_brake.active:
                if moving or cmd_drive:
                    gait = "drive"
                    phase_name = "teleop"
                else:
                    gait = "stand"
                    phase_name = "standby"
            elif moving and speed_m_s > 0.08:
                # Coast / residual motion after teleop ends
                gait = "drive"
                phase_name = "teleop"
            motion_factor = 1.0 if (moving or turning) else 0.0
            if abs_brake.active:
                drain_scale = 0.1
            elif moving or turning:
                drain_scale = 1.0 if (user_driving or turning) else 0.2
            else:
                drain_scale = 0.05
            if _teleop is not None:
                battery_pct = max(
                    5.0,
                    battery_pct
                    - _teleop.battery_drain_pct(total_draw, dt, drain_scale=drain_scale),
                )
            else:
                battery_pct = max(
                    5.0,
                    battery_pct - (total_draw * dt) / (480.0 * 3600.0) * 100.0 * 50.0 * drain_scale,
                )
            thermal_c = _update_thermal(thermal_c, total_draw, dt, motion_factor=motion_factor)

            local_throttle, local_msg = _local_throttle(battery_pct, thermal_c)
            remote_for_teleop = (
                None if (user_driving and not api_driving) else remote_throttle
            )
            throttle_factor = _merge_throttle(local_throttle, remote_for_teleop)
            agent_message = local_msg if throttle_factor < 1.0 else None

            if user_driving and not abs_brake.active:
                if api_driving:
                    _set_drive(motors, left_v, right_v, 1.0)
                else:
                    _apply_wheel_command(motors, sensors, left_v, right_v, throttle_factor)

            if user_driving:
                drive_log_elapsed += dt
                if drive_log_elapsed >= 2.0:
                    drive_log_elapsed = 0.0
                    throttle_note = (
                        f" agent={throttle_factor:.0%}"
                        if throttle_factor < 0.995
                        else ""
                    )
                    print(
                        f"Driving L={left_v:.1f} R={right_v:.1f} "
                        f"@ {_format_pose(gps)} {speed_m_s:.2f} m/s{throttle_note}"
                    )
            else:
                drive_log_elapsed = 0.0

            # --- HUD + twin publish (throttled) ---
            if hud is not None:
                _draw_hud(
                    hud,
                    battery_pct=battery_pct,
                    thermal_c=thermal_c,
                    throttle=throttle_factor,
                    message=agent_message,
                    teleop_active=user_driving,
                    api_source=cached_api_source if api_driving else "",
                    auto_loop=auto_loop and not user_driving,
                    speed_m_s=speed_m_s,
                    braking=abs_brake.active,
                )

            if tick % publish_every == 0:
                payload = build_payload(
                    joints,
                    gait=gait,
                    phase=phase_name,
                    speed_m_s=speed_m_s,
                    battery_pct=battery_pct,
                    pose=_read_pose(gps, imu),
                    sensors={
                        "imu_roll": _safe_imu_roll(imu),
                        "yaw_rate": round(yaw_rate, 3),
                        "left_wheel_rad_s": round(left_wv, 3),
                        "right_wheel_rad_s": round(right_wv, 3),
                        "thermal_c": round(thermal_c, 2),
                        "teleop_active": user_driving,
                        "agent_throttle": throttle_factor,
                        "braking": abs_brake.active,
                        "residual_spin": residual_spin,
                    },
                )
                result = publish_telemetry(payload, dashboard)
                if result.get("ok", False):
                    publish_fail_streak = 0
                else:
                    publish_fail_streak += 1
                    if publish_fail_streak == 1 or publish_fail_streak % 20 == 0:
                        print(f"Twin publish failed (non-fatal): {result.get('error', result)}")
        except Exception as exc:
            print(f"Controller step error (recovering): {exc}")
            traceback.print_exc()
            continue

    print("ButlerBot controller stopped (simulation ended)")


def main() -> None:
    """Webots entry: parse controllerArgs, construct Robot, run the twin loop."""
    opts = parse_controller_args()
    robot = Robot()
    try:
        _run_loop(robot, opts)
    except Exception as exc:
        print(f"ButlerBot controller fatal error: {exc}")
        traceback.print_exc()
        sys.exit(0)


# ---------------------------------------------------------------------------
# Telemetry helpers (joint power + pose for twin_publisher payload)
# ---------------------------------------------------------------------------


def _estimate_joint_power(
    motor_name: str,
    velocity: float,
    torque: float,
    *,
    speed_m_s: float = 0.0,
) -> float:
    if estimate_motor_power_w is not None:
        try:
            return estimate_motor_power_w(
                velocity,
                torque,
                motor_name=motor_name,
                speed_m_s=speed_m_s if speed_m_s > 0.02 else None,
            )
        except Exception:
            pass
    idle = 2.5 if "wheel" in motor_name else 1.6
    scale = 3.6 if "wheel" in motor_name else 4.5
    return round(idle + abs(torque * velocity) * scale, 2)


def _read_joints(
    motors: dict[str, Motor],
    sensors: dict[str, PositionSensor],
    *,
    wheel_vels: dict[str, float] | None = None,
    cmd_wheel_v: dict[str, float] | None = None,
    speed_m_s: float = 0.0,
) -> list[dict]:
    mapping = {
        "left_wheel": "left_wheel_sensor",
        "right_wheel": "right_wheel_sensor",
        "torso_joint": "torso_sensor",
        "left_arm": "left_arm_sensor",
        "right_arm": "right_arm_sensor",
    }
    joints = []
    for motor_name, sensor_name in mapping.items():
        motor = motors[motor_name]
        sensor = sensors[sensor_name]
        try:
            position = sensor.getValue()
        except Exception:
            position = 0.0
        if "wheel" in motor_name and wheel_vels is not None:
            velocity = float(wheel_vels.get(motor_name, 0.0))
        else:
            try:
                velocity = motor.getVelocity()
            except Exception:
                velocity = 0.0
        # Encoder can lag a step behind command; use cmd vel as floor while driving
        if "wheel" in motor_name and cmd_wheel_v:
            cmd_v = abs(float(cmd_wheel_v.get(motor_name, 0.0)))
            if cmd_v > abs(velocity):
                velocity = math.copysign(cmd_v, velocity if abs(velocity) > 1e-6 else cmd_v)
        # Only synthesize ω from body speed when we have a non-zero wheel cmd floor
        # (never invent spin from GPS alone — that kept Legs high after Stop).
        if (
            "wheel" in motor_name
            and abs(velocity) < 0.15
            and speed_m_s > 0.06
            and cmd_wheel_v
            and abs(float(cmd_wheel_v.get(motor_name, 0.0))) > 0.05
        ):
            velocity = speed_m_s / max(WHEEL_RADIUS_M, 0.02)

        # Velocity-based torque model (Nm ≈ k·|ω|). Torque feedback is often 0
        # when not enabled in the world — never overwrite a useful estimate with 0.
        torque_est = abs(velocity) * 0.45
        torque = torque_est
        if hasattr(motor, "getTorqueFeedback"):
            try:
                tf = abs(float(motor.getTorqueFeedback()))
                if tf > 1e-4:
                    torque = tf
            except Exception:
                pass
        # Prefer per-wheel |ω| for power; body speed only when wheel vel is weak
        joint_speed = speed_m_s if abs(velocity) < 0.2 else 0.0
        power_w = _estimate_joint_power(
            motor_name, velocity, torque, speed_m_s=joint_speed
        )
        joints.append({
            "name": motor_name,
            "position": round(position, 4),
            "velocity": round(velocity, 4),
            "torque": round(torque, 4),
            "power_w": power_w,
        })
    return joints


def _read_pose(gps: GPS, imu: InertialUnit) -> dict:
    pose: dict = {}
    try:
        pos = gps.getValues()
        pose["x_m"] = round(pos[0], 3)
        pose["y_m"] = round(pos[1], 3)
        pose["z_m"] = round(pos[2], 3)
    except Exception:
        return pose
    try:
        rpy = imu.getRollPitchYaw()
        pose["heading_rad"] = round(rpy[2], 4)
    except Exception:
        pass
    return pose


if __name__ == "__main__":
    main()