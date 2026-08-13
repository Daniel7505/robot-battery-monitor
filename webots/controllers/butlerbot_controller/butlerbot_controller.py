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

from controller import Camera, Robot, Motor, PositionSensor, GPS, InertialUnit, Keyboard, Display

try:
    from controller import Supervisor
except ImportError:  # pragma: no cover - Webots always provides Supervisor
    Supervisor = None  # type: ignore[misc, assignment]

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

# Must stay ≤ RotationalMotor maxVelocity in butlerbot.wbt (wheels = 15).
MOTOR_MAX_VELOCITY = 15.0
# Pololu 37D 50:1 (#4753) stall ≈ 2.06 N·m @ 12 V (datasheet extrapolated).
# Available torque slightly above stall so ABS park can still hold hubs.
# Must match butlerbot.wbt RotationalMotor maxTorque and hardware profile.
WHEEL_STALL_TORQUE_NM = 2.06
WHEEL_AVAILABLE_TORQUE_NM = 2.5
MAX_WHEEL_V = 10.0
MAX_JOINT_V = 1.8
WHEEL_RADIUS_M = 0.08
# Differential track width (left/right wheel y anchors ±0.17 m)
WHEEL_TRACK_M = 0.34
# Soft grip coupling gain while intentionally driving (0=ODE only, 1=full odo lock).
# Thin cylinder tires in ODE under-deliver continuous contact; this pulls body linear
# velocity toward wheel odometry so track free-roll matches v=ωr without hop/spin.
SOFT_GRIP_GAIN = 0.55
MAX_BRAKE_WHEEL_V = 12.0
MIN_BRAKE_WHEEL_V = 1.5
# Linear "stopped" gate — must be tight; looser values completed ABS while coasting.
STOP_SPEED_M_S = 0.012
STOP_WHEEL_RAD_S = 0.03
BRAKE_COAST_PHASE_S = 0.15
MAX_BRAKE_DURATION_S = 12.0
# Residual body yaw (rad/s). 0.12 was too loose: "look away 3s and it moved"
# (sub-visual creep). ~0.025 ≈ 1.4 deg/s still catches that class.
STOP_YAW_RATE_RAD_S = 0.025
# Hub L−R mismatch while "locked" (rad/s)
STOP_WHEEL_DIFF_RAD_S = 0.05
# ABS must stay calm this long before complete (was 0.45s — finished early).
ABS_CALM_HOLD_S = 0.80
# After a turn/spin stop, demand a longer calm window (pure circle residual).
ABS_SPIN_CALM_HOLD_S = 1.35
# Re-ABS if GPS still reports translation while we think we are idle.
RESIDUAL_SPEED_M_S = 0.018
# Pose window residual (m / rad over ~0.6 s) — catches ultra-slow drift GPS rate misses.
POSE_RESIDUAL_TRANS_M = 0.025
POSE_RESIDUAL_YAW_RAD = 0.04
POSE_RESIDUAL_WINDOW_S = 0.65
# Pure-spin oppose only when NOT translating (avoids Tokyo drift on coast).
PURE_SPIN_OPPOSE_SPEED_M_S = 0.04
# Position-mode approach rate when holding encoder angle — MUST be ≤ MOTOR_MAX_VELOCITY
# or Webots warns every step and the hold is weaker than we think.
WHEEL_HOLD_VEL = 15.0

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
    """Body speed from GPS with EMA smoothing.

    Prefer ``GPS.getSpeed()`` (physics speed) when available; fall back to
    finite-difference of position. A short EMA removes the classic
    alternating 0.00 / 0.2 m/s flicker from single-step FD under light slip
    while still tracking true body motion for residual-park logic.
    """

    def __init__(self, ema_tau_s: float = 0.12) -> None:
        self._prev_pos: list[float] | None = None
        self._ema_speed = 0.0
        self._ema_forward = 0.0
        self._ema_tau_s = ema_tau_s
        self._have_ema = False

    def estimate_motion(self, gps: GPS, dt: float) -> tuple[float, float]:
        """Return (scalar speed m/s, signed forward speed along +X)."""
        try:
            pos = list(gps.getValues())
        except Exception:
            return 0.0, 0.0

        fd_speed = 0.0
        fd_forward = 0.0
        if self._prev_pos is not None and dt > 0:
            dx = pos[0] - self._prev_pos[0]
            dy = pos[1] - self._prev_pos[1]
            inv_dt = 1.0 / max(dt, 0.001)
            fd_forward = dx * inv_dt
            fd_speed = math.sqrt(dx * dx + dy * dy) * inv_dt
        self._prev_pos = pos

        # Physics speed from Webots when available (less noisy than 1-step FD)
        speed = fd_speed
        try:
            phys = float(gps.getSpeed())
            if math.isfinite(phys) and phys >= 0.0:
                speed = phys
        except Exception:
            pass

        # Preserve forward sign from FD (getSpeed is scalar)
        if abs(fd_forward) > 1e-6:
            forward = math.copysign(speed, fd_forward)
        else:
            forward = fd_forward

        # EMA over ~ema_tau_s
        if dt > 0 and self._ema_tau_s > 0:
            alpha = 1.0 - math.exp(-dt / self._ema_tau_s)
            if not self._have_ema:
                self._ema_speed = speed
                self._ema_forward = forward
                self._have_ema = True
            else:
                self._ema_speed += alpha * (speed - self._ema_speed)
                self._ema_forward += alpha * (forward - self._ema_forward)
            speed = self._ema_speed
            forward = self._ema_forward

        if _teleop is not None:
            return _teleop.sanitize_motion(speed, forward)
        speed = min(max(0.0, speed), 0.85)
        forward = max(-0.85, min(0.85, forward))
        return round(speed, 3), round(forward, 3)

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
        # Fresh stop: clear lock targets so the next hard-zero latches *now*.
        # Tracking current encoder every tick while freewheeling does nothing
        # (zero position error) — that was the "never stops / horizon chase" bug.
        _clear_wheel_locks()
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
            and abs(left_wv - right_wv) < STOP_WHEEL_DIFF_RAD_S
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
        """Park-brake ABS — dual hub freeze only (no differential oppose).

        Soft reverse and yaw-oppose were producing "Tokyo drift": body still
        translating while hubs applied unequal torque. Live fix path is:

          track encoder while fast → freeze both hubs when slow → hold.

        Complete only when hubs + yaw + GPS linear stay quiet for ABS_CALM_HOLD_S.
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
        linear_ok = speed_m_s < STOP_SPEED_M_S and abs(forward_m_s) < STOP_SPEED_M_S
        if wheels_ok and yaw_ok and linear_ok:
            self._calm_hold_s += dt
        else:
            self._calm_hold_s = 0.0

        # ALWAYS freeze latched targets during ABS (never track). Tracking
        # freewheel = zero error = no brake torque = constant coast.
        _hard_zero_wheels(
            motors,
            sensors,
            left_wv=left_wv,
            right_wv=right_wv,
            track=False,
        )

        # Pure residual circle after hubs quiet: brief yaw oppose pulse only.
        hubs_still_spinning = (
            abs(left_wv) > STOP_WHEEL_RAD_S * 2.0
            or abs(right_wv) > STOP_WHEEL_RAD_S * 2.0
        )
        pure_spin = (
            self._spin_mode
            and not hubs_still_spinning
            and abs(yaw_rate) >= STOP_YAW_RATE_RAD_S
            and speed_m_s < PURE_SPIN_OPPOSE_SPEED_M_S
        )
        if pure_spin:
            _oppose_body_yaw(motors, yaw_rate, min_rate=0.01)
            # Re-assert freeze next tick after the pulse.

        calm_need = ABS_SPIN_CALM_HOLD_S if self._spin_mode else ABS_CALM_HOLD_S
        if self._calm_hold_s >= calm_need:
            self.active = False
            _hard_zero_wheels(
                motors, sensors, left_wv=left_wv, right_wv=right_wv, track=False
            )
            print(
                f"ABS park complete @ speed={speed_m_s:.3f} "
                f"wv L={left_wv:.2f} R={right_wv:.2f} yaw_rate={yaw_rate:.3f} "
                f"hold={self._calm_hold_s:.2f}s spin={self._spin_mode}"
            )
            return True

        if self._elapsed_s >= MAX_BRAKE_DURATION_S and wheels_ok and yaw_ok and linear_ok:
            self.active = False
            _hard_zero_wheels(
                motors, sensors, left_wv=left_wv, right_wv=right_wv, track=False
            )
            print("ABS park timeout complete")
            return True

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


def _clamp_motor_vel(value: float) -> float:
    """Clamp any wheel velocity command to the world motor maxVelocity."""
    return _clamp(value, MOTOR_MAX_VELOCITY)


def _safe_imu_pitch(imu: InertialUnit | None) -> float:
    if imu is None:
        return 0.0
    try:
        return float(imu.getRollPitchYaw()[1])
    except Exception:
        return 0.0


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
        ("setAvailableTorque", WHEEL_AVAILABLE_TORQUE_NM),
        ("setAvailableForce", WHEEL_AVAILABLE_TORQUE_NM),
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
    track: bool = False,
) -> None:
    """Stop BOTH hubs: encoder-rate oppose while spinning, then fixed position lock.

    Lessons:
      - Tracking live encoder every step → zero error → freewheel coast (horizon chase).
      - Position-lock alone can be weak while |ω| is high; first oppose measured hub rate
        (symmetric if both freewheel forward; differential if spinning in place).
      - When |ω| is small, latch a finite encoder angle and hold (never NaN).

    WHEEL_HOLD_VEL / oppose cmds must stay ≤ world maxVelocity (15).
    """
    hold_vel = min(WHEEL_HOLD_VEL, MOTOR_MAX_VELOCITY)
    rates = {"left_wheel": left_wv, "right_wheel": right_wv}
    for wheel in ("left_wheel", "right_wheel"):
        motor = motors[wheel]
        _enable_full_wheel_torque(motor)
        wv = rates.get(wheel)
        spinning = wv is not None and abs(float(wv)) > STOP_WHEEL_RAD_S * 2.5

        # Phase 1 — demand zero hub rate with full torque (velocity mode).
        # Do NOT command continuous reverse ω — that held |ω|≈7.5 and Legs~27W
        # forever (park never completed, locks never engaged).
        if spinning:
            try:
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)
            except Exception:
                pass
            # Still latch target so phase-2 lock is ready as soon as |ω| drops
            if sensors is not None:
                sensor = sensors.get(f"{wheel}_sensor")
                if sensor is not None:
                    try:
                        pos = float(sensor.getValue())
                        if math.isfinite(pos) and (
                            wheel not in _WHEEL_LOCK_POS or track
                        ):
                            _WHEEL_LOCK_POS[wheel] = pos
                    except Exception:
                        pass
            continue

        # Phase 2 — position lock at fixed target (hubs already near zero)
        if sensors is None:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            continue
        sensor = sensors.get(f"{wheel}_sensor")
        if sensor is None:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
            continue
        try:
            pos = float(sensor.getValue())
            if not math.isfinite(pos):
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)
                continue
            if wheel not in _WHEEL_LOCK_POS or track:
                _WHEEL_LOCK_POS[wheel] = pos
            hold = _WHEEL_LOCK_POS[wheel]
            motor.setPosition(hold)
            motor.setVelocity(hold_vel)
        except Exception:
            try:
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)
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
        _hard_zero_wheels(
            motors, sensors, left_wv=left_wv, right_wv=right_wv, track=True
        )
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
    gain: float = 4.0,
    min_rate: float | None = None,
) -> None:
    """Brief differential command opposing residual body spin (GPS may be ~0)."""
    floor = STOP_YAW_RATE_RAD_S if min_rate is None else min_rate
    if abs(yaw_rate) < floor:
        return
    # Positive yaw_rate ≈ CCW: oppose with left+, right- (and reverse for CW)
    # Floor magnitude so sub-visual creep still gets a real oppose pulse.
    mag = min(5.0, max(0.8, abs(yaw_rate) * gain))
    sign = 1.0 if yaw_rate > 0.0 else -1.0
    left_cmd = sign * mag
    right_cmd = -sign * mag
    for wheel, cmd in (("left_wheel", left_cmd), ("right_wheel", right_cmd)):
        motor = motors[wheel]
        _enable_full_wheel_torque(motor)
        motor.setPosition(float("inf"))
        motor.setVelocity(_clamp_motor_vel(_clamp(cmd, MAX_BRAKE_WHEEL_V)))


def _gps_xy(gps: GPS | None) -> tuple[float, float] | None:
    """Body planar position from GPS, or None if unavailable."""
    if gps is None:
        return None
    try:
        vals = list(gps.getValues())
        return float(vals[0]), float(vals[1])
    except Exception:
        return None


def _apply_soft_grip(
    robot: Robot,
    *,
    left_wv: float,
    right_wv: float,
    imu: InertialUnit | None,
    gain: float = SOFT_GRIP_GAIN,
) -> None:
    """Blend body linear (+ yaw) velocity toward wheel-odometry kinematics.

    ODE thin-cylinder tire contacts often deliver intermittent force, so hubs
    can hold commanded ω while the body crawls. While the operator/API is
    intentionally driving, softly couple body velocity to v=ωr so free-roll
    matches the track budget. Disabled during ABS park / idle.
    """
    if gain <= 0.0 or not hasattr(robot, "getSelf"):
        return
    try:
        node = robot.getSelf()
    except Exception:
        return
    if node is None:
        return
    try:
        v_odo = 0.5 * (left_wv + right_wv) * WHEEL_RADIUS_M
        # Yaw rate for differential: ω_z ≈ (ω_r - ω_l) * r / track
        yaw_rate_odo = (right_wv - left_wv) * WHEEL_RADIUS_M / max(WHEEL_TRACK_M, 0.05)
        yaw = 0.0
        if imu is not None:
            try:
                yaw = float(imu.getRollPitchYaw()[2])
            except Exception:
                yaw = 0.0
        tx = v_odo * math.cos(yaw)
        ty = v_odo * math.sin(yaw)
        vel = node.getVelocity()
        if vel is None or len(vel) < 6:
            return
        g = max(0.0, min(1.0, float(gain)))
        new_vel = [
            (1.0 - g) * float(vel[0]) + g * tx,
            (1.0 - g) * float(vel[1]) + g * ty,
            float(vel[2]),
            float(vel[3]),
            float(vel[4]),
            (1.0 - g) * float(vel[5]) + g * yaw_rate_odo,
        ]
        node.setVelocity(new_vel)
    except Exception:
        # Grip assist is best-effort; never break the control loop
        return


def _pitch_balance_delta(
    imu: InertialUnit | None,
    prev_pitch: float,
    dt: float,
    cfg: dict,
    *,
    abs_active: bool,
) -> tuple[float, float]:
    """Equal hub correction from IMU pitch (BNO085 stand-in). Returns (dω, pitch)."""
    pitch = _safe_imu_pitch(imu)
    if not cfg.get("enabled"):
        return 0.0, pitch
    if abs_active and not cfg.get("apply_while_abs"):
        return 0.0, pitch
    if abs(pitch) < float(cfg.get("deadband_rad") or 0.025):
        return 0.0, pitch
    rate = (pitch - prev_pitch) / dt if dt > 1e-4 else 0.0
    raw = -float(cfg["kp_pitch"]) * pitch - float(cfg["kd_pitch_rate"]) * rate
    lim = float(cfg.get("max_correct_rad_s") or 0.8)
    return _clamp(raw, lim), pitch


def _load_balance_cfg() -> dict:
    try:
        from src.hardware_profile import balance_control_spec

        return balance_control_spec()
    except Exception:
        return {
            "enabled": False,
            "kp_pitch": 2.0,
            "kd_pitch_rate": 0.85,
            "max_correct_rad_s": 0.8,
            "deadband_rad": 0.025,
            "apply_while_abs": False,
        }


def _load_lane_keep():
    try:
        from src.lane_keep import lane_keep_command, peak_score_bgra, red_score, yellow_score

        return lane_keep_command, yellow_score, red_score, peak_score_bgra
    except Exception as exc:
        print(f"WARNING: lane_keep policy not loaded ({exc})")
        return None, None, None, None


def _camera_max_rgb(cam: Camera | None) -> tuple[float, float, float]:
    """Debug: brightest R,G,B seen through Webots accessors."""
    if cam is None:
        return (0.0, 0.0, 0.0)
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
        get_r = cam.imageGetRed
        get_g = cam.imageGetGreen
        get_b = cam.imageGetBlue
    except Exception:
        return (0.0, 0.0, 0.0)
    if not image or w <= 0 or h <= 0:
        return (0.0, 0.0, 0.0)
    mr = mg = mb = 0
    step = 3
    for y in range(0, h, step):
        for x in range(0, w, step):
            mr = max(mr, get_r(image, w, x, y))
            mg = max(mg, get_g(image, w, x, y))
            mb = max(mb, get_b(image, w, x, y))
    return (mr / 255.0, mg / 255.0, mb / 255.0)


def _camera_peak_score(cam: Camera | None, score_fn, peak_fn) -> float:
    """Peak color score using Webots imageGetRed/Green/Blue (not raw BGRA)."""
    if cam is None or score_fn is None:
        return 0.0
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
    except Exception:
        return 0.0
    if not image or w <= 0 or h <= 0:
        return 0.0
    get_r = getattr(cam, "imageGetRed", None)
    get_g = getattr(cam, "imageGetGreen", None)
    get_b = getattr(cam, "imageGetBlue", None)
    if get_r is None or get_g is None or get_b is None:
        if peak_fn is None:
            return 0.0
        try:
            return float(peak_fn(image, w, h, score_fn))
        except Exception:
            return 0.0
    scale = 1.0 / 255.0
    # Floor lives in the lower part of a forward cam; sky washes the top.
    # Scan every pixel in that band so a 2 cm stripe can still register.
    best = 0.0
    y0 = h // 3
    for y in range(y0, h):
        for x in range(w):
            rgb = (
                get_r(image, w, x, y) * scale,
                get_g(image, w, x, y) * scale,
                get_b(image, w, x, y) * scale,
            )
            val = float(score_fn(rgb))
            if val > best:
                best = val
    return best


def _read_lane_eyes(cams: dict, yellow_fn, red_fn, peak_fn) -> dict:
    """Peak color scores so a thin paint stripe still registers."""
    ly = _camera_peak_score(cams.get("line_left"), yellow_fn, peak_fn)
    ry = _camera_peak_score(cams.get("line_right"), yellow_fn, peak_fn)
    fr_l = _camera_peak_score(cams.get("finish_cam"), red_fn, peak_fn)
    fr_r = _camera_peak_score(cams.get("finish_cam_r"), red_fn, peak_fn)
    return {
        "left_yellow": round(ly, 3),
        "right_yellow": round(ry, 3),
        "finish_red": round(max(fr_l, fr_r), 3),
    }


def _set_drive(motors: dict[str, Motor], left_v: float, right_v: float, throttle: float) -> None:
    """Velocity-mode wheel command with agent throttle scale (0–1)."""
    scale = max(0.0, min(1.0, throttle))
    for side, cmd in (("left_wheel", left_v * scale), ("right_wheel", right_v * scale)):
        motor = motors[side]
        _enable_full_wheel_torque(motor)
        motor.setPosition(float("inf"))
        motor.setVelocity(_clamp_motor_vel(_clamp(cmd, MAX_WHEEL_V)))


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
) -> tuple:
    """Enable motors, encoders, GPS, head GPS, IMU, keyboard, and optional HUD."""
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
    gps_head = None
    try:
        gps_head = robot.getDevice("gps_head")
        if gps_head is not None:
            gps_head.enable(timestep)
    except Exception:
        gps_head = None
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

    cams: dict[str, Camera | None] = {}
    for name in ("line_left", "line_right", "finish_cam", "finish_cam_r"):
        cam = None
        try:
            cam = robot.getDevice(name)
            if cam is not None:
                cam.enable(timestep)
        except Exception:
            cam = None
        cams[name] = cam
        if cam is None:
            print(f"WARNING: camera '{name}' not found — lane-keep eye missing")

    return motors, sensors, gps, gps_head, imu, keyboard, hud, cams


def _format_pose(gps: GPS) -> str:
    try:
        pos = gps.getValues()
        return f"({pos[0]:.2f}, {pos[1]:.2f})"
    except Exception:
        return "(?, ?)"


# Sideline onlooker: robot's right (−Y), ~14 ft out. Matches DEF VIEWPOINT.
# ENU identity looks −Z (down); +X rotation tilts toward +Y (at the bot).
# Daniel mouse-framed sideline 2026-08-12 (follow off).
_CAM_EYE = (0.02270, -3.81863, 0.81003)
_CAM_LOOK = (0.0, 0.0, 0.55)
_CAM_FOV = 0.85
_CAM_ORIENTATION = (0.0, 0.0, 1.0, 1.56)


def _apply_follow_camera(robot: Robot) -> None:
    """Reset the world Viewpoint onto ButlerBot (close 3/4 chase).

    Live Webots keeps a zoomed-out / look-down pose for the whole session even
    when the .wbt file already has follow=ButlerBot. Re-applying at controller
    start makes every launch (and every reload) land on the robot without a
    manual zoom. Does not touch motors / stop / power.
    """
    if Supervisor is None or not isinstance(robot, Supervisor):
        return
    try:
        vp = robot.getFromDef("VIEWPOINT")
    except Exception:
        vp = None
    if vp is None:
        print("Viewpoint DEF VIEWPOINT not found — skip follow-cam reset")
        return
    try:
        field = vp.getField
        follow = field("follow")
        if follow:
            follow.setSFString("ButlerBot")
        ftype = field("followType")
        if ftype:
            ftype.setSFString("Tracking Shot")
        smooth = field("followSmoothness")
        if smooth:
            smooth.setSFFloat(0.2)
        pos = field("position")
        if pos:
            pos.setSFVec3f(list(_CAM_EYE))
        ori = field("orientation")
        if ori:
            ori.setSFRotation(list(_CAM_ORIENTATION))
        fov = field("fieldOfView")
        if fov:
            fov.setSFFloat(_CAM_FOV)
        print(
            "Camera reset: Tracking Shot on ButlerBot "
            f"(Daniel sideline) eye={_CAM_EYE} fov={_CAM_FOV}"
        )
    except Exception as exc:
        print(f"Follow-cam reset skipped: {exc}")


_DUMP_CAM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "DUMP_CAM")


def _maybe_dump_viewpoint(robot: Robot) -> None:
    """If DUMP_CAM exists beside this controller, print live viewpoint and remove it."""
    if not os.path.isfile(_DUMP_CAM):
        return
    try:
        os.remove(_DUMP_CAM)
    except OSError:
        pass
    if Supervisor is None or not isinstance(robot, Supervisor):
        print("VIEWPOINT dump: not a Supervisor")
        return
    try:
        vp = robot.getFromDef("VIEWPOINT")
        if vp is None:
            print("VIEWPOINT dump: DEF VIEWPOINT missing")
            return
        pos = vp.getField("position").getSFVec3f()
        ori = vp.getField("orientation").getSFRotation()
        fol = vp.getField("follow").getSFString()
        print(
            "VIEWPOINT COPY "
            f"position {pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f} "
            f"orientation {ori[0]:.6f} {ori[1]:.6f} {ori[2]:.6f} {ori[3]:.6f} "
            f"follow={fol!r}"
        )
    except Exception as exc:
        print(f"VIEWPOINT dump failed: {exc}")


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
    motors, sensors, gps, gps_head, imu, keyboard, hud, cams = _init_devices(robot, timestep)
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
    prev_pitch = _safe_imu_pitch(imu)
    balance_cfg = _load_balance_cfg()
    if balance_cfg.get("enabled"):
        print(
            "Pitch-hold ON (BNO085-class IMU) "
            f"kp={balance_cfg['kp_pitch']} kd={balance_cfg['kd_pitch_rate']}"
        )
    lane_keep_fn, yellow_fn, red_fn, peak_fn = _load_lane_keep()
    lane_keep_on = False
    last_lane_sig = ""
    lane_eyes = {"left_yellow": 0.0, "right_yellow": 0.0, "finish_red": 0.0}
    yaw_rate = 0.0
    # Pose residual window — catches "look away 3s" ultra-slow drift GPS rate misses
    pose_anchor_xy: tuple[float, float] | None = None
    pose_anchor_yaw = prev_yaw
    pose_anchor_age_s = 0.0
    # After Stop, ignore stale "active" drive polls for this long (seconds).
    # Prevents a race from re-arming cruise mid-park (horizon chase / Tokyo drift).
    park_holdoff_s = 0.0
    # After intentional API drive, ignore residual "re-park on motion" for a while.
    # Without this: brief teleop gap → residual_hub/coast sees motion → ABS stop
    # → next re-assert drive → hop-stop-hop (user visual + 40s for ~3 m track).
    drive_grace_s = 0.0
    # Wheel rotation sensors (encoder-based): absolute angle + lock detection
    # These answer "are the wheels actually locking?" for park diagnosis.
    wheel_enc_prev: dict[str, float] = {}
    wheel_rot_abs: dict[str, float] = {"left_wheel": 0.0, "right_wheel": 0.0}
    wheel_rot_since_stop: dict[str, float] = {"left_wheel": 0.0, "right_wheel": 0.0}
    wheel_lock_hold_s: dict[str, float] = {"left_wheel": 0.0, "right_wheel": 0.0}
    was_abs_active = False

    _apply_follow_camera(robot)
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
            if tick % 8 == 0:
                _maybe_dump_viewpoint(robot)
            if park_holdoff_s > 0.0:
                park_holdoff_s = max(0.0, park_holdoff_s - dt)
            if drive_grace_s > 0.0:
                drive_grace_s = max(0.0, drive_grace_s - dt)
            # --- Input: keyboard + R (auto loop) / Space (ABS) ---
            if keyboard is not None:
                keys, pressed = key_tracker.poll(keyboard)
            else:
                keys, pressed = set(), set()

            if KEY_R in pressed:
                auto_loop = not auto_loop
                print(f"Auto mission loop: {'ON' if auto_loop else 'OFF'}")

            # --- Sense: GPS translation, encoder rates, body yaw rate ---
            # Keep RAW GPS for residual kill — never zero it away before that check.
            speed_m_s, forward_m_s = speed_estimator.estimate_motion(gps, dt)
            raw_speed_m_s = speed_m_s
            raw_forward_m_s = forward_m_s
            left_wv_early = abs_brake.wheel_rad_s(sensors, "left_wheel_sensor", dt)
            right_wv_early = abs_brake.wheel_rad_s(sensors, "right_wheel_sensor", dt)
            yaw_rate, prev_yaw = _yaw_rate(imu, prev_yaw, dt)
            gps_xy = _gps_xy(gps)

            hubs_locked = (
                abs(left_wv_early) < STOP_WHEEL_RAD_S
                and abs(right_wv_early) < STOP_WHEEL_RAD_S
                and abs(left_wv_early - right_wv_early) < STOP_WHEEL_DIFF_RAD_S
            )
            yaw_quiet = abs(yaw_rate) < STOP_YAW_RATE_RAD_S
            linear_quiet = (
                raw_speed_m_s < STOP_SPEED_M_S
                and abs(raw_forward_m_s) < STOP_SPEED_M_S
            )
            no_drive_cmd = (
                abs(last_teleop_left) < 0.05
                and abs(last_teleop_right) < 0.05
                and abs(cached_api_left) < 0.05
                and abs(cached_api_right) < 0.05
            )
            # ONLY zero GPS for mission/power labels when hubs + yaw + linear are quiet.
            # Old path zeroed on hubs+yaw alone → false idle while still coasting/creeping.
            if (
                hubs_locked
                and yaw_quiet
                and linear_quiet
                and no_drive_cmd
                and not abs_brake.active
            ):
                speed_m_s = 0.0
                forward_m_s = 0.0

            settled = (
                hubs_locked
                and yaw_quiet
                and linear_quiet
                and (
                    _teleop.motion_settled(raw_speed_m_s, left_wv_early, right_wv_early)
                    if _teleop is not None
                    else raw_speed_m_s < 0.08
                )
            )

            if KEY_SPACE in pressed:
                park_holdoff_s = max(park_holdoff_s, 2.0)
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
            # ORDER MATTERS: stop_epoch always wins over a stale "active" drive.
            # Empty/failed GET must NOT mean "drive expired" — that caused mid-cruise
            # ABS park → clear → drive → jerk (user: three short hops, not 6s free roll).
            if tick % state_poll_every == 0:
                twin_state = fetch_twin_state(dashboard)
                poll_ok = isinstance(twin_state, dict) and bool(twin_state)
                if not poll_ok:
                    # Keep last cached_api_* / abs state; do not park on network blip
                    pass
                else:
                    remote_throttle = remote_throttle_factor(twin_state)
                    api_cmd = teleop_from_twin_state(twin_state)
                    if api_cmd.get("battery_pct") is not None:
                        battery_pct = float(api_cmd["battery_pct"])
                        if api_cmd.get("reset_thermal"):
                            thermal_c = 22.0
                        print(f"Battery replenished → {battery_pct:.0f}% (dashboard command)")

                    stop_epoch = float(api_cmd.get("stop_epoch") or 0.0)
                    got_new_stop = stop_epoch > last_stop_epoch
                    api_source = str(api_cmd.get("source") or "")
                    api_active = bool(api_cmd.get("active"))
                    lane_keep_on = bool(api_cmd.get("lane_keep"))

                    if got_new_stop:
                        last_stop_epoch = stop_epoch
                        cached_api_left = 0.0
                        cached_api_right = 0.0
                        cached_api_source = "stop"
                        last_api_sig = ""
                        park_holdoff_s = 3.0
                        print("External drive stop — park brake")
                        if not abs_brake.active:
                            abs_brake.request(
                                forward_m_s,
                                speed_m_s,
                                last_left_v=last_teleop_left,
                                last_right_v=last_teleop_right,
                                left_wv=left_wv_early,
                                right_wv=right_wv_early,
                            )
                    elif api_active and park_holdoff_s <= 0.0:
                        if abs_brake.active:
                            abs_brake.clear()
                        _clear_wheel_locks()
                        cached_api_left = float(api_cmd.get("left_v") or 0.0)
                        cached_api_right = float(api_cmd.get("right_v") or 0.0)
                        cached_api_source = api_source or "api"
                        last_teleop_left = cached_api_left
                        last_teleop_right = cached_api_right
                        # Intentional cruise: suppress residual "stop on motion"
                        drive_grace_s = max(drive_grace_s, 2.5)
                        sig = f"{cached_api_left}:{cached_api_right}:{cached_api_source}"
                        if sig != last_api_sig:
                            last_api_sig = sig
                            print(
                                f"External drive from {cached_api_source}: "
                                f"L={cached_api_left} R={cached_api_right}"
                            )
                    elif api_active and park_holdoff_s > 0.0:
                        cached_api_left = 0.0
                        cached_api_right = 0.0
                    else:
                        # Confirmed inactive from a good poll only
                        was_api_driving = (
                            abs(cached_api_left) > 0.01 or abs(cached_api_right) > 0.01
                        )
                        # Explicit stop source or clean expiry (source still api/empty)
                        confirmed_end = (
                            api_source in ("", "stop", "api")
                            and not api_active
                            and not lane_keep_on
                        )
                        if (
                            was_api_driving
                            and not abs_brake.active
                            and confirmed_end
                        ):
                            print("External drive expired — park brake")
                            abs_brake.request(
                                forward_m_s,
                                speed_m_s,
                                last_left_v=last_teleop_left,
                                last_right_v=last_teleop_right,
                                left_wv=left_wv_early,
                                right_wv=right_wv_early,
                            )
                        if confirmed_end:
                            cached_api_left = 0.0
                            cached_api_right = 0.0
                            cached_api_source = api_source
                            if api_source != "stop":
                                last_api_sig = ""

            if yellow_fn and red_fn and peak_fn:
                lane_eyes = _read_lane_eyes(cams, yellow_fn, red_fn, peak_fn)

            if (
                lane_keep_on
                and lane_keep_fn is not None
                and not abs_brake.active
                and not user_driving
                and park_holdoff_s <= 0.0
            ):
                lk = lane_keep_fn(
                    lane_eyes["left_yellow"],
                    lane_eyes["right_yellow"],
                    lane_eyes["finish_red"],
                )
                if lk["brake"]:
                    print("Lane-keep: red finish — ABS")
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
                    cached_api_source = "agent"
                    last_lane_sig = "brake"
                else:
                    cached_api_left = float(lk["left"])
                    cached_api_right = float(lk["right"])
                    cached_api_source = "agent"
                    last_teleop_left = cached_api_left
                    last_teleop_right = cached_api_right
                    drive_grace_s = max(drive_grace_s, 1.0)
                    sig = (
                        f"{lk['left']:.1f}:{lk['right']:.1f}:"
                        f"{lane_eyes['left_yellow']:.2f}:{lane_eyes['right_yellow']:.2f}"
                    )
                    if sig != last_lane_sig:
                        last_lane_sig = sig
                        mx = _camera_max_rgb(cams.get("finish_cam"))
                        print(
                            "Lane-keep "
                            f"L={lk['left']:.2f} R={lk['right']:.2f} "
                            f"yL={lane_eyes['left_yellow']:.2f} "
                            f"yR={lane_eyes['right_yellow']:.2f} "
                            f"red={lane_eyes['finish_red']:.2f} "
                            f"navRGB=({mx[0]:.2f},{mx[1]:.2f},{mx[2]:.2f})"
                        )

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
                        motors,
                        sensors,
                        left_wv=left_wv,
                        right_wv=right_wv,
                        track=False,
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
                # Multi-layer residual kill:
                # 1) hub freewheel  2) yaw rate  3) GPS coast  4) pose window creep
                residual_hub = (
                    abs(left_wv) > STOP_WHEEL_RAD_S * 1.2
                    or abs(right_wv) > STOP_WHEEL_RAD_S * 1.2
                    or abs(left_wv - right_wv) > STOP_WHEEL_DIFF_RAD_S * 1.5
                )
                residual_yaw = abs(yaw_rate) >= STOP_YAW_RATE_RAD_S
                residual_coast = raw_speed_m_s >= RESIDUAL_SPEED_M_S

                pose_residual = False
                pose_anchor_age_s += dt
                if gps_xy is not None:
                    if pose_anchor_xy is None:
                        pose_anchor_xy = gps_xy
                        pose_anchor_yaw = prev_yaw
                        pose_anchor_age_s = 0.0
                    elif pose_anchor_age_s >= POSE_RESIDUAL_WINDOW_S:
                        dx = gps_xy[0] - pose_anchor_xy[0]
                        dy = gps_xy[1] - pose_anchor_xy[1]
                        trans = math.sqrt(dx * dx + dy * dy)
                        dyaw = prev_yaw - pose_anchor_yaw
                        while dyaw > math.pi:
                            dyaw -= 2.0 * math.pi
                        while dyaw < -math.pi:
                            dyaw += 2.0 * math.pi
                        if (
                            trans >= POSE_RESIDUAL_TRANS_M
                            or abs(dyaw) >= POSE_RESIDUAL_YAW_RAD
                        ):
                            pose_residual = True
                        pose_anchor_xy = gps_xy
                        pose_anchor_yaw = prev_yaw
                        pose_anchor_age_s = 0.0

                # Only residual-park when we are truly idle — NOT during/after a
                # recent intentional drive (drive_grace). Movement detection used
                # to re-ABS immediately → hop-stop-hop on the track.
                allow_residual_park = drive_grace_s <= 0.0
                if (
                    allow_residual_park
                    and (residual_hub or residual_yaw or residual_coast or pose_residual)
                ):
                    if not abs_brake.active:
                        why = []
                        if residual_hub:
                            why.append("hub")
                        if residual_yaw:
                            why.append("yaw")
                        if residual_coast:
                            why.append("coast")
                        if pose_residual:
                            why.append("pose")
                        print(
                            f"Residual ({'+'.join(why)}) — re-park "
                            f"L={left_wv:.2f} R={right_wv:.2f} "
                            f"yaw_rate={yaw_rate:.3f} speed={raw_speed_m_s:.3f}"
                        )
                        abs_brake.request(
                            raw_forward_m_s,
                            raw_speed_m_s,
                            last_left_v=last_teleop_left,
                            last_right_v=last_teleop_right,
                            left_wv=left_wv,
                            right_wv=right_wv,
                        )
                        pose_anchor_xy = gps_xy
                        pose_anchor_yaw = prev_yaw
                        pose_anchor_age_s = 0.0
                    else:
                        _hard_zero_wheels(
                            motors,
                            sensors,
                            left_wv=left_wv,
                            right_wv=right_wv,
                            track=False,
                        )
                else:
                    # Idle hold OR grace window after drive (do not ABS on motion)
                    _hard_zero_wheels(
                        motors,
                        sensors,
                        left_wv=left_wv,
                        right_wv=right_wv,
                        track=False,
                    )
                _hold_neutral_upper_body(motors)

            # While actively driving (keyboard OR API), keep pose residual fresh
            if user_driving or api_driving or abs_brake.active or auto_loop:
                pose_anchor_xy = gps_xy
                pose_anchor_yaw = prev_yaw
                pose_anchor_age_s = 0.0
                if api_driving or user_driving:
                    drive_grace_s = max(drive_grace_s, 2.5)

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
            # Real pack physics: always full P·dt drain (no demo accel / idle fudge).
            # total_draw already includes driver + DC-DC path losses when twin_power is used.
            if _teleop is not None:
                cap_wh = float(
                    getattr(_teleop, "BATTERY_CAPACITY_WH", 480.0) or 480.0
                )
                battery_pct = max(
                    5.0,
                    battery_pct
                    - _teleop.battery_drain_pct(
                        total_draw,
                        dt,
                        capacity_wh=cap_wh,
                        scale=1.0,
                        drain_scale=1.0,
                    ),
                )
            else:
                battery_pct = max(
                    5.0,
                    battery_pct
                    - (total_draw * dt) / (480.0 * 3600.0) * 100.0,
                )
            thermal_c = _update_thermal(thermal_c, total_draw, dt, motion_factor=motion_factor)

            local_throttle, local_msg = _local_throttle(battery_pct, thermal_c)
            remote_for_teleop = (
                None if (user_driving and not api_driving) else remote_throttle
            )
            throttle_factor = _merge_throttle(local_throttle, remote_for_teleop)
            agent_message = local_msg if throttle_factor < 1.0 else None

            dω, prev_pitch = _pitch_balance_delta(
                imu,
                prev_pitch,
                dt,
                balance_cfg,
                abs_active=abs_brake.active,
            )
            if user_driving and not abs_brake.active:
                left_v += dω
                right_v += dω
                if api_driving:
                    _set_drive(motors, left_v, right_v, 1.0)
                else:
                    _apply_wheel_command(motors, sensors, left_v, right_v, throttle_factor)
                # Soft no-slip coupling while intentionally driving (not during ABS)
                _apply_soft_grip(
                    robot,
                    left_wv=left_wv,
                    right_wv=right_wv,
                    imu=imu,
                    gain=SOFT_GRIP_GAIN,
                )

            if user_driving:
                drive_log_elapsed += dt
                # Interval body speed from pose (stable) — not single-step FD flicker
                try:
                    _log_pos = list(gps.getValues())
                except Exception:
                    _log_pos = None
                if drive_log_elapsed >= 2.0:
                    odo_v = 0.5 * (abs(left_wv) + abs(right_wv)) * WHEEL_RADIUS_M
                    avg_body = speed_m_s
                    if (
                        _log_pos is not None
                        and getattr(speed_estimator, "_log_anchor", None) is not None
                    ):
                        ax, ay = speed_estimator._log_anchor  # type: ignore[attr-defined]
                        avg_body = (
                            math.hypot(_log_pos[0] - ax, _log_pos[1] - ay)
                            / max(drive_log_elapsed, 1e-3)
                        )
                    if _log_pos is not None:
                        speed_estimator._log_anchor = (_log_pos[0], _log_pos[1])  # type: ignore[attr-defined]
                    drive_log_elapsed = 0.0
                    throttle_note = (
                        f" agent={throttle_factor:.0%}"
                        if throttle_factor < 0.995
                        else ""
                    )
                    print(
                        f"Driving L={left_v:.1f} R={right_v:.1f} "
                        f"@ {_format_pose(gps)} "
                        f"gps={avg_body:.2f} odo={odo_v:.2f} m/s"
                        f"{throttle_note}"
                    )
            else:
                drive_log_elapsed = 0.0
                if hasattr(speed_estimator, "_log_anchor"):
                    speed_estimator._log_anchor = None  # type: ignore[attr-defined]

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

            # --- Wheel rotation sensors (PositionSensor on each drive hub) ---
            # Tracks absolute encoder angle, cumulative rotation, and lock hold.
            hub_rates = {
                "left_wheel": left_wv,
                "right_wheel": right_wv,
            }
            # Reset "since stop" integrator when a new ABS park begins
            if abs_brake.active and not was_abs_active:
                wheel_rot_since_stop = {"left_wheel": 0.0, "right_wheel": 0.0}
                wheel_lock_hold_s = {"left_wheel": 0.0, "right_wheel": 0.0}
            was_abs_active = abs_brake.active

            for wname, rate in hub_rates.items():
                sname = f"{wname}_sensor"
                enc = None
                try:
                    enc = float(sensors[sname].getValue())
                except Exception:
                    enc = None
                if enc is not None and math.isfinite(enc):
                    prev = wheel_enc_prev.get(wname)
                    if prev is not None:
                        dth = enc - prev
                        # unwrap large jumps (shouldn't happen for continuous joint)
                        while dth > math.pi:
                            dth -= 2.0 * math.pi
                        while dth < -math.pi:
                            dth += 2.0 * math.pi
                        adth = abs(dth)
                        wheel_rot_abs[wname] = wheel_rot_abs.get(wname, 0.0) + adth
                        # Integrate rotation after stop / while parking or idle
                        if abs_brake.active or (
                            abs(left_v) < 0.05
                            and abs(right_v) < 0.05
                            and not user_driving
                            and not api_driving
                        ):
                            wheel_rot_since_stop[wname] = (
                                wheel_rot_since_stop.get(wname, 0.0) + adth
                            )
                    wheel_enc_prev[wname] = enc
                # Lock detector: |ω| quiet for a short hold
                if abs(rate) < STOP_WHEEL_RAD_S * 1.5:
                    wheel_lock_hold_s[wname] = wheel_lock_hold_s.get(wname, 0.0) + dt
                else:
                    wheel_lock_hold_s[wname] = 0.0

            wheel_locked = {
                w: wheel_lock_hold_s.get(w, 0.0) >= 0.25
                for w in ("left_wheel", "right_wheel")
            }
            wheels_both_locked = all(wheel_locked.values())

            # Publish faster while parking so suites/agents see control_diag live
            publish_ticks = 1 if abs_brake.active else publish_every
            if tick % publish_ticks == 0:
                lock_l = _WHEEL_LOCK_POS.get("left_wheel")
                lock_r = _WHEEL_LOCK_POS.get("right_wheel")
                wheel_sensors = {
                    "left": {
                        "encoder_rad": (
                            None
                            if "left_wheel" not in wheel_enc_prev
                            else round(wheel_enc_prev["left_wheel"], 4)
                        ),
                        "omega_rad_s": round(left_wv, 4),
                        "rot_abs_rad": round(wheel_rot_abs.get("left_wheel", 0.0), 4),
                        "rot_since_stop_rad": round(
                            wheel_rot_since_stop.get("left_wheel", 0.0), 4
                        ),
                        "rot_since_stop_deg": round(
                            math.degrees(wheel_rot_since_stop.get("left_wheel", 0.0)), 2
                        ),
                        "locked": wheel_locked["left_wheel"],
                        "lock_hold_s": round(wheel_lock_hold_s.get("left_wheel", 0.0), 3),
                    },
                    "right": {
                        "encoder_rad": (
                            None
                            if "right_wheel" not in wheel_enc_prev
                            else round(wheel_enc_prev["right_wheel"], 4)
                        ),
                        "omega_rad_s": round(right_wv, 4),
                        "rot_abs_rad": round(wheel_rot_abs.get("right_wheel", 0.0), 4),
                        "rot_since_stop_rad": round(
                            wheel_rot_since_stop.get("right_wheel", 0.0), 4
                        ),
                        "rot_since_stop_deg": round(
                            math.degrees(wheel_rot_since_stop.get("right_wheel", 0.0)), 2
                        ),
                        "locked": wheel_locked["right_wheel"],
                        "lock_hold_s": round(wheel_lock_hold_s.get("right_wheel", 0.0), 3),
                    },
                    "both_locked": wheels_both_locked,
                }
                control_diag = {
                    "schema": "butlerbot_control_diag_v1",
                    "hub_left_rad_s": round(left_wv, 4),
                    "hub_right_rad_s": round(right_wv, 4),
                    "hub_diff_rad_s": round(abs(left_wv - right_wv), 4),
                    "yaw_rate_rad_s": round(yaw_rate, 4),
                    "gps_speed_m_s": round(raw_speed_m_s, 4),
                    "gps_forward_m_s": round(raw_forward_m_s, 4),
                    "cmd_left": round(left_v, 3),
                    "cmd_right": round(right_v, 3),
                    "cmd_source": (
                        "abs_park"
                        if abs_brake.active
                        else (
                            cached_api_source
                            if api_driving
                            else ("keyboard" if user_driving else ("auto" if auto_loop else "idle"))
                        )
                    ),
                    "abs_active": abs_brake.active,
                    "abs_spin_mode": bool(getattr(abs_brake, "_spin_mode", False)),
                    "abs_elapsed_s": round(float(getattr(abs_brake, "_elapsed_s", 0.0)), 3),
                    "abs_calm_hold_s": round(float(getattr(abs_brake, "_calm_hold_s", 0.0)), 3),
                    "stop_epoch_seen": last_stop_epoch,
                    "park_holdoff_s": round(park_holdoff_s, 3),
                    "lock_left": None if lock_l is None else round(float(lock_l), 4),
                    "lock_right": None if lock_r is None else round(float(lock_r), 4),
                    "locks_engaged": lock_l is not None and lock_r is not None,
                    "residual_spin": residual_spin,
                    "phase": phase_name,
                    "gait": gait,
                    # Explicit wheel rotation sensors (encoder truth for lock)
                    "wheels": wheel_sensors,
                }
                payload = build_payload(
                    joints,
                    gait=gait,
                    phase=phase_name,
                    speed_m_s=speed_m_s,
                    battery_pct=battery_pct,
                    pose=_read_pose(gps, imu),
                    sensors={
                        "imu_roll": _safe_imu_roll(imu),
                        "imu_pitch": _safe_imu_pitch(imu),
                        **_read_head_gps(gps_head),
                        "yaw_rate": round(yaw_rate, 4),
                        "left_wheel_rad_s": round(left_wv, 4),
                        "right_wheel_rad_s": round(right_wv, 4),
                        "thermal_c": round(thermal_c, 2),
                        "teleop_active": user_driving,
                        "lane_keep": lane_keep_on,
                        "left_yellow": lane_eyes.get("left_yellow"),
                        "right_yellow": lane_eyes.get("right_yellow"),
                        "finish_red": lane_eyes.get("finish_red"),
                        "agent_throttle": throttle_factor,
                        "braking": abs_brake.active,
                        "residual_spin": residual_spin,
                        "control_diag": control_diag,
                        "wheels": wheel_sensors,
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
    """Webots entry: parse controllerArgs, construct Robot/Supervisor, run twin loop."""
    opts = parse_controller_args()
    # Supervisor enables soft grip coupling (node.setVelocity). Falls back to Robot.
    if Supervisor is not None:
        robot = Supervisor()
    else:
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
        cmd_v = 0.0
        if "wheel" in motor_name and cmd_wheel_v:
            cmd_v = abs(float(cmd_wheel_v.get(motor_name, 0.0)))
            # Encoder can lag a step behind command; use cmd as floor while driving
            if cmd_v > abs(velocity):
                velocity = math.copysign(cmd_v, velocity if abs(velocity) > 1e-6 else cmd_v)
        # Only synthesize ω from body speed when we have a non-zero wheel cmd floor
        # (never invent spin from GPS alone — that kept Legs high after Stop).
        if (
            "wheel" in motor_name
            and abs(velocity) < 0.15
            and speed_m_s > 0.06
            and cmd_v > 0.05
        ):
            velocity = speed_m_s / max(WHEEL_RADIUS_M, 0.02)

        # Parked / locked hubs: do NOT count position-hold torque or motor
        # maxVelocity target as cruise load. That produced Legs~27W while pose
        # was frozen (false high idle for agents / energy baseline).
        wheel_parked = (
            "wheel" in motor_name
            and cmd_v < 0.05
            and abs(velocity) < 0.15
            and speed_m_s < 0.05
        )
        if wheel_parked:
            joints.append({
                "name": motor_name,
                "position": round(position, 4),
                "velocity": 0.0,
                "torque": 0.0,
                "power_w": 2.0,  # profile wheel idle_w
            })
            continue

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
        # and we still have a drive command (parked path already returned).
        joint_speed = speed_m_s if (abs(velocity) < 0.2 and cmd_v > 0.05) else 0.0
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


def _read_head_gps(gps_head) -> dict:
    """Head-mounted GPS — sees pitch/rock as y/z wobble, not just +x travel."""
    if gps_head is None:
        return {}
    try:
        p = gps_head.getValues()
        return {
            "head_x_m": round(float(p[0]), 4),
            "head_y_m": round(float(p[1]), 4),
            "head_z_m": round(float(p[2]), 4),
        }
    except Exception:
        return {}


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
        pose["roll_rad"] = round(rpy[0], 4)
        pose["pitch_rad"] = round(rpy[1], 4)
        pose["heading_rad"] = round(rpy[2], 4)
    except Exception:
        pass
    return pose


if __name__ == "__main__":
    main()