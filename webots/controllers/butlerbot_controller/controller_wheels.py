"""Wheel ABS / residual-spin / soft-grip. Scar tissue — do not casual-rewrite.

Residual spin / ABS design notes (read before changing brake code):

* GPS reports translation only — pure in-place yaw shows ~0 m/s while a hub
  still spins on camera. Stops therefore require **finite wheel encoders**,
  **dual-hub hard-zero**, and **IMU yaw-rate** quiescence — not GPS alone.
* ``setPosition(NaN)`` freewheels a hub; locks use only finite encoder angles.
* ``stop_epoch`` from the dashboard ensures a stop is not missed if wheel
  commands already read as zero on the next poll.
"""
from __future__ import annotations

import math

from controller import GPS, InertialUnit, Motor, PositionSensor, Robot

_teleop = None


def bind_teleop(mod) -> None:
    global _teleop
    _teleop = mod


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
