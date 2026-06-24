"""
ButlerBot Webots controller — WASD teleop + onboard power agent HUD.

WASD drive (click the 3D view first). Space = stop. R = toggle auto mission loop.
Posts joint states + power estimates to the twin bridge; agent throttles when
battery or heat limits are exceeded.
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


class SpeedEstimator:
    def __init__(self) -> None:
        self._prev_pos: list[float] | None = None

    def estimate(self, gps: GPS, dt: float) -> float:
        try:
            pos = list(gps.getValues())
        except Exception:
            return 0.0
        if self._prev_pos is not None and dt > 0:
            dx = pos[0] - self._prev_pos[0]
            dy = pos[1] - self._prev_pos[1]
            speed = math.sqrt(dx * dx + dy * dy) / max(dt, 0.001)
            self._prev_pos = pos
            return round(speed, 3)
        self._prev_pos = pos
        return 0.0


class KeyTracker:
    """Track held keys from Webots keyboard events."""

    def __init__(self) -> None:
        self._active: set[int] = set()

    def poll(self, keyboard: Keyboard) -> tuple[set[int], set[int]]:
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


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _safe_imu_roll(imu: InertialUnit | None) -> float:
    if imu is None:
        return 0.0
    try:
        return float(imu.getRollPitchYaw()[0])
    except Exception:
        return 0.0


def _set_drive(motors: dict[str, Motor], left_v: float, right_v: float, throttle: float) -> None:
    scale = max(0.0, min(1.0, throttle))
    for side, cmd in (("left_wheel", left_v * scale), ("right_wheel", right_v * scale)):
        motor = motors[side]
        motor.setPosition(float("inf"))
        motor.setVelocity(_clamp(cmd, MAX_WHEEL_V))


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


def _expand_teleop_keys(keys: set[int]) -> set[int]:
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
    if _teleop is not None:
        return _teleop.local_agent_throttle(battery_pct, thermal_c)
    return 1.0, None


def _update_thermal(thermal_c: float, draw_w: float, dt: float, motion_factor: float) -> float:
    if _teleop is not None:
        return _teleop.update_thermal_c(thermal_c, draw_w, dt, motion_factor=motion_factor)
    return thermal_c


def _merge_throttle(local: float, remote: float | None) -> float:
    if _teleop is not None:
        return _teleop.merge_throttle(local, remote)
    if remote is None:
        return local
    return min(local, float(remote))


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
) -> None:
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
    display.drawText("Arrows/IJKL drive (not WASD)", 12, 124)

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
    timestep = int(robot.getBasicTimeStep())
    publish_every = max(1, int(opts["interval_s"] * 1000 / timestep))
    state_poll_every = max(publish_every * 2, int(1.0 * 1000 / timestep))
    motors, sensors, gps, imu, keyboard, hud = _init_devices(robot, timestep)
    speed_estimator = SpeedEstimator()
    key_tracker = KeyTracker()

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
    drive_log_elapsed = 0.0

    print(f"ButlerBot controller started — twin → {dashboard}/api/twin/telemetry")
    print(f"Battery synced from dashboard: {battery_pct:.1f}%")
    print("Teleop: Arrow keys or I/J/K/L — click the FLOOR (not the robot), Escape deselects gizmos")
    print("Or use Dashboard: Drive Forward (API) under the twin panel")

    while robot.step(timestep) != -1:
        try:
            tick += 1
            dt = timestep / 1000.0
            if keyboard is not None:
                keys, pressed = key_tracker.poll(keyboard)
            else:
                keys, pressed = set(), set()

            if KEY_R in pressed:
                auto_loop = not auto_loop
                print(f"Auto mission loop: {'ON' if auto_loop else 'OFF'}")

            teleop_keys = keys - {KEY_R}
            if KEY_SPACE in teleop_keys:
                teleop_keys.discard(KEY_SPACE)
            user_driving = bool(teleop_keys)

            phase = PHASES[phase_idx]
            phase_name = "teleop" if user_driving else phase["name"]
            gait = "drive" if user_driving else phase["gait"]

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
                    cached_api_left = float(api_cmd.get("left_v") or 0.0)
                    cached_api_right = float(api_cmd.get("right_v") or 0.0)
                    cached_api_source = str(api_cmd.get("source") or "api")
                    sig = f"{cached_api_left}:{cached_api_right}:{cached_api_source}"
                    if sig != last_api_sig:
                        last_api_sig = sig
                        print(
                            f"External drive from {cached_api_source}: "
                            f"L={cached_api_left} R={cached_api_right}"
                        )
                else:
                    cached_api_left = 0.0
                    cached_api_right = 0.0
                    cached_api_source = ""
                    last_api_sig = ""

            api_driving = abs(cached_api_left) > 0.01 or abs(cached_api_right) > 0.01
            if api_driving:
                user_driving = True
                left_v, right_v = cached_api_left, cached_api_right
                _set_drive(motors, left_v, right_v, 1.0)
                _hold_neutral_upper_body(motors)
            elif user_driving:
                left_v, right_v = _teleop_drive(teleop_keys)
                _set_drive(motors, left_v, right_v, 1.0)
                _hold_neutral_upper_body(motors)
            elif auto_loop:
                left_v, right_v = _apply_phase_motion(
                    motors, gait=gait, phase=phase, t=t, throttle=1.0
                )
            else:
                left_v = right_v = 0.0
                _set_drive(motors, 0.0, 0.0, 1.0)
                _hold_neutral_upper_body(motors)

            joints = _read_joints(motors, sensors)
            total_draw = sum(j.get("power_w", 0) for j in joints)
            speed_m_s = speed_estimator.estimate(gps, dt)
            wheel_motion = max(
                abs(j.get("velocity", 0.0))
                for j in joints
                if "wheel" in j.get("name", "")
            )
            moving = user_driving or speed_m_s > 0.04 or wheel_motion > 0.25
            motion_factor = 1.0 if moving else 0.0
            drain_scale = (1.0 if user_driving else 0.25) if moving else 0.05
            battery_pct = max(5.0, battery_pct - total_draw * dt / 480.0 * 100.0 * 0.004 * drain_scale)
            thermal_c = _update_thermal(thermal_c, total_draw, dt, motion_factor=motion_factor)

            local_throttle, local_msg = _local_throttle(battery_pct, thermal_c)
            throttle_factor = _merge_throttle(local_throttle, remote_throttle)
            agent_message = local_msg if throttle_factor < 1.0 else None

            if user_driving:
                _set_drive(motors, left_v, right_v, throttle_factor)

            if user_driving:
                drive_log_elapsed += dt
                if drive_log_elapsed >= 2.0:
                    drive_log_elapsed = 0.0
                    print(
                        f"Driving L={left_v:.1f} R={right_v:.1f} "
                        f"@ {_format_pose(gps)} throttle={throttle_factor:.0%}"
                    )
            else:
                drive_log_elapsed = 0.0

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
                        "thermal_c": round(thermal_c, 2),
                        "teleop_active": user_driving,
                        "agent_throttle": throttle_factor,
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
    opts = parse_controller_args()
    robot = Robot()
    try:
        _run_loop(robot, opts)
    except Exception as exc:
        print(f"ButlerBot controller fatal error: {exc}")
        traceback.print_exc()
        sys.exit(0)


def _estimate_joint_power(motor_name: str, velocity: float, torque: float) -> float:
    if estimate_motor_power_w is not None:
        try:
            return estimate_motor_power_w(velocity, torque, motor_name=motor_name)
        except Exception:
            pass
    idle = 2.5 if "wheel" in motor_name else 1.6
    scale = 3.6 if "wheel" in motor_name else 4.5
    return round(idle + abs(torque * velocity) * scale, 2)


def _read_joints(motors: dict[str, Motor], sensors: dict[str, PositionSensor]) -> list[dict]:
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
            velocity = motor.getVelocity()
            position = sensor.getValue()
        except Exception:
            velocity = 0.0
            position = 0.0
        torque = abs(velocity) * 0.45
        if hasattr(motor, "getTorqueFeedback"):
            try:
                torque = abs(motor.getTorqueFeedback())
            except Exception:
                pass
        power_w = _estimate_joint_power(motor_name, velocity, torque)
        joints.append({
            "name": motor_name,
            "position": round(position, 4),
            "velocity": round(velocity, 4),
            "torque": round(torque, 4),
            "power_w": power_w,
        })
    return joints


def _read_pose(gps: GPS, imu: InertialUnit) -> dict:
    try:
        pos = gps.getValues()
        rpy = imu.getRollPitchYaw()
        return {
            "x_m": round(pos[0], 3),
            "y_m": round(pos[1], 3),
            "z_m": round(pos[2], 3),
            "heading_rad": round(rpy[2], 4),
        }
    except Exception:
        return {}


if __name__ == "__main__":
    main()