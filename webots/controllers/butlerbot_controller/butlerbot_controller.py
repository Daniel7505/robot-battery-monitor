"""
ButlerBot Webots controller — drives robot and publishes twin telemetry.

Cycles: stand → drive → patrol → manipulate → idle
Posts joint states + estimated power to DigitalTwinBridge each step.
"""

from __future__ import annotations

import math
import sys
import traceback

from controller import Robot, Motor, PositionSensor, GPS, InertialUnit

from twin_publisher import build_payload, parse_controller_args, publish_telemetry

# Mission phases — drive speeds are wheel angular velocity (rad/s)
PHASES = [
    {
        "name": "standby",
        "gait": "stand",
        "duration_s": 6,
        "drive_speed": 0.0,
        "turn_amp": 0.0,
        "arm_amp": 0.08,
        "torso_amp": 0.05,
    },
    {
        "name": "walk_transit",
        "gait": "walk",
        "duration_s": 14,
        "drive_speed": 6.0,
        "turn_amp": 0.0,
        "arm_amp": 0.18,
        "torso_amp": 0.08,
    },
    {
        "name": "patrol",
        "gait": "patrol",
        "duration_s": 12,
        "drive_speed": 4.0,
        "turn_amp": 3.2,
        "arm_amp": 0.22,
        "torso_amp": 0.12,
    },
    {
        "name": "manipulate",
        "gait": "manipulate",
        "duration_s": 10,
        "drive_speed": 0.0,
        "turn_amp": 0.0,
        "arm_amp": 0.95,
        "torso_amp": 0.35,
    },
    {
        "name": "return_idle",
        "gait": "stand",
        "duration_s": 10,
        "drive_speed": -4.5,
        "turn_amp": 0.6,
        "arm_amp": 0.0,
        "torso_amp": 0.0,
    },
]

MOTOR_NAMES = ["left_wheel", "right_wheel", "torso_joint", "left_arm", "right_arm"]
SENSOR_NAMES = [
    "left_wheel_sensor",
    "right_wheel_sensor",
    "torso_sensor",
    "left_arm_sensor",
    "right_arm_sensor",
]

MAX_WHEEL_V = 8.0
MAX_JOINT_V = 1.8
WHEEL_RADIUS_M = 0.065


class SpeedEstimator:
    """Track GPS displacement without module-level globals."""

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


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _safe_imu_roll(imu: InertialUnit | None) -> float:
    if imu is None:
        return 0.0
    try:
        return float(imu.getRollPitchYaw()[0])
    except Exception:
        return 0.0


def _set_drive(motors: dict[str, Motor], left_v: float, right_v: float) -> None:
    for side, cmd in (("left_wheel", left_v), ("right_wheel", right_v)):
        motor = motors[side]
        motor.setPosition(float("inf"))
        motor.setVelocity(_clamp(cmd, MAX_WHEEL_V))


def _set_joint_position(motors: dict[str, Motor], name: str, position: float) -> None:
    motors[name].setPosition(_clamp(position, MAX_JOINT_V))


def _set_joint_velocity(motors: dict[str, Motor], name: str, velocity: float) -> None:
    motor = motors[name]
    motor.setPosition(float("inf"))
    motor.setVelocity(_clamp(velocity, MAX_JOINT_V))


def _apply_phase_motion(
    motors: dict[str, Motor],
    *,
    gait: str,
    phase: dict,
    t: float,
) -> tuple[float, float]:
    """Return commanded left/right wheel speeds for telemetry."""
    drive = phase["drive_speed"]
    turn = phase["turn_amp"]
    arm_amp = phase["arm_amp"]
    torso_amp = phase["torso_amp"]

    if gait == "walk":
        left_v = drive
        right_v = drive
        _set_drive(motors, left_v, right_v)
        _set_joint_velocity(motors, "torso_joint", torso_amp * math.sin(t * 0.7))
        arm_cmd = arm_amp * math.sin(t * 1.0)
        _set_joint_velocity(motors, "left_arm", arm_cmd)
        _set_joint_velocity(motors, "right_arm", -arm_cmd * 0.85)
        return left_v, right_v

    if gait == "patrol":
        # Differential drive weave — arcs across the floor
        turn_cmd = turn * math.sin(t * 0.65)
        speed_mod = 0.75 + 0.25 * math.sin(t * 1.1)
        left_v = drive * speed_mod - turn_cmd
        right_v = drive * speed_mod + turn_cmd
        _set_drive(motors, left_v, right_v)
        _set_joint_velocity(motors, "torso_joint", torso_amp * math.sin(t * 0.5))
        arm_cmd = arm_amp * math.sin(t * 0.9)
        _set_joint_velocity(motors, "left_arm", arm_cmd)
        _set_joint_velocity(motors, "right_arm", -arm_cmd * 0.9)
        return left_v, right_v

    if gait == "manipulate":
        _set_drive(motors, 0.0, 0.0)
        # Position targets produce large, visible arm/torso motion
        _set_joint_position(motors, "torso_joint", torso_amp * math.sin(t * 0.55))
        _set_joint_position(
            motors,
            "left_arm",
            arm_amp * math.sin(t * 1.15),
        )
        _set_joint_position(
            motors,
            "right_arm",
            arm_amp * math.sin(t * 1.15 + math.pi * 0.85),
        )
        return 0.0, 0.0

    # standby / return_idle — slow reverse or idle with optional gentle turn
    if abs(drive) > 0.05:
        turn_cmd = turn * math.sin(t * 0.4)
        left_v = drive - turn_cmd
        right_v = drive + turn_cmd
        _set_drive(motors, left_v, right_v)
    else:
        left_v = right_v = 0.0
        _set_drive(motors, 0.0, 0.0)

    if arm_amp > 0.0 or torso_amp > 0.0:
        _set_joint_velocity(motors, "torso_joint", torso_amp * math.sin(t * 0.45))
        arm_cmd = arm_amp * math.sin(t * 0.8)
        _set_joint_velocity(motors, "left_arm", arm_cmd)
        _set_joint_velocity(motors, "right_arm", -arm_cmd * 0.8)
    else:
        _set_joint_position(motors, "torso_joint", 0.0)
        _set_joint_position(motors, "left_arm", 0.0)
        _set_joint_position(motors, "right_arm", 0.0)

    return left_v, right_v


def _init_devices(robot: Robot, timestep: int) -> tuple[dict[str, Motor], dict[str, PositionSensor], GPS, InertialUnit]:
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
    return motors, sensors, gps, imu


def _format_pose(gps: GPS) -> str:
    try:
        pos = gps.getValues()
        return f"({pos[0]:.2f}, {pos[1]:.2f})"
    except Exception:
        return "(?, ?)"


def _run_loop(robot: Robot, opts: dict) -> None:
    timestep = int(robot.getBasicTimeStep())
    publish_every = max(1, int(opts["interval_s"] * 1000 / timestep))
    motors, sensors, gps, imu = _init_devices(robot, timestep)
    speed_estimator = SpeedEstimator()

    phase_idx = 0
    phase_elapsed = 0.0
    battery_pct = 92.0
    tick = 0
    publish_fail_streak = 0
    dashboard = opts["dashboard_url"]

    print(f"ButlerBot controller started — twin → {dashboard}/api/twin/telemetry")
    print("Phases: standby → walk_transit → patrol → manipulate → return_idle (loop)")

    while robot.step(timestep) != -1:
        try:
            tick += 1
            dt = timestep / 1000.0
            phase_elapsed += dt

            phase = PHASES[phase_idx]
            if phase_elapsed >= phase["duration_s"]:
                phase_idx = (phase_idx + 1) % len(PHASES)
                phase_elapsed = 0.0
                phase = PHASES[phase_idx]
                print(f"Phase → {phase['name']} ({phase['gait']}) @ {_format_pose(gps)}")

            t = phase_elapsed
            gait = phase["gait"]
            _apply_phase_motion(motors, gait=gait, phase=phase, t=t)

            joints = _read_joints(motors, sensors)
            total_draw = sum(j.get("power_w", 0) for j in joints)
            battery_pct = max(5.0, battery_pct - total_draw * dt / 480.0 * 100.0 * 0.02)
            speed_m_s = speed_estimator.estimate(gps, dt)

            if tick % publish_every == 0:
                payload = build_payload(
                    joints,
                    gait=gait,
                    phase=phase["name"],
                    speed_m_s=speed_m_s,
                    battery_pct=battery_pct,
                    pose=_read_pose(gps, imu),
                    sensors={"imu_roll": _safe_imu_roll(imu)},
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
        power_w = round(1.2 + abs(torque * velocity) * 4.5, 2)
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