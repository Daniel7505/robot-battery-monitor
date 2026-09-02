"""
ButlerBot Webots controller — nadir-only orchestrator.

Each simulation step:

1. Read GPS, IMU, wheel encoders, keyboard / dashboard teleop.
2. Drive wheels (nadir keep, manual, API) with local + remote throttle.
3. ABS stop on Space or dashboard ``drive_stop``.
4. Estimate joint power, drain a virtual battery, POST telemetry.
5. Poll ``GET /api/twin/state`` for throttle, API drive, ``stop_epoch``.

Eyes on the wheel: ``nadir_left`` / ``nadir_right`` only. LINE / finish /
forecast / SIDELOOK and aim pucks were removed from ``butlerbot.wbt``.

Choir harvest (picture-wins, wall meters, Z/W, GapPlanner, red-mark) is
archived at ``archives/butlerbot_controller_choir_2026-09-02.py``.
Copy that file back over this one to undo.

Siblings (do not re-merge):

* ``controller_hud.py`` — Display gauges + shoulder overlays
* ``controller_keys.py`` — keyboard teleop
* ``controller_wheels.py`` — ABS / residual-spin / soft-grip
* ``twin_publisher.py`` — telemetry HTTP

Click the **floor**, not the robot, or Webots steals keyboard focus.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import time
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
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)


def _load_teleop_agent():
    """Optionally load ``src/teleop_agent.py`` for shared brake/throttle math."""
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

from controller_hud import (
    _draw_hud,
    _label_eye_huds,
    _paint_eye_huds,
    bind_teleop as _bind_hud_teleop,
)
from controller_keys import (
    KEY_A,
    KEY_D,
    KEY_DOWN,
    KEY_I,
    KEY_J,
    KEY_K,
    KEY_L,
    KEY_LEFT,
    KEY_R,
    KEY_RIGHT,
    KEY_S,
    KEY_SPACE,
    KEY_UP,
    KEY_W,
    KeyTracker,
    _teleop_drive,
    bind_teleop as _bind_keys_teleop,
)
from controller_eyes import _nadir_lateral_from_cam
from controller_wheels import (
    MAX_JOINT_V,
    MAX_WHEEL_V,
    SOFT_GRIP_GAIN,
    POSE_RESIDUAL_TRANS_M,
    POSE_RESIDUAL_WINDOW_S,
    POSE_RESIDUAL_YAW_RAD,
    RESIDUAL_SPEED_M_S,
    STOP_SPEED_M_S,
    STOP_WHEEL_DIFF_RAD_S,
    STOP_WHEEL_RAD_S,
    STOP_YAW_RATE_RAD_S,
    WHEEL_RADIUS_M,
    AbsBrakeController,
    SpeedEstimator,
    _apply_soft_grip,
    _clamp,
    _clamp_motor_vel,
    _clear_wheel_locks,
    _enable_full_wheel_torque,
    _gps_xy,
    _halt_wheels,
    _hard_zero_wheels,
    _safe_imu_pitch,
    _safe_imu_roll,
    _WHEEL_LOCK_POS,
    bind_teleop as _bind_wheels_teleop,
)

_bind_hud_teleop(_teleop)
_bind_keys_teleop(_teleop)
_bind_wheels_teleop(_teleop)


def _imu_yaw(imu: InertialUnit | None) -> float:
    if imu is None:
        return 0.0
    try:
        return float(imu.getRollPitchYaw()[2])
    except Exception:
        return 0.0


def _yaw_rate(imu: InertialUnit | None, prev_yaw: float, dt: float) -> tuple[float, float]:
    yaw = _imu_yaw(imu)
    if dt <= 1e-6:
        return 0.0, yaw
    dy = yaw - prev_yaw
    while dy > math.pi:
        dy -= 2.0 * math.pi
    while dy < -math.pi:
        dy += 2.0 * math.pi
    return dy / dt, yaw


def _pitch_balance_delta(
    imu: InertialUnit | None,
    prev_pitch: float,
    dt: float,
    cfg: dict,
    *,
    abs_active: bool,
) -> tuple[float, float]:
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
    """Nadir policy only. Choir names stay in lane_keep.py until that file is stripped."""
    try:
        from src.lane_keep import (
            NadirGuard,
            SteerFilter,
            lane_keep_command,
            nadir_wheel_to_tape,
            track_cross_track_m,
        )

        return (
            lane_keep_command,
            nadir_wheel_to_tape,
            NadirGuard,
            SteerFilter,
            track_cross_track_m,
        )
    except Exception as exc:
        print(f"WARNING: lane_keep policy not loaded ({exc})")
        return None, None, None, None, None


def _set_drive(motors: dict[str, Motor], left_v: float, right_v: float, throttle: float) -> None:
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


_STEER_LOG = os.path.join(
    os.path.expanduser("~"),
    "OneDrive",
    "Desktop",
    "Grok Workspace",
    "steer-actions.csv",
)
_steer_log_ready = False
_steer_log_key = ""


def _log_steer_action(pose: dict, lane_eyes: dict) -> None:
    global _steer_log_ready, _steer_log_key
    steer = lane_eyes.get("steer")
    if steer is None:
        return
    x = pose.get("x_m")
    y = pose.get("y_m")
    key = f"{float(steer):.3f}|{x}|{y}"
    if key == _steer_log_key:
        return
    _steer_log_key = key
    try:
        if not _steer_log_ready:
            new = not os.path.isfile(_STEER_LOG)
            with open(_STEER_LOG, "a", encoding="ascii") as fh:
                if new:
                    fh.write("unix_s,x_m,y_m,steer,nL,nR,aL,aR,src\n")
            _steer_log_ready = True
        with open(_STEER_LOG, "a", encoding="ascii") as fh:
            fh.write(
                f"{time.time():.3f},{x},{y},{float(steer):.4f},"
                f"{lane_eyes.get('nadir_gap_px')},{lane_eyes.get('nadir_r_gap_px')},"
                f"{lane_eyes.get('nadir_ahead_px')},{lane_eyes.get('nadir_r_ahead_px')},"
                f"{lane_eyes.get('error_source')}\n"
            )
    except OSError:
        pass


def _format_pose(gps: GPS) -> str:
    try:
        pos = gps.getValues()
        return f"({pos[0]:.2f}, {pos[1]:.2f})"
    except Exception:
        return "(?, ?)"


_CAM_EYE = (-3.18573, -0.03020, 3.54536)
_CAM_FOV = 0.85
_CAM_ORIENTATION = (-0.001250, 0.999989, 0.004550, 0.536256)
_NADIR_PLACE_CHECK = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "NADIR_PLACE_CHECK"
)
_NADIR_STEER_ON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "NADIR_STEER_ON"
)
_FINISH_X_M = 24.5
_NADIR_CRUISE = 5.5
_NADIR_SNAP_DIR = os.path.join(
    os.path.expanduser("~"), "OneDrive", "Desktop", "Grok Workspace"
)


def _apply_follow_camera(robot: Robot) -> None:
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
            f"(Daniel chase) eye={_CAM_EYE} fov={_CAM_FOV}"
        )
    except Exception as exc:
        print(f"Follow-cam reset skipped: {exc}")


def _init_devices(
    robot: Robot, timestep: int
) -> tuple:
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
    for name in ("nadir_left", "nadir_right"):
        cam = None
        try:
            cam = robot.getDevice(name)
        except Exception:
            cam = None
        if cam is not None:
            cam.enable(timestep)
        cams[name] = cam
        if cam is None:
            print(f"WARNING: camera '{name}' not found — lane-keep eye missing")
    print("NADIR EYES ONLY — line/finish/forecast/sidelook not enabled")

    return motors, sensors, gps, gps_head, imu, keyboard, hud, cams


def _empty_lane_eyes() -> dict:
    return {
        "nadir_lateral_m": None,
        "nadir_gap_px": None,
        "nadir_ahead_px": None,
        "nadir_stripe_px": None,
        "nadir_tape_col": None,
        "nadir_wheel_col": None,
        "nadir_r_lateral_m": None,
        "nadir_r_gap_px": None,
        "nadir_r_ahead_px": None,
        "nadir_r_stripe_px": None,
        "nadir_r_tape_col": None,
        "nadir_r_wheel_col": None,
        "steer": None,
        "phase": None,
        "metric_active": False,
        "error_source": "nadir",
    }


def _harvest_nadir(cams: dict, nadir_fn, gps_xy, yaw: float) -> dict:
    eyes = _empty_lane_eyes()
    left = _nadir_lateral_from_cam(
        cams.get("nadir_left"), None, gps_xy, yaw, nadir_fn, side="left"
    )
    right = _nadir_lateral_from_cam(
        cams.get("nadir_right"), None, gps_xy, yaw, nadir_fn, side="right"
    )
    if left:
        eyes["nadir_lateral_m"] = left.get("m")
        eyes["nadir_gap_px"] = left.get("gap_px")
        eyes["nadir_ahead_px"] = left.get("gap_ahead_px")
        eyes["nadir_stripe_px"] = left.get("stripe_px")
        eyes["nadir_tape_col"] = left.get("tape_col")
        eyes["nadir_wheel_col"] = left.get("wheel_col")
    if right:
        eyes["nadir_r_lateral_m"] = right.get("m")
        eyes["nadir_r_gap_px"] = right.get("gap_px")
        eyes["nadir_r_ahead_px"] = right.get("gap_ahead_px")
        eyes["nadir_r_stripe_px"] = right.get("stripe_px")
        eyes["nadir_r_tape_col"] = right.get("tape_col")
        eyes["nadir_r_wheel_col"] = right.get("wheel_col")
    return eyes


def _run_loop(robot: Robot, opts: dict) -> None:
    """Priority: ABS → dashboard API → keyboard → nadir keep → residual park."""
    timestep = int(robot.getBasicTimeStep())
    publish_every = max(1, int(opts["interval_s"] * 1000 / timestep))
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
    last_stop_epoch = 0.0
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
    (
        lane_keep_fn,
        nadir_fn,
        nadir_guard_cls,
        steer_filter_cls,
        _track_ct,
    ) = _load_lane_keep()
    lane_keep_on = False
    last_lane_sig = ""
    lane_eyes = _empty_lane_eyes()
    yaw_rate = 0.0
    pose_anchor_xy: tuple[float, float] | None = None
    pose_anchor_yaw = prev_yaw
    pose_anchor_age_s = 0.0
    park_holdoff_s = 0.0
    drive_grace_s = 0.0
    wheel_enc_prev: dict[str, float] = {}
    wheel_rot_abs: dict[str, float] = {"left_wheel": 0.0, "right_wheel": 0.0}
    wheel_rot_since_stop: dict[str, float] = {"left_wheel": 0.0, "right_wheel": 0.0}
    wheel_lock_hold_s: dict[str, float] = {"left_wheel": 0.0, "right_wheel": 0.0}
    was_abs_active = False

    _apply_follow_camera(robot)
    steer_filter = None if steer_filter_cls is None else steer_filter_cls()
    nadir_guard = None if nadir_guard_cls is None else nadir_guard_cls()
    eye_huds = _label_eye_huds(robot, cams)
    nadir_steer_wanted = os.path.isfile(_NADIR_STEER_ON)
    nadir_lobe_done = False
    nadir_logged = False
    print(f"ButlerBot controller started — twin → {dashboard}/api/twin/telemetry")
    print(f"Battery synced from dashboard: {battery_pct:.1f}%")
    print("Teleop: Arrow keys or I/J/K/L — Space = stop. Click the FLOOR (not the robot)")
    print("Or use Dashboard: Drive Forward (API) under the twin panel")
    for _w in ("left_wheel", "right_wheel"):
        _enable_full_wheel_torque(motors[_w])
        motors[_w].setPosition(float("inf"))
        motors[_w].setVelocity(0.0)

    while robot.step(timestep) != -1:
        try:
            tick += 1
            dt = timestep / 1000.0
            if tick == 5 and nadir_steer_wanted and not lane_keep_on:
                lane_keep_on = True
                v_scale = max(1.0, min(2.2, (_NADIR_CRUISE * 0.08) / 0.21))
                print(
                    "NADIR STEER ON — pixel fan vs 32/29, "
                    f"cruise={_NADIR_CRUISE} rad/s ({_NADIR_CRUISE * 0.08:.2f} m/s), "
                    f"v-scale={v_scale:.2f}, "
                    f"full S to x={_FINISH_X_M} m. Two shoulder cams on the wheel."
                )
            if tick == 25 and os.path.isfile(_NADIR_PLACE_CHECK):
                cam = cams.get("nadir_left")
                if cam is not None:
                    try:
                        path = os.path.join(
                            _NADIR_SNAP_DIR, "butlerbot", "images", "nadir-left-place-check.png"
                        )
                        os.makedirs(os.path.dirname(path), exist_ok=True)
                        cam.saveImage(path, 90)
                        print(f"NADIR place-check snapshot → {path}")
                    except Exception as exc:
                        print(f"NADIR place-check snapshot skipped: {exc}")
                try:
                    os.remove(_NADIR_PLACE_CHECK)
                except OSError:
                    pass
            if park_holdoff_s > 0.0:
                park_holdoff_s = max(0.0, park_holdoff_s - dt)
            if drive_grace_s > 0.0:
                drive_grace_s = max(0.0, drive_grace_s - dt)

            if keyboard is not None:
                keys, pressed = key_tracker.poll(keyboard)
            else:
                keys, pressed = set(), set()

            if KEY_R in pressed:
                auto_loop = not auto_loop
                print(f"Auto mission loop: {'ON' if auto_loop else 'OFF'}")

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
                key_tracker.cancel_drive_keys()
                keys = key_tracker.active_keys()
            if abs_brake.active:
                key_tracker.cancel_drive_keys()
                keys = key_tracker.active_keys()
            teleop_keys = keys - {KEY_R, KEY_SPACE}
            stop_pressed = KEY_SPACE in pressed
            if teleop_keys and not settled and not abs_brake.active:
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

            if tick % state_poll_every == 0:
                twin_state = fetch_twin_state(dashboard)
                poll_ok = isinstance(twin_state, dict) and bool(twin_state)
                if poll_ok:
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
                    if nadir_steer_wanted and not nadir_lobe_done:
                        lane_keep_on = True
                    else:
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
                        was_api_driving = (
                            abs(cached_api_left) > 0.01 or abs(cached_api_right) > 0.01
                        )
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

            if nadir_fn is not None:
                lane_eyes = _harvest_nadir(cams, nadir_fn, gps_xy, prev_yaw)
                if not nadir_logged and (
                    lane_eyes.get("nadir_gap_px") is not None
                    or lane_eyes.get("nadir_r_gap_px") is not None
                ):
                    nadir_logged = True
                    print(
                        "Nadir L "
                        f"{lane_eyes.get('nadir_gap_px')} px / "
                        f"{lane_eyes.get('nadir_lateral_m')} m "
                        f"(stripe {lane_eyes.get('nadir_stripe_px')} px = 6 cm)"
                    )
                    print(
                        "Nadir R "
                        f"{lane_eyes.get('nadir_r_gap_px')} px / "
                        f"{lane_eyes.get('nadir_r_lateral_m')} m "
                        f"(stripe {lane_eyes.get('nadir_r_stripe_px')} px = 6 cm) "
                        "— both shoulders on the wheel"
                    )

            if (
                lane_keep_on
                and lane_keep_fn is not None
                and not abs_brake.active
                and not user_driving
                and park_holdoff_s <= 0.0
            ):
                lk = lane_keep_fn(
                    cruise=_NADIR_CRUISE,
                    k_steer=2.0,
                    steer_filter=steer_filter,
                    left_gap_px=lane_eyes.get("nadir_gap_px"),
                    right_gap_px=lane_eyes.get("nadir_r_gap_px"),
                    left_ahead_px=lane_eyes.get("nadir_ahead_px"),
                    right_ahead_px=lane_eyes.get("nadir_r_ahead_px"),
                    nadir_guard=nadir_guard,
                    nadir_primary=True,
                    dt=dt,
                )
                if (
                    nadir_steer_wanted
                    and gps_xy is not None
                    and float(gps_xy[0]) >= _FINISH_X_M
                ):
                    if not nadir_lobe_done:
                        nadir_lobe_done = True
                        print(
                            f"FULL S DONE at x={gps_xy[0]:.2f} y={gps_xy[1]:.2f} m — "
                            "stopping. Nadir was on the wheel."
                        )
                    lane_keep_on = False
                    lk = {
                        "left": 0.0,
                        "right": 0.0,
                        "brake": True,
                        "steer": 0.0,
                        "phase": "first_lobe_done",
                        "error_source": "nadir",
                    }
                lane_eyes["steer"] = lk.get("steer")
                lane_eyes["phase"] = lk.get("phase")
                lane_eyes["error_source"] = lk.get("error_source") or "nadir"
                if lk["brake"]:
                    print(
                        "Lane-keep: nadir stop "
                        f"{lk.get('reason')} "
                        f"nL={lane_eyes.get('nadir_gap_px')} "
                        f"nR={lane_eyes.get('nadir_r_gap_px')} "
                        f"v={speed_m_s:.2f}"
                    )
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
                        f"{lk.get('steer')}:"
                        f"{lane_eyes.get('nadir_gap_px')}:"
                        f"{lane_eyes.get('nadir_r_gap_px')}:"
                        f"{lane_eyes.get('nadir_ahead_px')}:"
                        f"{lane_eyes.get('nadir_r_ahead_px')}:"
                        f"{lk.get('phase')}"
                    )
                    if sig != last_lane_sig:
                        last_lane_sig = sig
                        print(
                            "Lane-keep "
                            f"L={lk['left']:.2f} R={lk['right']:.2f} "
                            f"steer={lk.get('steer')} "
                            f"nL={lane_eyes.get('nadir_gap_px')} "
                            f"nR={lane_eyes.get('nadir_r_gap_px')} "
                            f"aL={lane_eyes.get('nadir_ahead_px')} "
                            f"aR={lane_eyes.get('nadir_r_ahead_px')} "
                            f"src={lk.get('error_source')} "
                            f"phase={lk.get('phase')}"
                        )

            if tick % 4 == 0:
                _paint_eye_huds(eye_huds, lane_eyes)

            if stop_pressed:
                print("Keyboard stop — ABS braking")

            left_wv = left_wv_early
            right_wv = right_wv_early
            wheel_vels = {"left_wheel": left_wv, "right_wheel": right_wv}

            api_driving = abs(cached_api_left) > 0.01 or abs(cached_api_right) > 0.01
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
                        trans = math.hypot(dx, dy)
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
                    _hard_zero_wheels(
                        motors,
                        sensors,
                        left_wv=left_wv,
                        right_wv=right_wv,
                        track=False,
                    )
                _hold_neutral_upper_body(motors)

            if user_driving or api_driving or abs_brake.active or auto_loop:
                pose_anchor_xy = gps_xy
                pose_anchor_yaw = prev_yaw
                pose_anchor_age_s = 0.0
                if api_driving or user_driving:
                    drive_grace_s = max(drive_grace_s, 2.5)

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
                gait = "drive"
                phase_name = "teleop"
            motion_factor = 1.0 if (moving or turning) else 0.0
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
                _split = abs(left_v - right_v) > 0.2
                _apply_soft_grip(
                    robot,
                    left_wv=left_v,
                    right_wv=right_v,
                    imu=imu,
                    gain=1.0 if _split else SOFT_GRIP_GAIN,
                    yaw_lock=_split,
                )

            if user_driving:
                drive_log_elapsed += dt
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
                    ct_s = ""
                    if _track_ct is not None and gps_xy is not None:
                        try:
                            ct_s = f" ct={_track_ct(gps_xy[0], gps_xy[1]):+.3f}"
                        except Exception:
                            ct_s = ""
                    print(
                        f"Driving L={left_v:.1f} R={right_v:.1f} "
                        f"hub={left_wv:.1f}/{right_wv:.1f} "
                        f"hdg={_imu_yaw(imu):.3f} "
                        f"@ {_format_pose(gps)} "
                        f"{ct_s} "
                        f"gps={avg_body:.2f} odo={odo_v:.2f} m/s"
                        f"{throttle_note}"
                    )
            else:
                drive_log_elapsed = 0.0
                if hasattr(speed_estimator, "_log_anchor"):
                    speed_estimator._log_anchor = None  # type: ignore[attr-defined]

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

            hub_rates = {
                "left_wheel": left_wv,
                "right_wheel": right_wv,
            }
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
                        while dth > math.pi:
                            dth -= 2.0 * math.pi
                        while dth < -math.pi:
                            dth += 2.0 * math.pi
                        adth = abs(dth)
                        wheel_rot_abs[wname] = wheel_rot_abs.get(wname, 0.0) + adth
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
                if abs(rate) < STOP_WHEEL_RAD_S * 1.5:
                    wheel_lock_hold_s[wname] = wheel_lock_hold_s.get(wname, 0.0) + dt
                else:
                    wheel_lock_hold_s[wname] = 0.0

            wheel_locked = {
                w: wheel_lock_hold_s.get(w, 0.0) >= 0.25
                for w in ("left_wheel", "right_wheel")
            }
            wheels_both_locked = all(wheel_locked.values())

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
                try:
                    sim_t = float(robot.getTime())
                except Exception:
                    sim_t = 0.0
                odo_v = 0.5 * (float(left_wv) + float(right_wv)) * WHEEL_RADIUS_M
                cmd_v = 0.5 * (float(left_v) + float(right_v)) * WHEEL_RADIUS_M
                control_diag = {
                    "schema": "butlerbot_control_diag_v1",
                    "sim_time_s": round(sim_t, 4),
                    "hub_left_rad_s": round(left_wv, 4),
                    "hub_right_rad_s": round(right_wv, 4),
                    "hub_diff_rad_s": round(abs(left_wv - right_wv), 4),
                    "yaw_rate_rad_s": round(yaw_rate, 4),
                    "gps_speed_m_s": round(raw_speed_m_s, 4),
                    "gps_forward_m_s": round(raw_forward_m_s, 4),
                    "odo_speed_m_s": round(odo_v, 4),
                    "cmd_speed_m_s": round(cmd_v, 4),
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
                        "nadir_lateral_m": lane_eyes.get("nadir_lateral_m"),
                        "nadir_r_lateral_m": lane_eyes.get("nadir_r_lateral_m"),
                        "nadir_gap_px": lane_eyes.get("nadir_gap_px"),
                        "nadir_r_gap_px": lane_eyes.get("nadir_r_gap_px"),
                        "nadir_ahead_px": lane_eyes.get("nadir_ahead_px"),
                        "nadir_r_ahead_px": lane_eyes.get("nadir_r_ahead_px"),
                        "metric_active": False,
                        "error_source": lane_eyes.get("error_source") or "nadir",
                        "lane_phase": lane_eyes.get("phase"),
                        "steer": lane_eyes.get("steer"),
                        "agent_throttle": throttle_factor,
                        "braking": abs_brake.active,
                        "residual_spin": residual_spin,
                        "control_diag": control_diag,
                        "wheels": wheel_sensors,
                    },
                )
                if lane_keep_on:
                    _log_steer_action(payload.get("pose") or {}, lane_eyes)
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
            if cmd_v > abs(velocity):
                velocity = math.copysign(cmd_v, velocity if abs(velocity) > 1e-6 else cmd_v)
        if (
            "wheel" in motor_name
            and abs(velocity) < 0.15
            and speed_m_s > 0.06
            and cmd_v > 0.05
        ):
            velocity = speed_m_s / max(WHEEL_RADIUS_M, 0.02)

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
                "power_w": 2.0,
            })
            continue

        torque_est = abs(velocity) * 0.45
        torque = torque_est
        if hasattr(motor, "getTorqueFeedback"):
            try:
                tf = abs(float(motor.getTorqueFeedback()))
                if tf > 1e-4:
                    torque = tf
            except Exception:
                pass
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
