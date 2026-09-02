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

This file is the **orchestrator** (``_run_loop`` + device init + publish).
Siblings in this folder (2026-08-17 split, do not re-merge):

* ``controller_hud.py`` — Display gauges + eye overlays
* ``controller_keys.py`` — keyboard teleop
* ``controller_eyes.py`` — cameras, aim, LINE_CAM identity lock
* ``controller_wheels.py`` — ABS / residual-spin / soft-grip
* ``twin_publisher.py`` — telemetry HTTP

Residual spin / ABS notes live on ``controller_wheels`` (read before
changing brake code): GPS is translation-only; never ``setPosition(NaN)``;
dual-hub hard-zero + IMU yaw-rate; ``stop_epoch`` from the dashboard.
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

from controller_hud import (
    HUD_H,
    HUD_W,
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
from controller_eyes import (
    _aim_forecast_cameras,
    _aim_lookdown_cameras,
    _camera_max_rgb,
    _consume_nadir_shove_flag,
    _consume_sidelook_diag_flag,
    _load_lookdown_math,
    _maybe_dump_viewpoint,
    _read_lane_eyes,
    _revert_line_cams_identity,
    _run_lookdown_interlock,
    _run_preview_interlock,
    _run_nadir_shove_diag,
    _run_sidelook_offset_diag,
    _nadir_lateral_from_cam,
    _nadir_shove_requested,
    _sample_wall_pair,
    _sidelook_diag_requested,
    _sidelook_enabled,
    _yellow_ground_y,
    _LOOKDOWN_SNAP_DIR,
)
from controller_wheels import (
    MAX_JOINT_V,
    MAX_WHEEL_V,
    MOTOR_MAX_VELOCITY,
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
    _wheel_rad_s,
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
        from src.lane_keep import (
            lane_keep_command,
            offset_to_column,
            peak_score_bgra,
            red_score,
            yellow_band_fill,
            yellow_line_curve,
            yellow_line_offset,
            yellow_look_band,
            yellow_score,
        )

        return (
            lane_keep_command,
            yellow_score,
            red_score,
            peak_score_bgra,
            yellow_line_offset,
            yellow_look_band,
            offset_to_column,
            yellow_line_curve,
            yellow_band_fill,
        )
    except Exception as exc:
        print(f"WARNING: lane_keep policy not loaded ({exc})")
        return None, None, None, None, None, None, None, None, None

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
    """Append one GPS-tagged steer sample. Same facts the dashboard log uses."""
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
                    fh.write(
                        "unix_s,x_m,y_m,steer,oL,oR,yL,yR,red,"
                        "lDist,rDist,mCt,src\n"
                    )
            _steer_log_ready = True
        with open(_STEER_LOG, "a", encoding="ascii") as fh:
            fh.write(
                f"{time.time():.3f},{x},{y},{float(steer):.4f},"
                f"{lane_eyes.get('left_offset')},{lane_eyes.get('right_offset')},"
                f"{lane_eyes.get('left_yellow')},{lane_eyes.get('right_yellow')},"
                f"{lane_eyes.get('finish_red')},"
                f"{lane_eyes.get('left_wall_dist_m')},"
                f"{lane_eyes.get('right_wall_dist_m')},"
                f"{lane_eyes.get('metric_ct')},"
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


# Chase from behind and up. Daniel mouse-framed 2026-08-16 (not exact 45 deg).
# Position is the live follow offset; Tracking Shot walks with him.
_CAM_EYE = (-3.18573, -0.03020, 3.54536)
_CAM_LOOK = (0.0, 0.0, 0.55)
_CAM_FOV = 0.85
_CAM_ORIENTATION = (-0.001250, 0.999989, 0.004550, 0.536256)
# Back / eye-level, for checking shoulder-boom placement vs Daniel's sketch.
_BACK_EYE = (-1.70, 0.0, 0.55)
_BACK_ORIENTATION = (0.0, 1.0, 0.0, -1.5708)
_BACK_FOV = 0.90
_NADIR_PLACE_CHECK = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "NADIR_PLACE_CHECK"
)
_NADIR_STEER_ON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "NADIR_STEER_ON"
)
_FIRST_LOBE_X_M = None  # full S this sitting; was 8.0 for the plant probe
_FINISH_X_M = 24.5
_NADIR_CRUISE = 5.5  # 0.44 m/s — old picture-wins champ speed; nadir cleared 3.7 cm at 2.5


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
            f"(Daniel chase) eye={_CAM_EYE} fov={_CAM_FOV}"
        )
    except Exception as exc:
        print(f"Follow-cam reset skipped: {exc}")


def _apply_back_eye_view(robot: Robot) -> None:
    """Stand behind the robot at eye level. For shoulder-boom placement checks."""
    if Supervisor is None or not isinstance(robot, Supervisor):
        return
    try:
        vp = robot.getFromDef("VIEWPOINT")
    except Exception:
        vp = None
    if vp is None:
        return
    try:
        field = vp.getField
        follow = field("follow")
        if follow:
            follow.setSFString("")
        pos = field("position")
        if pos:
            pos.setSFVec3f(list(_BACK_EYE))
        ori = field("orientation")
        if ori:
            ori.setSFRotation(list(_BACK_ORIENTATION))
        fov = field("fieldOfView")
        if fov:
            fov.setSFFloat(_BACK_FOV)
        print(
            "Viewpoint: back/eye-level for nadir place-check "
            f"eye={_BACK_EYE}"
        )
    except Exception as exc:
        print(f"Back-eye view skipped: {exc}")


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
    for name in (
        "line_left",
        "line_right",
        "finish_cam",
        "finish_cam_r",
        "forecast_z",
        "forecast_w",
        "nadir_left",
        "nadir_right",
    ):
        cam = None
        try:
            cam = robot.getDevice(name)
        except Exception:
            cam = None
        # Shoulder nadirs only. Enabling the rest is the 0.08× molasses.
        if cam is not None and name in ("nadir_left", "nadir_right"):
            cam.enable(timestep)
        cams[name] = cam if name in ("nadir_left", "nadir_right") else None
        if name in ("nadir_left", "nadir_right") and cams[name] is None:
            print(f"WARNING: camera '{name}' not found — lane-keep eye missing")
    print("NADIR EYES ONLY — line/finish/forecast cameras not enabled")
    if _sidelook_enabled():
        for name in ("line_side_l", "line_side_r"):
            cam = None
            try:
                cam = robot.getDevice(name)
                if cam is not None:
                    cam.enable(timestep)
            except Exception:
                cam = None
            cams[name] = cam
            if cam is None:
                print(f"WARNING: experimental camera '{name}' missing")
        print("SIDELOOK experiment ON — observation only, not on the wheel")

    return motors, sensors, gps, gps_head, imu, keyboard, hud, cams


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
    (
        lane_keep_fn,
        yellow_fn,
        red_fn,
        peak_fn,
        offset_fn,
        band_fn,
        col_fn,
        curve_fn,
        fill_fn,
    ) = _load_lane_keep()
    lane_keep_on = False
    both_eyes_seen = False
    last_lane_sig = ""
    lane_eyes = {
        "left_yellow": 0.0,
        "right_yellow": 0.0,
        "left_offset": None,
        "right_offset": None,
        "finish_red": 0.0,
        "finish_red_row": None,
    }
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
    lookdown_math = _load_lookdown_math()
    steer_filter = None
    gap_planner = None
    _track_ct = None
    try:
        from src.lane_keep import (
            GapPlanner as _GapPlanner,
            SteerFilter as _SteerFilter,
            track_cross_track_m as _track_ct,
        )

        steer_filter = _SteerFilter()
        gap_planner = _GapPlanner()
    except Exception:
        steer_filter = None
        gap_planner = None
        _track_ct = None
    pixel_fn = None
    ground_fn = None
    corridor_watch = None
    forecast_lookout = None
    try:
        from src.lane_keep import (
            CorridorWatch as _CorridorWatch,
            ForecastLookout as _ForecastLookout,
            pixel_to_ground_m as _pixel_to_ground_m,
            yellow_ahead_pixel as _yellow_ahead_pixel,
        )

        pixel_fn = _yellow_ahead_pixel
        ground_fn = _pixel_to_ground_m
        corridor_watch = _CorridorWatch()
        forecast_lookout = _ForecastLookout()
    except Exception:
        pixel_fn = None
        ground_fn = None
        corridor_watch = None
        forecast_lookout = None
    look_ahead_m = 1.09
    # Skip look-down / forecast / LINE_CAM aim — those cameras are off.
    eye_huds = _label_eye_huds(robot, cams)
    classify_fn = None if not lookdown_math else lookdown_math["classify"]
    mark_stop = lookdown_math["tracker_cls"]() if lookdown_math else None
    lookdown_view = "pending"
    preview_state = {"aim_ok": False, "preview_ok": False, "range_m": None}
    preview_checked = False
    far_fn = None
    preview_range_fn = None
    t_mark_fn = None
    forecast_hit_fn = None
    line_wall_fn = None
    nadir_fn = None
    nadir_guard = None
    metric_ok_fn = None
    metric_logged = False
    nadir_logged = False
    try:
        from src.lane_keep import (
            preview_look_ahead_m as _preview_look_ahead_m,
            time_to_mark_s as _time_to_mark_s,
            yellow_far_offset as _yellow_far_offset,
            forecast_wall_hit as _forecast_wall_hit,
            line_wall_hit as _line_wall_hit,
            metric_walls_plausible as _metric_ok,
            nadir_wheel_to_tape as _nadir_wheel_to_tape,
            NadirGuard as _NadirGuard,
        )

        far_fn = _yellow_far_offset
        preview_range_fn = _preview_look_ahead_m
        t_mark_fn = _time_to_mark_s
        forecast_hit_fn = _forecast_wall_hit
        line_wall_fn = _line_wall_hit
        metric_ok_fn = _metric_ok
        nadir_fn = _nadir_wheel_to_tape
        nadir_guard = _NadirGuard()
    except Exception:
        far_fn = None
        preview_range_fn = None
        t_mark_fn = None
        forecast_hit_fn = None
        line_wall_fn = None
        metric_ok_fn = None
        nadir_fn = None
        nadir_guard = None
    nadir_steer_wanted = os.path.isfile(_NADIR_STEER_ON)
    nadir_lobe_done = False
    # Last Z/W fills + hits. Refreshed every 8 steps (~10 Hz); lookout
    # still steps at 8 ms on these cached fills.
    zw_cache = {
        "z_fill": None,
        "w_fill": None,
        "z_offset": None,
        "w_offset": None,
        "z_hit": None,
        "w_hit": None,
    }
    mark_plan = {"phase": "seek", "remaining_m": None, "t_to_mark_s": None}
    finish_cam_node = None
    line_cam_l_node = None
    line_cam_r_node = None
    side_cam_l_node = None
    side_cam_r_node = None
    nadir_cam_l_node = None
    forecast_z_node = None
    forecast_w_node = None
    sidelook_on = _sidelook_enabled()
    sidelook_diag_done = False
    nadir_shove_done = False
    if Supervisor is not None and isinstance(robot, Supervisor):
        try:
            finish_cam_node = robot.getFromDef("FINISH_CAM")
        except Exception:
            finish_cam_node = None
        try:
            line_cam_l_node = robot.getFromDef("LINE_CAM_L")
            line_cam_r_node = robot.getFromDef("LINE_CAM_R")
        except Exception:
            line_cam_l_node = None
            line_cam_r_node = None
        if sidelook_on:
            try:
                side_cam_l_node = robot.getFromDef("SIDE_CAM_L")
                side_cam_r_node = robot.getFromDef("SIDE_CAM_R")
            except Exception:
                side_cam_l_node = None
                side_cam_r_node = None
        try:
            nadir_cam_l_node = robot.getFromDef("NADIR_CAM_L")
        except Exception:
            nadir_cam_l_node = None
        try:
            forecast_z_node = robot.getFromDef("FORECAST_CAM_Z")
            forecast_w_node = robot.getFromDef("FORECAST_CAM_W")
        except Exception:
            forecast_z_node = None
            forecast_w_node = None
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
            if tick == 5 and nadir_steer_wanted and not lane_keep_on:
                lane_keep_on = True
                print(
                    "NADIR STEER ON — pixel fan vs 32/29, "
                    f"cruise={_NADIR_CRUISE} rad/s ({_NADIR_CRUISE * 0.08:.2f} m/s), "
                    f"v-scale={max(1.0, min(2.2, (_NADIR_CRUISE * 0.08) / 0.21)):.2f}, "
                    f"full S to x={_FINISH_X_M} m. Two shoulder cams only."
                )
            if tick == 25 and lookdown_view == "pending":
                lookdown_view = "nadir_only"
                preview_checked = True
                if (
                    sidelook_on
                    and not sidelook_diag_done
                    and _sidelook_diag_requested()
                ):
                    _run_sidelook_offset_diag(
                        robot,
                        timestep,
                        cams,
                        motors,
                        line_cam_l_node,
                        line_cam_r_node,
                        side_cam_l_node,
                        side_cam_r_node,
                        line_wall_fn,
                        gps,
                    )
                    _consume_sidelook_diag_flag()
                    sidelook_diag_done = True
                if os.path.isfile(_NADIR_PLACE_CHECK):
                    cam = cams.get("nadir_left")
                    if cam is not None:
                        try:
                            path = os.path.join(
                                _LOOKDOWN_SNAP_DIR,
                                "butlerbot",
                                "images",
                                "nadir-left-place-check.png",
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
                if not nadir_shove_done and _nadir_shove_requested():
                    _run_nadir_shove_diag(
                        robot,
                        timestep,
                        cams,
                        motors,
                        nadir_cam_l_node,
                        nadir_fn,
                        gps,
                    )
                    _consume_nadir_shove_flag()
                    nadir_shove_done = True
                    if nadir_steer_wanted:
                        lane_keep_on = True
                        print(
                            "NADIR STEER ON — pixel fan vs 32/29, "
                            f"cruise={_NADIR_CRUISE} rad/s, full S to "
                            f"x={_FINISH_X_M} m. Picture-wins is not on the wheel."
                        )
                elif nadir_steer_wanted and not lane_keep_on:
                    lane_keep_on = True
                    print(
                        "NADIR STEER ON — pixel fan vs 32/29, "
                        f"cruise={_NADIR_CRUISE} rad/s, full S to "
                        f"x={_FINISH_X_M} m. Picture-wins is not on the wheel."
                    )
                if not preview_state.get("aim_ok"):
                    for key, fname in (
                        ("line_left", "line-left-identity.png"),
                        ("line_right", "line-right-identity.png"),
                    ):
                        cam = cams.get(key)
                        if cam is None:
                            continue
                        try:
                            path = os.path.join(_LOOKDOWN_SNAP_DIR, fname)
                            cam.saveImage(path, 90)
                            print(f"Identity snapshot {key} → {path}")
                        except Exception as exc:
                            print(f"Identity snapshot {key} skipped: {exc}")
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
                    was_lane = lane_keep_on
                    if nadir_steer_wanted and not nadir_lobe_done:
                        lane_keep_on = True
                    else:
                        lane_keep_on = bool(api_cmd.get("lane_keep"))
                    if mark_stop is not None and was_lane != lane_keep_on:
                        mark_stop.reset()
                        mark_plan = {"phase": "seek", "remaining_m": None, "t_to_mark_s": None}
                    if lane_keep_on and not was_lane:
                        both_eyes_seen = False
                        if corridor_watch is not None:
                            corridor_watch.reset()
                        if forecast_lookout is not None:
                            forecast_lookout.reset()
                        if gap_planner is not None:
                            gap_planner.reset()

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
                scan_zw = tick % 8 == 0
                lane_eyes = _read_lane_eyes(
                    cams,
                    yellow_fn,
                    red_fn,
                    peak_fn,
                    offset_fn,
                    curve_fn,
                    fill_fn,
                    far_fn,
                    scan_forecast=scan_zw,
                )
                if scan_zw:
                    zw_cache["z_fill"] = lane_eyes.get("z_fill")
                    zw_cache["w_fill"] = lane_eyes.get("w_fill")
                    zw_cache["z_offset"] = lane_eyes.get("z_offset")
                    zw_cache["w_offset"] = lane_eyes.get("w_offset")
                else:
                    lane_eyes["z_fill"] = zw_cache["z_fill"]
                    lane_eyes["w_fill"] = zw_cache["w_fill"]
                    lane_eyes["z_offset"] = zw_cache["z_offset"]
                    lane_eyes["w_offset"] = zw_cache["w_offset"]
                if (
                    float(lane_eyes.get("left_yellow") or 0.0) >= 0.12
                    and float(lane_eyes.get("right_yellow") or 0.0) >= 0.12
                ):
                    both_eyes_seen = True
                lane_eyes["left_y_m"] = _yellow_ground_y(
                    cams.get("line_left"), line_cam_l_node, pixel_fn, ground_fn
                )
                lane_eyes["right_y_m"] = _yellow_ground_y(
                    cams.get("line_right"), line_cam_r_node, pixel_fn, ground_fn
                )
                ident = _sample_wall_pair(
                    cams,
                    "line_left",
                    "line_right",
                    line_cam_l_node,
                    line_cam_r_node,
                    gps_xy,
                    prev_yaw,
                    line_wall_fn,
                )
                lane_eyes["left_wall_dist_m"] = ident["left"]
                lane_eyes["right_wall_dist_m"] = ident["right"]
                lane_eyes["metric_ct"] = ident["ct"]
                lane_eyes["left_wall_col"] = ident["l_col"]
                lane_eyes["right_wall_col"] = ident["r_col"]
                exp = {
                    "left": None,
                    "right": None,
                    "ct": None,
                    "ok": False,
                }
                if sidelook_on:
                    exp = _sample_wall_pair(
                        cams,
                        "line_side_l",
                        "line_side_r",
                        side_cam_l_node,
                        side_cam_r_node,
                        gps_xy,
                        prev_yaw,
                        line_wall_fn,
                    )
                lane_eyes["exp_left_wall_dist_m"] = exp["left"]
                lane_eyes["exp_right_wall_dist_m"] = exp["right"]
                lane_eyes["exp_metric_ct"] = exp["ct"]
                lane_eyes["exp_metric_ok"] = bool(exp["ok"])
                nadir_hit = _nadir_lateral_from_cam(
                    cams.get("nadir_left"),
                    nadir_cam_l_node,
                    gps_xy,
                    prev_yaw,
                    nadir_fn,
                    side="left",
                )
                nadir_r = _nadir_lateral_from_cam(
                    cams.get("nadir_right"),
                    None,
                    gps_xy,
                    prev_yaw,
                    nadir_fn,
                    side="right",
                )
                if nadir_hit:
                    lane_eyes["nadir_lateral_m"] = nadir_hit.get("m")
                    lane_eyes["nadir_gap_px"] = nadir_hit.get("gap_px")
                    lane_eyes["nadir_ahead_px"] = nadir_hit.get("gap_ahead_px")
                    lane_eyes["nadir_stripe_px"] = nadir_hit.get("stripe_px")
                    lane_eyes["nadir_tape_col"] = nadir_hit.get("tape_col")
                    lane_eyes["nadir_wheel_col"] = nadir_hit.get("wheel_col")
                else:
                    lane_eyes["nadir_lateral_m"] = None
                    lane_eyes["nadir_gap_px"] = None
                    lane_eyes["nadir_ahead_px"] = None
                if nadir_r:
                    lane_eyes["nadir_r_lateral_m"] = nadir_r.get("m")
                    lane_eyes["nadir_r_gap_px"] = nadir_r.get("gap_px")
                    lane_eyes["nadir_r_ahead_px"] = nadir_r.get("gap_ahead_px")
                    lane_eyes["nadir_r_stripe_px"] = nadir_r.get("stripe_px")
                    lane_eyes["nadir_r_tape_col"] = nadir_r.get("tape_col")
                    lane_eyes["nadir_r_wheel_col"] = nadir_r.get("wheel_col")
                else:
                    lane_eyes["nadir_r_lateral_m"] = None
                    lane_eyes["nadir_r_gap_px"] = None
                    lane_eyes["nadir_r_ahead_px"] = None
                # Production stays on picture-wins. Nadir is log-only
                # until the 20 cm shove probe validates live.
                lane_eyes["metric_active"] = False
                lane_eyes["error_source"] = "picture"
                if not nadir_logged and (nadir_hit or nadir_r):
                    nadir_logged = True
                    print(
                        "Nadir L "
                        f"{None if not nadir_hit else nadir_hit.get('gap_px')} px / "
                        f"{None if not nadir_hit else nadir_hit.get('m')} m "
                        f"(stripe {None if not nadir_hit else nadir_hit.get('stripe_px')} px = 6 cm)"
                    )
                    print(
                        "Nadir R "
                        f"{None if not nadir_r else nadir_r.get('gap_px')} px / "
                        f"{None if not nadir_r else nadir_r.get('m')} m "
                        f"(stripe {None if not nadir_r else nadir_r.get('stripe_px')} px = 6 cm; "
                        "observation, not on the wheel)"
                    )
                if not metric_logged and (
                    ident["left"] is not None
                    or exp["left"] is not None
                ):
                    metric_logged = True
                    print(
                        "Metric walls id "
                        f"L={ident['left']} R={ident['right']} ct={ident['ct']} "
                        f"| exp L={exp['left']} R={exp['right']} "
                        f"ct={exp['ct']} ok={int(bool(exp['ok']))} "
                        "(exp observation only)"
                    )
                z_hit = w_hit = None
                if forecast_hit_fn is not None and scan_zw:
                    xy = gps_xy
                    yaw = prev_yaw
                    for label, cam, node in (
                        ("z", cams.get("forecast_z"), forecast_z_node),
                        ("w", cams.get("forecast_w"), forecast_w_node),
                    ):
                        if cam is None or node is None:
                            continue
                        try:
                            img = cam.getImage()
                            hit = forecast_hit_fn(
                                img,
                                int(cam.getWidth()),
                                int(cam.getHeight()),
                                tuple(node.getField("translation").getSFVec3f()),
                                tuple(node.getField("rotation").getSFRotation()),
                                float(cam.getFov()),
                                robot_xy=xy,
                                yaw_rad=yaw,
                            )
                        except Exception:
                            hit = None
                        if label == "z":
                            z_hit = hit
                        else:
                            w_hit = hit
                    zw_cache["z_hit"] = z_hit
                    zw_cache["w_hit"] = w_hit
                else:
                    z_hit = zw_cache["z_hit"]
                    w_hit = zw_cache["w_hit"]
                z_y = None if not z_hit else z_hit.get("y_m")
                w_y = None if not w_hit else w_hit.get("y_m")
                aheads = []
                if z_hit and z_hit.get("ahead_m") is not None:
                    aheads.append(float(z_hit["ahead_m"]))
                if w_hit and w_hit.get("ahead_m") is not None:
                    aheads.append(float(w_hit["ahead_m"]))
                live_range = None if not aheads else sum(aheads) / len(aheads)
                forecast_ok = z_y is not None or w_y is not None
                lane_eyes["z_y_m"] = None if z_y is None else round(float(z_y), 3)
                lane_eyes["w_y_m"] = None if w_y is None else round(float(w_y), 3)
                lane_eyes["forecast_ok"] = forecast_ok
                # HUD / log only. Forecast hits are 12 cm of floor, not preview.
                lane_eyes["preview_ok"] = bool(preview_state.get("preview_ok"))
                t_ahead = None
                if lane_eyes.get("preview_ok") and float(speed_m_s) >= 0.05:
                    t_ahead = 2.0
                lane_eyes["t_ahead"] = t_ahead
                lane_eyes["preview_range_m"] = live_range

            if (
                lane_keep_on
                and lane_keep_fn is not None
                and not abs_brake.active
                and not user_driving
                and park_holdoff_s <= 0.0
            ):
                if mark_stop is not None:
                    measured = None
                    row = lane_eyes.get("finish_red_row")
                    eye = cams.get("finish_cam")
                    if (
                        lookdown_math
                        and finish_cam_node is not None
                        and row is not None
                        and eye is not None
                        and lane_eyes["finish_red"] >= 0.28
                    ):
                        try:
                            cam_pos = tuple(finish_cam_node.getField("translation").getSFVec3f())
                            cam_rot = tuple(finish_cam_node.getField("rotation").getSFRotation())
                            measured = lookdown_math["row_range"](
                                cam_pos,
                                cam_rot,
                                float(row),
                                int(eye.getHeight()),
                                float(eye.getFov()),
                            )
                        except Exception:
                            measured = None
                    gx = 0.0
                    try:
                        gx = float(gps.getValues()[0])
                    except Exception:
                        gx = 0.0
                    # Magenta pucks / desert dirt can score "red" far from the stripe.
                    near_finish = gx >= 21.0
                    saw_red = bool(lane_eyes["finish_red"] >= 0.28 and near_finish)
                    stepped = mark_stop.step(
                        saw_red,
                        look_ahead_m,
                        speed_m_s,
                        dt,
                        measured_range_m=measured,
                    )
                    if stepped:
                        mark_plan = stepped
                gx = 0.0
                try:
                    gx = float(gps.getValues()[0])
                except Exception:
                    gx = 0.0
                finish_red = (
                    lane_eyes["finish_red"] if gx >= 21.0 else 0.0
                )
                watch_plan = None
                if corridor_watch is not None:
                    watch_plan = corridor_watch.step(
                        lane_eyes.get("left_y_m"),
                        lane_eyes.get("right_y_m"),
                        lane_eyes.get("steer"),
                        dt,
                    )
                lk = lane_keep_fn(
                    lane_eyes["left_yellow"],
                    lane_eyes["right_yellow"],
                    finish_red,
                    mark_plan=mark_plan,
                    cruise=_NADIR_CRUISE if nadir_steer_wanted else 5.5,
                    k_steer=2.0 if nadir_steer_wanted else 3.2,
                    left_offset=lane_eyes.get("left_offset"),
                    right_offset=lane_eyes.get("right_offset"),
                    left_curve=lane_eyes.get("left_curve"),
                    right_curve=lane_eyes.get("right_curve"),
                    left_fill=lane_eyes.get("left_fill"),
                    right_fill=lane_eyes.get("right_fill"),
                    left_y_m=lane_eyes.get("left_y_m"),
                    right_y_m=lane_eyes.get("right_y_m"),
                    left_far_offset=lane_eyes.get("left_far_offset"),
                    right_far_offset=lane_eyes.get("right_far_offset"),
                    t_ahead=lane_eyes.get("t_ahead"),
                    preview_ok=bool(lane_eyes.get("preview_ok")),
                    allow_one_eye=both_eyes_seen,
                    steer_filter=steer_filter,
                    planner=None if nadir_steer_wanted else gap_planner,
                    watch_plan=None if nadir_steer_wanted else watch_plan,
                    lookout=None if nadir_steer_wanted else forecast_lookout,
                    z_fill=lane_eyes.get("z_fill"),
                    w_fill=lane_eyes.get("w_fill"),
                    left_gap_px=lane_eyes.get("nadir_gap_px"),
                    right_gap_px=lane_eyes.get("nadir_r_gap_px"),
                    left_ahead_px=lane_eyes.get("nadir_ahead_px"),
                    right_ahead_px=lane_eyes.get("nadir_r_ahead_px"),
                    nadir_guard=nadir_guard,
                    nadir_primary=bool(nadir_steer_wanted),
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
                lane_eyes["left_pressure"] = lk.get("left_pressure")
                lane_eyes["right_pressure"] = lk.get("right_pressure")
                lane_eyes["metric_ct"] = lk.get("metric_ct")
                lane_eyes["metric_active"] = lk.get("metric_active")
                lane_eyes["error_source"] = lk.get("error_source")
                if lk["brake"]:
                    rem = lk.get("remaining_m")
                    if lk.get("phase") == "watch" or "watch" in str(
                        lk.get("reason") or ""
                    ):
                        print(
                            "Lane-keep: geometry watch — stop "
                            f"{lk.get('reason')} "
                            f"yLm={lane_eyes.get('left_y_m')} "
                            f"yRm={lane_eyes.get('right_y_m')}"
                        )
                    elif lk.get("phase") == "lookout" or "lookout" in str(
                        lk.get("reason") or ""
                    ):
                        print(
                            "Lane-keep: lookout — paint gone ahead "
                            f"zF={lane_eyes.get('z_fill')} "
                            f"wF={lane_eyes.get('w_fill')} "
                            f"yL={lane_eyes['left_yellow']:.2f} "
                            f"yR={lane_eyes['right_yellow']:.2f} "
                            f"v={speed_m_s:.2f}"
                        )
                    elif lk.get("phase") == "lost" or "lost paint" in str(
                        lk.get("reason") or ""
                    ):
                        print(
                            "Lane-keep: lost paint — stop "
                            f"x~ GPS yL={lane_eyes['left_yellow']:.2f} "
                            f"yR={lane_eyes['right_yellow']:.2f} "
                            f"v={speed_m_s:.2f}"
                        )
                    else:
                        print(
                            "Lane-keep: red mark — ABS to stop on line "
                            f"D={look_ahead_m:.2f}m rem={rem} "
                            f"row={lane_eyes.get('finish_red_row')} v={speed_m_s:.2f}"
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
                        f"{lk.get('phase')}:{lk.get('remaining_m')}"
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

            if yellow_fn and red_fn and peak_fn and tick % 4 == 0:
                _paint_eye_huds(eye_huds, lane_eyes, band_fn, col_fn)

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
                # Straight: blend v=ωr. Split: lock ω_z to the commanded
                # differential. Sticky drive-wheel contact (μ=80, no slip)
                # otherwise holds heading 0 and both hubs sit on min(cmd).
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
                        "left_offset": lane_eyes.get("left_offset"),
                        "right_offset": lane_eyes.get("right_offset"),
                        "left_y_m": lane_eyes.get("left_y_m"),
                        "right_y_m": lane_eyes.get("right_y_m"),
                        "left_wall_dist_m": lane_eyes.get("left_wall_dist_m"),
                        "right_wall_dist_m": lane_eyes.get("right_wall_dist_m"),
                        "metric_ct": lane_eyes.get("metric_ct"),
                        "metric_active": False,
                        "error_source": "picture",
                        "exp_left_wall_dist_m": lane_eyes.get("exp_left_wall_dist_m"),
                        "exp_right_wall_dist_m": lane_eyes.get("exp_right_wall_dist_m"),
                        "exp_metric_ct": lane_eyes.get("exp_metric_ct"),
                        "exp_metric_ok": lane_eyes.get("exp_metric_ok"),
                        "nadir_lateral_m": lane_eyes.get("nadir_lateral_m"),
                        "left_fill": lane_eyes.get("left_fill"),
                        "right_fill": lane_eyes.get("right_fill"),
                        "z_fill": lane_eyes.get("z_fill"),
                        "w_fill": lane_eyes.get("w_fill"),
                        "lane_phase": lane_eyes.get("phase"),
                        "steer": lane_eyes.get("steer"),
                        "finish_red": lane_eyes.get("finish_red"),
                        "lookdown_view": lookdown_view,
                        "look_ahead_m": round(look_ahead_m, 3),
                        "finish_red_row": lane_eyes.get("finish_red_row"),
                        "mark_remaining_m": None
                        if not mark_plan or mark_plan.get("remaining_m") is None
                        else round(float(mark_plan["remaining_m"]), 3),
                        "mark_phase": None if not mark_plan else mark_plan.get("phase"),
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