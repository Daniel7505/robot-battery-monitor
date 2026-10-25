"""Setup and wheel commands for the ButlerBot Webots controller.

Device enable, the BoM body reload, and the per-tick motor command
(ABS, API, keyboard, R-key phase, residual park).
"""

from __future__ import annotations

import math
import os

from controller import (
    Camera,
    Display,
    GPS,
    InertialUnit,
    Keyboard,
    Motor,
    PositionSensor,
    Robot,
)

try:
    from controller import Gyro
except ImportError:  # pragma: no cover
    Gyro = None  # type: ignore[misc, assignment]

from controller_hud import _label_eye_huds
from controller_inventory import InventoryRuntime
from controller_keys import KeyTracker, _teleop_drive
from controller_mission import (
    PHASES,
    _LANE_MODE,
    _RUN_ID,
    _load_finish_referee,
    _load_s_cross_track,
    _record_mission_lines,
)
from controller_obstacles import ObstacleRuntime, obstacles_mode as _obstacles_mode
from controller_rowfit import (
    LaneVisionLog,
    RowfitRuntime,
    world_check_warnings,
)
from controller_sensing import (
    _apply_follow_camera,
    _empty_lane_eyes,
    _imu_yaw,
    _load_balance_cfg,
)
from controller_wheels import (
    _safe_imu_pitch,
    MAX_JOINT_V,
    MAX_WHEEL_V,
    POSE_RESIDUAL_TRANS_M,
    POSE_RESIDUAL_WINDOW_S,
    POSE_RESIDUAL_YAW_RAD,
    RESIDUAL_SPEED_M_S,
    STOP_WHEEL_DIFF_RAD_S,
    STOP_WHEEL_RAD_S,
    STOP_YAW_RATE_RAD_S,
    WHEEL_RADIUS_M,
    AbsBrakeController,
    SpeedEstimator,
    _clamp,
    _clamp_motor_vel,
    _clear_wheel_locks,
    _enable_full_wheel_torque,
    _halt_wheels,
    _hard_zero_wheels,
    bind_drive_radius,
)
from twin_publisher import battery_from_twin_state, fetch_twin_state

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

_NEUTRAL_TORSO = 0.0
_NEUTRAL_LEFT_ARM = 0.14
_NEUTRAL_RIGHT_ARM = -0.14

MOTOR_NAMES = ["left_wheel", "right_wheel", "torso_joint", "left_arm", "right_arm"]
SENSOR_NAMES = [
    "left_wheel_sensor", "right_wheel_sensor", "torso_sensor",
    "left_arm_sensor", "right_arm_sensor",
]

# Physics / robot.step stay at basicTimeStep (8 ms). Nadir Camera.enable
# is the OpenGL pass. Fair trade: 128×128 @ 320 ms (~3 Hz, ~14 cm at
# 0.44 m/s). Harvest only on a new frame. Next sitting: 640 ms.
_NADIR_CAM_PERIOD_MS = 320

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
    gyro = None
    if Gyro is not None:
        try:
            gyro = robot.getDevice("gyro")
            gyro.enable(timestep)
        except Exception:
            gyro = None
    gyro_found = gyro is not None
    if gyro_found:
        print("Gyro: device found and enabled (pivot rate + integrated heading)")
    elif Gyro is not None:
        print(
            "Gyro: device not found — pivot falls back to IMU yaw "
            "(open-loop rate tracking; no gyro feedback)"
        )
    else:
        print(
            "Gyro: API unavailable in this Webots build — pivot open-loop (IMU yaw only)"
        )

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
        print("Keyboard device unavailable (debug key input off)")

    hud: Display | None = None
    try:
        hud = robot.getDevice("hud")
    except Exception:
        hud = None

    cams: dict[str, Camera | None] = {}
    cam_period = max(int(timestep), int(_NADIR_CAM_PERIOD_MS))
    cam_period = int(round(cam_period / timestep) * timestep)
    for name in ("nadir_left", "nadir_right"):
        cam = None
        try:
            cam = robot.getDevice(name)
        except Exception:
            cam = None
        if cam is not None:
            cam.enable(cam_period)
        cams[name] = cam
        if cam is None:
            print(f"WARNING: camera '{name}' not found — lane-keep eye missing")
    print(
        "NADIR EYES ONLY — line/finish/forecast/sidelook not enabled. "
        f"Camera {128}×{128} sample {cam_period} ms, physics {timestep} ms, "
        f"harvest every {max(1, cam_period // timestep)} ticks."
    )

    return motors, sensors, gps, gps_head, imu, keyboard, hud, cams, cam_period, gyro

def _sync_bom_body(robot: Robot) -> bool:
    """Load BoM mass. True means stop for a reload.

    The shipped world keeps the 0.08 m wheel. A selected ``bom_build`` writes
    a sibling world (see ``src.webots_body``) whose body mass and inertia
    follow the BoM, then reloads it. The contact cylinder stays 0.08 m, and
    ``v = ω · r`` stays on that radius, matching odometry and the viscous
    energy bill. The catalog wheel diameter is only logged. No build
    selected: this does nothing.
    """
    try:
        from src.webots_body import (
            DRAWN_WHEEL_RADIUS_M,
            bom_body_physics,
            format_bom_body_log_line,
            install_command_radius,
            plan_bom_world,
        )
    except Exception as exc:
        print(f"BOM BODY skipped ({type(exc).__name__}: {exc})")
        return False
    physics = bom_body_physics()
    if physics is None:
        # Blank bom_build still names the shipped 3.2 kg / 0.08 m body.
        # print() stays in the webots-bin console, which Windows cannot redirect.
        line = format_bom_body_log_line()
        if line:
            print(line)
            _record_mission_lines([line])
        return False
    try:
        loaded = str(robot.getWorldPath() or "")
    except Exception:
        loaded = ""
    if not loaded:
        print("BOM BODY skipped: controller has no world path")
        return False
    try:
        plan = plan_bom_world(loaded)
    except Exception as exc:
        print(f"BOM BODY patch failed ({type(exc).__name__}: {exc})")
        return False
    target = plan.path
    if plan.reload:
        print(
            f"BOM BODY build={physics.build} mass_kg={physics.mass_kg:g} "
            f"catalog_wheel_radius_m={physics.wheel_radius_m:g} "
            f"contact_radius_m={physics.contact_radius_m:g} "
            f"drawn_wheel_radius_m={DRAWN_WHEEL_RADIUS_M:g} "
            f"-> {os.path.basename(target)} (reloading; contact stays {physics.contact_radius_m:g} m)"
        )
        _record_mission_lines([format_bom_body_log_line(physics.build) or ""])
        if not hasattr(robot, "worldLoad"):
            print(f"BOM BODY open this world and restart: {target}")
            return False
        try:
            robot.worldLoad(os.path.abspath(target))
        except Exception as exc:
            print(f"BOM BODY reload failed ({type(exc).__name__}: {exc}); drawn wheel stays")
            return False
        return True
    install_command_radius(physics.contact_radius_m)
    bind_drive_radius(physics.contact_radius_m)
    global WHEEL_RADIUS_M
    WHEEL_RADIUS_M = physics.contact_radius_m
    body_line = format_bom_body_log_line(physics.build) or (
        f"BOM BODY build={physics.build} mass_kg={physics.mass_kg:g} "
        f"contact_radius_m={physics.contact_radius_m:g}"
    )
    print(body_line)
    _record_mission_lines([body_line])
    return False

class RunState:
    """Locals of the old ``_run_loop``, shared by the step functions.

    Each step unpacks the names it reads, runs the original block, and
    writes back the names it assigns. Call order is ``_run_loop``.
    """


def prepare_run(robot, opts):
    """Bring devices, route, and mission limits up. Same order as the old _run_loop prologue."""
    if _sync_bom_body(robot):
        return
    timestep = int(robot.getBasicTimeStep())
    publish_every = max(1, int(opts["interval_s"] * 1000 / timestep))
    state_poll_every = max(3, min(publish_every, int(0.1 * 1000 / timestep)))
    motors, sensors, gps, gps_head, imu, keyboard, hud, cams, cam_period, gyro = _init_devices(
        robot, timestep
    )
    harvest_every = max(1, int(cam_period) // max(1, int(timestep)))
    last_harvest_tick = -harvest_every
    speed_estimator = SpeedEstimator()
    key_tracker = KeyTracker()
    abs_brake = AbsBrakeController()

    phase_idx = 0
    phase_elapsed = 0.0
    dashboard = opts["dashboard_url"]
    startup_state = fetch_twin_state(dashboard)
    battery_pct = battery_from_twin_state(startup_state, default=100.0)
    # One id per controller process. A Webots restart without a dashboard
    # restart changes this, and the energy log summarizes the old mission.
    controller_run_id = f"{_RUN_ID}-{os.getpid()}"
    pack_meter = None
    controller_energy_log = None
    try:
        from src.twin_energy import SimPackMeter

        pack_meter = SimPackMeter()
    except Exception as exc:
        print(f"Pack energy meter failed ({type(exc).__name__}: {exc})")
    try:
        from src.mission_energy_log import MissionEnergyLog

        controller_energy_log = MissionEnergyLog()
    except Exception as exc:
        print(f"Mission energy log failed ({type(exc).__name__}: {exc})")
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
    _track_ct = _load_s_cross_track()
    rowfit_rt = None
    lane_vision_log = None
    if _LANE_MODE == "rowfit":
        try:
            rowfit_rt = RowfitRuntime()
            rowfit_rt.gyro_found = gyro is not None
            lane_vision_log = LaneVisionLog()
            _lane_csv = os.path.abspath(lane_vision_log.path)
            print(
                f"LANE MODE rowfit (run_id {_RUN_ID}) — lane_vision fit, "
                f"log → {_lane_csv}"
            )
            _record_mission_lines([f"LANE_VISION file={_lane_csv}"])
            print(rowfit_rt.route_line())
            print(rowfit_rt.limits_line())
            print(rowfit_rt.coast_line())
            for _wmsg in rowfit_rt.route_warnings:
                print(f"WARNING {_wmsg}")
        except Exception as exc:
            rowfit_rt = None
            print(
                f"WARNING: rowfit not loaded ({exc}) — lane keep off "
                "(the pixel-gap keeper is retired)"
            )
    finish_ref = _load_finish_referee(robot)
    print(f"FINISH REFEREE {finish_ref.describe()} — GPS finish line, not used to steer")
    try:
        from src.track_geometry import steer_track_enabled

        _loaded_world = str(robot.getWorldPath() or "")
        _steer_track = steer_track_enabled(_loaded_world)
    except Exception:
        _steer_track = True
    if rowfit_rt is not None and _steer_track and getattr(finish_ref, "track", None) is not None:
        rowfit_rt.ctl.set_track(finish_ref.track)
        print(
            f"ROWFIT corners use track {finish_ref.name} for pivot distance "
            "(a short camera view cannot start the turn early). "
            "The track does not set wheel curvature."
        )
    elif rowfit_rt is not None:
        print(
            "ROWFIT steer track off — wheels follow the nadir cameras. "
            "GPS still scores the finish line and cross-track. "
            "It is not given to the wheel command."
        )
    for _wmsg in world_check_warnings(robot, finish_ref):
        print("!" * 78)
        print(f"WARNING {_wmsg}")
        print("!" * 78)
    if lane_vision_log is not None:
        lane_vision_log.write_run_meta(robot, finish_ref, rowfit_rt)
    lane_keep_on = False
    ct_peak_m = 0.0
    ct_latest_m = None
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
    hud_paint = False
    try:
        from src.config import config as _hud_cfg
        from src.sim_trace import overlay_status_lines, resolve_hud_overlays

        hud_paint = resolve_hud_overlays(
            config_value=_hud_cfg.get("robot", "hud_overlays", None),
            env_value=os.environ.get("RBM_HUD"),
        )
        for _line in overlay_status_lines(hud_paint):
            print(_line)
    except Exception as exc:
        print(f"HUD paint failed ({type(exc).__name__}: {exc})")
    # Paint is the optional 256×256 paste. Cameras stay enabled either way.
    eye_huds = _label_eye_huds(robot, cams) if hud_paint else []
    if rowfit_rt is not None:
        for msg in rowfit_rt.bind_cameras(robot, cams) + rowfit_rt.check_cameras(cams):
            print(f"WARNING rowfit camera model: {msg}")
        # Stereo obstacles (RBM_OBSTACLES=1, or the world's '# OBSTACLES on' tag): slow / stop / resume
        _obs_on, _obs_why = _obstacles_mode(robot)
        print(f"OBSTACLES {'on' if _obs_on else 'off'} ({_obs_why})")
        if _obs_on:
            try:
                _top = rowfit_rt.limits.gate_top_speed_m_s
                _ort = ObstacleRuntime(
                    v_cruise_m_s=_top,
                    bom_build=rowfit_rt.limits.bom_build,
                )
                _msgs = _ort.enable_devices(robot, cam_period)
                _msgs += _ort.bind(robot, rowfit_rt.cams_model)
                for msg in _msgs:
                    print(f"WARNING obstacles: {msg}")
                if _ort.ok:
                    rowfit_rt.obstacles = _ort
            except Exception as exc:
                print(f"WARNING: obstacle sensing not loaded ({type(exc).__name__}: {exc}) — driving without it")
        # Pretend pick when RBM_PICK_SKU is set (demo / SKU-A05-B0-L).
        # Shopping list when RBM_SHOPPING_LIST is set. No trolley prop.
        try:
            _inv = InventoryRuntime()
            _track = getattr(finish_ref, "name", None) or "warehouse"
            for msg in _inv.enable(robot, track_name=str(_track)):
                print(f"WARNING inventory: {msg}")
            if _inv.pick is not None:
                rowfit_rt.inventory = _inv
                print(_inv.describe())
            else:
                print(f"INVENTORY off ({_inv.why})")
        except Exception as exc:
            print(f"WARNING: inventory not loaded ({type(exc).__name__}: {exc})")
    nadir_lobe_done = False
    mission_complete = False
    turn_log_sent = False
    finish_age_s = 0.0
    mission_travel_m = 0.0
    mission_prev_xy: tuple[float, float] | None = None
    mission_capped = False
    mission_limit_m = None
    from src.sim_trace import SpeedTrace

    speed_trace = SpeedTrace(False)
    try:
        from src.config import config as _rbm_config
        from src.mission_park import (
            mission_distance_reached,
            resolve_mission_limit_m,
            warehouse_perimeter_m,
        )
        from src.teleop_agent import plausible_travel_m

        _perim_m = warehouse_perimeter_m()
        mission_limit_m = resolve_mission_limit_m(
            config_m=_rbm_config.get("robot", "mission_max_m", None),
            env_m=os.environ.get("RBM_MISSION_MAX_M"),
            env_laps=os.environ.get("RBM_MISSION_LAPS"),
            lap_m=_perim_m,
        )
        if mission_limit_m:
            _perim_txt = f"{_perim_m:.0f} m" if _perim_m else "unknown"
            print(
                f"MISSION LIMIT on — stop at {mission_limit_m:.1f} m "
                f"(warehouse perimeter {_perim_txt}). "
                "Leave mission_max_m blank and unset RBM_MISSION_MAX_M "
                "to run until the track finish."
            )
        else:
            print("MISSION LIMIT off — run until the track finish")
        from src.sim_trace import resolve_speed_trace, resolve_speed_trace_console
        from twin_publisher import project_env

        _trace_on = resolve_speed_trace(
            config_value=_rbm_config.get("robot", "speed_trace", None),
            env_value=os.environ.get("RBM_SPEED_TRACE"),
            mission_limit_m=mission_limit_m,
            lap_m=_perim_m,
        )
        _trace_console = resolve_speed_trace_console(
            config_value=_rbm_config.get("robot", "speed_trace_console", None),
            env_value=os.environ.get("RBM_SPEED_TRACE_CONSOLE"),
        )
        _trace_dir = project_env("RBM_LOG_DIR") or os.path.join(_PROJECT_ROOT, "logs")
        speed_trace = SpeedTrace(
            _trace_on,
            console=_trace_console,
            log_dir=_trace_dir if _trace_on else None,
        )
    except Exception as exc:
        mission_distance_reached = lambda traveled_m, limit_m: False  # noqa: E731
        plausible_travel_m = lambda step_m, dt_s: 0.0  # noqa: E731
        print(f"MISSION LIMIT off ({type(exc).__name__}: {exc})")
    if speed_trace.enabled:
        _where = speed_trace.path or speed_trace.open_error or "no file"
        _console = (
            "console on"
            if speed_trace.console
            else "console off (set RBM_SPEED_TRACE_CONSOLE=1 to print)"
        )
        print(
            f"SPEED_TRACE on — file {_where}. {_console}. "
            "Rows only while the wheels are moving. Stops at mission end. "
            "Not the dashboard log."
        )
    else:
        print(
            "SPEED_TRACE off — set RBM_SPEED_TRACE=1, or run one warehouse "
            "lap (RBM_MISSION_LAPS=1)."
        )
    print(f"ButlerBot controller started — twin → {dashboard}/api/twin/telemetry")
    print(f"Battery synced from dashboard: {battery_pct:.1f}%")
    for _w in ("left_wheel", "right_wheel"):
        _enable_full_wheel_torque(motors[_w])
        motors[_w].setPosition(float("inf"))
        motors[_w].setVelocity(0.0)

    # Kept across ticks. A new camera frame rarely lands on a telemetry
    # post; clearing this every step dropped width and center errors
    # before MISSION_SUMMARY could see them. Each frame is also noted
    # once on the energy log (see note_lane_frame) so a later post of
    # the same frame does not count twice.
    lane_fit_fields: dict = {}
    st = RunState()
    st.robot = robot
    st.opts = opts
    st.timestep = timestep
    st.publish_every = publish_every
    st.state_poll_every = state_poll_every
    st.motors = motors
    st.sensors = sensors
    st.gps = gps
    st.gps_head = gps_head
    st.imu = imu
    st.keyboard = keyboard
    st.hud = hud
    st.cams = cams
    st.gyro = gyro
    st.harvest_every = harvest_every
    st.last_harvest_tick = last_harvest_tick
    st.speed_estimator = speed_estimator
    st.key_tracker = key_tracker
    st.abs_brake = abs_brake
    st.phase_idx = phase_idx
    st.phase_elapsed = phase_elapsed
    st.dashboard = dashboard
    st.battery_pct = battery_pct
    st.controller_run_id = controller_run_id
    st.pack_meter = pack_meter
    st.controller_energy_log = controller_energy_log
    st.thermal_c = thermal_c
    st.tick = tick
    st.publish_fail_streak = publish_fail_streak
    st.auto_loop = auto_loop
    st.remote_throttle = remote_throttle
    st.agent_message = agent_message
    st.throttle_factor = throttle_factor
    st.keys_logged = keys_logged
    st.cached_api_left = cached_api_left
    st.cached_api_right = cached_api_right
    st.cached_api_source = cached_api_source
    st.last_api_sig = last_api_sig
    st.last_stop_epoch = last_stop_epoch
    st.drive_log_elapsed = drive_log_elapsed
    st.last_teleop_left = last_teleop_left
    st.last_teleop_right = last_teleop_right
    st.prev_yaw = prev_yaw
    st.prev_pitch = prev_pitch
    st.balance_cfg = balance_cfg
    st._track_ct = _track_ct
    st.rowfit_rt = rowfit_rt
    st.lane_vision_log = lane_vision_log
    st.finish_ref = finish_ref
    st.lane_keep_on = lane_keep_on
    st.ct_peak_m = ct_peak_m
    st.ct_latest_m = ct_latest_m
    st.lane_eyes = lane_eyes
    st.yaw_rate = yaw_rate
    st.pose_anchor_xy = pose_anchor_xy
    st.pose_anchor_yaw = pose_anchor_yaw
    st.pose_anchor_age_s = pose_anchor_age_s
    st.park_holdoff_s = park_holdoff_s
    st.drive_grace_s = drive_grace_s
    st.wheel_enc_prev = wheel_enc_prev
    st.wheel_rot_abs = wheel_rot_abs
    st.wheel_rot_since_stop = wheel_rot_since_stop
    st.wheel_lock_hold_s = wheel_lock_hold_s
    st.was_abs_active = was_abs_active
    st.hud_paint = hud_paint
    st.eye_huds = eye_huds
    st.nadir_lobe_done = nadir_lobe_done
    st.mission_complete = mission_complete
    st.turn_log_sent = turn_log_sent
    st.finish_age_s = finish_age_s
    st.mission_travel_m = mission_travel_m
    st.mission_prev_xy = mission_prev_xy
    st.mission_capped = mission_capped
    st.mission_limit_m = mission_limit_m
    st.speed_trace = speed_trace
    st.mission_distance_reached = mission_distance_reached
    st.plausible_travel_m = plausible_travel_m
    st.lane_fit_fields = lane_fit_fields
    return st

def command_motion(st):
    """ABS, API, keyboard, the R-key phase, or residual park. Then the pose anchor."""
    stop_pressed = st.stop_pressed
    left_wv_early = st.left_wv_early
    right_wv_early = st.right_wv_early
    cached_api_left = st.cached_api_left
    cached_api_right = st.cached_api_right
    abs_brake = st.abs_brake
    motors = st.motors
    sensors = st.sensors
    speed_m_s = st.speed_m_s
    forward_m_s = st.forward_m_s
    dt = st.dt
    yaw_rate = st.yaw_rate
    user_driving = st.user_driving
    teleop_keys = st.teleop_keys
    auto_loop = st.auto_loop
    gait = st.gait
    phase = st.phase
    t = st.t
    raw_speed_m_s = st.raw_speed_m_s
    pose_anchor_age_s = st.pose_anchor_age_s
    gps_xy = st.gps_xy
    pose_anchor_xy = st.pose_anchor_xy
    prev_yaw = st.prev_yaw
    pose_anchor_yaw = st.pose_anchor_yaw
    drive_grace_s = st.drive_grace_s
    raw_forward_m_s = st.raw_forward_m_s
    last_teleop_left = st.last_teleop_left
    last_teleop_right = st.last_teleop_right
    phase_name = st.phase_name
    abs_just_finished = st.abs_just_finished
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
            abs_just_finished = True
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
    st.left_wv = left_wv
    st.right_wv = right_wv
    st.wheel_vels = wheel_vels
    st.api_driving = api_driving
    st.left_v = left_v
    st.right_v = right_v
    st.user_driving = user_driving
    st.gait = gait
    st.phase_name = phase_name
    st.last_teleop_left = last_teleop_left
    st.last_teleop_right = last_teleop_right
    st.cached_api_left = cached_api_left
    st.cached_api_right = cached_api_right
    st.abs_just_finished = abs_just_finished
    st.pose_anchor_age_s = pose_anchor_age_s
    st.pose_anchor_xy = pose_anchor_xy
    st.pose_anchor_yaw = pose_anchor_yaw
    st.drive_grace_s = drive_grace_s
