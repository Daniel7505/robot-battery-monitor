"""Sensing and camera pose for the ButlerBot Webots controller.

IMU yaw, pitch hold, the follow camera, and the per-tick pose read
(GPS, wheel rates, gyro). ``butlerbot_controller.py`` calls these in
the same order the single loop used to.
"""

from __future__ import annotations

import math

from controller import GPS, InertialUnit, Robot

try:
    from controller import Supervisor
except ImportError:  # pragma: no cover - Webots always provides Supervisor
    Supervisor = None  # type: ignore[misc, assignment]

from controller_wheels import (
    STOP_SPEED_M_S,
    STOP_WHEEL_DIFF_RAD_S,
    STOP_WHEEL_RAD_S,
    STOP_YAW_RATE_RAD_S,
    _clamp,
    _gps_xy,
    _safe_imu_pitch,
)

_teleop = None


def bind_teleop(mod) -> None:
    global _teleop
    _teleop = mod


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

def _format_pose(gps: GPS) -> str:
    try:
        pos = gps.getValues()
        return f"({pos[0]:.2f}, {pos[1]:.2f})"
    except Exception:
        return "(?, ?)"


_CAM_EYE = (-3.18573, -0.03020, 3.54536)
_CAM_FOV = 0.85
_CAM_ORIENTATION = (-0.001250, 0.999989, 0.004550, 0.536256)

def _is_top_down(rot) -> bool:
    """World Viewpoint looks straight down (ENU axis-angle ~ 0 1 0 pi/2), e.g. the warehouse follow cam."""
    try:
        ax, ay, az, ang = (float(c) for c in rot)
    except Exception:
        return False
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-9:
        return False
    ax, ay, az = ax / n, ay / n, az / n
    if ay < 0:  # same rotation written as -axis / -angle
        ax, ay, az, ang = -ax, -ay, -az, -ang
    return ay > 0.999 and abs(ang - math.pi / 2) < 0.02


def _apply_top_down_follow(robot, vp, pos_field) -> None:
    """Top-down follow pose authored by the world (height / fov / orientation derived from
    the world's builder settings, e.g. scripts/warehouse_builder.py follow_viewpoint()).

    Keep the world's height, straight-down orientation and fov; re-centre the eye over
    the robot and follow it with a Tracking Shot (translation only, so the map keeps a
    fixed world orientation and the eye only moves horizontally above the walls).
    """
    field = vp.getField
    eye = list(pos_field.getSFVec3f())
    try:
        rp = robot.getSelf().getPosition()
        eye[0], eye[1] = float(rp[0]), float(rp[1])
    except Exception:
        pass
    follow = field("follow")
    if follow:
        follow.setSFString("ButlerBot")
    ftype = field("followType")
    if ftype:
        ftype.setSFString("Tracking Shot")
    pos_field.setSFVec3f(eye)
    fov = field("fieldOfView")
    fov_v = fov.getSFFloat() if fov else float("nan")
    print(
        "Camera: top-down follow (world Viewpoint) Tracking Shot on ButlerBot "
        f"eye=({eye[0]:.2f}, {eye[1]:.2f}, {eye[2]:.2f}) fov={fov_v:.2f} "
        f"floor across={2 * eye[2] * math.tan(fov_v / 2):.1f} m"
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
        if follow is not None and not follow.getSFString().strip():
            # World authored a FIXED view (no follow): keep it, no chase cam.
            print("Camera: fixed world Viewpoint (no follow) kept, chase reset skipped")
            return
        ori0 = field("orientation")
        pos0 = field("position")
        if ori0 is not None and pos0 is not None and _is_top_down(ori0.getSFRotation()):
            _apply_top_down_follow(robot, vp, pos0)
            return
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
        "error_source": "nadir",
    }

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

def sense_motion(st):
    """GPS, encoders, IMU, gyro, cross-track, and the settled gate, in that order."""
    import controller_wheels as _wheelmod
    WHEEL_RADIUS_M = _wheelmod.WHEEL_RADIUS_M
    speed_estimator = st.speed_estimator
    gps = st.gps
    dt = st.dt
    abs_brake = st.abs_brake
    sensors = st.sensors
    imu = st.imu
    prev_yaw = st.prev_yaw
    gyro = st.gyro
    finish_ref = st.finish_ref
    ct_peak_m = st.ct_peak_m
    rowfit_rt = st.rowfit_rt
    mission_capped = st.mission_capped
    mission_prev_xy = st.mission_prev_xy
    plausible_travel_m = st.plausible_travel_m
    mission_travel_m = st.mission_travel_m
    last_teleop_left = st.last_teleop_left
    last_teleop_right = st.last_teleop_right
    cached_api_left = st.cached_api_left
    cached_api_right = st.cached_api_right
    yaw_rate = st.yaw_rate
    ct_latest_m = st.ct_latest_m
    speed_m_s, forward_m_s = speed_estimator.estimate_motion(gps, dt)
    raw_speed_m_s = speed_m_s
    raw_forward_m_s = forward_m_s
    left_wv_early = abs_brake.wheel_rad_s(sensors, "left_wheel_sensor", dt)
    right_wv_early = abs_brake.wheel_rad_s(sensors, "right_wheel_sensor", dt)
    yaw_rate, prev_yaw = _yaw_rate(imu, prev_yaw, dt)
    if gyro is not None:
        try:
            yaw_rate = float(gyro.getValues()[2])
        except Exception:
            pass
    gps_xy = _gps_xy(gps)
    if gps_xy is not None and hasattr(finish_ref, "cross_track"):
        try:
            _ct_now = finish_ref.cross_track(float(gps_xy[0]), float(gps_xy[1]))
        except Exception:
            _ct_now = None
        if _ct_now is not None:
            ct_latest_m = float(_ct_now)
            ct_peak_m = max(ct_peak_m, abs(ct_latest_m))
            if rowfit_rt is not None:
                rowfit_rt.ctl.note_cross_track(ct_latest_m)
    if not mission_capped and gps_xy is not None:
        if mission_prev_xy is not None:
            _step_m = math.hypot(
                float(gps_xy[0]) - mission_prev_xy[0],
                float(gps_xy[1]) - mission_prev_xy[1],
            )
            mission_travel_m += plausible_travel_m(_step_m, dt)
        mission_prev_xy = (float(gps_xy[0]), float(gps_xy[1]))
    if rowfit_rt is not None:
        rowfit_rt.odometry(left_wv_early, right_wv_early, dt, WHEEL_RADIUS_M)

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
    st.speed_m_s = speed_m_s
    st.forward_m_s = forward_m_s
    st.raw_speed_m_s = raw_speed_m_s
    st.raw_forward_m_s = raw_forward_m_s
    st.left_wv_early = left_wv_early
    st.right_wv_early = right_wv_early
    st.yaw_rate = yaw_rate
    st.prev_yaw = prev_yaw
    st.gps_xy = gps_xy
    st.ct_latest_m = ct_latest_m
    st.ct_peak_m = ct_peak_m
    st.mission_travel_m = mission_travel_m
    st.mission_prev_xy = mission_prev_xy
    st.settled = settled

def accumulate_wheel_encoders(st):
    """Encoder deltas and hub-lock hold, after the wheel command."""
    left_wv = st.left_wv
    right_wv = st.right_wv
    abs_brake = st.abs_brake
    was_abs_active = st.was_abs_active
    sensors = st.sensors
    wheel_enc_prev = st.wheel_enc_prev
    wheel_rot_abs = st.wheel_rot_abs
    left_v = st.left_v
    right_v = st.right_v
    user_driving = st.user_driving
    api_driving = st.api_driving
    wheel_rot_since_stop = st.wheel_rot_since_stop
    wheel_lock_hold_s = st.wheel_lock_hold_s
    dt = st.dt
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
    st.wheel_rot_since_stop = wheel_rot_since_stop
    st.wheel_lock_hold_s = wheel_lock_hold_s
    st.was_abs_active = was_abs_active
    st.wheel_locked = wheel_locked
    st.wheels_both_locked = wheels_both_locked
