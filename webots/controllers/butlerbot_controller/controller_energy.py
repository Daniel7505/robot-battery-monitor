"""Energy and telemetry posting for the ButlerBot Webots controller.

Joint watts, the pack meter, battery percent, and ``POST /api/twin/telemetry``.
The drain math is the same block the loop used to run after the wheel command.
"""

from __future__ import annotations

import math
import os
import time

from controller_devices import _apply_wheel_command, _set_drive
from controller_mission import _log_steer_action
from controller_sensing import (
    _format_pose,
    _imu_yaw,
    _pitch_balance_delta,
    _read_head_gps,
    _read_pose,
)
from controller_wheels import (
    SOFT_GRIP_GAIN,
    STOP_YAW_RATE_RAD_S,
    _WHEEL_LOCK_POS,
    _apply_soft_grip,
    _safe_imu_pitch,
    _safe_imu_roll,
)
from twin_publisher import (
    build_payload,
    estimate_motor_power_w,
    publish_telemetry,
)

_teleop = None


def bind_teleop(mod) -> None:
    global _teleop
    _teleop = mod


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

def _publish_controller_limits(loco: dict, rowfit_rt) -> None:
    """Copy the effective limits into telemetry so MISSION_SUMMARY can prove them."""
    try:
        lim = rowfit_rt.limits
    except AttributeError:
        return
    loco["ctrl_cruise_m_s"] = round(float(lim.cruise_m_s), 4)
    loco["ctrl_straight_m_s"] = round(float(lim.straight_m_s), 4)
    if lim.v_max_m_s is not None:
        loco["ctrl_v_max_m_s"] = round(float(lim.v_max_m_s), 4)
    eff_c = lim.effective_cruise_m_s if lim.effective_cruise_m_s is not None else lim.cruise_m_s
    loco["ctrl_effective_cruise_m_s"] = round(float(eff_c), 4)
    loco["ctrl_speed_limit_active"] = bool(lim.speed_limit_active)
    cmd = getattr(rowfit_rt, "last_cmd", None) or {}
    if cmd.get("yaw_rate_rad_s") is not None:
        loco["ctrl_yaw_rate_rad_s"] = round(float(cmd["yaw_rate_rad_s"]), 4)
    if cmd.get("pivot_heading_err_rad") is not None:
        loco["ctrl_pivot_heading_err_rad"] = round(float(cmd["pivot_heading_err_rad"]), 4)
    if cmd.get("pivot_integrated_deg") is not None:
        loco["ctrl_pivot_integrated_deg"] = round(float(cmd["pivot_integrated_deg"]), 2)
    loco["ctrl_pivot_w_max_rad_s"] = round(float(lim.pivot_w_max_rad_s), 4)
    loco["ctrl_k_steer"] = round(float(lim.k_steer), 4)
    loco["ctrl_bom_build"] = "" if not lim.bom_build else str(lim.bom_build)
    loco["gyro_found"] = bool(getattr(rowfit_rt, "gyro_found", False))
    if cmd.get("coast_wh_saved") is not None:
        loco["coast_wh_saved"] = round(float(cmd["coast_wh_saved"]), 6)
    if cmd.get("coast_starts") is not None:
        loco["coast_starts"] = int(cmd["coast_starts"])
    coast_cfg = getattr(getattr(rowfit_rt, "ctl", None), "coast_into_turn", None)
    if coast_cfg is not None:
        loco["ctrl_coast_enabled"] = 1 if coast_cfg.enabled else 0
    loco["ctrl_pivot_count"] = int(getattr(getattr(rowfit_rt, "ctl", None), "pivots", 0))
    for env_key, field in (
        ("RBM_OPT_RUN_ID", "opt_run_id"),
        ("RBM_OPT_TRIAL_ID", "opt_trial_id"),
        ("RBM_LAP_START_WALLCLOCK", "lap_start_wallclock"),
    ):
        stamped = os.environ.get(env_key)
        if stamped:
            loco[field] = str(stamped)
    try:
        from src.webots_body import bom_body_physics

        physics = bom_body_physics(lim.bom_build)
        if physics is not None:
            loco["body_mass_kg"] = round(float(physics.mass_kg), 4)
    except Exception:
        pass


def _uses_bom_pack() -> bool:
    """True when config selects a BoM build and the pack/draw should follow it."""
    try:
        from src.hardware_profile import active_bom_build

        return bool(active_bom_build())
    except Exception:
        return False


def _pack_capacity_wh() -> float:
    """Wh to drain against when the preview did not name a pack.

    A failed pack read on a light build must use the BoM pack (240 Wh),
    not the wheeled YAML 480. Blank ``bom_build`` stays 480.
    """
    try:
        from src.hardware_profile import battery_capacity_wh

        cap = float(battery_capacity_wh())
        if cap > 0:
            return cap
    except Exception:
        pass
    return 480.0


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
    import controller_wheels as _wheelmod
    WHEEL_RADIUS_M = _wheelmod.WHEEL_RADIUS_M
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
        if wheel_parked and not _uses_bom_pack():
            # Legacy sim: a flat 2 W so a parked hub is not free. A BoM build
            # falls through and uses the published encoder idle instead.
            # torque_feedback is left off, so this step counts as an
            # electrical gap. Shaft power is ~0 here, so the gap does not
            # move the moving-lap watt-hour integral. A miss while the
            # hinge is turning does undercount that step.
            joints.append({
                "name": motor_name,
                "position": round(position, 4),
                "velocity": 0.0,
                "torque": 0.0,
                "power_w": 2.0,
            })
            continue

        feedback = None
        if hasattr(motor, "getTorqueFeedback"):
            try:
                tf = float(motor.getTorqueFeedback())
                if math.isfinite(tf):
                    feedback = tf
            except Exception:
                feedback = None
        try:
            from src.twin.webots_power import reported_wheel_torque_nm

            # Model load. A BoM build passes 0 so Legs stay on rolling
            # resistance. The raw hinge reading is torque_feedback_nm,
            # which the energy log integrates as |τ|·|ω|.
            torque = reported_wheel_torque_nm(
                velocity, feedback, bom=_uses_bom_pack()
            )
        except Exception:
            torque = 0.0 if _uses_bom_pack() else (
                abs(feedback) if feedback is not None else abs(velocity) * 0.45
            )
        joint_speed = speed_m_s if (abs(velocity) < 0.2 and cmd_v > 0.05) else 0.0
        power_w = _estimate_joint_power(
            motor_name, velocity, torque, speed_m_s=joint_speed
        )
        joint = {
            "name": motor_name,
            "position": round(position, 4),
            "velocity": round(velocity, 4),
            "torque": round(torque, 4),
            "power_w": power_w,
        }
        if "wheel" in motor_name:
            # None when getTorqueFeedback did not report. 0.0 is a reading.
            joint["torque_feedback_nm"] = (
                None if feedback is None else round(feedback, 4)
            )
        joints.append(joint)
    return joints

def integrate_energy(st):
    """Joint power, pack Wh, battery percent, thermal, throttle, and pitch hold."""
    import controller_wheels as _wheelmod
    WHEEL_RADIUS_M = _wheelmod.WHEEL_RADIUS_M
    left_v = st.left_v
    right_v = st.right_v
    left_wv = st.left_wv
    right_wv = st.right_wv
    speed_m_s = st.speed_m_s
    motors = st.motors
    sensors = st.sensors
    wheel_vels = st.wheel_vels
    dt = st.dt
    yaw_rate = st.yaw_rate
    abs_brake = st.abs_brake
    user_driving = st.user_driving
    api_driving = st.api_driving
    pack_meter = st.pack_meter
    gait = st.gait
    phase_name = st.phase_name
    battery_pct = st.battery_pct
    thermal_c = st.thermal_c
    remote_throttle = st.remote_throttle
    imu = st.imu
    prev_pitch = st.prev_pitch
    balance_cfg = st.balance_cfg
    robot = st.robot
    throttle_factor = st.throttle_factor
    agent_message = st.agent_message
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
    drain_dt = dt
    pack_step_wh = None
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
    cap_wh = 0.0
    pack_step_wh = None
    if pack_meter is not None:
        # Pack draw is measured electrical drive plus the non-drive
        # channels. Legs in the preview are the rolling prediction;
        # substituting that for hinge torque made wh_measured_soc
        # disagree with the electrical total (16646d5: pack 2.256 Wh,
        # shaft+idle 3.724 Wh). Missing feedback adds no drive watts.
        # The integral is the simulation step, posted as energy_wh_*.
        # Wall time between POSTs is not the drain clock: a few
        # seconds blocked in HTTP used to bill that many seconds of
        # sim watts, and a hole over 15 s was dropped by the log
        # (run #11, wh_measured 6.381 vs wh_measured_soc 7.224).
        # A blank bom_build still integrates. Shaft |τ|·|ω| uses the
        # YAML motor line. Only the battery percent stays on the old
        # joint-watt drain unless a BoM pack is selected.
        try:
            from src.twin.webots_power import measured_drive_split

            preview = build_payload(
                joints,
                gait=gait,
                phase=phase_name,
                speed_m_s=speed_m_s,
                battery_pct=battery_pct,
            )
            channels = preview.get("channel_draws") or (
                (preview.get("power") or {}).get("channel_draws") or {}
            )
            split = measured_drive_split(joints, power_speed)
            elec_w = None if split is None else split.get("elec_w")
            shaft_w = None if split is None else split.get("shaft_w")
            idle_w = sum(
                float(watts)
                for name, watts in channels.items()
                if name != "Legs" and watts is not None
            )
            pack_meter.step(
                dt, shaft_w=shaft_w, elec_w=elec_w, idle_w=idle_w
            )
            if _uses_bom_pack():
                pack_step_wh = pack_meter.last_step_wh
                total_draw = float(pack_meter.last_pack_w)
                cap_wh = float((preview.get("robot") or {}).get("battery_capacity_wh") or 0.0)
                drain_dt = dt
        except Exception:
            if _uses_bom_pack():
                cap_wh = 0.0
                drain_dt = dt
                pack_step_wh = None
    # A failed pack read leaves cap_wh at 0. Drain the active pack:
    # BoM Wh on a light build, wheeled YAML 480 when bom_build is blank.
    if cap_wh <= 0:
        cap_wh = _pack_capacity_wh()
    if pack_step_wh is not None:
        if cap_wh > 0:
            battery_pct = max(
                5.0, battery_pct - pack_step_wh / cap_wh * 100.0
            )
    elif _teleop is not None:
        battery_pct = max(
            5.0,
            battery_pct
            - _teleop.battery_drain_pct(
                total_draw,
                drain_dt,
                capacity_wh=cap_wh,
                scale=1.0,
                drain_scale=1.0,
            ),
        )
    elif cap_wh > 0:
        battery_pct = max(
            5.0,
            battery_pct
            - (total_draw * drain_dt) / (cap_wh * 3600.0) * 100.0,
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
    st.joints = joints
    st.residual_spin = residual_spin
    st.gait = gait
    st.phase_name = phase_name
    st.battery_pct = battery_pct
    st.thermal_c = thermal_c
    st.throttle_factor = throttle_factor
    st.agent_message = agent_message
    st.prev_pitch = prev_pitch
    st.left_v = left_v
    st.right_v = right_v

def publish_tick(st):
    """Build the twin payload and POST it on the telemetry cadence."""
    import controller_wheels as _wheelmod
    WHEEL_RADIUS_M = _wheelmod.WHEEL_RADIUS_M
    abs_brake = st.abs_brake
    publish_every = st.publish_every
    tick = st.tick
    wheel_enc_prev = st.wheel_enc_prev
    left_wv = st.left_wv
    wheel_rot_abs = st.wheel_rot_abs
    wheel_rot_since_stop = st.wheel_rot_since_stop
    wheel_locked = st.wheel_locked
    wheel_lock_hold_s = st.wheel_lock_hold_s
    right_wv = st.right_wv
    wheels_both_locked = st.wheels_both_locked
    robot = st.robot
    left_v = st.left_v
    right_v = st.right_v
    yaw_rate = st.yaw_rate
    raw_speed_m_s = st.raw_speed_m_s
    raw_forward_m_s = st.raw_forward_m_s
    api_driving = st.api_driving
    cached_api_source = st.cached_api_source
    user_driving = st.user_driving
    auto_loop = st.auto_loop
    last_stop_epoch = st.last_stop_epoch
    park_holdoff_s = st.park_holdoff_s
    residual_spin = st.residual_spin
    phase_name = st.phase_name
    gait = st.gait
    joints = st.joints
    speed_m_s = st.speed_m_s
    battery_pct = st.battery_pct
    gps = st.gps
    imu = st.imu
    gps_head = st.gps_head
    thermal_c = st.thermal_c
    lane_keep_on = st.lane_keep_on
    lane_eyes = st.lane_eyes
    ct_latest_m = st.ct_latest_m
    ct_peak_m = st.ct_peak_m
    lane_fit_fields = st.lane_fit_fields
    throttle_factor = st.throttle_factor
    mission_event = st.mission_event
    lane_vision_log = st.lane_vision_log
    rowfit_rt = st.rowfit_rt
    prev_yaw = st.prev_yaw
    controller_run_id = st.controller_run_id
    pack_meter = st.pack_meter
    speed_trace = st.speed_trace
    controller_energy_log = st.controller_energy_log
    dashboard = st.dashboard
    publish_fail_streak = st.publish_fail_streak
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
                "error_source": lane_eyes.get("error_source") or "nadir",
                "lane_phase": lane_eyes.get("phase"),
                "steer": lane_eyes.get("steer"),
                "steer_source": lane_eyes.get("steer_source"),
                "steer_lateral": lane_eyes.get("steer_lateral"),
                "fit_reject": lane_eyes.get("fit_reject"),
                "cross_track_m": None if ct_latest_m is None else round(ct_latest_m, 4),
                "ct_max_m": None if ct_latest_m is None else round(ct_peak_m, 4),
                **lane_fit_fields,
                "agent_throttle": throttle_factor,
                "braking": abs_brake.active,
                "residual_spin": residual_spin,
                "mission_event": mission_event,
                "control_diag": control_diag,
                "wheels": wheel_sensors,
            },
        )
        if lane_keep_on:
            _log_steer_action(payload.get("pose") or {}, lane_eyes)
            if lane_vision_log is not None and rowfit_rt is not None:
                _pose = payload.get("pose") or {}
                lane_vision_log.write(
                    x_m=_pose.get("x_m"),
                    y_m=_pose.get("y_m"),
                    mode="rowfit",
                    est=rowfit_rt.est,
                    cmd={
                        "steer": lane_eyes.get("steer"),
                        "target_speed": rowfit_rt.ctl.target_speed,
                        **rowfit_rt.ctl.corner_fields(),
                        **(rowfit_rt.obstacles.fields() if rowfit_rt.obstacles is not None else {}),
                    },
                    new_frame=False,
                    yaw=prev_yaw,
                )
        loco = payload.get("locomotion")
        if not isinstance(loco, dict):
            loco = {}
            payload["locomotion"] = loco
        loco["controller_run_id"] = controller_run_id
        if rowfit_rt is not None:
            _publish_controller_limits(loco, rowfit_rt)
        sensors_out = payload.get("sensors")
        if isinstance(sensors_out, dict):
            sensors_out["controller_run_id"] = controller_run_id
        if pack_meter is not None:
            loco.update(pack_meter.snapshot())
        if lane_eyes.get("steer_source"):
            loco["steer_source"] = lane_eyes.get("steer_source")
            loco["steer_lateral"] = lane_eyes.get("steer_lateral")
            loco["fit_reject"] = lane_eyes.get("fit_reject") or ""
        if ct_latest_m is not None:
            loco["cross_track_m"] = round(ct_latest_m, 4)
            loco["ct_max_m"] = round(ct_peak_m, 4)
        loco.update(lane_fit_fields)
        if speed_trace.steer_text:
            loco["steer_split"] = speed_trace.steer_text
            if isinstance(sensors_out, dict):
                sensors_out["steer_split"] = speed_trace.steer_text
        if controller_energy_log is not None:
            try:
                from src.mission_summary_file import observe_controller_payload

                _sum_lines, _sum_path = observe_controller_payload(
                    controller_energy_log, payload
                )
                if any(line.startswith("MISSION_SUMMARY") for line in _sum_lines):
                    print(f"MISSION_SUMMARY written → {_sum_path}")
            except Exception as exc:
                print(f"MISSION_SUMMARY file skipped ({type(exc).__name__}: {exc})")
        result = publish_telemetry(payload, dashboard)
        if result.get("ok", False):
            publish_fail_streak = 0
        else:
            publish_fail_streak += 1
            if publish_fail_streak == 1 or publish_fail_streak % 20 == 0:
                print(f"Twin publish failed (non-fatal): {result.get('error', result)}")
    st.publish_fail_streak = publish_fail_streak
