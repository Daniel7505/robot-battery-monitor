"""Mission and route state for the ButlerBot Webots controller.

Finish referee, the R-key phase clock, row-fit harvest, and the
mission-complete turn log. Device reads stay in ``controller_sensing``.
"""

from __future__ import annotations

import math
import os
import time

from controller_hud import _draw_hud
from controller_inventory import InventoryRuntime
from controller_keys import (
    KEY_R,
    KEY_SPACE,
)
from controller_wheels import STOP_WHEEL_RAD_S, _clear_wheel_locks
from controller_obstacles import ObstacleRuntime, obstacles_mode as _obstacles_mode
from controller_rowfit import (
    RUN_ID as _RUN_ID,
    LaneVisionLog,
    RowfitRuntime,
    apply_lane_steer,
    lane_frame_fields,
    lane_mode as _lane_mode,
    log_dir as _log_dir,
    world_check_warnings,
)
from controller_sensing import _empty_lane_eyes, _format_pose, _imu_yaw
from twin_publisher import (
    fetch_twin_state,
    project_env,
    remote_throttle_factor,
    teleop_from_twin_state,
)

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

_FINISH_X_M = 16.5

PHASES = [
    {"name": "standby", "gait": "stand", "duration_s": 6, "drive_speed": 0.0, "turn_amp": 0.0, "arm_amp": 0.0, "torso_amp": 0.0},
    {"name": "drive_transit", "gait": "drive", "duration_s": 14, "drive_speed": 5.5, "turn_amp": 0.0, "arm_amp": 0.0, "torso_amp": 0.0},
    {"name": "patrol", "gait": "patrol", "duration_s": 12, "drive_speed": 3.8, "turn_amp": 0.28, "arm_amp": 0.0, "torso_amp": 0.0},
    {"name": "manipulate", "gait": "manipulate", "duration_s": 10, "drive_speed": 0.0, "turn_amp": 0.0, "arm_amp": 0.95, "torso_amp": 0.35},
    {"name": "return_idle", "gait": "stand", "duration_s": 10, "drive_speed": -4.5, "turn_amp": 0.0, "arm_amp": 0.0, "torso_amp": 0.0},
]

def _load_s_cross_track():
    """S centre line for the driving log when the world has no track file."""
    try:
        from src.lane_keep import track_cross_track_m

        return track_cross_track_m
    except Exception as exc:
        print(f"WARNING: S centre line not loaded ({exc})")
        return None


class _XFinish:
    """Fallback referee: the original S rule (GPS x >= _FINISH_X_M)."""

    name = "s"

    def crossed(self, x: float, y: float) -> bool:
        return float(x) >= _FINISH_X_M

    def describe(self) -> str:
        return f"finish x>={_FINISH_X_M:g} m (built-in S)"


def _load_finish_referee(robot):
    """Finish line of the loaded track (GPS referee only; never used to steer).

    RBM_TRACK (track name or file) > ``# TRACK_FILE`` tag in the loaded world
    (written by scripts/track_builder.py) > the built-in S rule.
    """
    world = None
    try:
        world = robot.getWorldPath()
    except Exception:
        world = None
    try:
        from src.track_geometry import referee_for

        return referee_for(world, finish_x_m=_FINISH_X_M)
    except Exception as exc:
        print(f"WARNING finish referee: track not loaded ({type(exc).__name__}: {exc}) — using x>={_FINISH_X_M:g} m")
        return _XFinish()


# Default folder unchanged (~/OneDrive/Desktop/Grok Workspace); RBM_LOG_DIR overrides.
_STEER_LOG = os.path.join(_log_dir(), "steer-actions.csv")
# Lane mode is row-fit. A leftover RBM_LANE_MODE=gap is ignored (see lane_mode).
_LANE_MODE = _lane_mode()
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

def _emit_turn_log(rowfit_rt) -> None:
    """Print TURN_LOG when the mission finishes.

    Webots can kill the controller before the post-loop hook, so the
    checksum has to go out on the mission-complete path, flushed, and
    into the mission file that already survives that kill.
    """
    if rowfit_rt is None:
        return
    try:
        lines = rowfit_rt.ctl.finish_turn_log()
    except Exception as exc:
        print(f"TURN_LOG skipped ({type(exc).__name__}: {exc})", flush=True)
        return
    for line in lines:
        print(line, flush=True)
    _record_mission_lines(lines)


def _record_mission_lines(lines: list[str]) -> str | None:
    """Append lines to logs/mission_summary_<ts>.json and the repo dashboard log."""
    try:
        from src.mission_summary_file import append_mission_lines

        return append_mission_lines(lines, source="controller", mirror_dashboard_log=True)
    except Exception as exc:
        print(f"MISSION log file skipped ({type(exc).__name__}: {exc})")
        return None

def begin_tick(st):
    """Advance the simulation clock and the lane-keep arming gate."""
    tick = st.tick
    timestep = st.timestep
    mission_complete = st.mission_complete
    nadir_lobe_done = st.nadir_lobe_done
    finish_age_s = st.finish_age_s
    lane_keep_on = st.lane_keep_on
    rowfit_rt = st.rowfit_rt
    finish_ref = st.finish_ref
    park_holdoff_s = st.park_holdoff_s
    drive_grace_s = st.drive_grace_s
    tick += 1
    dt = timestep / 1000.0
    mission_event = "complete" if mission_complete else None
    if nadir_lobe_done and not mission_complete:
        finish_age_s += dt
    abs_just_finished = False
    if tick == 5 and not lane_keep_on and rowfit_rt is not None:
        lane_keep_on = True
        print(
            "ROWFIT STEER ON — fitted lane (offset/heading/curvature), "
            "pure pursuit + curve speed governor, "
            f"to the finish ({finish_ref.describe()})."
        )
    if park_holdoff_s > 0.0:
        park_holdoff_s = max(0.0, park_holdoff_s - dt)
    if drive_grace_s > 0.0:
        drive_grace_s = max(0.0, drive_grace_s - dt)
    st.tick = tick
    st.dt = dt
    st.mission_event = mission_event
    st.finish_age_s = finish_age_s
    st.abs_just_finished = abs_just_finished
    st.lane_keep_on = lane_keep_on
    st.park_holdoff_s = park_holdoff_s
    st.drive_grace_s = drive_grace_s

def poll_keyboard(st):
    """Read the keyboard, then the R-key demo toggle, before any pose read."""
    keyboard = st.keyboard
    key_tracker = st.key_tracker
    auto_loop = st.auto_loop
    if keyboard is not None:
        keys, pressed = key_tracker.poll(keyboard)
    else:
        keys, pressed = set(), set()

    if KEY_R in pressed:
        auto_loop = not auto_loop
        print(f"Auto mission loop: {'ON' if auto_loop else 'OFF'}")
    st.keys = keys
    st.pressed = pressed
    st.auto_loop = auto_loop

def handle_manual_and_api(st):
    """Space/teleop brake, the R-key phase clock, then the twin state poll."""
    pressed = st.pressed
    park_holdoff_s = st.park_holdoff_s
    abs_brake = st.abs_brake
    forward_m_s = st.forward_m_s
    speed_m_s = st.speed_m_s
    last_teleop_left = st.last_teleop_left
    last_teleop_right = st.last_teleop_right
    left_wv_early = st.left_wv_early
    right_wv_early = st.right_wv_early
    key_tracker = st.key_tracker
    keys = st.keys
    settled = st.settled
    phase_idx = st.phase_idx
    auto_loop = st.auto_loop
    dt = st.dt
    phase_elapsed = st.phase_elapsed
    gps = st.gps
    keys_logged = st.keys_logged
    tick = st.tick
    state_poll_every = st.state_poll_every
    dashboard = st.dashboard
    last_stop_epoch = st.last_stop_epoch
    mission_capped = st.mission_capped
    nadir_lobe_done = st.nadir_lobe_done
    drive_grace_s = st.drive_grace_s
    last_api_sig = st.last_api_sig
    cached_api_left = st.cached_api_left
    cached_api_right = st.cached_api_right
    remote_throttle = st.remote_throttle
    battery_pct = st.battery_pct
    thermal_c = st.thermal_c
    lane_keep_on = st.lane_keep_on
    cached_api_source = st.cached_api_source
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
            # A distance cap stays stopped. A real finish can still
            # hand lane-keep back to the dashboard.
            if mission_capped:
                lane_keep_on = False
            elif not nadir_lobe_done:
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
    st.park_holdoff_s = park_holdoff_s
    st.keys = keys
    st.teleop_keys = teleop_keys
    st.stop_pressed = stop_pressed
    st.user_driving = user_driving
    st.phase = phase
    st.phase_name = phase_name
    st.gait = gait
    st.phase_elapsed = phase_elapsed
    st.phase_idx = phase_idx
    st.t = t
    st.keys_logged = keys_logged
    st.remote_throttle = remote_throttle
    st.battery_pct = battery_pct
    st.thermal_c = thermal_c
    st.lane_keep_on = lane_keep_on
    st.last_stop_epoch = last_stop_epoch
    st.cached_api_left = cached_api_left
    st.cached_api_right = cached_api_right
    st.cached_api_source = cached_api_source
    st.last_api_sig = last_api_sig
    st.last_teleop_left = last_teleop_left
    st.last_teleop_right = last_teleop_right
    st.drive_grace_s = drive_grace_s

def harvest_and_steer(st):
    """Nadir harvest and the row-fit command, including the finish / distance cap."""
    tick = st.tick
    last_harvest_tick = st.last_harvest_tick
    harvest_every = st.harvest_every
    rowfit_rt = st.rowfit_rt
    robot = st.robot
    cams = st.cams
    prev_yaw = st.prev_yaw
    lane_keep_on = st.lane_keep_on
    abs_brake = st.abs_brake
    user_driving = st.user_driving
    park_holdoff_s = st.park_holdoff_s
    gps_xy = st.gps_xy
    dt = st.dt
    imu = st.imu
    yaw_rate = st.yaw_rate
    ct_latest_m = st.ct_latest_m
    finish_ref = st.finish_ref
    mission_capped = st.mission_capped
    mission_distance_reached = st.mission_distance_reached
    mission_travel_m = st.mission_travel_m
    mission_limit_m = st.mission_limit_m
    nadir_lobe_done = st.nadir_lobe_done
    lane_eyes = st.lane_eyes
    controller_energy_log = st.controller_energy_log
    lane_vision_log = st.lane_vision_log
    speed_m_s = st.speed_m_s
    forward_m_s = st.forward_m_s
    last_teleop_left = st.last_teleop_left
    last_teleop_right = st.last_teleop_right
    left_wv_early = st.left_wv_early
    right_wv_early = st.right_wv_early
    drive_grace_s = st.drive_grace_s
    lane_fit_fields = st.lane_fit_fields
    cached_api_left = st.cached_api_left
    cached_api_right = st.cached_api_right
    cached_api_source = st.cached_api_source
    harvest_now = tick - last_harvest_tick >= harvest_every
    if rowfit_rt is not None and harvest_now:
        lane_eyes = _empty_lane_eyes()
        try:
            _sim_t = float(robot.getTime())
        except Exception:
            _sim_t = None
        rowfit_rt.harvest(cams, yaw=prev_yaw, sim_t=_sim_t)
        rowfit_rt.fill_eyes(lane_eyes)
        last_harvest_tick = tick

    if (
        lane_keep_on
        and rowfit_rt is not None
        and not abs_brake.active
        and not user_driving
        and park_holdoff_s <= 0.0
    ):
        rowfit_new = rowfit_rt._has_pending
        # Pretend-pick stop after obstacle apply (harvest), before wheel command
        if getattr(rowfit_rt, "inventory", None) is not None:
            rowfit_rt.inventory.update(gps_xy, prev_yaw, dt, ctl=rowfit_rt.ctl)
        lk = rowfit_rt.command(
            dt,
            yaw=prev_yaw if imu is not None else None,
            pose=None if gps_xy is None else (float(gps_xy[0]), float(gps_xy[1])),
            yaw_rate_rad_s=yaw_rate if imu is not None else None,
            cross_track_m=ct_latest_m,
        )
        _crossed = (
            gps_xy is not None
            and finish_ref.crossed(float(gps_xy[0]), float(gps_xy[1]))
        )
        # Stay on the brake after the cap. A later poll can turn
        # lane-keep back on for one step; the cap must still win.
        _at_cap = mission_capped or mission_distance_reached(
            mission_travel_m, mission_limit_m
        )
        if _crossed or _at_cap:
            if not nadir_lobe_done:
                nadir_lobe_done = True
                if _at_cap and not _crossed:
                    mission_capped = True
                    _gx = 0.0 if gps_xy is None else float(gps_xy[0])
                    _gy = 0.0 if gps_xy is None else float(gps_xy[1])
                    print(
                        f"MISSION LIMIT {mission_travel_m:.1f} m "
                        f"(cap {float(mission_limit_m):.1f} m) at "
                        f"x={_gx:.2f} y={_gy:.2f} — "
                        "distance cap, braking like a finish"
                    )
                else:
                    _what = "FULL S DONE" if finish_ref.name == "s" else f"TRACK {finish_ref.name} DONE"
                    _exit = finish_ref.exit_name(float(gps_xy[0]), float(gps_xy[1])) if hasattr(
                        finish_ref, "exit_name") else None
                    if _exit:
                        _what += f" via exit {_exit}"
                    print(
                        f"{_what} at x={gps_xy[0]:.2f} y={gps_xy[1]:.2f} m — "
                        "GPS finish, not a red camera. Nadir was on the wheel."
                    )
                if getattr(rowfit_rt, "inventory", None) is not None and _crossed:
                    rowfit_rt.inventory.mark_shipped()
            elif _at_cap:
                mission_capped = True
            lane_keep_on = False
            lk = {
                "left": 0.0,
                "right": 0.0,
                "brake": True,
                "steer": 0.0,
                "phase": "first_lobe_done",
                "error_source": "rowfit",
                "steer_source": "stopped",
                "steer_lateral": "stopped",
                "fit_reject": "",
                "reason": "finish",
            }
        # Every command, not only the finish ``stopped`` override.
        # Telemetry is what STEER_SOURCE integrates.
        apply_lane_steer(lane_eyes, lk)
        if rowfit_rt is not None and rowfit_new:
            try:
                _meas = lane_frame_fields(
                    rowfit_rt.est,
                    getattr(finish_ref, "track", None),
                    None if gps_xy is None else gps_xy[0],
                    None if gps_xy is None else gps_xy[1],
                    ct_latest_m,
                )
            except Exception:
                _meas = {}
            lane_fit_fields = {k: v for k, v in _meas.items() if v is not None}
            lane_fit_fields["lane_frame_n"] = int(rowfit_rt.frames)
            if controller_energy_log is not None:
                controller_energy_log.note_lane_frame(lane_fit_fields)
            if lane_vision_log is not None:
                lane_vision_log.write(
                    x_m=None if gps_xy is None else round(float(gps_xy[0]), 3),
                    y_m=None if gps_xy is None else round(float(gps_xy[1]), 3),
                    mode="rowfit",
                    est=rowfit_rt.est,
                    cmd=lk,
                    new_frame=True,
                    yaw=prev_yaw,
                    measure=_meas,
                )
            _e = rowfit_rt.est
            if _e is not None and getattr(_e, "held", False):
                _rj = _e.rejected or {}
                print(
                    f"Rowfit REJECTED fit ({_e.reject_reason}) — steer={lk.get('steer_source')} holding last lane "
                    f"off={_e.offset_m:+.3f} hd={_e.heading_rad:+.3f} k={_e.curvature_1pm:+.2f}; "
                    f"rejected off={_rj.get('offset_m', float('nan')):+.3f} "
                    f"hd={_rj.get('heading_rad', float('nan')):+.3f} "
                    f"nL={_rj.get('n_left')} nR={_rj.get('n_right')} "
                    f"steer={lk.get('steer')} phase={lk.get('phase')}"
                )
            elif rowfit_rt.frames % 3 == 1 and _e is not None:
                print(
                    "Rowfit "
                    f"L={lk['left']:.2f} R={lk['right']:.2f} steer={lk.get('steer')} "
                    f"off={_e.offset_m:+.3f} hd={_e.heading_rad:+.3f} "
                    f"k={_e.curvature_1pm:+.2f} conf={_e.confidence:.2f} "
                    f"nL={_e.n_left} nR={_e.n_right} "
                    f"w={_e.lane_width_m:.3f}"
                    + (
                        ""
                        if _meas.get("true_width_near_m") is None
                        else f" true={_meas['true_width_near_m']:.3f}"
                    )
                    + (" live" if _e.width_live else "")
                    + f" v*={lk.get('target_speed')} phase={lk.get('phase')} "
                    f"steer_source={lk.get('steer_source')}"
                    if _e.valid
                    else f"Rowfit no lane (frame {rowfit_rt.frames}) phase={lk.get('phase')}"
                )
        if lk["brake"]:
            print(
                f"Lane-keep: {lk.get('error_source') or 'nadir'} stop "
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
        else:
            cached_api_left = float(lk["left"])
            cached_api_right = float(lk["right"])
            cached_api_source = "agent"
            last_teleop_left = cached_api_left
            last_teleop_right = cached_api_right
            drive_grace_s = max(drive_grace_s, 1.0)
    st.harvest_now = harvest_now
    st.lane_eyes = lane_eyes
    st.last_harvest_tick = last_harvest_tick
    st.nadir_lobe_done = nadir_lobe_done
    st.mission_capped = mission_capped
    st.lane_keep_on = lane_keep_on
    st.lane_fit_fields = lane_fit_fields
    st.cached_api_left = cached_api_left
    st.cached_api_right = cached_api_right
    st.cached_api_source = cached_api_source
    st.last_teleop_left = last_teleop_left
    st.last_teleop_right = last_teleop_right
    st.drive_grace_s = drive_grace_s

def log_drive_and_overlays(st):
    """Driving line, speed trace, and the optional HUD gauges."""
    import controller_wheels as _wheelmod
    WHEEL_RADIUS_M = _wheelmod.WHEEL_RADIUS_M
    user_driving = st.user_driving
    dt = st.dt
    drive_log_elapsed = st.drive_log_elapsed
    gps = st.gps
    left_wv = st.left_wv
    right_wv = st.right_wv
    speed_m_s = st.speed_m_s
    speed_estimator = st.speed_estimator
    throttle_factor = st.throttle_factor
    gps_xy = st.gps_xy
    finish_ref = st.finish_ref
    _track_ct = st._track_ct
    left_v = st.left_v
    right_v = st.right_v
    imu = st.imu
    speed_trace = st.speed_trace
    robot = st.robot
    hud = st.hud
    battery_pct = st.battery_pct
    thermal_c = st.thermal_c
    agent_message = st.agent_message
    api_driving = st.api_driving
    cached_api_source = st.cached_api_source
    auto_loop = st.auto_loop
    abs_brake = st.abs_brake
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
            if gps_xy is not None:
                try:
                    # loaded track's centre line (same sampler as drift_report);
                    # the S default keeps the S formula
                    _ct = finish_ref.cross_track(gps_xy[0], gps_xy[1]) if hasattr(
                        finish_ref, "cross_track") else None
                    if _ct is None and _track_ct is not None:
                        _ct = _track_ct(gps_xy[0], gps_xy[1])
                    ct_s = "" if _ct is None else f" ct={_ct:+.3f}"
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

    for _trace_line in speed_trace.lines(
        t_sim=float(robot.getTime()),
        t_wall=time.monotonic(),
        v_body=float(speed_m_s),
        v_odo=0.5 * (abs(left_wv) + abs(right_wv)) * WHEEL_RADIUS_M,
        w_l_cmd=float(left_v),
        w_r_cmd=float(right_v),
        w_l=float(left_wv),
        w_r=float(right_wv),
    ):
        if speed_trace.console:
            print(_trace_line)

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
    st.drive_log_elapsed = drive_log_elapsed

def finish_mission_if_parked(st):
    """Parked-after-finish mission complete, turn log, and speed-trace close."""
    mission_complete = st.mission_complete
    left_wv = st.left_wv
    right_wv = st.right_wv
    user_driving = st.user_driving
    api_driving = st.api_driving
    left_v = st.left_v
    right_v = st.right_v
    nadir_lobe_done = st.nadir_lobe_done
    abs_just_finished = st.abs_just_finished
    finish_age_s = st.finish_age_s
    turn_log_sent = st.turn_log_sent
    rowfit_rt = st.rowfit_rt
    speed_trace = st.speed_trace
    mission_event = st.mission_event
    if not mission_complete:
        try:
            from src.mission_park import mission_should_complete

            hubs_quiet = (
                abs(left_wv) < STOP_WHEEL_RAD_S
                and abs(right_wv) < STOP_WHEEL_RAD_S
            )
            no_cmd = (
                not user_driving
                and not api_driving
                and abs(left_v) < 0.05
                and abs(right_v) < 0.05
            )
            # nadir_lobe_done is set on FULL S DONE / route end, or when
            # an optional distance cap (off by default) is reached.
            # A no-drive pause before that must not complete the mission.
            # After it, hub twitch and an ABS hold that never closes
            # still complete once the drive command has been zero.
            if mission_should_complete(
                track_done=nadir_lobe_done,
                already_complete=False,
                abs_done=abs_just_finished,
                hubs_quiet=hubs_quiet,
                no_drive_command=no_cmd,
                finish_age_s=finish_age_s,
            ):
                mission_complete = True
                mission_event = "complete"
                print(
                    "MISSION complete — parked after track finish; "
                    "dashboard will log MISSION_SUMMARY"
                )
        except Exception:
            if abs_just_finished and nadir_lobe_done:
                mission_complete = True
                mission_event = "complete"
    if mission_complete and not turn_log_sent:
        _emit_turn_log(rowfit_rt)
        turn_log_sent = True
    if mission_complete and not speed_trace.finished:
        _steer_line = speed_trace.finish()
        if _steer_line:
            print(_steer_line)
    st.mission_complete = mission_complete
    st.mission_event = mission_event
    st.turn_log_sent = turn_log_sent

def shutdown_run(st):
    """Final speed-trace line, turn log, and the stopped print."""
    speed_trace = st.speed_trace
    turn_log_sent = st.turn_log_sent
    rowfit_rt = st.rowfit_rt
    if not speed_trace.finished:
        _steer_line = speed_trace.finish()
        if _steer_line:
            print(_steer_line)
    if not turn_log_sent:
        _emit_turn_log(rowfit_rt)
    print("ButlerBot controller stopped (simulation ended)", flush=True)
