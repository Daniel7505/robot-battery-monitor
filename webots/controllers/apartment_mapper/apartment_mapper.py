"""Explore the apartment and build a 2D occupancy grid.

Sensors: oak_depth, oak_rgb, oak_gyro, oak_imu roll/pitch, the two wheel
position sensors, a ring of short-range sonars, and a downward floor
camera. Wheel angles are quantized in the odometry module. The inertial
unit's yaw is not used for heading. A separate controller records the
simulator pose for scoring; this process does not.

Stop condition: no reachable frontier longer than 0.3 m, or
RBM_MAP_MAX_S if that is set. Artifacts go to logs/mapping_runs/.
"""

from __future__ import annotations

import os
import sys
import traceback

import numpy as np
from controller import Robot

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from src.mapping.body import FLOOR_CAM, OAK_DEPTH
from src.mapping.bubble import _FRONT, _LEFT, _REAR, _RIGHT
from src.mapping.explore import Explorer
from src.mapping.run_log import RunLog, default_run_dir, publish_current


_DEPTH_PERIOD_S = 0.096
_UPPER = ("torso_joint", "left_arm", "right_arm")


def _env_float(name: str) -> float | None:
    raw = (os.environ.get(name) or "").strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        print(f"MAPPING ignoring {name}={raw!r}")
        return None


def _depth_image(device, width: int, height: int):
    try:
        raw = device.getRangeImage(data_type="buffer")
        flat = np.frombuffer(raw, dtype=np.float32)
        if flat.size == width * height:
            return np.array(flat, dtype=np.float64).reshape(height, width)
    except Exception:
        pass
    try:
        raw = device.getRangeImage()
    except Exception:
        return None
    flat = np.asarray(raw, dtype=np.float64).ravel()
    if flat.size != width * height:
        return None
    return flat.reshape(height, width)


def _radar_blocked(radar) -> bool:
    if radar is None:
        return False
    try:
        targets = radar.getTargets()
    except Exception:
        return False
    if not targets:
        return False
    for target in targets:
        try:
            dist = float(target.distance)
            az = float(target.azimuth)
        except Exception:
            continue
        if dist < 0.45 and abs(az) < 0.6:
            return True
    return False


_PROXIMITY = _FRONT + _LEFT + _REAR + _RIGHT


def _flag(name: str, default: bool = True) -> bool:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return default
    return str(raw).strip().lower() not in ("0", "false", "no", "off")


def _enable(robot, name: str, period: int):
    try:
        device = robot.getDevice(name)
    except Exception:
        return None
    if device is None:
        return None
    try:
        device.enable(period)
    except Exception:
        return None
    return device


def _camera_gray(device, width: int, height: int):
    if device is None:
        return None
    try:
        raw = device.getImage()
    except Exception:
        return None
    if raw is None:
        return None
    flat = np.frombuffer(raw, dtype=np.uint8)
    if flat.size < width * height * 4:
        return None
    bgra = flat[: width * height * 4].reshape(height, width, 4)
    return bgra[:, :, :3].mean(axis=2)


def _proximity(devices: dict) -> dict:
    ranges = {}
    for name, device in devices.items():
        try:
            value = float(device.getValue())
        except Exception:
            continue
        if value == value and 0.0 < value < 1.15:
            ranges[name] = value
    return ranges


def _hold_upper(robot) -> None:
    for name in _UPPER:
        try:
            motor = robot.getDevice(name)
        except Exception:
            continue
        if motor is None:
            continue
        try:
            motor.setPosition(0.0)
            motor.setVelocity(1.0)
        except Exception:
            continue


def _finish(folder: str, now: float) -> None:
    try:
        with open(os.path.join(folder, "mapping_done"), "w", encoding="utf-8") as handle:
            handle.write(f"{now:.3f}\n")
    except OSError:
        pass


def _straight_probe(robot, timestep_ms: int, log: RunLog, folder: str, seconds: float) -> None:
    """Drive straight on open floor. Used to measure wheel contact and slip.

    Both hubs get the same speed. The pose logger records true speed and
    which solids are touching. This does not build a map.
    """
    from src.mapping.body import FLOOR_CAM, WHEEL_RADIUS_M
    from src.mapping.floor_flow import FloorFlow

    left_motor = robot.getDevice("left_wheel")
    right_motor = robot.getDevice("right_wheel")
    left_enc = robot.getDevice("left_wheel_sensor")
    right_enc = robot.getDevice("right_wheel_sensor")
    gyro = robot.getDevice("oak_gyro")
    # Same speed on both hubs. 0.22 m/s matches the mapping cruise.
    omega = 0.22 / WHEEL_RADIUS_M
    print(f"MAPPING straight probe {seconds:.1f}s omega={omega:.2f}")
    raw_yaw = 0.0
    prev_t = None
    flow = FloorFlow(noise_px=0.0)
    floor = None
    try:
        floor = robot.getDevice("floor_cam")
    except Exception:
        floor = None
    flow_dx = 0.0
    flow_frames = 0
    flow_std = 0.0
    while robot.step(timestep_ms) != -1:
        now = float(robot.getTime())
        dt = 0.0 if prev_t is None else now - prev_t
        prev_t = now
        try:
            raw_yaw += float(gyro.getValues()[2]) * dt
        except Exception:
            pass
        driving = 0.4 <= now <= 0.4 + float(seconds)
        cmd = omega if driving else 0.0
        left_motor.setVelocity(cmd)
        right_motor.setVelocity(cmd)
        log.write_wheels(now, float(left_enc.getValue()), float(right_enc.getValue()), cmd, cmd)
        if driving and floor is not None and dt > 0.0 and int(now * 8) != int((now - dt) * 8):
            image = _camera_gray(floor, FLOOR_CAM.width, FLOOR_CAM.height)
            if image is not None:
                flow_std = float(np.std(image))
                sample = flow.measure(image)
                if sample is not None and sample.ok:
                    flow_dx += float(sample.dx_m)
                    flow_frames += 1
        if now >= 0.4 + float(seconds) + 0.8:
            note = (
                f"t={now:.2f} gyro_raw_yaw={raw_yaw:.4f} "
                f"floor_dx={flow_dx:.3f} floor_frames={flow_frames} floor_std={flow_std:.1f}\n"
            )
            print("MAPPING straight probe done " + note.strip())
            try:
                with open(os.path.join(folder, "floor_probe.txt"), "w", encoding="utf-8") as handle:
                    handle.write(note)
            except OSError:
                pass
            _finish(folder, now)
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            log.close()
            return


def main() -> None:
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0
    left_motor = robot.getDevice("left_wheel")
    right_motor = robot.getDevice("right_wheel")
    for motor in (left_motor, right_motor):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    _hold_upper(robot)
    left_enc = robot.getDevice("left_wheel_sensor")
    right_enc = robot.getDevice("right_wheel_sensor")
    left_enc.enable(timestep)
    right_enc.enable(timestep)
    gyro = robot.getDevice("oak_gyro")
    gyro.enable(timestep)
    imu = robot.getDevice("oak_imu")
    imu.enable(timestep)
    depth = robot.getDevice("oak_depth")
    depth.enable(int(_DEPTH_PERIOD_S * 1000))
    width = int(depth.getWidth()) if hasattr(depth, "getWidth") else OAK_DEPTH.width
    height = int(depth.getHeight()) if hasattr(depth, "getHeight") else OAK_DEPTH.height
    radar = _enable(robot, "radar_fwd", timestep)
    use_floor = _flag("RBM_FLOOR", True)
    prox_devices = {}
    for name in dict.fromkeys(_PROXIMITY):
        device = _enable(robot, name, timestep)
        if device is not None:
            prox_devices[name] = device
    rgb = _enable(robot, "oak_rgb", int(_DEPTH_PERIOD_S * 1000))
    floor_cam = _enable(robot, "floor_cam", timestep) if use_floor else None
    rgb_w = int(rgb.getWidth()) if rgb is not None and hasattr(rgb, "getWidth") else OAK_DEPTH.width
    rgb_h = int(rgb.getHeight()) if rgb is not None and hasattr(rgb, "getHeight") else OAK_DEPTH.height
    floor_w = int(floor_cam.getWidth()) if floor_cam is not None and hasattr(floor_cam, "getWidth") else FLOOR_CAM.width
    floor_h = int(floor_cam.getHeight()) if floor_cam is not None and hasattr(floor_cam, "getHeight") else FLOOR_CAM.height

    stride = int(_env_float("RBM_MAP_STRIDE") or 4)
    max_s = _env_float("RBM_MAP_MAX_S")
    run_dir = (os.environ.get("RBM_MAP_DIR") or "").strip()
    folder = run_dir if run_dir else default_run_dir(_PROJECT_ROOT)
    publish_current(_PROJECT_ROOT, folder)
    log = RunLog(folder)
    explorer = Explorer(
        stride=max(1, stride),
        fusion=_flag("RBM_FUSION", True),
        floor=use_floor,
        loop=_flag("RBM_LOOP", True),
    )
    print(f"MAPPING run folder: {folder}")
    straight_s = _env_float("RBM_STRAIGHT_S")
    if straight_s is not None and straight_s > 0.0:
        _straight_probe(robot, timestep, log, folder, straight_s)
        return
    print(f"MAPPING depth {width}x{height} stride {explorer.stride} period {_DEPTH_PERIOD_S:.3f}s")
    next_depth = _DEPTH_PERIOD_S
    next_save = 5.0
    next_note = 2.0
    saved = False
    try:
        while robot.step(timestep) != -1:
            now = float(robot.getTime())
            image = None
            if now + 1e-6 >= next_depth:
                image = _depth_image(depth, width, height)
                next_depth = now + _DEPTH_PERIOD_S
            try:
                rpy = imu.getRollPitchYaw()
                roll = float(rpy[0]) if rpy[0] == rpy[0] else 0.0
                pitch = float(rpy[1]) if rpy[1] == rpy[1] else 0.0
            except Exception:
                roll, pitch = 0.0, 0.0
            try:
                yaw_rate = float(gyro.getValues()[2])
            except Exception:
                yaw_rate = 0.0
            left_cmd, right_cmd = explorer.step(
                dt_s=dt,
                left_wheel_rad=float(left_enc.getValue()),
                right_wheel_rad=float(right_enc.getValue()),
                yaw_rate_rad_s=yaw_rate,
                roll_rad=roll,
                pitch_rad=pitch,
                depth=image,
                radar_blocked=_radar_blocked(radar),
                proximity=_proximity(prox_devices),
                floor=_camera_gray(floor_cam, floor_w, floor_h),
                rgb=_camera_gray(rgb, rgb_w, rgb_h) if image is not None else None,
            )
            left_motor.setVelocity(left_cmd)
            right_motor.setVelocity(right_cmd)
            log.write_wheels(
                now,
                float(left_enc.getValue()),
                float(right_enc.getValue()),
                left_cmd,
                right_cmd,
            )
            x, y, yaw = explorer.pose()
            log.write_pose(now, x, y, yaw)
            odom = explorer.odometry
            fuse = explorer._fusion
            log.write_yaw(
                now,
                odom.yaw_unwrapped_rad,
                odom.gyro_raw_yaw,
                odom.gyro_unbiased_yaw,
                odom.wheel_yaw,
                fuse.visual_meas_yaw,
                fuse.visual_applied_yaw,
                fuse.floor_applied_yaw,
                explorer.wall_applied_yaw,
                odom.bias_rad_s,
                odom.still_s,
                odom.moving_s,
            )
            timed_out = max_s is not None and now >= max_s
            if timed_out and not explorer.done:
                explorer.done = True
                explorer.phase = "done"
                print(f"MAPPING time cap {max_s:.1f}s")
            if explorer.done and not saved:
                log.save_map(explorer.grid.labels(), explorer.grid.meta())
                saved = True
                print(f"MAPPING done t={now:.1f}s pose=({x:.2f},{y:.2f},{yaw:.2f})")
                # This controller cannot stop the world. The pose logger
                # watches this file and quits.
                _finish(folder, now)
            elif now >= next_save:
                log.save_map(explorer.grid.labels(), explorer.grid.meta())
                next_save = now + 5.0
            if now >= next_note:
                nose, left_c, right_c = explorer._last_clear
                cmd_l, cmd_r = explorer._commanded
                gap = explorer._bubble_gap
                gap_txt = "nan" if gap is None else f"{gap:.2f}"
                print(
                    f"MAPPING t={now:.1f}s phase={explorer.phase} pose=({x:.2f},{y:.2f}) "
                    f"nose={nose:.2f} L={left_c:.2f} R={right_c:.2f} bubble={gap_txt} "
                    f"cmd=({cmd_l:.2f},{cmd_r:.2f})"
                )
                next_note = now + 2.0
            if explorer.done:
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
    except Exception:
        traceback.print_exc()
    finally:
        try:
            log.save_map(explorer.grid.labels(), explorer.grid.meta())
        except Exception:
            traceback.print_exc()
        log.close()


if __name__ == "__main__":
    main()
