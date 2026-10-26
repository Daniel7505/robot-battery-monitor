"""Explore the apartment and build a 2D occupancy grid.

Sensors: oak_depth, oak_gyro, oak_imu roll/pitch, and the two wheel
position sensors. Wheel angles are quantized in the odometry module.
The inertial unit's yaw is not used for heading. A separate controller
records the simulator pose for scoring; this process does not.

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

from src.mapping.body import OAK_DEPTH
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
    radar = None
    try:
        radar = robot.getDevice("radar_fwd")
        radar.enable(timestep)
    except Exception:
        radar = None

    stride = int(_env_float("RBM_MAP_STRIDE") or 4)
    max_s = _env_float("RBM_MAP_MAX_S")
    run_dir = (os.environ.get("RBM_MAP_DIR") or "").strip()
    folder = run_dir if run_dir else default_run_dir(_PROJECT_ROOT)
    publish_current(_PROJECT_ROOT, folder)
    log = RunLog(folder)
    explorer = Explorer(stride=max(1, stride))
    print(f"MAPPING run folder: {folder}")
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
            )
            left_motor.setVelocity(left_cmd)
            right_motor.setVelocity(right_cmd)
            x, y, yaw = explorer.pose()
            log.write_pose(now, x, y, yaw)
            timed_out = max_s is not None and now >= max_s
            if timed_out and not explorer.done:
                explorer.done = True
                explorer.phase = "done"
                print(f"MAPPING time cap {max_s:.1f}s")
            if explorer.done and not saved:
                log.save_map(explorer.grid.labels(), explorer.grid.meta())
                saved = True
                print(f"MAPPING done t={now:.1f}s pose=({x:.2f},{y:.2f},{yaw:.2f})")
            elif now >= next_save:
                log.save_map(explorer.grid.labels(), explorer.grid.meta())
                next_save = now + 5.0
            if now >= next_note:
                print(f"MAPPING t={now:.1f}s phase={explorer.phase} pose=({x:.2f},{y:.2f})")
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
