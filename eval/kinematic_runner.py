"""Kinematic stand-in for a mapping run. Not used by the robot.

The true pose advances with the wheel command. Depth is a numpy ray
cast from that pose. The mapper only receives wheel angles, a yaw
rate, and the depth image. Optional noise matches the oak_* devices.
There is no wheel slip. A command that would intersect a box stalls
the hubs instead of driving through the wall.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from eval.depth_scene import Box, circle_hits, render_xdepth
from src.mapping.body import WHEEL_RADIUS_M, WHEEL_TRACK_M, DepthCamera
from src.mapping.explore import Explorer
from src.mapping.noise import noisy_depth, noisy_tilt, noisy_yaw_rate
from src.mapping.run_log import RunLog

# Hub contact. The planner inflates a little further than this.
_BODY_RADIUS_M = 0.18


@dataclass
class TruePose:
    x: float
    y: float
    yaw: float
    left_rad: float = 0.0
    right_rad: float = 0.0


@dataclass
class KinematicResult:
    explorer: Explorer
    pose: TruePose
    steps: int
    stalled: int = 0
    log: RunLog | None = None
    extra: dict = field(default_factory=dict)


def _vw(left_rad_s: float, right_rad_s: float) -> tuple[float, float]:
    v = 0.5 * (left_rad_s + right_rad_s) * WHEEL_RADIUS_M
    w = (right_rad_s - left_rad_s) * WHEEL_RADIUS_M / WHEEL_TRACK_M
    return v, w


def run_scene(
    boxes: list[Box],
    *,
    start: tuple[float, float, float] = (0.0, 0.0, 0.0),
    steps: int = 400,
    dt_s: float = 0.2,
    noise: bool = False,
    seed: int = 0,
    camera: DepthCamera,
    stride: int = 2,
    floor_z: float = 0.0,
    run_dir=None,
) -> KinematicResult:
    pose = TruePose(*start)
    explorer = Explorer(camera=camera, stride=stride)
    rng = np.random.default_rng(seed)
    yaw_rate = 0.0
    log = RunLog(run_dir) if run_dir is not None else None
    stalled = 0
    done_step = 0
    for i in range(int(steps)):
        depth = render_xdepth(camera, (pose.x, pose.y, pose.yaw), boxes, floor_z=floor_z)
        if noise:
            depth = noisy_depth(depth, rng, camera.max_range_m)
            rate = noisy_yaw_rate(yaw_rate, rng)
            roll, pitch = noisy_tilt(0.0, 0.0, rng)
        else:
            rate = yaw_rate
            roll, pitch = 0.0, 0.0
        left_cmd, right_cmd = explorer.step(
            dt_s=dt_s,
            left_wheel_rad=pose.left_rad,
            right_wheel_rad=pose.right_rad,
            yaw_rate_rad_s=rate,
            roll_rad=roll,
            pitch_rad=pitch,
            depth=depth,
        )
        v, w = _vw(left_cmd, right_cmd)
        nx = pose.x + v * math.cos(pose.yaw) * dt_s
        ny = pose.y + v * math.sin(pose.yaw) * dt_s
        if circle_hits(nx, ny, boxes, _BODY_RADIUS_M):
            stalled += 1
            yaw_rate = 0.0
        else:
            pose.x, pose.y = nx, ny
            pose.yaw += w * dt_s
            pose.left_rad += left_cmd * dt_s
            pose.right_rad += right_cmd * dt_s
            yaw_rate = w
        if log is not None:
            ox, oy, oyaw = explorer.pose()
            log.write_pose(explorer.t_s, ox, oy, oyaw)
        done_step = i + 1
        if explorer.done:
            break
    if log is not None:
        log.save_map(explorer.grid.labels(), explorer.grid.meta())
        log.close()
    return KinematicResult(explorer=explorer, pose=pose, steps=done_step, stalled=stalled, log=log)
