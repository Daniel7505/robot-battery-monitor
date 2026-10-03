"""Tiny numpy raycaster for offline stereo tests and the offline sim.

Renders what a ``CameraModel`` on the robot sees of a flat textured floor
(1 m chequer like the Webots Parquetry floor + fine grain), optional tape
stripes and axis-aligned textured boxes (obstacles, table tops and legs,
racks). Grey levels 0..255, float32. It is a stand-in for Webots rendering,
good enough to check the stereo geometry, the ground filter and the corridor
logic; it is NOT a lighting or texture model of the real scene.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.lane_vision import CameraModel


@dataclass
class SceneBox:
    """World-frame axis-aligned box (x0 < x1, y0 < y1, z0 < z1)."""

    x0: float
    y0: float
    z0: float
    x1: float
    y1: float
    z1: float
    albedo: float = 0.55
    seed: int = 1
    name: str = "box"

    @staticmethod
    def at(cx, cy, sx, sy, h, z0=0.0, **kw) -> "SceneBox":
        return SceneBox(cx - sx / 2, cy - sy / 2, z0, cx + sx / 2, cy + sy / 2, z0 + h, **kw)


def table(cx, cy, sx, sy, h=0.9, top=0.05, leg=0.05, seed=5, name="table") -> list[SceneBox]:
    """Packing table like the warehouse builder's: a top slab and four legs inset 0.05 m."""
    out = [SceneBox.at(cx, cy, sx, sy, top, z0=h - top, albedo=0.45, seed=seed, name=name + "_top")]
    for dx in (-1, 1):
        for dy in (-1, 1):
            out.append(SceneBox.at(cx + dx * (sx / 2 - 0.05), cy + dy * (sy / 2 - 0.05), leg, leg, h - top,
                                   albedo=0.4, seed=seed + 1, name=name + "_leg"))
    return out


def _hash(ix, iy, seed):
    v = np.sin(ix * 12.9898 + iy * 78.233 + seed * 37.719) * 43758.5453
    return v - np.floor(v)


def value_noise(u, v, cell, seed):
    gu, gv = u / cell, v / cell
    iu, iv = np.floor(gu), np.floor(gv)
    fu, fv = gu - iu, gv - iv
    fu = fu * fu * (3 - 2 * fu)
    fv = fv * fv * (3 - 2 * fv)
    a, b = _hash(iu, iv, seed), _hash(iu + 1, iv, seed)
    c, d = _hash(iu, iv + 1, seed), _hash(iu + 1, iv + 1, seed)
    return (a * (1 - fu) + b * fu) * (1 - fv) + (c * (1 - fu) + d * fu) * fv


def _fade(footprint, cell):
    """1 while a texture cell spans >= 2 pixels, 0 once it is under ~0.7 px (mipmap-like)."""
    return np.clip((cell / np.maximum(footprint, 1e-9) - 0.7) / 1.3, 0.0, 1.0)


def render_gray(cam: CameraModel, pose=(0.0, 0.0, 0.0), boxes=(), *, tapes=(), supersample: int = 2,
                floor: bool = True) -> np.ndarray:
    """Grey image (H, W) from robot pose (x, y, yaw) in the world."""
    W, H = cam.width, cam.height
    s = max(1, int(supersample))
    offs = (np.arange(s) + 0.5) / s - 0.5
    cols = (np.arange(W)[None, :, None, None] + offs[None, None, None, :])
    rows = (np.arange(H)[:, None, None, None] + offs[None, None, :, None])
    cols, rows = np.broadcast_arrays(cols, rows)
    R = np.array(cam.R)
    d_cam = np.stack([np.full(cols.shape, cam.focal_px), cam.cx - cols, cam.cy - rows], axis=-1)
    d_bot = d_cam @ R.T
    px, py, yaw = pose
    c, sn = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
    d = d_bot @ Rz.T
    t = np.array(cam.translation)
    o = np.array([px, py, 0.0]) + Rz @ t
    dn = np.linalg.norm(d, axis=-1)
    t_best = np.full(d.shape[:-1], np.inf)
    shade = np.full(d.shape[:-1], 200.0)  # background (walls far away / sky): flat, no texture
    with np.errstate(divide="ignore", invalid="ignore"):
        if floor:
            tf = np.where(d[..., 2] < -1e-9, -o[2] / d[..., 2], np.inf)
            hit = tf < t_best
            t_best = np.where(hit, tf, t_best)
            fx, fy = o[0] + tf * d[..., 0], o[1] + tf * d[..., 1]
            # pixel footprint on the floor (m): fades detail finer than a pixel, like GPU mipmapping
            foot = tf * dn / cam.focal_px / np.maximum(np.abs(d[..., 2]) / dn, 1e-3)
            chk = ((np.floor(fx) + np.floor(fy)) % 2 == 0)
            g = 0.535 + _fade(foot, 1.0) * np.where(chk, -0.065, 0.065) \
                + _fade(foot, 0.03) * 0.22 * (value_noise(fx, fy, 0.03, 11) - 0.5) \
                + _fade(foot, 0.11) * 0.10 * (value_noise(fx, fy, 0.11, 12) - 0.5)
            for (ax, ay, bx, by, w) in tapes:
                vx, vy = bx - ax, by - ay
                L2 = vx * vx + vy * vy
                u = np.clip(((fx - ax) * vx + (fy - ay) * vy) / max(L2, 1e-9), 0, 1)
                dist = np.hypot(fx - (ax + u * vx), fy - (ay + u * vy))
                g = np.where(dist <= w / 2, 0.85, g)
            shade = np.where(hit, 255.0 * g, shade)
        for k, b in enumerate(boxes):
            lo = np.array([b.x0, b.y0, b.z0])
            hi = np.array([b.x1, b.y1, b.z1])
            inv = 1.0 / np.where(np.abs(d) < 1e-12, 1e-12, d)
            t0 = (lo - o) * inv
            t1 = (hi - o) * inv
            tmin = np.minimum(t0, t1)
            tmax = np.maximum(t0, t1)
            tn = tmin.max(axis=-1)
            tx = tmax.min(axis=-1)
            hit = (tn <= tx) & (tn > 1e-6) & (tn < t_best)
            if not hit.any():
                continue
            axis = tmin.argmax(axis=-1)  # 0: x face, 1: y face, 2: top / bottom
            P = o + tn[..., None] * d
            u = np.where(axis == 0, P[..., 1], P[..., 0])
            v = np.where(axis == 2, P[..., 1], P[..., 2])
            lit = np.choose(axis, [0.85, 0.72, 1.0])
            cosn = np.abs(np.take_along_axis(d, axis[..., None], axis=-1)[..., 0]) / dn
            foot = tn * dn / cam.focal_px / np.maximum(cosn, 1e-3)
            g = b.albedo * lit * (1.0 + _fade(foot, 0.025) * 0.55 * (value_noise(u, v, 0.025, b.seed + 3 * axis) - 0.5)
                                  + _fade(foot, 0.09) * 0.25 * (value_noise(u, v, 0.09, b.seed + 7) - 0.5))
            shade = np.where(hit, 255.0 * g, shade)
            t_best = np.where(hit, tn, t_best)
    return np.clip(shade.mean(axis=(2, 3)), 0, 255).astype(np.float32)


def render_pair(rig, pose=(0.0, 0.0, 0.0), boxes=(), **kw):
    return render_gray(rig.left, pose, boxes, **kw), render_gray(rig.right, pose, boxes, **kw)
