"""Fuse wheel travel, gyro yaw, visual odometry, and the floor sensor.

The pose still lives in the wheel-and-gyro odometry. This is the update
that used to be a capped scan-match step. Each measurement is a body
displacement over the same interval as the wheels. The gain is a scalar
Kalman weight from a 3-state covariance (x, y, yaw). When the floor or
the camera says the scene did not move while the wheels turned, the
wheel translation is undone and counted as slip.
"""

from __future__ import annotations

import math

from src.mapping.floor_flow import FloorFlow
from src.mapping.odometry import WheelGyroOdometry
from src.mapping.visual_odometry import VisualOdometry


class _Interval:
    def __init__(self) -> None:
        self.raw_m = 0.0
        self.applied_m = 0.0
        self.yaw_rad = 0.0
        self.armed = False

    def add(self, raw_m: float, applied_m: float, yaw_rad: float) -> None:
        self.raw_m += float(raw_m)
        self.applied_m += float(applied_m)
        self.yaw_rad += float(yaw_rad)

    def clear(self) -> None:
        self.raw_m = 0.0
        self.applied_m = 0.0
        self.yaw_rad = 0.0
        self.armed = True


class MotionFusion:
    def __init__(self, *, floor: bool = True, visual: bool = True, rng=None) -> None:
        self.enabled = bool(visual or floor)
        self.use_floor = bool(floor)
        self.use_visual = bool(visual)
        self.floor = FloorFlow(rng=rng)
        self.visual = VisualOdometry()
        self.p_xy = 0.01
        self.p_yaw = 0.002
        self.slip_events = 0
        self.floor_updates = 0
        self.visual_updates = 0
        # Yaw the fusion actually wrote, and the yaw the camera measured.
        # The floor camera is a translation sensor. Its applied yaw stays 0.
        self.visual_applied_yaw = 0.0
        self.visual_meas_yaw = 0.0
        self.floor_applied_yaw = 0.0
        self.landmarks = None
        self.last_slip = ""
        # Latest scene translation, even when a fast yaw keeps it out of the pose.
        # None means this step had no reading. A pin needs all three.
        self.scene_floor_m: float | None = None
        self.scene_visual_m: float | None = None
        self._floor = _Interval()
        self._visual = _Interval()
        self._visual_still = 0
        self._floor_dx = 0.0
        self._floor_dy = 0.0
        self._floor_q = 0.0

    def observe(
        self,
        odom: WheelGyroOdometry,
        *,
        ds_raw_m: float,
        ds_applied_m: float,
        dyaw_rad: float,
        floor=None,
        depth=None,
        camera=None,
        rgb=None,
    ) -> str:
        """Fold this step into the pose. Returns ``slip``, ``floor``, ``visual``, or ``''``."""
        self.landmarks = None
        self.last_slip = ""
        self.scene_floor_m = None
        self.scene_visual_m = None
        if not self.enabled:
            return ""
        self._floor.add(ds_raw_m, ds_applied_m, dyaw_rad)
        self._visual.add(ds_raw_m, ds_applied_m, dyaw_rad)
        self.p_xy += 0.002 * abs(float(ds_raw_m)) + 1e-6
        self.p_yaw += 5e-4 * abs(float(dyaw_rad)) + 1e-7
        note = ""
        if self.use_floor and floor is not None:
            noted = self._observe_floor(odom, floor)
            if noted:
                note = noted
        if self.use_visual and depth is not None and camera is not None:
            sample_v = self.visual.measure(depth, camera, rgb)
            if sample_v is None:
                self._visual.clear()
            elif sample_v.reason == "still":
                # An unchanged picture with no corners is a pin vote.
                # It is not a pose update: writing dyaw 0 here pulled the
                # heading by several degrees on a flat wall.
                self.landmarks = sample_v.landmarks
                self.scene_visual_m = 0.0
                self._visual.clear()
                sample_v = None
            else:
                self.landmarks = sample_v.landmarks
                if sample_v.ok:
                    self.visual_meas_yaw += float(sample_v.dyaw_rad)
                    # The pose update below ignores a fast yaw. The pin
                    # still needs the translation the corners measured.
                    self.scene_visual_m = math.hypot(float(sample_v.dx_m), float(sample_v.dy_m))
                expect_px = 0.0
                if camera is not None and sample_v.range_m > 0.2:
                    expect_px = abs(self._visual.raw_m) / sample_v.range_m * float(camera.focal_px)
                noted = self._apply_shift(
                    odom,
                    self._visual,
                    sample_v.dx_m,
                    sample_v.dy_m,
                    sample_v.dyaw_rad,
                    ok=sample_v.ok,
                    quality=sample_v.quality,
                    kind="visual",
                    var_xy=0.008,
                    var_yaw=0.01,
                    pixel_shift=sample_v.pixel_shift,
                    expect_px=expect_px,
                )
                if noted:
                    note = noted
                self._visual.clear()
                if note == "slip":
                    self._floor.applied_m = 0.0
                    self._floor_dx = 0.0
                    self._floor_dy = 0.0
        return note

    def _reset_floor(self) -> None:
        self._floor.clear()
        self._floor_dx = 0.0
        self._floor_dy = 0.0
        self._floor_q = 0.0

    def _observe_floor(self, odom: WheelGyroOdometry, floor) -> str:
        """Integrate the downward camera until the wheels have moved enough to judge.

        The apartment world steps at 8 ms. One frame at cruise is a couple
        of millimetres, under the stall threshold, so a per-frame check
        never flags slip. The shift is summed until the hubs claim about
        2 cm, then compared once. A fast yaw, or a stretch of floor with
        nothing to track, drops the window instead of calling it a stall.
        """
        sample = self.floor.measure(floor)
        if sample is None:
            if not self._floor.armed:
                self._reset_floor()
            return ""
        if sample.ok:
            self.scene_floor_m = math.hypot(float(sample.dx_m), float(sample.dy_m))
        if not sample.ok or abs(self._floor.yaw_rad) > 0.08:
            if abs(self._floor.raw_m) > 0.05 or abs(self._floor.yaw_rad) > 0.08:
                self._reset_floor()
            return ""
        self._floor_dx += float(sample.dx_m)
        self._floor_dy += float(sample.dy_m)
        self._floor_q = float(sample.quality)
        if abs(self._floor.raw_m) < 0.02:
            return ""
        raw_m = self._floor.raw_m
        measured = self._floor_dx
        note = self._apply_shift(
            odom,
            self._floor,
            self._floor_dx,
            self._floor_dy,
            0.0,
            ok=True,
            quality=self._floor_q,
            kind="floor",
            var_xy=0.004 if self._floor_q > 0.4 else 0.05,
            var_yaw=10.0,
        )
        if note != "slip":
            self.floor.learn_scale(raw_m, measured)
        self._reset_floor()
        if note == "slip":
            self._visual.applied_m = 0.0
        return note

    def _apply_shift(
        self,
        odom: WheelGyroOdometry,
        interval: _Interval,
        dx_m: float,
        dy_m: float,
        dyaw_rad: float,
        *,
        ok: bool,
        quality: float,
        kind: str,
        var_xy: float,
        var_yaw: float,
        pixel_shift: float = 0.0,
        expect_px: float = 0.0,
    ) -> str:
        if not ok or quality < 0.15:
            return ""
        raw = interval.raw_m
        applied = interval.applied_m
        # A fast yaw between two frames breaks a short patch search and a
        # floor phase correlation. The fit looks like a sideways shove and
        # walks the pose in the middle of a scan. Wheels and the gyro own
        # that step.
        if abs(interval.yaw_rad) > 0.08:
            return ""
        scene = math.hypot(float(dx_m), float(dy_m))
        # Wheels turned, the sensor says the body did not. Drop the travel.
        # A forward camera only counts when that travel should have moved
        # the tracked pixels by several pixels and they stayed put.
        visible_stall = kind != "visual" or (expect_px >= 3.0 and pixel_shift < 0.75)
        if visible_stall and abs(raw) > 0.012 and scene < 0.30 * abs(raw) and abs(float(dyaw_rad)) < 0.05:
            # One missed track looks like a still scene. The floor sensor is
            # a direct ground measurement, so it counts immediately. The
            # forward camera has to say so twice.
            if kind == "visual":
                self._visual_still += 1
                if self._visual_still < 2 or quality < 0.45:
                    return ""
            self._visual_still = 0
            self._add_body(odom, -applied, 0.0, 0.0)
            self.slip_events += 1
            self.last_slip = kind
            return "slip"
        self._visual_still = 0
        var = float(var_xy)
        if kind == "floor" and abs(interval.yaw_rad) > 0.04:
            var += 0.05
        innov_x = float(dx_m) - applied
        innov_y = float(dy_m)
        innov_w = float(dyaw_rad) - interval.yaw_rad
        if kind == "floor":
            innov_w = 0.0
        # A stall was handled above. Anything else that disagrees is not
        # applied: a wrong visual step walks the pose, and that is worse
        # than keeping the wheels.
        if abs(innov_x) > max(0.015, 0.20 * abs(raw)) or abs(innov_y) > 0.02:
            return ""
        if kind == "visual" and abs(innov_w) > 0.04:
            return ""
        gain_x = min(0.55, self.p_xy / (self.p_xy + var))
        gain_y = min(0.45, self.p_xy / (self.p_xy + var))
        gain_w = 0.0 if kind != "visual" else min(0.40, self.p_yaw / (self.p_yaw + float(var_yaw)))
        applied_w = gain_w * innov_w
        if kind == "floor":
            applied_w = 0.0
        else:
            self.visual_applied_yaw += applied_w
        self._add_body(odom, gain_x * innov_x, gain_y * innov_y, applied_w)
        self.p_xy *= max(0.05, 1.0 - gain_x)
        self.p_yaw *= max(0.05, 1.0 - max(gain_w, 0.0))
        if kind == "floor":
            self.floor_updates += 1
        else:
            self.visual_updates += 1
        return kind

    def _add_body(self, odom: WheelGyroOdometry, forward_m: float, lateral_m: float, dyaw_rad: float) -> None:
        yaw = odom.yaw_rad - float(dyaw_rad)
        c, s = math.cos(yaw), math.sin(yaw)
        odom.apply_correction(
            c * float(forward_m) - s * float(lateral_m),
            s * float(forward_m) + c * float(lateral_m),
            float(dyaw_rad),
        )
