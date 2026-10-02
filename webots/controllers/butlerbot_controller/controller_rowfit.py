"""Row-fit lane mode glue for the Webots controller. No physics, no Webots import.

Row-fit is the DEFAULT lane mode (``RBM_LANE_MODE`` unset, empty or
``rowfit``). ``RBM_LANE_MODE=gap`` (process env or the repo ``.env``) selects
the old pixel-fan lane keep, unchanged.

* :func:`lane_mode`   — resolve the mode once at start-up.
* :meth:`RowfitRuntime.bind_cameras` — build the camera models from the LIVE
  camera pose (Supervisor), falling back to the loaded .wbt, then constants.
* :func:`log_dir`     — ``RBM_LOG_DIR`` or the usual Grok Workspace folder.
* :class:`RowfitRuntime` — per-frame LaneTracker + RowfitController.
* :class:`LaneVisionLog` — appends ``lane-vision.csv`` (one run_id per process).
"""
from __future__ import annotations

import math
import os
import time

try:
    from twin_publisher import project_env
except Exception:  # pragma: no cover - twin_publisher sits next to this file
    def project_env(name, default=None):  # type: ignore[misc]
        return os.environ.get(name) or default

LANE_MODES = ("gap", "rowfit")
DEFAULT_LANE_MODE = "rowfit"
# Webots DEF names of the lane cameras (only used if getFromDevice is missing).
CAMERA_DEFS = {"nadir_left": "NADIR_CAM_L", "nadir_right": "NADIR_CAM_R"}
DEFAULT_LOG_DIR = os.path.join(os.path.expanduser("~"), "OneDrive", "Desktop", "Grok Workspace")
LANE_VISION_LOG_NAME = "lane-vision.csv"
LANE_VISION_HEADER = (
    "unix_s,x_m,y_m,mode,offset_m,heading_rad,curvature,lookahead_m,confidence,"
    "nL_pts,nR_pts,steer,target_speed,new_frame,run_id"
)
# Controller process start = run label in lane-vision.csv.
RUN_ID = time.strftime("%Y%m%d-%H%M%S")


def lane_mode() -> str:
    raw = (project_env("RBM_LANE_MODE", DEFAULT_LANE_MODE) or DEFAULT_LANE_MODE).strip().lower()
    if not raw:
        return DEFAULT_LANE_MODE
    if raw not in LANE_MODES:
        print(f"WARNING: RBM_LANE_MODE={raw!r} unknown — using {DEFAULT_LANE_MODE!r}")
        return DEFAULT_LANE_MODE
    return raw


def guard_per_frame() -> bool:
    """Opt-in NadirGuard per-frame unison counting for the gap path."""
    raw = (project_env("RBM_NADIR_GUARD_PER_FRAME", "") or "").strip().lower()
    return raw in ("1", "true", "yes", "on")


def log_dir() -> str:
    d = project_env("RBM_LOG_DIR")
    if d:
        d = os.path.expanduser(os.path.expandvars(d))
        try:
            os.makedirs(d, exist_ok=True)
        except OSError:
            pass
        return d
    return DEFAULT_LOG_DIR


def _fmt(v, nd=4) -> str:
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        return f"{v:.{nd}f}" if math.isfinite(v) else ""
    return str(v)


class LaneVisionLog:
    def __init__(self, path: str | None = None, run_id: str = RUN_ID) -> None:
        self.path = path or os.path.join(log_dir(), LANE_VISION_LOG_NAME)
        self.run_id = run_id
        self._ready = False

    def write(self, *, x_m, y_m, mode, est, cmd: dict, new_frame: bool) -> None:
        try:
            if not self._ready:
                new = not os.path.isfile(self.path) or os.path.getsize(self.path) == 0
                with open(self.path, "a", encoding="ascii") as fh:
                    if new:
                        fh.write(LANE_VISION_HEADER + "\n")
                self._ready = True
            valid = est is not None and getattr(est, "valid", False)
            vals = [
                f"{time.time():.3f}",
                _fmt(x_m, 3),
                _fmt(y_m, 3),
                str(mode),
                _fmt(est.offset_m if valid else None),
                _fmt(est.heading_rad if valid else None),
                _fmt(est.curvature_1pm if valid else None),
                _fmt(est.lookahead_m if valid else None, 3),
                _fmt(est.confidence if est is not None else 0.0, 3),
                _fmt(est.n_left if est is not None else 0),
                _fmt(est.n_right if est is not None else 0),
                _fmt(cmd.get("steer")),
                _fmt(cmd.get("target_speed"), 3),
                "1" if new_frame else "0",
                self.run_id,
            ]
            with open(self.path, "a", encoding="ascii") as fh:
                fh.write(",".join(vals) + "\n")
        except OSError:
            pass


class RowfitRuntime:
    """Owns the tracker/controller; fed images on harvest ticks, wheels every tick."""

    def __init__(self) -> None:
        from src.lane_vision import NADIR_CAMS, LaneTracker
        from src.rowfit_control import RowfitController

        self.cams_model = dict(NADIR_CAMS)
        self.pose_source: dict[str, str] = {n: "constants" for n in self.cams_model}
        self.tracker = LaneTracker(self.cams_model)
        self.ctl = RowfitController()
        self.est = None  # last estimate (for logs / HUD)
        self._pending = None
        self._has_pending = False
        self._odo_dx = 0.0
        self._frame_yaw: float | None = None
        self._frame_t: float | None = None
        self.frame_dt = 0.32
        self.frames = 0

    def bind_cameras(self, robot, cams: dict, *, log=print) -> list[str]:
        """Rebuild the camera models from the live robot and re-derive the look-ahead window.

        Pose per camera: Supervisor node fields (relative to the Robot node)
        -> the loaded world's .wbt -> NADIR_*_POSE constants. Width, height
        and fov come from the device. Prints one ``CAM POSE`` line per camera
        and returns warnings (missing device, wbt/constants fallback).
        """
        from src.lane_vision import (
            LaneTracker,
            camera_coverage,
            camera_look_angles,
            camera_model_from_device,
        )
        from src.rowfit_control import lookahead_bounds_from_coverage

        msgs: list[str] = []
        models = {}
        for name in list(self.cams_model):
            dev = cams.get(name)
            if dev is None:
                msgs.append(f"{name}: device missing")
            why: list[str] = []
            try:
                model, src = camera_model_from_device(
                    robot, dev, name, def_name=CAMERA_DEFS.get(name), errors=why
                )
            except Exception as exc:
                msgs.append(f"{name}: pose lookup failed ({exc}) — keeping constants")
                model, src = self.cams_model[name], "constants"
            models[name] = model
            self.pose_source[name] = src
            pitch, yaw = camera_look_angles(model)
            t = ", ".join(f"{v:.4f}" for v in model.translation)
            rot = " ".join(f"{v:.6g}" for v in model.rotation)
            cov = camera_coverage(model)
            log(
                f"CAM POSE {name} from {src}: t=({t}) rot=({rot}) "
                f"pitch={math.degrees(pitch):.1f}deg yaw={math.degrees(yaw):+.1f}deg "
                f"{model.width}x{model.height} fov={model.fov_rad:.3f} | floor rows "
                f"bottom {_fmt(cov['bottom_row_x_m'], 2)} m, mid {_fmt(cov['mid_row_x_m'], 2)} m, "
                f"top {_fmt(cov['top_row_x_m'], 2)} m"
            )
            if src != "supervisor":
                reason = "; ".join(why) or "no reason recorded"
                msgs.append(f"{name}: live pose not read from supervisor, using {src} ({reason})")
        self.cams_model = models
        self.tracker = LaneTracker(models)
        ld_min, ld_max = lookahead_bounds_from_coverage([camera_coverage(m) for m in models.values()])
        self.ctl.ld_min, self.ctl.ld_max = ld_min, ld_max
        tr = self.tracker
        log(
            f"ROWFIT look-ahead {ld_min:.2f}–{ld_max:.2f} m, near fit x<={tr.near_max_x:.2f} m, "
            f"far fit x>={tr.far_min_x:.2f} m (from camera coverage)"
        )
        return msgs

    def check_cameras(self, cams: dict) -> list[str]:
        """Warn if the live devices disagree with the active camera models."""
        msgs = []
        for name, model in self.cams_model.items():
            cam = cams.get(name)
            if cam is None:
                msgs.append(f"{name}: device missing")
                continue
            try:
                w, h, fov = int(cam.getWidth()), int(cam.getHeight()), float(cam.getFov())
            except Exception:
                continue
            if (w, h) != (model.width, model.height) or abs(fov - model.fov_rad) > 1e-3:
                msgs.append(
                    f"{name}: live {w}x{h} fov {fov:.3f} != model "
                    f"{model.width}x{model.height} fov {model.fov_rad:.3f}"
                )
        return msgs

    def odometry(self, left_wv: float, right_wv: float, dt: float, wheel_r: float = 0.08) -> None:
        self._odo_dx += 0.5 * (float(left_wv) + float(right_wv)) * wheel_r * float(dt)

    def harvest(self, cams: dict, *, yaw: float, sim_t: float | None = None):
        images = {}
        for name in self.cams_model:
            cam = cams.get(name)
            if cam is None:
                continue
            try:
                img = cam.getImage()
                images[name] = (img, int(cam.getWidth()), int(cam.getHeight()))
            except Exception:
                continue
        dyaw = 0.0
        if self._frame_yaw is not None:
            dyaw = math.atan2(math.sin(yaw - self._frame_yaw), math.cos(yaw - self._frame_yaw))
        if sim_t is not None and self._frame_t is not None and sim_t > self._frame_t:
            self.frame_dt = float(sim_t) - float(self._frame_t)
        self._frame_t = sim_t
        self._frame_yaw = float(yaw)
        dx = self._odo_dx
        self._odo_dx = 0.0
        est = self.tracker.process(images, dx_m=dx, dyaw_rad=dyaw)
        self.est = est
        self._pending = est
        self._has_pending = True
        self.frames += 1
        return est

    def command(self, dt: float) -> dict:
        new = self._has_pending
        est = self._pending if new else None
        self._has_pending = False
        self._pending = None
        cmd = self.ctl.step(est, new_frame=new, dt=dt, frame_dt=self.frame_dt)
        return cmd

    def fill_eyes(self, lane_eyes: dict) -> None:
        est = self.est
        lane_eyes["error_source"] = "rowfit"
        if est is not None and getattr(est, "held", False):
            lane_eyes["rowfit_text"] = [
                "rowfit: HELD (fit rejected)",
                (est.reject_reason or "")[:28],
                f"off {est.offset_m * 100:+.0f}cm hd {math.degrees(est.heading_rad):+.0f}d",
            ]
            lane_eyes["rowfit_px"] = {}
            return
        if est is None or not est.valid:
            lane_eyes["rowfit_text"] = ["rowfit: no lane"]
            lane_eyes["rowfit_px"] = {}
            return
        lane_eyes["rowfit_px"] = est.pixels
        lane_eyes["rowfit_text"] = [
            f"off {est.offset_m * 100:+.0f}cm hd {math.degrees(est.heading_rad):+.0f}d",
            f"k {est.curvature_1pm:+.2f} conf {est.confidence:.2f}",
            f"L{est.n_left} R{est.n_right} look {est.lookahead_m:.1f}m",
        ]
        lane_eyes["rowfit_offset_m"] = round(est.offset_m, 4)
