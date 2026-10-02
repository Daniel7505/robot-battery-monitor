"""Row-fit lane mode glue for the Webots controller. No physics, no Webots import.

Turned on with ``RBM_LANE_MODE=rowfit`` (process env or the repo ``.env``).
Anything else (or unset) keeps the default ``gap`` pixel-fan lane keep.

* :func:`lane_mode`   — resolve the mode once at start-up.
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
DEFAULT_LOG_DIR = os.path.join(os.path.expanduser("~"), "OneDrive", "Desktop", "Grok Workspace")
LANE_VISION_LOG_NAME = "lane-vision.csv"
LANE_VISION_HEADER = (
    "unix_s,x_m,y_m,mode,offset_m,heading_rad,curvature,lookahead_m,confidence,"
    "nL_pts,nR_pts,steer,target_speed,new_frame,run_id"
)
# Controller process start = run label in lane-vision.csv.
RUN_ID = time.strftime("%Y%m%d-%H%M%S")


def lane_mode() -> str:
    raw = (project_env("RBM_LANE_MODE", "gap") or "gap").strip().lower()
    if raw not in LANE_MODES:
        print(f"WARNING: RBM_LANE_MODE={raw!r} unknown — using 'gap'")
        return "gap"
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

        self.cams_model = NADIR_CAMS
        self.tracker = LaneTracker()
        self.ctl = RowfitController()
        self.est = None  # last estimate (for logs / HUD)
        self._pending = None
        self._has_pending = False
        self._odo_dx = 0.0
        self._frame_yaw: float | None = None
        self._frame_t: float | None = None
        self.frame_dt = 0.32
        self.frames = 0

    def check_cameras(self, cams: dict) -> list[str]:
        """Warn if the live devices disagree with the mirrored wbt constants."""
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
