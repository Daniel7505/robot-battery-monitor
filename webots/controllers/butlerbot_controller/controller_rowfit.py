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
    "nL_pts,nR_pts,steer,target_speed,new_frame,run_id,"
    "state,corner_dir,corner_m,corner_conf,yaw_deg,yaw_target_deg,"
    "junction_type,junction_m,openings,route_choice,blind_m"
)
# Corner columns sit AFTER run_id, junction columns after those, so rows
# appended to an older lane-vision.csv (old header) still line up for every
# column the old header names.
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

    def write(self, *, x_m, y_m, mode, est, cmd: dict, new_frame: bool, yaw=None) -> None:
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
                str(cmd.get("state") or ""),
                str(cmd.get("corner_dir") or ""),
                _fmt(cmd.get("corner_m"), 3),
                _fmt(cmd.get("corner_conf"), 2),
                _fmt(None if yaw is None else math.degrees(float(yaw)), 2),
                _fmt(cmd.get("yaw_target_deg"), 2),
                str(cmd.get("junction_type") or ""),
                _fmt(cmd.get("junction_m"), 3),
                str(cmd.get("openings") or ""),
                str(cmd.get("route_choice") or ""),
                _fmt(cmd.get("blind_m"), 3),
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
        self.ctl.tracker = self.tracker  # re-acquire after a corner pivot
        self.last_cmd: dict = {}
        self.est = None  # last estimate (for logs / HUD)
        self._pending = None
        self._has_pending = False
        self._odo_dx = 0.0
        self._frame_yaw: float | None = None
        self._frame_t: float | None = None
        self.frame_dt = 0.32
        self.frames = 0
        self.route_text, self.route_warnings = "", []
        self.configure_route()

    def configure_route(self) -> None:
        """RBM_ROUTE (junction choices, e.g. S,L,R), RBM_MAX_BLIND_M (default 4.0),
        RBM_LOOK_AROUND (default on) from the process env or the repo .env."""
        from src.route_policy import parse_route
        from src.rowfit_control import MAX_BLIND_M

        raw = (project_env("RBM_ROUTE", "") or "").strip()
        route, warn = parse_route(raw)
        mb_raw = (project_env("RBM_MAX_BLIND_M", "") or "").strip()
        try:
            mb = float(mb_raw) if mb_raw else MAX_BLIND_M
            if mb <= 0:
                raise ValueError
        except ValueError:
            warn.append(f"RBM_MAX_BLIND_M={mb_raw!r} not a positive number — using {MAX_BLIND_M:g}")
            mb = MAX_BLIND_M
        la = (project_env("RBM_LOOK_AROUND", "1") or "1").strip().lower() not in ("0", "false", "no", "off")
        self.ctl.set_route(route, max_blind_m=mb, look_around=la)
        self.route_text = raw
        self.route_warnings = warn

    def route_line(self) -> str:
        c = self.ctl
        src = f"RBM_ROUTE={self.route_text}" if self.route_text else "RBM_ROUTE unset"
        return (f"ROUTE {src} -> {c.policy.describe()}; max blind {c.max_blind_m:.2f} m (RBM_MAX_BLIND_M), "
                f"look-around {'on' if c.look_around else 'off'}")

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
        self.ctl.tracker = self.tracker
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

    def command(self, dt: float, yaw: float | None = None, *, log=print) -> dict:
        """Wheel command for this tick. ``yaw`` = IMU yaw (rad); it enables the
        sharp-corner pivot (without it a corner ends in the lost-lane brake).
        Corner state changes are printed through ``log``."""
        new = self._has_pending
        est = self._pending if new else None
        self._has_pending = False
        self._pending = None
        cmd = self.ctl.step(est, new_frame=new, dt=dt, frame_dt=self.frame_dt, yaw=yaw)
        for msg in self.ctl.drain_events():
            log(msg)
        self.last_cmd = cmd
        return cmd

    def state_line(self) -> str:
        """One HUD line for the corner state machine."""
        c = self.ctl
        side = "L" if c.corner_dir > 0 else "R"
        if c.state == "CORNER_APPROACH":
            rem = c.remaining_m if c.remaining_m is not None else 0.0
            if c.turn_src != "corner":
                return f"TURN {side} {c.route_choice} pv{rem:+.2f}"
            return f"CORNER {side} {c.corner_dist_m or 0:.2f}m pv{rem:+.2f}"
        if c.state == "PIVOT":
            yaw = self._frame_yaw
            if yaw is not None and c.yaw_target is not None:
                err = math.atan2(math.sin(c.yaw_target - yaw), math.cos(c.yaw_target - yaw))
                return f"PIVOT {side} err {math.degrees(err):+.0f}d"
            return f"PIVOT {side}"
        if c.state == "REACQUIRE":
            return f"REACQUIRE {c.reacq_ok}/3"
        if c.state == "JUNCTION_APPROACH" and c.junction is not None:
            return f"JCT {c.junction.kind} {c.jn_center_m or 0:.2f}m {c.junction.openings().replace(' ', '')}"
        if c.state == "GAP_CROSS":
            return f"GAP {c.route_choice or 'S'} blind {c.blind_m:.2f}/{c.max_blind_m:.1f}m"
        if c.state == "LOOK_AROUND":
            return f"LOOK {min(c.look_i + 1, 3)}/3"
        if c.state == "STOPPED":
            return "STOPPED no lane"
        est = self.est
        corner = getattr(est, "corner", None) if est is not None else None
        if corner is not None:
            return f"LANE cue {corner.side[0].upper()} {corner.distance_m:.2f}m c{corner.confidence:.1f}"
        j = getattr(est, "junction", None) if est is not None else None
        if j is not None:
            return f"LANE {j.kind} {j.near_m:.2f}m {j.openings().replace(' ', '')}"
        return "LANE"

    def fill_eyes(self, lane_eyes: dict) -> None:
        est = self.est
        lane_eyes["error_source"] = "rowfit"
        lane_eyes["rowfit_state"] = self.ctl.state
        state = self.state_line()
        corner = getattr(est, "corner", None) if est is not None else None
        # crossbar rows for the HUD: cam -> [(row, col, role)]
        lane_eyes["rowfit_corner"] = {} if corner is None else {
            c.cam: [(c.row, c.col, c.role)] for c in corner.cues
        }
        if est is not None and getattr(est, "held", False):
            lane_eyes["rowfit_text"] = [
                state,
                "rowfit: HELD (fit rejected)",
                (est.reject_reason or "")[:28],
                f"off {est.offset_m * 100:+.0f}cm hd {math.degrees(est.heading_rad):+.0f}d",
            ]
            lane_eyes["rowfit_px"] = {}
            return
        if est is None or not est.valid:
            lane_eyes["rowfit_text"] = [state, "rowfit: no lane"]
            lane_eyes["rowfit_px"] = {}
            return
        lane_eyes["rowfit_px"] = est.pixels
        lane_eyes["rowfit_text"] = [
            state,
            f"off {est.offset_m * 100:+.0f}cm hd {math.degrees(est.heading_rad):+.0f}d",
            f"k {est.curvature_1pm:+.2f} conf {est.confidence:.2f}",
            f"L{est.n_left} R{est.n_right} look {est.lookahead_m:.1f}m",
        ]
        lane_eyes["rowfit_offset_m"] = round(est.offset_m, 4)
