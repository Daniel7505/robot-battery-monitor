"""Stereo obstacle glue for the Webots controller. No physics, no Webots import.

* :func:`obstacles_mode` — on / off: ``RBM_OBSTACLES`` (1 / 0) wins; unset ->
  the loaded world's ``# OBSTACLES on`` tag (only
  ``butlerbot_warehouse_obstacles.wbt`` has it) -> off.
* :class:`ObstacleRuntime` — enables the two stereo cameras, builds the rig
  from their LIVE pose (Supervisor, like the shoulder cams), derives the
  corridor from the live robot (shoulder-camera width, stereo-bar height),
  runs ``StereoObstacleSensor`` on every camera harvest and feeds the
  ``ObstacleGate``; the gate's speed cap goes to ``RowfitController``.
"""
from __future__ import annotations

import os

try:
    from twin_publisher import project_env
except Exception:  # pragma: no cover
    def project_env(name, default=None):  # type: ignore[misc]
        return os.environ.get(name) or default

OBSTACLES_ENV = "RBM_OBSTACLES"
WORLD_TAG = "# OBSTACLES "
CORRIDOR_MARGIN_M = 0.10  # each side, past the widest part of the robot
HEADROOM_MARGIN_M = 0.15  # above the robot's highest part (the stereo bar)
SHOULDER_BOX_HALF_M = 0.025  # NADIR_BOX 5 cm cube: the box sticks out this far past the lens


def world_obstacles_tag(world_path: str) -> str | None:
    try:
        with open(world_path, encoding="utf-8") as fh:
            for _ in range(80):
                line = fh.readline()
                if not line:
                    break
                if line.startswith(WORLD_TAG):
                    rest = line[len(WORLD_TAG):].split()
                    return rest[0].lower() if rest else ""
    except OSError:
        return None
    return None


def obstacles_mode(robot=None, env=None) -> tuple[bool, str]:
    """(enabled, why)."""
    raw = (project_env(OBSTACLES_ENV, "") if env is None else env.get(OBSTACLES_ENV, "")) or ""
    raw = raw.strip().lower()
    if raw in ("1", "true", "yes", "on"):
        return True, f"{OBSTACLES_ENV}={raw}"
    if raw in ("0", "false", "no", "off"):
        return False, f"{OBSTACLES_ENV}={raw}"
    world = ""
    try:
        world = str(robot.getWorldPath() or "") if robot is not None else ""
    except Exception:
        world = ""
    tag = world_obstacles_tag(world) if world else None
    if tag in ("on", "1", "true"):
        return True, f"world tag '{WORLD_TAG.strip()} {tag}' in {os.path.basename(world)} ({OBSTACLES_ENV} unset)"
    return False, f"{OBSTACLES_ENV} unset, no '{WORLD_TAG.strip()} on' tag in the world: off (default)"


class ObstacleRuntime:
    def __init__(self) -> None:
        from src.obstacle_gate import ObstacleFusion, ObstacleGate
        from src.rowfit_control import DECEL_M_S2, V_CRUISE_M_S

        self.gate = ObstacleGate(decel_m_s2=DECEL_M_S2, v_cruise_m_s=V_CRUISE_M_S)
        self.fusion = ObstacleFusion()
        self.sensor = None
        self.rig = None
        self.devs: dict = {}
        self.report = None
        self.frames = 0
        self.errors = 0
        self.robot = None
        # test harness only: a 'person' takes the prop away after the robot waited this long
        raw = (project_env("RBM_OBS_REMOVE_AFTER_S", "") or "").strip()
        try:
            self.remove_after_s = float(raw) if raw else None
        except ValueError:
            self.remove_after_s = None
        self.stopped_s = 0.0
        self.removed: list[str] = []

    # ---- devices ------------------------------------------------------------
    def enable_devices(self, robot, period_ms: int) -> list[str]:
        from src.stereo_depth import STEREO_NAMES

        msgs = []
        for name in STEREO_NAMES:
            dev = None
            try:
                dev = robot.getDevice(name)
            except Exception:
                dev = None
            if dev is None:
                msgs.append(f"stereo camera '{name}' not in this world — obstacles off")
            else:
                dev.enable(int(period_ms))
            self.devs[name] = dev
        return msgs

    @property
    def ok(self) -> bool:
        return self.sensor is not None

    def bind(self, robot, lane_models: dict | None = None, *, log=print) -> list[str]:
        """Live pose -> rig; corridor half width and headroom from the live robot."""
        from src.lane_vision import camera_model_from_device
        from src.stereo_depth import (
            ROBOT_FRONT_X_M,
            STEREO_DEFS,
            STEREO_LEFT_POSE,
            STEREO_RIGHT_POSE,
            CorridorParams,
            StereoObstacleSensor,
            StereoRig,
        )

        msgs: list[str] = []
        self.robot = robot
        if any(d is None for d in self.devs.values()) or not self.devs:
            return ["stereo cameras missing — obstacle sensing off"]
        models = {}
        for name, const in (("stereo_left", STEREO_LEFT_POSE), ("stereo_right", STEREO_RIGHT_POSE)):
            why: list[str] = []
            model, src = camera_model_from_device(robot, self.devs[name], name, def_name=STEREO_DEFS[name],
                                                  fallback_pose=const, errors=why)
            models[name] = model
            t = ", ".join(f"{v:.4f}" for v in model.translation)
            rot = " ".join(f"{v:.6g}" for v in model.rotation)
            log(f"CAM POSE {name} from {src}: t=({t}) rot=({rot}) {model.width}x{model.height} fov={model.fov_rad:.3f}")
            if src != "supervisor":
                msgs.append(f"{name}: live pose not read from supervisor, using {src} ({'; '.join(why) or '-'})")
        self.rig = StereoRig(models["stereo_left"], models["stereo_right"])
        lane_models = lane_models or {}
        ys = [abs(m.translation[1]) for m in lane_models.values()]
        body_half = (max(ys) + SHOULDER_BOX_HALF_M) if ys else 0.443
        top = max([m.translation[2] for m in models.values()] + [m.translation[2] + SHOULDER_BOX_HALF_M
                                                                  for m in lane_models.values()]) + 0.02
        front = max(ROBOT_FRONT_X_M, max(m.translation[0] for m in models.values()) + 0.01)
        prm = CorridorParams(half_width_m=round(body_half + CORRIDOR_MARGIN_M, 3), x_front_m=round(front, 3),
                             clearance_h_m=round(top + HEADROOM_MARGIN_M, 3))
        backend = (project_env("RBM_STEREO_BACKEND", "auto") or "auto").strip().lower()
        self.sensor = StereoObstacleSensor(self.rig, prm, backend=backend if backend in ("auto", "sgbm", "numpy") else "auto")
        log(self.rig.describe())
        g = self.gate
        log(f"OBSTACLES ON — backend {self.sensor.backend}"
            + ("" if self.sensor.backend == "sgbm" else " (no OpenCV: pip install opencv-python for SGBM, ~10x faster)")
            + f", {self.sensor.num_disp} disparities; corridor +-{prm.half_width_m:.2f} m (robot half width "
            f"{body_half:.3f} + {CORRIDOR_MARGIN_M:.2f}), x {prm.x_front_m:.2f}..{prm.max_range_m:.1f} m, headroom "
            f"{prm.clearance_h_m:.2f} m; stop = {g.standoff_m:.2f} m + v*latency + v^2/(2*{g.decel:.2f}) "
            f"({g.stop_distance(g.v_cruise):.2f} m at {g.v_cruise:.2f} m/s), slow zone {g.slow_distance:.2f} m")
        return msgs

    # ---- per frame ----------------------------------------------------------
    def harvest(self, *, dx_m: float, v_m_s: float, kappa: float = 0.0, frame_dt: float | None = None, log=print):
        if self.sensor is None:
            return None
        rep = None
        try:
            imgs = [self.devs[n].getImage() for n in ("stereo_left", "stereo_right")]
            if all(imgs):
                rep = self.sensor.process_bgra(imgs[0], imgs[1], kappa=kappa)
        except Exception as exc:
            self.errors += 1
            if self.errors <= 3:
                log(f"WARNING stereo frame failed: {type(exc).__name__}: {exc}")
        self.frames += 1
        self.report = rep
        self.fusion.add_report(rep)
        # radar later: self.fusion.add_source(radar) here; the gate takes the fused report
        fused = self.fusion.fused() if rep is not None else None
        self.gate.on_frame(fused, v_m_s=v_m_s, dx_m=dx_m, frame_dt_s=frame_dt)
        for e in self.gate.drain_events():
            log(e)
        if self.remove_after_s is not None:
            from src.obstacle_gate import STATE_STOPPED

            self.stopped_s = self.stopped_s + (frame_dt or 0.32) if self.gate.state == STATE_STOPPED else 0.0
            if self.stopped_s >= self.remove_after_s:
                self.stopped_s = 0.0
                self.remove_nearest_prop(log=log)
        return rep

    def remove_nearest_prop(self, *, log=print) -> str | None:
        """TEST HARNESS (RBM_OBS_REMOVE_AFTER_S): the Supervisor deletes the WH_OBSTACLES prop nearest
        the robot, like a person taking it away. Perception is not told; it has to see the lane clear."""
        sup = self.robot
        try:
            grp = sup.getFromDef("WH_OBSTACLES")
            fld = grp.getField("children") if grp is not None else None
            n = fld.getCount() if fld is not None else 0
            if not n:
                log("TEST HARNESS: no WH_OBSTACLES props left to remove")
                return None
            me = sup.getSelf().getPosition()
            best = min((fld.getMFNode(i) for i in range(n)),
                       key=lambda nd: (nd.getPosition()[0] - me[0]) ** 2 + (nd.getPosition()[1] - me[1]) ** 2)
            name = best.getDef() or "prop"
            best.remove()
            self.removed.append(name)
            log(f"TEST HARNESS (RBM_OBS_REMOVE_AFTER_S={self.remove_after_s:g}): removed {name} after the wait — "
                "the robot is not told; it resumes only when the stereo sees the corridor clear")
            return name
        except Exception as exc:
            log(f"TEST HARNESS: could not remove a prop ({type(exc).__name__}: {exc})")
            return None

    def apply(self, ctl) -> None:
        ctl.obstacle_v_cap = self.gate.v_cap

    def fields(self) -> dict:
        return self.gate.fields()

    def hud_line(self) -> str:
        return self.gate.hud_line()


def no_obstacle_fields() -> dict:
    return {"obstacle_state": "", "obstacle_m": None, "obstacle_band": "", "obstacle_conf": None}
