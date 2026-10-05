"""Minimal stand-in for the Webots ``controller`` module (smoke tests only).

Cameras return a synthetic straight lane (robot 5 cm left of centre) rendered
with src.lane_vision; everything else is a permissive no-op device.
"""
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from src.lane_vision import NADIR_CAMS, lane_polyline_robot, offset_polyline, render_lane_bgra
_c = lane_polyline_robot([(-2 + 0.1 * i, 0.0) for i in range(60)], (0.0, 0.05), 0.0)
_IMG = {n: render_lane_bgra(m, [offset_polyline(_c, 0.65), offset_polyline(_c, -0.65)]) for n, m in NADIR_CAMS.items()}

_TAGS = iter(range(10, 10000))
class _Any:
    def __init__(self, name=""):
        self.name = name
        self._tag = next(_TAGS)
    def __getattr__(self, k):
        return lambda *a, **kw: 0.0
class Motor(_Any): pass
class PositionSensor(_Any): pass
class GPS(_Any):
    def getValues(self): return [0.0, 0.0, 0.0]
class InertialUnit(_Any):
    def getRollPitchYaw(self): return [0.0, 0.0, 0.0]
class Keyboard(_Any):
    def getKey(self): return -1
class Display(_Any):
    BGRA = 1
    def getWidth(self): return 256
    def getHeight(self): return 256
_STEREO = {}


def _stereo_img(name):
    """Stereo pair of a textured box 1.0 m ahead (front face), rendered once (src.stereo_synth)."""
    if not _STEREO:
        import numpy as np
        from src.stereo_depth import STEREO_LEFT_CAM, STEREO_RIGHT_CAM
        from src.stereo_synth import SceneBox, render_gray
        box = [SceneBox.at(1.0 + 0.25, 0.0, 0.5, 0.5, 0.5)]
        for n, cam in (("stereo_left", STEREO_LEFT_CAM), ("stereo_right", STEREO_RIGHT_CAM)):
            g = render_gray(cam, (0.0, 0.0, 0.0), box, supersample=1).astype(np.uint8)
            _STEREO[n] = np.dstack([g, g, g, np.full_like(g, 255)]).tobytes()
    return _STEREO[name]


class RadarTarget:
    def __init__(self, distance=2.0, received_power=-40.0, speed=0.0, azimuth=0.0):
        self.distance = distance
        self.received_power = received_power
        self.speed = speed
        self.azimuth = azimuth

class Radar(_Any):
    """Empty radar (no targets) unless FAKE_WEBOTS_RADAR_TARGETS is set."""
    def getNumberOfTargets(self):
        return len(self.getTargets())
    def getTargets(self):
        raw = os.environ.get("FAKE_WEBOTS_RADAR_TARGETS", "").strip()
        if not raw:
            return []
        out = []
        for part in raw.split(";"):
            # distance,speed,azimuth
            bits = [float(x) for x in part.split(",")]
            out.append(RadarTarget(bits[0], -40.0, bits[1] if len(bits) > 1 else 0.0,
                                   bits[2] if len(bits) > 2 else 0.0))
        return out

class Camera(_Any):
    def getImage(self): return _stereo_img(self.name) if self.name.startswith("stereo") else _IMG[self.name]
    def getWidth(self): return 320 if self.name.startswith("stereo") else 128
    def getHeight(self): return 240 if self.name.startswith("stereo") else 128
    def getFov(self): return 1.4 if self.name.startswith("stereo") else 1.2
class Robot:
    MAX_STEPS = int(os.environ.get("FAKE_WEBOTS_STEPS", "400"))
    def __init__(self):
        self.n = 0
        self.devs = {}
    def getBasicTimeStep(self): return 8.0
    def getTime(self): return self.n * 0.008
    def step(self, ts):
        self.n += 1
        return -1 if self.n > self.MAX_STEPS else 0
    def getKeyboard(self): return Keyboard("kb")
    def getWorldPath(self): return os.environ.get("FAKE_WEBOTS_WORLD", "")
    def getFromDef(self, name): return None
    def getSelf(self): return _Any("self")
    def getDevice(self, name):
        if name not in self.devs:
            if name.startswith(("nadir", "stereo")): d = Camera(name)
            elif name.startswith("radar"): d = Radar(name)
            elif name.startswith("hud"): d = Display(name)
            elif name in ("gps", "gps_head"): d = GPS(name)
            elif name == "imu": d = InertialUnit(name)
            elif name.endswith("sensor"): d = PositionSensor(name)
            else: d = Motor(name)
            self.devs[name] = d
        return self.devs[name]
class _Field:
    def __init__(self, v): self.v = list(v)
    def getSFVec3f(self): return self.v
    def getSFRotation(self): return self.v
class _Node:
    def __init__(self, nid, parent=None, t=(0, 0, 0), r=(0, 0, 1, 0)):
        self.nid, self.parent, self.f = nid, parent, {"translation": _Field(t), "rotation": _Field(r)}
    def getId(self): return self.nid
    def getParentNode(self): return self.parent
    def getField(self, k): return self.f.get(k)
class Supervisor(Robot):
    """Mimics R2025a's Python Supervisor: getFromDevice takes the INTEGER device
    tag and passes it to ctypes (a Camera object raises ctypes.ArgumentError).
    Camera nodes are direct Robot children with the repo .wbt pose."""
    _self = _Node(1)
    _defs = {"NADIR_CAM_L": "nadir_left", "NADIR_CAM_R": "nadir_right",
             "STEREO_CAM_L": "stereo_left", "STEREO_CAM_R": "stereo_right"}
    def getSelf(self): return self._self
    def _cam_node(self, name):
        from src.lane_vision import parse_wbt_cameras
        p = parse_wbt_cameras().get(name)
        return None if p is None else _Node(2, self._self, p["translation"], p["rotation"])
    def getFromDevice(self, tag):
        import ctypes
        if not isinstance(tag, int):
            raise ctypes.ArgumentError("argument 1: TypeError: Don't know how to convert parameter 1")
        for d in self.devs.values():
            if getattr(d, "_tag", None) == tag:
                return self._cam_node(d.name)
        return None
    def getFromDef(self, name):
        return self._cam_node(self._defs[name]) if name in self._defs else None