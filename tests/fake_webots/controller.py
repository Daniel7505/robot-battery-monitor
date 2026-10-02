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

class _Any:
    def __init__(self, name=""):
        self.name = name
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
class Camera(_Any):
    def getImage(self): return _IMG[self.name]
    def getWidth(self): return 128
    def getHeight(self): return 128
    def getFov(self): return 1.2
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
    def getFromDef(self, name): return None
    def getSelf(self): return _Any("self")
    def getDevice(self, name):
        if name not in self.devs:
            if name.startswith("nadir"): d = Camera(name)
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
    """Camera nodes are direct Robot children with the repo .wbt pose."""
    _self = _Node(1)
    def getSelf(self): return self._self
    def getFromDevice(self, dev):
        from src.lane_vision import parse_wbt_cameras
        p = parse_wbt_cameras().get(getattr(dev, "name", ""))
        return None if p is None else _Node(2, self._self, p["translation"], p["rotation"])
