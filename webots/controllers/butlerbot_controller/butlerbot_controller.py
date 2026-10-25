"""
ButlerBot Webots controller — row-fit orchestrator.

Each simulation step:

1. Read GPS, IMU, wheel encoders, agent / dashboard API commands.
2. Drive wheels (row-fit, manual, API) with local + remote throttle.
3. ABS stop on dashboard / agent ``drive_stop``.
4. Estimate joint power, drain a virtual battery, POST telemetry.
5. Poll ``GET /api/twin/state`` for throttle, API drive, ``stop_epoch``.

Eyes on the wheel: ``nadir_left`` / ``nadir_right`` only, read by row-fit.
The pixel-gap keeper is retired. LINE / finish / forecast / SIDELOOK
were removed from ``butlerbot.wbt`` earlier.

This file is the Webots entry point (``controller "butlerbot_controller"``).
The step bodies live in siblings and run in the order below. Behavior of
those blocks is unchanged.

* ``controller_devices.py`` — device setup, BoM reload, wheel commands
* ``controller_sensing.py`` — IMU, follow camera, GPS / encoder reads
* ``controller_mission.py`` — route, R-key phases, nadir harvest, mission end
* ``controller_energy.py`` — pack watt-hours and telemetry POST
* ``controller_hud.py`` — Display gauges + shoulder overlays.
  Paint is off unless ``RBM_HUD=1`` (``robot.hud_overlays``). Lane
  keeping still reads the nadir cameras.
* ``controller_keys.py`` — legacy debug key input (not a user feature;
  the agent steers through the twin API)
* ``controller_wheels.py`` — ABS / residual-spin / soft-grip
* ``twin_publisher.py`` — telemetry HTTP

"""

from __future__ import annotations

import importlib.util
import os
import sys
import traceback

from controller import Robot

try:
    from controller import Supervisor
except ImportError:  # pragma: no cover - Webots always provides Supervisor
    Supervisor = None  # type: ignore[misc, assignment]

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)


def _load_teleop_agent():
    """Optionally load ``src/teleop_agent.py`` for shared brake/throttle math."""
    module_path = os.path.join(_PROJECT_ROOT, "src", "teleop_agent.py")
    if not os.path.isfile(module_path):
        return None
    spec = importlib.util.spec_from_file_location("teleop_agent", module_path)
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception:
        return None
    return module


_teleop = _load_teleop_agent()

from controller_hud import (
    _paint_eye_huds,
    bind_teleop as _bind_hud_teleop,
)
from controller_keys import bind_teleop as _bind_keys_teleop
from controller_wheels import bind_teleop as _bind_wheels_teleop
from controller_sensing import (
    _apply_follow_camera,
    accumulate_wheel_encoders,
    bind_teleop as _bind_sensing_teleop,
    sense_motion,
)
from controller_mission import (
    begin_tick,
    finish_mission_if_parked,
    handle_manual_and_api,
    harvest_and_steer,
    log_drive_and_overlays,
    poll_keyboard,
    shutdown_run,
)
from controller_devices import command_motion, prepare_run
from controller_energy import (
    bind_teleop as _bind_energy_teleop,
    integrate_energy,
    publish_tick,
)
from twin_publisher import parse_controller_args

_bind_hud_teleop(_teleop)
_bind_keys_teleop(_teleop)
_bind_wheels_teleop(_teleop)
_bind_sensing_teleop(_teleop)
_bind_energy_teleop(_teleop)


def _run_loop(robot: Robot, opts: dict) -> None:
    """Priority: ABS → dashboard API → keyboard → nadir keep → residual park."""
    st = prepare_run(robot, opts)
    if st is None:
        return
    while robot.step(st.timestep) != -1:
        try:
            begin_tick(st)
            poll_keyboard(st)
            sense_motion(st)
            handle_manual_and_api(st)
            harvest_and_steer(st)
            if st.hud_paint and st.harvest_now:
                _paint_eye_huds(st.eye_huds, st.lane_eyes)
            command_motion(st)
            integrate_energy(st)
            log_drive_and_overlays(st)
            accumulate_wheel_encoders(st)
            finish_mission_if_parked(st)
            publish_tick(st)
        except Exception as exc:
            print(f"Controller step error (recovering): {exc}")
            traceback.print_exc()
            continue
    shutdown_run(st)


class _TeeStream:
    """Mirror controller stdout/stderr to a host log file (optimizer laps)."""

    def __init__(self, stream, file_obj):
        self._stream = stream
        self._file = file_obj

    def write(self, data):
        self._stream.write(data)
        self._file.write(data)

    def flush(self):
        self._stream.flush()
        self._file.flush()

    def __getattr__(self, name):
        return getattr(self._stream, name)


def _install_controller_log_tee() -> None:
    path = (os.environ.get("RBM_CONTROLLER_LOG") or "").strip()
    if not path:
        return
    folder = os.path.dirname(path)
    if folder:
        os.makedirs(folder, exist_ok=True)
    handle = open(path, "a", encoding="utf-8", buffering=1)
    sys.stdout = _TeeStream(sys.stdout, handle)
    sys.stderr = _TeeStream(sys.stderr, handle)
    print(f"Controller console tee -> {path}")


def _prefer_utf8_stdio() -> None:
    """Windows pipes default to cp1252, which cannot encode the arrows in the startup log."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(encoding="utf-8", errors="replace")
        except (OSError, ValueError):
            continue


def main() -> None:
    _prefer_utf8_stdio()
    _install_controller_log_tee()
    opts = parse_controller_args()
    if Supervisor is not None:
        robot = Supervisor()
    else:
        robot = Robot()
    try:
        _run_loop(robot, opts)
    except Exception as exc:
        print(f"ButlerBot controller fatal error: {exc}")
        traceback.print_exc()
        sys.exit(0)


if __name__ == "__main__":
    main()
