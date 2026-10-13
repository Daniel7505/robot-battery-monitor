"""Speed-trace file and the shoulder-HUD paint switch.

Neither of these bills energy. The trace is so a warehouse lap can
separate a real wheel-speed jump from a render hitch, and so a lap can
report how hard the left and right wheels disagreed. The HUD switch
skips painting the two shoulder Displays. Lane keeping still reads the
nadir cameras.

The controller writes the trace itself. Webots ``--stdout`` redirected
with Start-Process stays a 0-byte file, so printing is not a log.
Console lines are off unless asked. The CSV is ``logs/speed_trace_<timestamp>.csv``
under the repo (or ``RBM_LOG_DIR``), gitignored. Rows are written only
while the robot is moving, and they stop when the mission ends.

Straight-line contact above the published hinge damping is not given a
coefficient here. See ``assumptions.notes.straight_contact`` in
``config/bom/bom.json``.
"""

from __future__ import annotations

import csv
import os
import time

_TRUE = {"1", "true", "yes", "on"}
_FALSE = {"0", "false", "no", "off"}
_UNSET = {"", "none", "null", "default", "yaml", "~"}

# One warehouse lap, with a little slack for the perimeter rounding.
_ONE_LAP_LO = 0.98
_ONE_LAP_HI = 1.02
# Quiet floors. Below these the trace does not append a row, so a parked
# robot does not grow the file. The lap split still ignores those samples.
_MOVE_SPEED_M_S = 0.02
_MOVE_WHEEL_RAD_S = 0.05
# A forgotten RBM_SPEED_TRACE=1 on a long sit-and-drive must not fill the disk.
_MAX_TRACE_ROWS = 20000
_TRACE_FIELDS = (
    "kind",
    "t_sim",
    "t_wall",
    "rtf",
    "dt_sim",
    "dt_wall",
    "v_body",
    "v_odo",
    "wL_cmd",
    "wR_cmd",
    "wL",
    "wR",
    "n",
    "mean_cmd_split",
    "var_cmd_split",
    "mean_hub_split",
    "var_hub_split",
    "mean_abs_cmd_split",
    "mean_abs_hub_split",
    "mean_rtf",
    "mean_v_body",
)


def _parse_bool(value) -> bool | None:
    """True, False, or None when the value does not choose."""
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in _UNSET:
        return None
    if text in _TRUE:
        return True
    if text in _FALSE:
        return False
    return None


def resolve_hud_overlays(*, config_value=None, env_value: str | None = None) -> bool:
    """Whether to paint ``hud_nadir`` and ``hud_nadir_r``.

    ``RBM_HUD`` wins over YAML ``robot.hud_overlays``. Blank means off.
    Off skips the paint only. The nadir cameras stay enabled.
    """
    env = _parse_bool(env_value)
    if env is not None:
        return env
    cfg = _parse_bool(config_value)
    if cfg is not None:
        return cfg
    return False


def resolve_speed_trace_console(*, config_value=None, env_value: str | None = None) -> bool:
    """Whether to also print the trace on the Webots console.

    ``RBM_SPEED_TRACE_CONSOLE`` wins, then YAML ``robot.speed_trace_console``.
    Blank means off. The CSV is written either way when the trace itself is on.
    """
    env = _parse_bool(env_value)
    if env is not None:
        return env
    cfg = _parse_bool(config_value)
    if cfg is not None:
        return cfg
    return False


def resolve_speed_trace(
    *,
    config_value=None,
    env_value: str | None = None,
    mission_limit_m: float | None = None,
    lap_m: float | None = None,
) -> bool:
    """Whether to record ``SPEED_TRACE`` / ``SIM_RTF``.

    ``RBM_SPEED_TRACE`` wins, then YAML ``robot.speed_trace``. When both
    are blank, the trace is on only for a one-lap cap: the mission limit
    within 2% of one warehouse perimeter. The S track has no cap, so it
    stays quiet. Recording writes a CSV. Printing is a separate switch.
    """
    env = _parse_bool(env_value)
    if env is not None:
        return env
    cfg = _parse_bool(config_value)
    if cfg is not None:
        return cfg
    try:
        limit = float(mission_limit_m)
        lap = float(lap_m)
    except (TypeError, ValueError):
        return False
    if lap <= 0.0 or limit <= 0.0:
        return False
    ratio = limit / lap
    return _ONE_LAP_LO <= ratio <= _ONE_LAP_HI


def overlay_status_lines(hud_paint: bool) -> list[str]:
    """One startup explanation of the 3D-view overlays.

    Webots opens a Camera overlay when the node exists, including a
    camera the controller never enables, and a Display overlay for every
    Display node. There is no controller call that hides them. The close
    button and Robot → Camera Devices / Display Devices write the
    gitignored ``.wbproj``.
    """
    paint = (
        "HUD paint is on (RBM_HUD=1). controller_hud.py paints both."
        if hud_paint
        else "HUD paint is off. Set RBM_HUD=1 to paint. Lane keeping still reads the cameras."
    )
    return [
        "OVERLAYS — drawn by Webots on the 3D view. The controller cannot hide them. "
        "Close a window with its X, or Robot → Camera Devices / Display Devices. "
        "That choice is saved in the gitignored .wbproj.",
        "  Camera nadir_left (magenta border): enabled for lane keeping. "
        "Webots draws the raw image. Lane vision reads getImage. This code does not paint it.",
        "  Camera nadir_right (magenta border): same, right shoulder.",
        "  Camera stereo_left and stereo_right (magenta border): not enabled unless obstacles are on. "
        "Webots still opens the overlay when the world loads, so the picture stays black. "
        "That is the unused window. The cameras stay in the world for the obstacle path.",
        "  Display hud_nadir and hud_nadir_r (cyan border): shoulder lane HUDs. "
        f"{paint} Unpainted, these rectangles stay black.",
        "  No Display named hud. The old gauge node is gone; _draw_hud does nothing without it.",
        "HUD paint, when on, scales two 128x128 images to 256x256 in Python and pastes them "
        "every camera sample (320 ms), on the controller thread. Skipping that does not stop "
        "the two nadir cameras, which steering needs. A 64x64 paint test on 2026-08-17 was not "
        "the realtime limit; these Displays are 256x256. SPEED_TRACE is how a hitch in the "
        "picture is told from a real wheel-speed jump.",
    ]


def format_speed_trace(
    *,
    t_sim: float,
    t_wall: float,
    rtf: float | None,
    v_body: float,
    v_odo: float,
    w_l_cmd: float,
    w_r_cmd: float,
    w_l: float,
    w_r: float,
) -> str:
    """One low-rate sample. ``t_wall`` is ``time.monotonic`` seconds."""
    rtf_s = "na" if rtf is None else f"{rtf:.3f}"
    return (
        f"SPEED_TRACE t_sim={t_sim:.3f} t_wall={t_wall:.3f} rtf={rtf_s} "
        f"v_body={v_body:.3f} v_odo={v_odo:.3f} "
        f"wL_cmd={w_l_cmd:.3f} wR_cmd={w_r_cmd:.3f} "
        f"wL={w_l:.3f} wR={w_r:.3f}"
    )


def format_steer_split(
    *,
    n: int,
    mean_cmd_split: float,
    var_cmd_split: float,
    mean_abs_cmd_split: float,
    mean_hub_split: float,
    var_hub_split: float,
    mean_abs_hub_split: float,
    mean_rtf: float | None,
    mean_v_body: float,
) -> str:
    """One lap of left/right disagreement. Not an energy term.

    Splits are signed ``left − right`` in rad/s, over controller steps
    that were actually moving. Variance is the population variance
    (divide by n). ``mean_abs_*`` is the mean of the absolute split.
    A 0.2 rad/s split is under 1% of electrical power. This line is how
    a HUD-on lap is compared with a HUD-off lap. It is not billed.
    """
    rtf_s = "na" if mean_rtf is None else f"{mean_rtf:.3f}"
    return (
        f"STEER_SPLIT n={int(n)} "
        f"mean_cmd_split={mean_cmd_split:.4f} var_cmd_split={var_cmd_split:.4f} "
        f"mean_abs_cmd_split={mean_abs_cmd_split:.4f} "
        f"mean_hub_split={mean_hub_split:.4f} var_hub_split={var_hub_split:.4f} "
        f"mean_abs_hub_split={mean_abs_hub_split:.4f} "
        f"mean_rtf={rtf_s} mean_v_body={mean_v_body:.3f}"
    )


def default_speed_trace_dir(log_dir: str | None = None) -> str:
    """Directory for ``speed_trace_<timestamp>.csv``.

    ``RBM_LOG_DIR`` when set, otherwise the repo ``logs/`` folder.
    """
    raw = (log_dir if log_dir is not None else os.environ.get("RBM_LOG_DIR", "")).strip()
    if raw:
        return os.path.expanduser(os.path.expandvars(raw))
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    return os.path.join(root, "logs")


def _welford_add(n: int, mean: float, m2: float, x: float) -> tuple[int, float, float]:
    n += 1
    delta = x - mean
    mean += delta / n
    m2 += delta * (x - mean)
    return n, mean, m2


def format_sim_rtf(
    *,
    t_sim: float,
    dt_sim: float,
    dt_wall: float,
    rtf: float | None,
    mean_cmd_split: float,
    mean_hub_split: float,
    n: int,
) -> str:
    """Real-time factor and the mean left/right wheel split over the window.

    ``mean_cmd_split`` is the mean of |commanded left − commanded right|
    in rad/s. ``mean_hub_split`` is the same for the measured hub rates.
    A split is logged here. It is not an energy term.
    """
    rtf_s = "na" if rtf is None else f"{rtf:.3f}"
    return (
        f"SIM_RTF t_sim={t_sim:.3f} dt_sim={dt_sim:.3f} dt_wall={dt_wall:.3f} "
        f"rtf={rtf_s} mean_cmd_split={mean_cmd_split:.3f} "
        f"mean_hub_split={mean_hub_split:.3f} n={int(n)}"
    )


class SpeedTrace:
    """Emit ``SPEED_TRACE`` every ``step_s`` of sim time and ``SIM_RTF`` every ``rtf_period_s``.

    A sim hitch that jumps across several sample times prints one line,
    not a burst. The gap shows up as sim time moving further than the
    wall gap, which drops ``rtf``.

    ``lines`` always returns those strings so a test can read them.
    The controller prints them only when ``console`` is true. When
    ``log_dir`` or ``path`` is set, the same rows are appended to a CSV
    while the robot is moving. Parked samples advance the schedule but
    do not grow the file. ``finish`` writes one ``STEER_SPLIT`` row and
    stops further trace rows.
    """

    def __init__(
        self,
        enabled: bool,
        *,
        step_s: float = 0.1,
        rtf_period_s: float = 10.0,
        console: bool = False,
        log_dir: str | None = None,
        path: str | None = None,
    ) -> None:
        self.enabled = bool(enabled)
        self.console = bool(console)
        self.step_s = float(step_s)
        self.rtf_period_s = float(rtf_period_s)
        self.path: str | None = None
        self.open_error: str | None = None
        self._fh = None
        self._rows = 0
        self._capped = False
        self._stop = False
        self._finished = False
        self._steer_text = ""
        self._origin_sim: float | None = None
        self._next_trace = 0.0
        self._next_rtf = 0.0
        self._prev_sim = 0.0
        self._prev_wall = 0.0
        self._win_sim = 0.0
        self._win_wall = 0.0
        self._cmd_sum = 0.0
        self._hub_sum = 0.0
        self._n = 0
        self._steer_n = 0
        self._cmd_mean = 0.0
        self._cmd_m2 = 0.0
        self._hub_mean = 0.0
        self._hub_m2 = 0.0
        self._abs_cmd = 0.0
        self._abs_hub = 0.0
        self._v_sum = 0.0
        self._move_sim = 0.0
        self._move_wall = 0.0
        self._prev_moving = False
        self._last_sim: float | None = None
        self._last_wall: float | None = None
        self._last_emit_sim = 0.0
        if self.enabled and (path or log_dir is not None):
            self._open(path=path, log_dir=log_dir)

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def steer_text(self) -> str:
        return self._steer_text

    def _open(self, *, path: str | None, log_dir: str | None) -> None:
        try:
            if path:
                dest = path
            else:
                directory = default_speed_trace_dir(log_dir)
                os.makedirs(directory, exist_ok=True)
                stamp = time.strftime("%Y%m%d-%H%M%S")
                dest = os.path.join(directory, f"speed_trace_{stamp}.csv")
                if os.path.exists(dest):
                    dest = os.path.join(directory, f"speed_trace_{stamp}_{os.getpid()}.csv")
            parent = os.path.dirname(dest)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self._fh = open(dest, "w", encoding="utf-8", newline="", buffering=1)
            csv.writer(self._fh).writerow(_TRACE_FIELDS)
            self._fh.flush()
            self.path = dest
        except OSError as exc:
            self.open_error = f"{type(exc).__name__}: {exc}"
            self._fh = None
            self.path = path

    def _moving(
        self,
        *,
        v_body: float,
        v_odo: float,
        w_l_cmd: float,
        w_r_cmd: float,
        w_l: float,
        w_r: float,
    ) -> bool:
        return (
            abs(v_body) >= _MOVE_SPEED_M_S
            or abs(v_odo) >= _MOVE_SPEED_M_S
            or abs(w_l_cmd) >= _MOVE_WHEEL_RAD_S
            or abs(w_r_cmd) >= _MOVE_WHEEL_RAD_S
            or abs(w_l) >= _MOVE_WHEEL_RAD_S
            or abs(w_r) >= _MOVE_WHEEL_RAD_S
        )

    def _note_sample(
        self,
        *,
        t_sim: float,
        t_wall: float,
        v_body: float,
        moving: bool,
        cmd_split: float,
        hub_split: float,
    ) -> None:
        if (
            moving
            and self._prev_moving
            and self._last_sim is not None
            and self._last_wall is not None
        ):
            ds = t_sim - self._last_sim
            dw = t_wall - self._last_wall
            if ds > 0.0 and dw > 0.0:
                self._move_sim += ds
                self._move_wall += dw
        if moving:
            self._steer_n, self._cmd_mean, self._cmd_m2 = _welford_add(
                self._steer_n, self._cmd_mean, self._cmd_m2, cmd_split
            )
            # Hub uses the same n. Add it without incrementing again.
            n = self._steer_n
            delta = hub_split - self._hub_mean
            self._hub_mean += delta / n
            self._hub_m2 += delta * (hub_split - self._hub_mean)
            self._abs_cmd += abs(cmd_split)
            self._abs_hub += abs(hub_split)
            self._v_sum += abs(v_body)
        self._prev_moving = moving
        self._last_sim = t_sim
        self._last_wall = t_wall

    def steer_split_line(self) -> str:
        n = self._steer_n
        var_cmd = (self._cmd_m2 / n) if n else 0.0
        var_hub = (self._hub_m2 / n) if n else 0.0
        mean_rtf = (self._move_sim / self._move_wall) if self._move_wall > 1e-9 else None
        return format_steer_split(
            n=n,
            mean_cmd_split=self._cmd_mean if n else 0.0,
            var_cmd_split=var_cmd,
            mean_abs_cmd_split=(self._abs_cmd / n) if n else 0.0,
            mean_hub_split=self._hub_mean if n else 0.0,
            var_hub_split=var_hub,
            mean_abs_hub_split=(self._abs_hub / n) if n else 0.0,
            mean_rtf=mean_rtf,
            mean_v_body=(self._v_sum / n) if n else 0.0,
        )

    def finish(self) -> str:
        """Write the lap split once and stop the trace file."""
        if self._finished:
            return self._steer_text
        self._finished = True
        self._stop = True
        if not self.enabled:
            self._steer_text = ""
            return ""
        self._steer_text = self.steer_split_line()
        self._write_steer_row(self._steer_text)
        self._close_file()
        return self._steer_text

    def _close_file(self) -> None:
        fh = self._fh
        self._fh = None
        if fh is None:
            return
        try:
            fh.flush()
            fh.close()
        except OSError:
            pass

    def _write_fields(self, fields: dict) -> None:
        if self._fh is None or self._capped:
            return
        if self._rows >= _MAX_TRACE_ROWS and fields.get("kind") != "steer":
            self._capped = True
            return
        row = [fields.get(name, "") for name in _TRACE_FIELDS]
        try:
            csv.writer(self._fh).writerow(row)
            self._fh.flush()
            if fields.get("kind") != "steer":
                self._rows += 1
        except OSError as exc:
            self.open_error = f"{type(exc).__name__}: {exc}"
            self._close_file()

    def _write_text_row(self, text: str) -> None:
        parts = text.split()
        kind = "trace" if text.startswith("SPEED_TRACE") else "rtf"
        fields = {"kind": kind}
        for part in parts:
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key] = value
        self._write_fields(fields)

    def _write_steer_row(self, text: str) -> None:
        if self._fh is None or not text:
            return
        fields = {"kind": "steer", "t_sim": "" if self._last_sim is None else f"{self._last_sim:.3f}"}
        for part in text.split():
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key] = value
        # The summary is one row past the cap. Bypass the trace cap.
        capped = self._capped
        self._capped = False
        self._write_fields(fields)
        self._capped = capped

    def lines(
        self,
        *,
        t_sim: float,
        t_wall: float,
        v_body: float,
        v_odo: float,
        w_l_cmd: float,
        w_r_cmd: float,
        w_l: float,
        w_r: float,
    ) -> list[str]:
        if not self.enabled or self._stop:
            return []
        t_sim = float(t_sim)
        t_wall = float(t_wall)
        v_body = float(v_body)
        v_odo = float(v_odo)
        w_l_cmd = float(w_l_cmd)
        w_r_cmd = float(w_r_cmd)
        w_l = float(w_l)
        w_r = float(w_r)
        moving = self._moving(
            v_body=v_body,
            v_odo=v_odo,
            w_l_cmd=w_l_cmd,
            w_r_cmd=w_r_cmd,
            w_l=w_l,
            w_r=w_r,
        )
        self._note_sample(
            t_sim=t_sim,
            t_wall=t_wall,
            v_body=v_body,
            moving=moving,
            cmd_split=w_l_cmd - w_r_cmd,
            hub_split=w_l - w_r,
        )
        if self._origin_sim is None:
            self._origin_sim = t_sim
            self._prev_sim = t_sim
            self._prev_wall = t_wall
            self._win_sim = t_sim
            self._win_wall = t_wall
            self._next_trace = t_sim + self.step_s
            self._next_rtf = t_sim + self.rtf_period_s
        self._cmd_sum += abs(float(w_l_cmd) - float(w_r_cmd))
        self._hub_sum += abs(float(w_l) - float(w_r))
        self._n += 1
        out: list[str] = []
        if t_sim + 1e-9 >= self._next_trace:
            dt_wall = t_wall - self._prev_wall
            dt_sim = t_sim - self._prev_sim
            rtf = (dt_sim / dt_wall) if dt_wall > 1e-9 else None
            out.append(
                format_speed_trace(
                    t_sim=t_sim,
                    t_wall=t_wall,
                    rtf=rtf,
                    v_body=v_body,
                    v_odo=v_odo,
                    w_l_cmd=w_l_cmd,
                    w_r_cmd=w_r_cmd,
                    w_l=w_l,
                    w_r=w_r,
                )
            )
            self._prev_sim = t_sim
            self._prev_wall = t_wall
            while self._next_trace <= t_sim + 1e-9:
                self._next_trace += self.step_s
        if t_sim + 1e-9 >= self._next_rtf:
            dt_wall = t_wall - self._win_wall
            dt_sim = t_sim - self._win_sim
            rtf = (dt_sim / dt_wall) if dt_wall > 1e-9 else None
            n = self._n
            out.append(
                format_sim_rtf(
                    t_sim=t_sim,
                    dt_sim=dt_sim,
                    dt_wall=dt_wall,
                    rtf=rtf,
                    mean_cmd_split=(self._cmd_sum / n) if n else 0.0,
                    mean_hub_split=(self._hub_sum / n) if n else 0.0,
                    n=n,
                )
            )
            self._win_sim = t_sim
            self._win_wall = t_wall
            self._cmd_sum = 0.0
            self._hub_sum = 0.0
            self._n = 0
            while self._next_rtf <= t_sim + 1e-9:
                self._next_rtf += self.rtf_period_s
        # Parked samples still advance the schedule above, so the next
        # move is one row and not a burst. They are not written.
        if not moving:
            return []
        for text in out:
            self._write_text_row(text)
        return out
