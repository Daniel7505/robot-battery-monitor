"""Webots-console speed trace and the shoulder-HUD paint switch.

Neither of these bills energy. The trace is so a warehouse lap can
separate a real wheel-speed jump from a render hitch. The HUD switch
skips painting the two shoulder Displays. Lane keeping still reads the
nadir cameras.

Straight-line contact above the published hinge damping is not given a
coefficient here. See ``assumptions.notes.straight_contact`` in
``config/bom/bom.json``.
"""

from __future__ import annotations

_TRUE = {"1", "true", "yes", "on"}
_FALSE = {"0", "false", "no", "off"}
_UNSET = {"", "none", "null", "default", "yaml", "~"}

# One warehouse lap, with a little slack for the perimeter rounding.
_ONE_LAP_LO = 0.98
_ONE_LAP_HI = 1.02


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


def resolve_speed_trace(
    *,
    config_value=None,
    env_value: str | None = None,
    mission_limit_m: float | None = None,
    lap_m: float | None = None,
) -> bool:
    """Whether to print ``SPEED_TRACE`` / ``SIM_RTF`` on the Webots console.

    ``RBM_SPEED_TRACE`` wins, then YAML ``robot.speed_trace``. When both
    are blank, the trace is on only for a one-lap cap: the mission limit
    within 2% of one warehouse perimeter. The S track has no cap, so it
    stays quiet.
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
    """

    def __init__(
        self,
        enabled: bool,
        *,
        step_s: float = 0.1,
        rtf_period_s: float = 10.0,
    ) -> None:
        self.enabled = bool(enabled)
        self.step_s = float(step_s)
        self.rtf_period_s = float(rtf_period_s)
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
        if not self.enabled:
            return []
        t_sim = float(t_sim)
        t_wall = float(t_wall)
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
        return out
