"""Lane-keep + red-line brake — the onboard agent's first driving job.

Yellow edge cameras and a forward finish camera produce 0–1 color scores.
This module is the policy the Webots controller runs every sim step (PMS
ticks are too slow to steer). OnboardAgent uses the same function so the
dashboard log matches what the wheels do.

Steer: more yellow on the left means we drifted toward +Y → add right yaw.
Red above threshold → brake (ABS in the controller).
"""

from __future__ import annotations

DEFAULT_CRUISE_RAD_S = 5.5
DEFAULT_K_STEER = 2.4
DEFAULT_RED_THRESH = 0.28
MAX_WHEEL_RAD_S = 8.0


def yellow_score(rgb: tuple[float, float, float]) -> float:
    """0–1: yellow paint (high R+G, low B) vs gray floor."""
    r, g, b = rgb
    return max(0.0, min(1.0, (r + g) * 0.5 - b))


def red_score(rgb: tuple[float, float, float]) -> float:
    """0–1: red finish bar vs gray / yellow / green."""
    r, g, b = rgb
    return max(0.0, min(1.0, r - max(g, b)))


def mean_rgb_bgra(image: bytes | bytearray, width: int, height: int) -> tuple[float, float, float]:
    """Average RGB from a Webots Camera BGRA buffer (channels 0–1)."""
    n = max(1, width * height)
    rs = gs = bs = 0
    buf = memoryview(image)
    pixels = min(n, len(buf) // 4)
    for i in range(pixels):
        o = i * 4
        bs += buf[o]
        gs += buf[o + 1]
        rs += buf[o + 2]
    if pixels <= 0:
        return (0.0, 0.0, 0.0)
    scale = 1.0 / (pixels * 255.0)
    return (rs * scale, gs * scale, bs * scale)


def peak_score_bgra(
    image: bytes | bytearray,
    width: int,
    height: int,
    score_fn,
    *,
    percentile: float = 0.98,
) -> float:
    """Score every pixel; return a high percentile so a thin stripe still counts.

    Mean-of-frame washed a 2 cm red finish line to 0.0. Tracks stay as painted —
    the eye has to find the mark, not the other way around.
    """
    buf = memoryview(image)
    pixels = min(width * height, len(buf) // 4)
    if pixels <= 0:
        return 0.0
    scale = 1.0 / 255.0
    stride = 1 if pixels <= 5000 else 2
    scores: list[float] = []
    for i in range(0, pixels, stride):
        o = i * 4
        rgb = (buf[o + 2] * scale, buf[o + 1] * scale, buf[o] * scale)
        scores.append(float(score_fn(rgb)))
    scores.sort()
    idx = min(len(scores) - 1, max(0, int(len(scores) * percentile)))
    return scores[idx]


def lane_keep_command(
    left_yellow: float,
    right_yellow: float,
    finish_red: float,
    *,
    cruise: float = DEFAULT_CRUISE_RAD_S,
    k_steer: float = DEFAULT_K_STEER,
    red_thresh: float = DEFAULT_RED_THRESH,
) -> dict:
    """Return wheel cmds. ``brake`` is True when the red mark is in view."""
    if finish_red >= red_thresh:
        return {
            "left": 0.0,
            "right": 0.0,
            "brake": True,
            "error": 0.0,
            "reason": "red finish — brake",
        }
    err = float(right_yellow) - float(left_yellow)
    left = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, cruise - k_steer * err))
    right = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, cruise + k_steer * err))
    return {
        "left": round(left, 3),
        "right": round(right, 3),
        "brake": False,
        "error": round(err, 4),
        "reason": "lane keep",
    }
