"""Yellow mask and stripe-centre points."""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.lane_camera import CameraModel

YELLOW_THRESH = 0.22  # same cutoff as lane_keep.yellow_score
YELLOW_MODE = "rg"  # yellow_mask: "rg" = min(R,G)-B (rejects red/green bars), "lane_keep" = gap score
MAX_RANGE_M = 3.5

# --------------------------------------------------------------------------
# Detection
# --------------------------------------------------------------------------


def yellow_mask(
    image, width: int, height: int, thresh: float = YELLOW_THRESH, mode: str | None = None
) -> bytearray:
    """1 where a BGRA pixel is lane yellow.

    ``mode="rg"`` (default, YELLOW_MODE): min(R, G) - B >= thresh. Yellow needs
    BOTH red and green; red or green marks (finish/start bars) don't pass.
    For r == g (the yellow paint and its blend with a grey floor) it equals
    the gap-mode score.
    ``mode="lane_keep"``: ``lane_keep.yellow_score`` = (r+g)/2 - b >= thresh
    (gap-mode test; scores the red finish bar 0.9/0.15/0.12 as 0.41 = yellow).
    """
    n = int(width) * int(height)
    buf = bytes(image[: n * 4]) if not isinstance(image, (bytes, bytearray)) else image
    if len(buf) < n * 4:
        return bytearray(n)
    bs = buf[0: n * 4: 4]
    gs = buf[1: n * 4: 4]
    rs = buf[2: n * 4: 4]
    if (mode or YELLOW_MODE) == "lane_keep":
        lim = 510.0 * float(thresh)
        return bytearray(1 if (r + g - 2 * b) >= lim else 0 for b, g, r in zip(bs, gs, rs))
    lim = 255.0 * float(thresh)
    return bytearray(1 if (min(r, g) - b) >= lim else 0 for b, g, r in zip(bs, gs, rs))


def _runs(seq) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    start = -1
    for i, v in enumerate(seq):
        if v:
            if start < 0:
                start = i
        elif start >= 0:
            out.append((start, i - 1))
            start = -1
    if start >= 0:
        out.append((start, len(seq) - 1))
    return out


@dataclass
class LinePoint:
    x: float
    y: float
    cam: str
    axis: str  # "row" or "col"
    seg_dx: float  # floor vector edge->edge (for the width check)
    seg_dy: float
    run_px: int
    col: float
    row: float
    side: int = 0  # +1 left line, -1 right line, 0 unassigned


def detect_points(
    cam: CameraModel,
    image,
    width: int | None = None,
    height: int | None = None,
    *,
    thresh: float = YELLOW_THRESH,
    max_run_px: int = 60,
    yellow_mode: str | None = None,
    mask=None,
) -> list[LinePoint]:
    """Stripe-centre floor points from row and column runs of yellow.

    ``mask``: a precomputed :func:`yellow_mask` (then ``image`` may be None)."""
    w = int(width or cam.width)
    h = int(height or cam.height)
    if (not image and mask is None) or w < 4 or h < 4:
        return []
    if mask is None:
        mask = yellow_mask(image, w, h, thresh, yellow_mode)
    pts: list[LinePoint] = []

    def _add(e0, e1, axis, run_px, ccol, crow):
        p0 = cam.pixel_to_ground(*e0)
        p1 = cam.pixel_to_ground(*e1)
        if p0 is None or p1 is None:
            return
        mx, my = 0.5 * (p0[0] + p1[0]), 0.5 * (p0[1] + p1[1])
        if math.hypot(mx, my) > MAX_RANGE_M:
            return
        pts.append(LinePoint(mx, my, cam.name, axis, p1[0] - p0[0], p1[1] - p0[1], run_px, ccol, crow))

    for row in range(h):
        line = mask[row * w: (row + 1) * w]
        for a, b in _runs(line):
            if a == 0 or b == w - 1 or (b - a + 1) > max_run_px:
                continue
            _add((a - 0.5, row), (b + 0.5, row), "row", b - a + 1, 0.5 * (a + b), float(row))
    for col in range(w):
        line = mask[col::w]
        for a, b in _runs(line):
            if a == 0 or b == h - 1 or (b - a + 1) > max_run_px:
                continue
            _add((col, a - 0.5), (col, b + 0.5), "col", b - a + 1, float(col), 0.5 * (a + b))
    return pts
