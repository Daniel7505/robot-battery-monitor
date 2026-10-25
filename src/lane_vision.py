"""Row-fit lane vision: nadir pixels -> ground points -> fitted lane model.

Stdlib only (the Webots controller runs on the host python and imports
nothing heavier than ``math``). Nothing here knows the track: the lane
is measured from the two shoulder cameras every frame.

Pipeline (one call per new camera frame, see :class:`LaneTracker`):

1. ``CameraModel`` maps a pixel (col, row) to a ray in the robot frame and
   intersects it with the flat paint plane. Webots R2025a camera axes:
   the camera looks along its local +X, +Y is image-left, +Z is image-up
   (doc "camera.md": s=0 <-> +hFov/2, t=0 <-> +vFov/2). Robot frame:
   origin on the floor under the drive axle, +x forward, +y left, +z up.
2. Yellow pixels (min(R,G) - B >= 0.22; equals ``lane_keep.yellow_score``
   for r == g but rejects red/green marks) are grouped
   into runs along every image ROW and every image COLUMN. Each run's two
   edges are projected to the floor; the midpoint of the two floor points
   lies on the painted stripe's centre line for ANY stripe angle, so
   sideways lines (90 deg turns) work. Runs clipped by the image border
   are dropped (their midpoint is not the stripe centre).
3. Points are labelled left/right line (camera + sign at start-up, then
   the previous frame's model, motion-compensated by odometry/IMU).
4. One lane model is fitted to both lines at once: a centre line that is a
   circle arc (straight when curvature = 0) through (0, a0) with direction
   psi and curvature kappa; left/right paint sit at +/- width/2 from it.
   Robust Gauss-Newton (Huber + gate). When both lines are in the near
   window the width is the live fit. The constant 1.30 m, and the smoothed
   width from earlier frames, are used only when a line is missing.
5. The known 6 cm stripe width is only a CHECK: each run's floor length
   times |cos| to the fitted line normal should be ~6 cm.

The pieces live in sibling modules. This module re-exports what callers
already import.
"""

from __future__ import annotations

from src.lane_camera import *  # noqa: F403
from src.lane_camera import _WBT_DEFAULT, _axis_angle_matrix, _mat_mul
from src.lane_mask import *  # noqa: F403
from src.lane_lines import *  # noqa: F403
from src.lane_lines import _median
from src.lane_geom import *  # noqa: F403
from src.lane_draw import *  # noqa: F403
