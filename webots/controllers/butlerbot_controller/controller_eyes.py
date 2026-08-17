"""Lane cameras, look-down / forecast aim, identity LINE_CAM lock.

LINE_CAM look-at is forbidden. The identity revert lives here; the call
that enforces it stays visible in butlerbot_controller._run_loop.
"""
from __future__ import annotations

import os

from controller import Camera, Robot

try:
    from controller import Supervisor
except ImportError:
    Supervisor = None

def _load_lookdown_math():
    try:
        from src.lane_keep import (
            MarkStopTracker,
            beam_sf_rotation,
            camera_fov_pyramid,
            classify_view,
            FORECAST_IMAGE_ROLL_RAD,
            roll_camera_sf,
            cruise_speed_m_s,
            ground_hit_ahead_m,
            image_row_to_ground_ahead_m,
            look_at_sf_rotation,
            stopping_distance_m,
            time_to_mark_s,
        )

        return {
            "look_at": look_at_sf_rotation,
            "beam_rot": beam_sf_rotation,
            "fov_pyramid": camera_fov_pyramid,
            "roll_cam": roll_camera_sf,
            "forecast_roll": FORECAST_IMAGE_ROLL_RAD,
            "classify": classify_view,
            "ground_hit": ground_hit_ahead_m,
            "row_range": image_row_to_ground_ahead_m,
            "tracker_cls": MarkStopTracker,
            "cruise_v": cruise_speed_m_s,
            "d_stop": stopping_distance_m,
            "t_mark": time_to_mark_s,
        }
    except Exception as exc:
        print(f"WARNING: look-down math not loaded ({exc})")
        return None


def _camera_mean_rgb(cam: Camera | None) -> tuple[float, float, float]:
    """Mean RGB via Webots accessors — SKY/GROUND interlock."""
    if cam is None:
        return (0.0, 0.0, 0.0)
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
        get_r = cam.imageGetRed
        get_g = cam.imageGetGreen
        get_b = cam.imageGetBlue
    except Exception:
        return (0.0, 0.0, 0.0)
    if not image or w <= 0 or h <= 0:
        return (0.0, 0.0, 0.0)
    rs = gs = bs = 0
    n = 0
    step = 1 if w * h <= 4000 else 2
    for y in range(0, h, step):
        for x in range(0, w, step):
            rs += get_r(image, w, x, y)
            gs += get_g(image, w, x, y)
            bs += get_b(image, w, x, y)
            n += 1
    if n <= 0:
        return (0.0, 0.0, 0.0)
    return (rs / (n * 255.0), gs / (n * 255.0), bs / (n * 255.0))


def _camera_max_rgb(cam: Camera | None) -> tuple[float, float, float]:
    """Debug: brightest R,G,B seen through Webots accessors."""
    if cam is None:
        return (0.0, 0.0, 0.0)
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
        get_r = cam.imageGetRed
        get_g = cam.imageGetGreen
        get_b = cam.imageGetBlue
    except Exception:
        return (0.0, 0.0, 0.0)
    if not image or w <= 0 or h <= 0:
        return (0.0, 0.0, 0.0)
    mr = mg = mb = 0
    step = 3
    for y in range(0, h, step):
        for x in range(0, w, step):
            mr = max(mr, get_r(image, w, x, y))
            mg = max(mg, get_g(image, w, x, y))
            mb = max(mb, get_b(image, w, x, y))
    return (mr / 255.0, mg / 255.0, mb / 255.0)


def _camera_peak_score_and_row(
    cam: Camera | None, score_fn, peak_fn
) -> tuple[float, int | None]:
    """Peak color score and the image row it came from (Webots accessors)."""
    if cam is None or score_fn is None:
        return 0.0, None
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
    except Exception:
        return 0.0, None
    if not image or w <= 0 or h <= 0:
        return 0.0, None
    get_r = getattr(cam, "imageGetRed", None)
    get_g = getattr(cam, "imageGetGreen", None)
    get_b = getattr(cam, "imageGetBlue", None)
    if get_r is None or get_g is None or get_b is None:
        if peak_fn is None:
            return 0.0, None
        try:
            return float(peak_fn(image, w, h, score_fn)), None
        except Exception:
            return 0.0, None
    scale = 1.0 / 255.0
    # Floor lives in the lower part of a forward cam; sky washes the top.
    # Scan every pixel in that band so a 2 cm stripe can still register.
    best = 0.0
    closest_y: int | None = None
    # Finish cams: skip sky in the top third. Tiny yellow eyes: full frame.
    y0 = 0 if h <= 20 else h // 3
    # Bottom row is nearest floor. Prefer the closest pixel that still
    # looks like the mark so range is the leading edge, not a far bloom.
    for y in range(h - 1, y0 - 1, -1):
        row_best = 0.0
        for x in range(w):
            rgb = (
                get_r(image, w, x, y) * scale,
                get_g(image, w, x, y) * scale,
                get_b(image, w, x, y) * scale,
            )
            val = float(score_fn(rgb))
            if val > row_best:
                row_best = val
            if val > best:
                best = val
        if closest_y is None and row_best >= 0.28:
            closest_y = y
    return best, closest_y if closest_y is not None else None


def _camera_peak_score(cam: Camera | None, score_fn, peak_fn) -> float:
    score, _row = _camera_peak_score_and_row(cam, score_fn, peak_fn)
    return score


def _yellow_ground_y(cam, node, pixel_fn, ground_fn) -> float | None:
    """Robot-frame Y of the nearest yellow pixel on the floor."""
    if cam is None or node is None or pixel_fn is None or ground_fn is None:
        return None
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
        fov = float(cam.getFov())
        pos = tuple(node.getField("translation").getSFVec3f())
        rot = tuple(node.getField("rotation").getSFRotation())
    except Exception:
        return None
    if not image or w <= 0 or h <= 0:
        return None
    try:
        pix = pixel_fn(image, w, h)
    except Exception:
        return None
    if pix is None:
        return None
    try:
        hit = ground_fn(pos, rot, pix[0], pix[1], w, h, fov)
    except Exception:
        return None
    if hit is None:
        return None
    return float(hit[1])


def _forecast_offset(cam: Camera | None, offset_fn) -> float | None:
    """Offset on the 90° CW buffer — same picture the Z/W HUD shows."""
    if cam is None or offset_fn is None:
        return None
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
    except Exception:
        return None
    if not image or w <= 0 or h <= 0:
        return None
    try:
        from src.lane_keep import rotate_bgra_90_cw

        rot_buf, rw, rh = rotate_bgra_90_cw(image, w, h)
        return offset_fn(rot_buf, rw, rh)
    except Exception:
        return None


def _line_offset(cam: Camera | None, offset_fn) -> float | None:
    if cam is None or offset_fn is None:
        return None
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
    except Exception:
        return None
    if not image or w <= 0 or h <= 0:
        return None
    try:
        return offset_fn(image, w, h)
    except Exception:
        return None


def _read_lane_eyes(
    cams: dict,
    yellow_fn,
    red_fn,
    peak_fn,
    offset_fn=None,
    curve_fn=None,
    fill_fn=None,
    far_fn=None,
) -> dict:
    """Peak color scores so a thin paint stripe still registers."""
    ly = _camera_peak_score(cams.get("line_left"), yellow_fn, peak_fn)
    ry = _camera_peak_score(cams.get("line_right"), yellow_fn, peak_fn)
    lo = _line_offset(cams.get("line_left"), offset_fn)
    ro = _line_offset(cams.get("line_right"), offset_fn)
    lc = _line_offset(cams.get("line_left"), curve_fn)
    rc = _line_offset(cams.get("line_right"), curve_fn)
    lf = _line_offset(cams.get("line_left"), fill_fn)
    rf = _line_offset(cams.get("line_right"), fill_fn)
    lfo = _line_offset(cams.get("line_left"), far_fn)
    rfo = _line_offset(cams.get("line_right"), far_fn)
    zo = _line_offset(cams.get("forecast_z"), offset_fn)
    wo = _line_offset(cams.get("forecast_w"), offset_fn)
    zf = _line_offset(cams.get("forecast_z"), fill_fn)
    wf = _line_offset(cams.get("forecast_w"), fill_fn)
    fr_l, row_l = _camera_peak_score_and_row(cams.get("finish_cam"), red_fn, peak_fn)
    fr_r, row_r = _camera_peak_score_and_row(cams.get("finish_cam_r"), red_fn, peak_fn)
    if fr_r > fr_l:
        red, row = fr_r, row_r
    else:
        red, row = fr_l, row_l
    return {
        "left_yellow": round(ly, 3),
        "right_yellow": round(ry, 3),
        "left_offset": None if lo is None else round(float(lo), 3),
        "right_offset": None if ro is None else round(float(ro), 3),
        "left_curve": None if lc is None else round(float(lc), 3),
        "right_curve": None if rc is None else round(float(rc), 3),
        "left_fill": None if lf is None else round(float(lf), 3),
        "right_fill": None if rf is None else round(float(rf), 3),
        "left_far_offset": None if lfo is None else round(float(lfo), 3),
        "right_far_offset": None if rfo is None else round(float(rfo), 3),
        "z_offset": None if zo is None else round(float(zo), 3),
        "w_offset": None if wo is None else round(float(wo), 3),
        "z_fill": None if zf is None else round(float(zf), 3),
        "w_fill": None if wf is None else round(float(wf), 3),
        "finish_red": round(red, 3),
        "finish_red_row": row,
    }



_LOOKDOWN_PAIRS = (
    ("FINISH_CAM", "LOOKDOWN_AIM_L", None),
    ("FINISH_CAM_R", "LOOKDOWN_AIM_R", None),
    # Purple LOOKDOWN_BEAM_* removed — they cluttered the Z/W view.
    # LINE_CAM_* stay identity. Forecast Z/W are a separate pair.
)
_FORECAST_PAIRS = (
    ("FORECAST_CAM_Z", "FORECAST_AIM_Z", "Z_BEAM", "Z_FOV_COORD"),
    ("FORECAST_CAM_W", "FORECAST_AIM_W", "W_BEAM", "W_FOV_COORD"),
)
# Daniel mouse-frames these. Copy DUMP_CAM numbers; do not guess ENU.
# None = do not overwrite rotation (he is lining Z/W up to the wire).
_FORECAST_LOCK = {
    # Daniel 2026-08-16: this boot's view is the first correct Z/W.
    # DUMP_CAM: identity 0 0 1 0. Do not look-at overwrite.
    "FORECAST_CAM_Z": (0.0, 0.0, 1.0, 0.0),
    "FORECAST_CAM_W": (0.0, 0.0, 1.0, 0.0),
}
_LINE_CAM_IDENTITY = (0.0, 1.0, 0.0, 0.0)
_LOOKDOWN_SNAP_DIR = os.path.join(
    os.path.expanduser("~"), "OneDrive", "Desktop", "Grok Workspace"
)


def _aim_lookdown_cameras(robot: Robot, mathkit: dict | None) -> float:
    """Point finish cams at the magenta floor pucks. Returns ground-hit D (m)."""
    look_ahead = 1.0
    if not mathkit or Supervisor is None or not isinstance(robot, Supervisor):
        print("Look-down aim skipped — Supervisor or math unavailable")
        return look_ahead
    look_at = mathkit["look_at"]
    beam_rot = mathkit["beam_rot"]
    ground_hit = mathkit["ground_hit"]
    for cam_def, aim_def, beam_def in _LOOKDOWN_PAIRS:
        try:
            cam_node = robot.getFromDef(cam_def)
            aim_node = robot.getFromDef(aim_def)
            beam_node = robot.getFromDef(beam_def) if beam_def else None
        except Exception as exc:
            print(f"Look-down aim: missing {cam_def}/{aim_def}: {exc}")
            continue
        if cam_node is None or aim_node is None:
            print(f"Look-down aim: DEF {cam_def} or {aim_def} not found")
            continue
        try:
            cam_pos = tuple(cam_node.getField("translation").getSFVec3f())
            aim_pos = tuple(aim_node.getField("translation").getSFVec3f())
            rot = look_at(cam_pos, aim_pos)
            cam_node.getField("rotation").setSFRotation(list(rot))
            if beam_node is not None and beam_rot is not None:
                mid = (
                    0.5 * (cam_pos[0] + aim_pos[0]),
                    0.5 * (cam_pos[1] + aim_pos[1]),
                    0.5 * (cam_pos[2] + aim_pos[2]),
                )
                beam_node.getField("translation").setSFVec3f(list(mid))
                beam_node.getField("rotation").setSFRotation(list(beam_rot(cam_pos, aim_pos)))
            d_hit = float(ground_hit(cam_pos, aim_pos))
            if cam_def.startswith("FINISH_CAM"):
                look_ahead = d_hit
            dx = aim_pos[0] - cam_pos[0]
            dy = aim_pos[1] - cam_pos[1]
            dz = aim_pos[2] - cam_pos[2]
            extra = ""
            if cam_def.startswith("FINISH_CAM"):
                v_cruise = float(mathkit["cruise_v"]())
                t_mark = mathkit["t_mark"](d_hit, v_cruise)
                d_stop = float(mathkit["d_stop"](v_cruise))
                t_coast = None if t_mark is None else max(0.0, (d_hit - d_stop) / v_cruise)
                extra = (
                    f" D={d_hit:.2f}m (v={v_cruise:.2f} m/s  t_mark={t_mark:.2f}s  "
                    f"d_stop={d_stop:.2f}m  coast={t_coast:.2f}s)"
                )
            print(
                f"Look-down aimed {cam_def} → {aim_def} "
                f"look=({dx:.2f},{dy:.2f},{dz:.2f}){extra}"
            )
        except Exception as exc:
            print(f"Look-down aim failed for {cam_def}: {exc}")
    return look_ahead


def _write_bgra_bmp(path: str, buf: bytes | bytearray, width: int, height: int) -> None:
    """Uncompressed 24-bit BMP, top-down-ish via flipped rows. No extra deps."""
    w = int(width)
    h = int(height)
    row_b = (w * 3 + 3) & ~3
    pixels = bytearray(row_b * h)
    src = memoryview(buf)
    for y in range(h):
        # BMP is bottom-up
        dest_y = h - 1 - y
        for x in range(w):
            s = (y * w + x) * 4
            d = dest_y * row_b + x * 3
            if s + 3 <= len(src):
                pixels[d] = src[s]
                pixels[d + 1] = src[s + 1]
                pixels[d + 2] = src[s + 2]
    pixel_off = 54
    size = pixel_off + len(pixels)
    hdr = bytearray(54)
    hdr[0:2] = b"BM"
    hdr[2:6] = size.to_bytes(4, "little")
    hdr[10:14] = pixel_off.to_bytes(4, "little")
    hdr[14:18] = (40).to_bytes(4, "little")
    hdr[18:22] = w.to_bytes(4, "little", signed=True)
    hdr[22:26] = h.to_bytes(4, "little", signed=True)
    hdr[26:28] = (1).to_bytes(2, "little")
    hdr[28:30] = (24).to_bytes(2, "little")
    with open(path, "wb") as fh:
        fh.write(hdr)
        fh.write(pixels)


def _set_fov_wire(robot: Robot, coord_def: str, pts: list) -> None:
    """Push apex+4 corners into an IndexedLineSet Coordinate."""
    try:
        node = robot.getFromDef(coord_def)
    except Exception:
        node = None
    if node is None:
        print(f"FOV wire {coord_def} missing")
        return
    try:
        field = node.getField("point")
        for i, p in enumerate(pts):
            field.setMFVec3f(i, [float(p[0]), float(p[1]), float(p[2])])
    except Exception as exc:
        print(f"FOV wire {coord_def} skip: {exc}")


def _aim_forecast_cameras(robot: Robot, mathkit: dict | None) -> None:
    """Point Z/W at the 2 m wall pucks and draw look beam + FOV pyramid."""
    if not mathkit or Supervisor is None or not isinstance(robot, Supervisor):
        print("Forecast aim skipped — Supervisor or math unavailable")
        return
    look_at = mathkit["look_at"]
    beam_rot = mathkit["beam_rot"]
    roll_fn = mathkit.get("roll_cam")
    roll_rad = mathkit.get("forecast_roll", 0.0)
    fov_fn = mathkit.get("fov_pyramid")
    for cam_def, aim_def, beam_def, fov_def in _FORECAST_PAIRS:
        try:
            cam_node = robot.getFromDef(cam_def)
            aim_node = robot.getFromDef(aim_def)
            beam_node = robot.getFromDef(beam_def)
        except Exception as exc:
            print(f"Forecast aim: missing {cam_def}/{aim_def}: {exc}")
            continue
        if cam_node is None or aim_node is None:
            print(f"Forecast aim: DEF {cam_def} or {aim_def} not found")
            continue
        try:
            cam_pos = tuple(cam_node.getField("translation").getSFVec3f())
            aim_pos = tuple(aim_node.getField("translation").getSFVec3f())
            lock = _FORECAST_LOCK.get(cam_def)
            if lock is not None:
                rot = tuple(lock)
                cam_node.getField("rotation").setSFRotation(list(rot))
            else:
                # Wire stays on the puck. Camera rotation is Daniel's.
                rot = look_at(cam_pos, aim_pos)
                print(
                    f"Forecast {cam_def} UNLOCKED — frame it to the wire, "
                    "then DUMP_CAM"
                )
            if beam_node is not None and beam_rot is not None:
                mid = (
                    0.5 * (cam_pos[0] + aim_pos[0]),
                    0.5 * (cam_pos[1] + aim_pos[1]),
                    0.5 * (cam_pos[2] + aim_pos[2]),
                )
                beam_node.getField("translation").setSFVec3f(list(mid))
                beam_node.getField("rotation").setSFRotation(
                    list(beam_rot(cam_pos, aim_pos))
                )
            fov_rad = 0.85
            try:
                fov_rad = float(cam_node.getField("fieldOfView").getSFFloat())
            except Exception:
                pass
            if fov_fn is not None:
                pts = fov_fn(cam_pos, rot, fov_rad, 64, 64, 2.0)
                if len(pts) >= 5:
                    _set_fov_wire(robot, fov_def, pts)
            dx = aim_pos[0] - cam_pos[0]
            dy = aim_pos[1] - cam_pos[1]
            dz = aim_pos[2] - cam_pos[2]
            deg = fov_rad * 180.0 / 3.141592653589793
            print(
                f"Forecast aimed {cam_def} → {aim_def} "
                f"look=({dx:.2f},{dy:.2f},{dz:.2f}) FOV={deg:.1f}deg "
                f"roll={float(roll_rad)*180.0/3.141592653589793:.0f}deg"
            )
        except Exception as exc:
            print(f"Forecast aim failed for {cam_def}: {exc}")


def _run_lookdown_interlock(cams: dict, classify_fn) -> str:
    """Classify what finish_cam actually sees. Save a still for the inbox."""
    rgb = _camera_mean_rgb(cams.get("finish_cam"))
    label = "unknown"
    if classify_fn is not None:
        try:
            label = str(classify_fn(rgb))
        except Exception:
            label = "unknown"
    print(
        f"Look-down interlock: {label.upper()}  "
        f"meanRGB=({rgb[0]:.2f},{rgb[1]:.2f},{rgb[2]:.2f})"
    )
    for key in ("line_left", "line_right", "forecast_z", "forecast_w"):
        eye = _camera_mean_rgb(cams.get(key))
        tag = ""
        if classify_fn is not None:
            try:
                tag = f" {classify_fn(eye)}"
            except Exception:
                tag = ""
        print(
            f"  {key}{tag} meanRGB=({eye[0]:.2f},{eye[1]:.2f},{eye[2]:.2f})"
        )
    for key, fname in (
        ("finish_cam", "lookdown-cam.png"),
        ("line_left", "line-left.png"),
        ("line_right", "line-right.png"),
        ("forecast_z", "forecast-z.png"),
        ("forecast_w", "forecast-w.png"),
    ):
        cam = cams.get(key)
        if cam is None:
            continue
        try:
            path = os.path.join(_LOOKDOWN_SNAP_DIR, fname)
            cam.saveImage(path, 90)
            print(f"Look-down snapshot {key} → {path}")
        except Exception as exc:
            print(f"Look-down snapshot {key} skipped: {exc}")
    return label


def _revert_line_cams_identity(robot: Robot) -> None:
    """Undo a drunk line-cam aim. Identity = look −Z / down on the stripe."""
    if Supervisor is None or not isinstance(robot, Supervisor):
        return
    for cam_def in ("LINE_CAM_L", "LINE_CAM_R"):
        try:
            node = robot.getFromDef(cam_def)
        except Exception:
            node = None
        if node is None:
            continue
        try:
            node.getField("rotation").setSFRotation(list(_LINE_CAM_IDENTITY))
            print(f"Line-cam {cam_def} reverted to identity (aim failed interlock)")
        except Exception as exc:
            print(f"Line-cam {cam_def} identity revert skipped: {exc}")


def _run_preview_interlock(cams: dict, robot: Robot) -> dict:
    """Far sliver must be dirt ahead, not sky or the robot.

    Aim is already pointed at YELLOW_AIM (2 m). This only *measures*
    whether that pose is useful. Broken dirt box → revert identity.
    Far sliver sky / no range → keep the forward view, drop preview.
    """
    out = {
        "aim_ok": False,
        "preview_ok": False,
        "range_m": None,
        "left": None,
        "right": None,
    }
    try:
        from src.lane_keep import (
            band_mean_rgb,
            band_sky_frac,
            classify_view,
            dirt_band_ok,
            preview_band_ok,
            preview_look_ahead_m,
            yellow_far_band,
            yellow_far_offset,
            yellow_look_band,
        )
    except Exception as exc:
        print(f"Preview interlock skipped — lane_keep helpers ({exc})")
        return out
    if Supervisor is None or not isinstance(robot, Supervisor):
        print("Preview interlock skipped — not a Supervisor")
        return out
    reports = {}
    for key, cam_def in (("left", "LINE_CAM_L"), ("right", "LINE_CAM_R")):
        cam = cams.get(f"line_{key}")
        try:
            node = robot.getFromDef(cam_def)
        except Exception:
            node = None
        if cam is None or node is None:
            print(f"Preview interlock {key}: missing cam/node")
            reports[key] = {"dirt_ok": False, "far_ok": False}
            continue
        try:
            image = cam.getImage()
            w = int(cam.getWidth())
            h = int(cam.getHeight())
            fov = float(cam.getFov())
            pos = tuple(node.getField("translation").getSFVec3f())
            rot = tuple(node.getField("rotation").getSFRotation())
        except Exception as exc:
            print(f"Preview interlock {key}: read failed ({exc})")
            reports[key] = {"dirt_ok": False, "far_ok": False}
            continue
        if not image or w < 2 or h < 2:
            reports[key] = {"dirt_ok": False, "far_ok": False}
            continue
        y0, y1 = yellow_look_band(h)
        fy0, fy1 = yellow_far_band(h)
        dirt_label = classify_view(band_mean_rgb(image, w, h, y0, y1))
        far_label = classify_view(band_mean_rgb(image, w, h, fy0, fy1))
        dirt_sky = band_sky_frac(image, w, h, y0, y1)
        far_sky = band_sky_frac(image, w, h, fy0, fy1)
        dirt_ok = dirt_band_ok(image, w, h)
        rng = preview_look_ahead_m(pos, rot, h, fov)
        off = yellow_far_offset(image, w, h)
        # Identity eyes often have no meter range (look is not +X). If
        # the far sliver still sees paint, that is the second reading.
        far_ok = bool(preview_band_ok(image, w, h) and (rng is not None or off is not None))
        reports[key] = {
            "dirt_ok": dirt_ok,
            "far_ok": far_ok,
            "dirt_label": dirt_label,
            "far_label": far_label,
            "dirt_sky": round(dirt_sky, 3),
            "far_sky": round(far_sky, 3),
            "range_m": None if rng is None else round(float(rng), 3),
            "far_offset": None if off is None else round(float(off), 3),
            "far_rows": (fy0, fy1),
        }
        print(
            f"Preview interlock {key}: dirt={dirt_label} sky={dirt_sky:.2f} "
            f"far={far_label} far_sky={far_sky:.2f} range={rng} "
            f"off={off} rows={fy0}-{fy1} "
            f"{'OK' if dirt_ok and far_ok else 'FAIL'}"
        )
    out["left"] = reports.get("left")
    out["right"] = reports.get("right")
    aim_ok = bool(
        reports.get("left", {}).get("dirt_ok")
        and reports.get("right", {}).get("dirt_ok")
    )
    preview_ok = bool(
        aim_ok
        and (
            reports.get("left", {}).get("far_ok")
            or reports.get("right", {}).get("far_ok")
        )
    )
    ranges = [
        reports[k]["range_m"]
        for k in ("left", "right")
        if reports.get(k) and reports[k].get("range_m") is not None
    ]
    out["aim_ok"] = aim_ok
    out["preview_ok"] = preview_ok
    out["range_m"] = None if not ranges else sum(ranges) / len(ranges)
    if not aim_ok:
        print(
            "Preview DISABLED — dirt box is not ground/yellow. "
            "Reverting line cams to identity (the 90° roll trap)."
        )
        _revert_line_cams_identity(robot)
    elif not preview_ok:
        print(
            "Preview DISABLED — far sliver has no paint and no 0.4–4 m floor hit. "
            "Planner uses error-now only."
        )
    elif out["range_m"] is None:
        print(
            "Preview ENABLED  image-far (no meter range, t_fallback=2.0s) "
            "— identity eyes, dirt sliver still has paint"
        )
    else:
        print(
            f"Preview ENABLED  look-ahead={out['range_m']:.2f}m "
            "(far dirt just under horizon)"
        )
    return out


_DUMP_CAM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "DUMP_CAM")


def _maybe_dump_viewpoint(robot: Robot) -> None:
    """If DUMP_CAM exists beside this controller, print live viewpoint and remove it."""
    if not os.path.isfile(_DUMP_CAM):
        return
    try:
        os.remove(_DUMP_CAM)
    except OSError:
        pass
    if Supervisor is None or not isinstance(robot, Supervisor):
        print("VIEWPOINT dump: not a Supervisor")
        return
    try:
        vp = robot.getFromDef("VIEWPOINT")
        if vp is None:
            print("VIEWPOINT dump: DEF VIEWPOINT missing")
            return
        pos = vp.getField("position").getSFVec3f()
        ori = vp.getField("orientation").getSFRotation()
        fol = vp.getField("follow").getSFString()
        print(
            "VIEWPOINT COPY "
            f"position {pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f} "
            f"orientation {ori[0]:.6f} {ori[1]:.6f} {ori[2]:.6f} {ori[3]:.6f} "
            f"follow={fol!r}"
        )
    except Exception as exc:
        print(f"VIEWPOINT dump failed: {exc}")
    for cam_def in ("FORECAST_CAM_Z", "FORECAST_CAM_W"):
        try:
            node = robot.getFromDef(cam_def)
            if node is None:
                print(f"{cam_def} COPY: missing")
                continue
            t = node.getField("translation").getSFVec3f()
            r = node.getField("rotation").getSFRotation()
            print(
                f"{cam_def} COPY "
                f"translation {t[0]:.5f} {t[1]:.5f} {t[2]:.5f} "
                f"rotation {r[0]:.6f} {r[1]:.6f} {r[2]:.6f} {r[3]:.6f}"
            )
        except Exception as exc:
            print(f"{cam_def} COPY failed: {exc}")

