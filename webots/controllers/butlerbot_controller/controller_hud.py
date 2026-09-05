"""Webots Display HUD + shoulder nadir overlays. No physics.

Choir overlay (yellow_look_band / picture-wins ticks / L P vs R P) is
archived at archives/controller_hud_choir_2026-09-02.py. Restore that
file to undo.

The main ``hud`` Display node is gone from the world. Gauge paint stays
so a missing device is a no-op, not a crash.
"""
from __future__ import annotations

from controller import Display, Robot

_teleop = None


def bind_teleop(mod) -> None:
    global _teleop
    _teleop = mod


HUD_W = 320
HUD_H = 180


def _draw_gauge(display: Display, x: int, y: int, w: int, h: int, ratio: float, color: int, label: str) -> None:
    display.setColor(0x1A2230)
    display.fillRectangle(x, y, w, h)
    display.setColor(0x3A4A5A)
    display.drawRectangle(x, y, w, h)
    fill_w = max(0, int((w - 4) * ratio))
    if fill_w > 0:
        display.setColor(color)
        display.fillRectangle(x + 2, y + 2, fill_w, h - 4)
    display.setColor(0xCCDDEE)
    display.drawText(label, x + 6, y + h - 18)


def _draw_hud(
    display: Display,
    *,
    battery_pct: float,
    thermal_c: float,
    throttle: float,
    message: str | None,
    teleop_active: bool,
    auto_loop: bool,
    api_source: str = "",
    speed_m_s: float = 0.0,
    braking: bool = False,
) -> None:
    """Paint battery/heat gauges. No-op if the world has no ``hud`` Display."""
    display.setAlpha(0.92)
    display.setColor(0x080C12)
    display.fillRectangle(0, 0, HUD_W, HUD_H)

    batt_ratio = battery_pct / 100.0
    if _teleop is not None:
        batt_ratio = _teleop.battery_gauge_ratio(battery_pct)
        heat_ratio = _teleop.thermal_gauge_ratio(thermal_c)
        batt_color = _teleop.gauge_color_hex(1.0 - batt_ratio)
        heat_color = _teleop.gauge_color_hex(heat_ratio)
    else:
        heat_ratio = max(0.0, min(1.0, (thermal_c - 22.0) / 46.0))
        batt_color = 0x33DD66 if batt_ratio > 0.2 else 0xFF4444
        heat_color = 0x33DD66 if heat_ratio < 0.55 else 0xFFAA22

    _draw_gauge(display, 12, 18, 130, 22, batt_ratio, batt_color, f"BATT {battery_pct:.0f}%")
    _draw_gauge(display, 12, 52, 130, 22, heat_ratio, heat_color, f"HEAT {thermal_c:.0f}C")

    display.setColor(0x8899AA)
    if api_source:
        mode = f"API:{api_source[:8]}"
    else:
        mode = "TELEOP" if teleop_active else ("AUTO LOOP" if auto_loop else "STANDBY")
    display.drawText(mode, 12, 88)
    display.drawText(f"Agent cap {throttle * 100:.0f}%", 12, 106)
    speed_kmh = speed_m_s * 3.6
    speed_mph = speed_m_s * 2.237
    display.setColor(0x66EEFF if not braking else 0xFFAA44)
    display.drawText(f"{speed_m_s:.2f} m/s", 168, 22)
    display.setColor(0xAABBCC)
    display.drawText(f"{speed_kmh:.1f} km/h", 168, 42)
    display.drawText(f"{speed_mph:.1f} mph", 168, 58)
    if braking:
        display.setColor(0xFF8844)
        display.drawText("BRAKING", 168, 78)
    display.setColor(0x8899AA)
    display.drawText("I/J/K/L drive · Space stop", 12, 124)

    if message:
        display.setColor(0x3A1808)
        display.fillRectangle(0, HUD_H - 44, HUD_W, 44)
        display.setColor(0xFFAA33)
        display.drawRectangle(0, HUD_H - 44, HUD_W, 44)
        display.setColor(0xFFEECC)
        display.drawText(message[:42], 8, HUD_H - 28)


def _scale_bgra_nn(
    raw: bytes | bytearray,
    src_w: int,
    src_h: int,
    dst_w: int,
    dst_h: int,
) -> bytes:
    """Nearest-neighbor scale. Overlay Display can be larger than 64×64."""
    if src_w == dst_w and src_h == dst_h:
        return bytes(raw)
    src = memoryview(raw)
    out = bytearray(dst_w * dst_h * 4)
    for y in range(dst_h):
        sy = (y * src_h) // dst_h
        for x in range(dst_w):
            sx = (x * src_w) // dst_w
            si = (sy * src_w + sx) * 4
            di = (y * dst_w + x) * 4
            out[di : di + 4] = src[si : si + 4]
    return bytes(out)


def _label_eye_huds(robot: Robot, cams: dict) -> list[dict]:
    """Two shoulder nadir overlays. LINE / finish / Z/W are not painted."""
    specs = (
        ("hud_nadir", "nadir_left", 0x66EEFF, "L SHOULDER", False),
        ("hud_nadir_r", "nadir_right", 0xFFAA66, "R SHOULDER", True),
    )
    handles: list[dict] = []
    for dname, cname, color, text, right in specs:
        try:
            disp = robot.getDevice(dname)
        except Exception:
            disp = None
        cam = cams.get(cname)
        if disp is None or cam is None:
            print(f"HUD {dname}: missing display or camera")
            continue
        try:
            disp.attachCamera(cam)
            try:
                disp.setFont("Arial", 10, True)
            except Exception:
                pass
            try:
                disp.detachCamera()
            except Exception:
                pass
            handles.append(
                {
                    "disp": disp,
                    "cam": cam,
                    "color": int(color),
                    "text": text,
                    "right": bool(right),
                }
            )
            print(f"HUD {dname} ← {cname} labeled {text}")
        except Exception as exc:
            print(f"HUD {dname} skip: {exc}")
    return handles


def _paint_eye_huds(handles: list[dict], lane_eyes: dict) -> None:
    """Paste the nadir crop, tick tape/wheel columns, print gap px / cm."""
    for handle in handles:
        disp = handle["disp"]
        try:
            w = int(disp.getWidth())
            h = int(disp.getHeight())
        except Exception:
            continue
        if w < 2 or h < 2:
            continue
        try:
            cam = handle.get("cam")
            if cam is not None:
                raw = cam.getImage()
                if raw:
                    cw = int(cam.getWidth())
                    ch = int(cam.getHeight())
                    if cw != w or ch != h:
                        raw = _scale_bgra_nn(raw, cw, ch, w, h)
                    ref = disp.imageNew(raw, Display.BGRA, w, h)
                    disp.imagePaste(ref, 0, 0, False)
                    disp.imageDelete(ref)
            disp.setColor(int(handle["color"]))
            disp.drawRectangle(0, 0, max(1, w - 1), max(1, h - 1))
            disp.drawText(str(handle["text"]), 2, 1)
            right = bool(handle.get("right"))
            px = lane_eyes.get("nadir_r_gap_px" if right else "nadir_gap_px")
            ny = lane_eyes.get("nadir_r_lateral_m" if right else "nadir_lateral_m")
            disp.drawText("—" if px is None else f"{int(px)} px", 2, 16)
            disp.drawText("—" if ny is None else f"{float(ny)*100:.0f} cm", 2, 31)
            cam_w = 64
            if cam is not None:
                try:
                    cam_w = max(1, int(cam.getWidth()))
                except Exception:
                    cam_w = 64
            tape = lane_eyes.get("nadir_r_tape_col" if right else "nadir_tape_col")
            wheel = lane_eyes.get("nadir_r_wheel_col" if right else "nadir_wheel_col")
            if tape is not None:
                x = int(round(float(tape) * w / cam_w))
                disp.setColor(0xFFE000)
                disp.drawLine(x, 0, x, h - 1)
            if wheel is not None:
                x = int(round(float(wheel) * w / cam_w))
                disp.setColor(0xFFFFFF)
                disp.drawLine(x, 0, x, h - 1)
        except Exception:
            continue
