"""Webots Display HUD + camera overlay paint. No physics."""
from __future__ import annotations

from controller import Camera, Display, Robot

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
    """Paint battery/heat gauges and teleop mode strip on the robot HUD."""
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


def _init_devices(
    robot: Robot, timestep: int
) -> tuple:
    """Enable motors, encoders, GPS, head GPS, IMU, keyboard, and optional HUD."""
    motors: dict[str, Motor] = {}
    sensors: dict[str, PositionSensor] = {}
    for name in MOTOR_NAMES:
        motor = robot.getDevice(name)
        motors[name] = motor
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        if hasattr(motor, "enableTorqueFeedback"):
            try:
                motor.enableTorqueFeedback(timestep)
            except Exception:
                pass
    for name in SENSOR_NAMES:
        sensor = robot.getDevice(name)
        sensors[name] = sensor
        sensor.enable(timestep)

    gps: GPS = robot.getDevice("gps")
    gps.enable(timestep)
    gps_head = None
    try:
        gps_head = robot.getDevice("gps_head")
        if gps_head is not None:
            gps_head.enable(timestep)
    except Exception:
        gps_head = None
    imu: InertialUnit = robot.getDevice("imu")
    imu.enable(timestep)

    keyboard: Keyboard | None = None
    if hasattr(robot, "getKeyboard"):
        keyboard = robot.getKeyboard()
    else:
        try:
            keyboard = robot.getDevice("keyboard")
        except Exception:
            keyboard = None
    if keyboard is not None:
        keyboard.enable(timestep)
    else:
        print("WARNING: Keyboard device unavailable — WASD teleop disabled")

    hud: Display | None = None
    try:
        hud = robot.getDevice("hud")
    except Exception:
        hud = None

    cams: dict[str, Camera | None] = {}
    for name in (
        "line_left",
        "line_right",
        "finish_cam",
        "finish_cam_r",
        "forecast_z",
        "forecast_w",
    ):
        cam = None
        try:
            cam = robot.getDevice(name)
            if cam is not None:
                cam.enable(timestep)
        except Exception:
            cam = None
        cams[name] = cam
        if cam is None:
            print(f"WARNING: camera '{name}' not found — lane-keep eye missing")

    return motors, sensors, gps, gps_head, imu, keyboard, hud, cams


def _label_eye_huds(robot: Robot, cams: dict) -> list[dict]:
    """Attach overlays to the live camera buffers. Returns handles for paint."""
    specs = (
        ("hud_left", "line_left", 0xFFE000, "L", "left_offset", False),
        ("hud_red", "finish_cam", 0xFF40A0, "RED", None, False),
        ("hud_right", "line_right", 0xFFE000, "R", "right_offset", False),
        # Full frame — do not black out the top. Daniel needs the whole FOV.
        ("hud_z", "forecast_z", 0xFF8800, "Z", "z_offset", True),
        ("hud_w", "forecast_w", 0x22DD55, "W", "w_offset", True),
    )
    handles: list[dict] = []
    for dname, cname, color, text, offset_key, full_frame in specs:
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
                    "offset_key": offset_key,
                    "full_frame": bool(full_frame),
                }
            )
            print(f"HUD {dname} ← {cname} labeled {text}")
        except Exception as exc:
            print(f"HUD {dname} skip: {exc}")
    return handles


def _paint_eye_huds(
    handles: list[dict],
    lane_eyes: dict,
    band_fn,
    col_fn,
) -> None:
    """Dim ignored rows, outline the brain's band, tick the measured column.

    Same yellow_look_band() the steer law uses — change the band in
    lane_keep.py and this box moves with it.
    """
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
                    ref = disp.imageNew(raw, Display.BGRA, w, h)
                    disp.imagePaste(ref, 0, 0, False)
                    disp.imageDelete(ref)
            if handle.get("full_frame"):
                y0, y1 = 0, h
                disp.setColor(int(handle["color"]))
                disp.drawRectangle(0, 0, max(1, w - 1), max(1, h - 1))
                off = lane_eyes.get(handle["offset_key"])
                if off is not None and col_fn is not None:
                    col = int(col_fn(float(off), w))
                    disp.setColor(0xFFFF00)
                    disp.drawLine(col, 0, col, h - 1)
            elif handle["offset_key"] is not None and band_fn is not None:
                y0, y1 = band_fn(h)
                y0 = max(0, min(h, int(y0)))
                y1 = max(y0 + 1, min(h, int(y1)))
                try:
                    disp.setAlpha(0.5)
                except Exception:
                    pass
                disp.setColor(0x000000)
                if y0 > 0:
                    disp.fillRectangle(0, 0, w, y0)
                if y1 < h:
                    disp.fillRectangle(0, y1, w, h - y1)
                try:
                    disp.setAlpha(1.0)
                except Exception:
                    pass
                disp.setColor(0x00FFFF)
                disp.drawRectangle(0, y0, max(1, w - 1), max(1, y1 - y0 - 1))
                fy0 = fy1 = None
                try:
                    from src.lane_keep import yellow_far_band as _far_band

                    fy0, fy1 = _far_band(h)
                    fy0 = max(y0, min(y1, int(fy0)))
                    fy1 = max(fy0 + 1, min(y1, int(fy1)))
                    disp.setColor(0xFF8800)
                    disp.drawRectangle(0, fy0, max(1, w - 1), max(1, fy1 - fy0 - 1))
                except Exception:
                    fy0 = fy1 = None
                split = y0 + (y1 - y0) // 2
                disp.setColor(0x00FFFF)
                disp.drawLine(0, split, max(1, w - 1), split)
                off = lane_eyes.get(handle["offset_key"])
                if off is not None and col_fn is not None:
                    col = int(col_fn(float(off), w))
                    disp.setColor(0xFFFF00)
                    disp.drawLine(col, y0, col, y1 - 1)
                far_key = None
                if handle["offset_key"] == "left_offset":
                    far_key = "left_far_offset"
                elif handle["offset_key"] == "right_offset":
                    far_key = "right_far_offset"
                far_off = None if far_key is None else lane_eyes.get(far_key)
                if (
                    far_off is not None
                    and col_fn is not None
                    and fy0 is not None
                    and fy1 is not None
                    and lane_eyes.get("preview_ok")
                ):
                    far_col = int(col_fn(float(far_off), w))
                    disp.setColor(0xFF8800)
                    disp.drawLine(far_col, fy0, far_col, fy1 - 1)
                fused = lane_eyes.get("steer")
                if fused is not None and col_fn is not None:
                    fcol = int(col_fn(float(fused), w))
                    disp.setColor(0xFFFFFF)
                    disp.drawLine(fcol, y0, fcol, y1 - 1)
            disp.setColor(int(handle["color"]))
            label = str(handle["text"])
            if handle.get("offset_key") == "left_offset":
                label = "L M" if lane_eyes.get("metric_active") else "L P"
            elif handle.get("offset_key") == "right_offset":
                label = "R M" if lane_eyes.get("metric_active") else "R P"
            disp.drawText(label, 2, 1)
        except Exception:
            continue

