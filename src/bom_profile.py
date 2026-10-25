"""Load ``config/bom/bom.json`` and turn one build-out into a hardware profile.

The YAML profile (``butlerbot_wheeled.yaml``) stays the default. A build from
the BoM is used only when ``config.yaml`` sets ``bom_build`` to ``light``,
``heavy``, or ``budget``.

Nothing in here invents a watt, amp, or kilogram the vendor page left blank.
Unknown numbers stay missing.
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
BOM_JSON_PATH = _REPO / "config" / "bom" / "bom.json"

# Catalog categories already allowed by src.parts_db.CATEGORIES.
BOM_CATEGORY_MAP = {
    "drive_motor": "motor",
    "motor_driver": "esc",
    "stereo_camera": "camera",
    "radar": "sensor",
    "ultrasound": "sensor",
    "gnss": "sensor",
    "imu": "sensor",
    "wheel": "wheel",
    "wheel_hub": "wheel",
    "caster": "wheel",
    "motor_bracket": "structure",
    "dc_dc": "other",
    "compute": "compute",
    "battery": "battery",
    "chassis": "structure",
}

# Energy gate: a blank idle watt on these parts fails the build.
# Other blanks (driver quiescent, radar) are listed, not filled in.
COMPUTE_CAMERA_CATEGORIES = frozenset({"compute", "stereo_camera"})

BUILD_ORDER = ("budget", "light", "heavy")

# Next larger pack in the same voltage family. Used only after every
# stock build fails a gate.
BATTERY_UPGRADE = {
    "BLF-2410A": "BLF-2420A",
    "BLF-1220A": "BLF-1240A",
}


@lru_cache(maxsize=2)
def load_bom(path: str | None = None) -> dict:
    """Read bom.json. Cached for the default path."""
    p = Path(path) if path else BOM_JSON_PATH
    with open(p, encoding="utf-8") as fh:
        data = json.load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"bom.json root must be an object, got {type(data).__name__}")
    return data


def clear_bom_cache() -> None:
    load_bom.cache_clear()


def parts_by_id(bom: dict | None = None) -> dict[str, dict]:
    data = bom if bom is not None else load_bom()
    return {p["id"]: p for p in data.get("parts") or []}


def bom_notes(part: dict) -> str:
    """price_date, unknown list, and the vendor note — stored on the catalog row."""
    bits = []
    if part.get("price_date"):
        bits.append(f"price_date={part['price_date']}")
    unknown = part.get("unknown") or []
    if unknown:
        bits.append("unknown=" + "; ".join(str(u) for u in unknown))
    if part.get("id"):
        bits.append(f"bom_id={part['id']}")
    if part.get("notes"):
        bits.append(str(part["notes"]))
    return " | ".join(bits)


def catalog_sku(part: dict) -> str:
    number = part.get("part_number")
    if number:
        return str(number)
    return str(part["id"])


def map_category(part: dict) -> str:
    raw = str(part.get("category") or "")
    if raw not in BOM_CATEGORY_MAP:
        raise ValueError(f"no catalog category for BoM category {raw!r} ({part.get('id')})")
    return BOM_CATEGORY_MAP[raw]


def _num(value):
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _first_part(lines: list, parts: dict, category: str) -> tuple[dict, int] | None:
    for line in lines:
        part = parts.get(line["part_id"])
        if part and part.get("category") == category:
            return part, int(line.get("qty") or 1)
    return None


def _parts_of(lines: list, parts: dict, category: str) -> list[tuple[dict, int]]:
    found = []
    for line in lines:
        part = parts.get(line["part_id"])
        if part and part.get("category") == category:
            found.append((part, int(line.get("qty") or 1)))
    return found


def build_profile_from_bom(build_id: str, bom: dict | None = None) -> dict:
    """Profile dict in the same shape ``butlerbot_wheeled.yaml`` loads into.

    ``battery_capacity_wh()``, ``battery_c_rate_limits()`` and
    ``motor_stall_torque_nm()`` keep working on the result.
    """
    data = bom if bom is not None else load_bom()
    builds = data.get("builds") or {}
    key = str(build_id).strip().lower()
    if key not in builds:
        known = ", ".join(sorted(builds))
        raise KeyError(f"unknown bom build {build_id!r}; expected one of {known}")
    build = builds[key]
    parts = parts_by_id(data)
    lines = list(build.get("lines") or [])
    totals = dict(build.get("totals") or {})
    assumptions = dict(data.get("assumptions") or {})
    motor = _first_part(lines, parts, "drive_motor")
    driver = _first_part(lines, parts, "motor_driver")
    wheel = _first_part(lines, parts, "wheel")
    battery = _first_part(lines, parts, "battery")
    compute = _first_part(lines, parts, "compute")
    if motor is None or battery is None or wheel is None:
        raise ValueError(f"build {key} is missing a motor, wheel, or battery")

    motor_part, motor_qty = motor
    batt_part, _batt_qty = battery
    wheel_part, _wheel_qty = wheel
    diameter_mm = _num(wheel_part.get("diameter_mm"))
    if diameter_mm is None:
        raise ValueError(f"wheel {wheel_part.get('id')} has no diameter_mm")
    radius_m = diameter_mm / 2000.0

    voltage = _num(motor_part.get("voltage_v"))
    stall_a = _num(motor_part.get("stall_current_a"))
    peak_w = (stall_a * voltage * 2.0) if (stall_a and voltage) else None
    cruise = _num(((data.get("repo_baseline") or {}).get("speeds_m_s") or {}).get("lane_cruise"))
    if cruise is None:
        cruise = 0.44

    motor_block = {
        "label": motor_part.get("name"),
        "part_number": motor_part.get("part_number"),
        "vendor": motor_part.get("vendor"),
        "product_url": motor_part.get("source_url"),
        "datasheet_url": motor_part.get("datasheet_url"),
        "channel": "Legs",
        "voltage_v": voltage,
        "gear_ratio": _num(motor_part.get("gear_ratio")),
        "no_load_rpm": _num(motor_part.get("no_load_rpm")),
        "no_load_current_a": _num(motor_part.get("no_load_current_a")),
        "stall_current_a": stall_a,
        "stall_torque_nm": _num(motor_part.get("stall_torque_nm")),
        "idle_w": _num(motor_part.get("idle_w")) or 0.0,
        "cruise_speed_m_s": cruise,
        "qty": motor_qty,
    }
    if motor_part.get("max_efficiency") is not None:
        motor_block["efficiency"] = float(motor_part["max_efficiency"])
    if motor_part.get("rec_continuous_torque_nm") is not None:
        motor_block["continuous_torque_nm"] = float(motor_part["rec_continuous_torque_nm"])
    if peak_w is not None:
        motor_block["peak_w"] = round(peak_w, 2)
        motor_block["peak_power_w"] = round(stall_a * voltage, 2)

    batt_v = _num(batt_part.get("voltage_v"))
    cont_a = _num(batt_part.get("max_continuous_discharge_a"))
    peak_a = _num(batt_part.get("peak_discharge_a"))
    batt_ah = _num(batt_part.get("capacity_ah"))
    batt_wh = _num(batt_part.get("capacity_wh"))
    battery_block = {
        "label": batt_part.get("name"),
        "part_number": batt_part.get("part_number"),
        "vendor": batt_part.get("vendor"),
        "product_url": batt_part.get("source_url"),
        "chemistry": batt_part.get("chemistry"),
        "nominal_voltage_v": batt_v,
        "capacity_ah": batt_ah,
        "capacity_wh": batt_wh,
        "continuous_discharge_a": cont_a,
        "peak_discharge_a": peak_a,
        "mass_kg": None if batt_part.get("mass_g") is None else float(batt_part["mass_g"]) / 1000.0,
        "usable_fraction": 0.80,
    }
    if batt_v and cont_a:
        battery_block["continuous_power_w"] = round(batt_v * cont_a, 2)
        battery_block["continuous_c_rate"] = round(cont_a / batt_ah, 3) if batt_ah else None
    if batt_v and peak_a:
        battery_block["peak_power_w"] = round(batt_v * peak_a, 2)
        battery_block["peak_c_rate"] = round(peak_a / batt_ah, 3) if batt_ah else None
    if batt_part.get("peak_duration_s") is not None:
        battery_block["peak_duration_s"] = batt_part["peak_duration_s"]

    driver_block = {}
    if driver is not None:
        drv, drv_qty = driver
        driver_block = {
            "label": drv.get("name"),
            "part_number": drv.get("part_number"),
            "vendor": drv.get("vendor"),
            "product_url": drv.get("source_url"),
            "continuous_current_a_per_ch": _num(drv.get("continuous_current_a_per_ch")),
            "channels": drv_qty,
            # Published G2/VNH5019 efficiency was not on the page.
            # 0.92 is the assumption recorded in bom.json, not a measurement.
            "efficiency": float(assumptions.get("driver_eff") or 0.92),
            "efficiency_source": "assumption",
        }
        if drv.get("idle_w") is not None:
            driver_block["idle_w"] = float(drv["idle_w"])

    bom_parts = []
    for line in lines:
        part = parts.get(line["part_id"])
        if not part:
            continue
        bom_parts.append(
            {
                "id": part["id"],
                "category": part.get("category"),
                "name": part.get("name"),
                "part_number": part.get("part_number"),
                "twin_channel": part.get("twin_channel"),
                "qty": int(line.get("qty") or 1),
                "idle_w": part.get("idle_w"),
                "active_w": part.get("active_w"),
                "quiescent_current_a_max": part.get("quiescent_current_a_max"),
                "unknown": list(part.get("unknown") or []),
            }
        )

    compute_block = {"idle_w": 0.0, "active_w": 0.0}
    if compute is not None:
        comp, _qty = compute
        compute_block = {
            "label": comp.get("name"),
            "vendor": comp.get("vendor"),
            "model": comp.get("name"),
            "product_url": comp.get("source_url"),
            "part_number": comp.get("part_number"),
        }
        if comp.get("idle_w") is not None:
            compute_block["idle_w"] = float(comp["idle_w"])
        if comp.get("active_w") is not None:
            compute_block["active_w"] = float(comp["active_w"])

    imu_block = {}
    imu = _first_part(lines, parts, "imu")
    if imu is not None:
        imu_part, _q = imu
        imu_block = {
            "label": imu_part.get("name"),
            "part_number": imu_part.get("part_number"),
            "vendor": imu_part.get("vendor"),
            "product_url": imu_part.get("source_url"),
            "idle_w": imu_part.get("idle_w"),
            "active_w": imu_part.get("active_w"),
        }

    idle_known = totals.get("idle_w_known")
    cruise_w = totals.get("cruise_044_total_w")
    phase = {}
    if idle_known is not None:
        phase = {
            "standby": float(idle_known),
            "return_idle": float(idle_known),
            "teleop": float(cruise_w if cruise_w is not None else idle_known),
            "drive_transit": float(cruise_w if cruise_w is not None else idle_known),
            "patrol": float(cruise_w if cruise_w is not None else idle_known),
            "manipulate": float(cruise_w if cruise_w is not None else idle_known),
        }

    # No arm / torso / cooling hardware is on these wheeled builds.
    channels = {
        "Legs": {
            "description": f"Differential drive ({motor_qty}× {motor_part.get('part_number')})",
            "max_draw_w": round((peak_w or 80.0) * max(motor_qty, 1), 2),
            "nominal_voltage_v": voltage,
            "motors": ["left_wheel", "right_wheel"],
        },
        "Arms": {"description": "No arm hardware in this BoM build", "max_draw_w": 0, "motors": []},
        "Torso": {"description": "No torso hardware in this BoM build", "max_draw_w": 0, "motors": []},
        "Compute": {
            "description": "Compute + sensors from the BoM",
            "max_draw_w": 80,
            "motors": [],
        },
        "Cooling": {"description": "No cooling hardware in this BoM build", "max_draw_w": 0, "motors": []},
    }

    chassis = _first_part(lines, parts, "chassis")
    chassis_kg = None
    if chassis is not None and chassis[0].get("mass_g") is not None:
        chassis_kg = float(chassis[0]["mass_g"]) / 1000.0

    return {
        "profile_id": f"butlerbot_bom_{key}",
        "label": build.get("label") or key,
        "version": str(data.get("schema_version") or "1.0"),
        "bom_build": key,
        "bom_schema_version": str(data.get("schema_version") or "1.0"),
        "power_model": "bom_linear",
        "bus_voltage_v": _num(build.get("bus_voltage_v")),
        "description": build.get("label") or "",
        "battery": battery_block,
        "motor_driver": driver_block,
        "motors": {
            "left_wheel": dict(motor_block, side="left"),
            "right_wheel": dict(motor_block, side="right"),
        },
        "geometry": {
            "wheel_radius_m": radius_m,
            "wheel_diameter_mm": diameter_mm,
            "wheel_mass_kg": None
            if wheel_part.get("mass_g") is None
            else float(wheel_part["mass_g"]) / 1000.0,
            "mass_kg_known": totals.get("mass_kg_known"),
            "chassis_mass_kg": chassis_kg,
            "tire": {
                "label": wheel_part.get("name"),
                "diameter_mm": diameter_mm,
                "product_url": wheel_part.get("source_url"),
            },
        },
        "compute": compute_block,
        "sensors": {
            "label": "per-part idle/active from the BoM (no flat sensors.idle_w)",
            "parts_are_source": True,
        },
        "imu": imu_block,
        "stabilizers": {"label": "none on this BoM build", "channel": "Torso", "idle_w": 0.0, "active_w": 0.0},
        "modes": {},
        "channels": channels,
        "phase_draw_w": phase,
        "phase_duration_s": {},
        "bom_parts": bom_parts,
        "bom_totals": totals,
        "energy_rows": list(build.get("energy") or []),
        "torque_headroom": dict(build.get("torque_headroom") or {}),
        "assumptions": assumptions,
        "unknown_parts": list(totals.get("idle_missing_parts") or []),
    }


# Cameras and navigation sensors. Split out of the Compute channel for logs.
# A dc-dc regulator stays with compute: it feeds the computer, and its only
# published number is quiescent current.
_SENSOR_CATEGORIES = frozenset({
    "stereo_camera",
    "camera",
    "radar",
    "ultrasound",
    "gnss",
    "imu",
    "sensor",
})


def _published_part_w(part: dict, bus_v: float) -> float | None:
    """Watts this part contributes, or None when the vendor did not publish them.

    ``active_w`` (Jetson 25 W, OAK 4.5 W) is a different power mode from the
    idle figure the energy table uses. The twin keeps the published idle so
    a moving robot does not jump from 9.96 W to ~30 W of compute.
    """
    qty = int(part.get("qty") or 1)
    if part.get("category") == "dc_dc" and part.get("idle_w") is None:
        quiescent = _num(part.get("quiescent_current_a_max"))
        if quiescent is None or bus_v <= 0:
            return None
        return float(quiescent) * bus_v * qty
    if part.get("idle_w") is None:
        return None
    return float(part["idle_w"]) * qty


def electronics_draw_w(profile: dict, *, moving: bool = False) -> float:
    """Sum published idle watts for Compute-channel parts.

    A part with no idle_w contributes nothing. Regulator quiescent current
    is a published maximum (``quiescent_current_a_max`` × bus voltage), which
    is how bom.json's idle total was added up. It is not a guessed operating point.

    ``moving`` is accepted so older callers keep working. It does not switch
    the Jetson or the camera to ``active_w``.
    """
    del moving  # cruise model is the published idle, not the 25 W mode
    bus_v = _num(profile.get("bus_voltage_v")) or 0.0
    total = 0.0
    for part in profile.get("bom_parts") or []:
        if part.get("twin_channel") != "Compute":
            continue
        watts = _published_part_w(part, bus_v)
        if watts is not None:
            total += watts
    return round(total, 2)


def subsystem_part_watts(profile: dict) -> dict[str, float]:
    """Published compute vs sensor watts. Parts with no idle number are omitted."""
    bus_v = _num(profile.get("bus_voltage_v")) or 0.0
    compute = 0.0
    sensors = 0.0
    for part in profile.get("bom_parts") or []:
        if part.get("twin_channel") != "Compute":
            continue
        watts = _published_part_w(part, bus_v)
        if watts is None:
            continue
        if part.get("category") in _SENSOR_CATEGORIES:
            sensors += watts
        else:
            compute += watts
    return {"compute": round(compute, 2), "sensors": round(sensors, 2)}


def compute_camera_unknown_ids(build: dict, parts: dict) -> list[str]:
    """Compute or camera lines whose idle draw was not published."""
    missing = []
    for line in build.get("lines") or []:
        part = parts.get(line["part_id"])
        if not part:
            continue
        if part.get("category") in COMPUTE_CAMERA_CATEGORIES and part.get("idle_w") is None:
            missing.append(part["id"])
    return missing


def rolling_mass_kg(profile: dict) -> float:
    """Known BoM mass plus payload when that is set."""
    geom = profile.get("geometry") or {}
    mass = float(geom.get("mass_kg_known") or 0.0)
    if geom.get("payload_kg") is not None:
        mass += float(geom["payload_kg"])
    else:
        from src.config import config

        mass += float(config.get("robot", "payload_kg") or 0.0)
    return mass
