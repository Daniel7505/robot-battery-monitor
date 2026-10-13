"""Thin design agent — assemble and score BOMs from the parts catalog.

North-star item 3, first cut. Reads ``src.parts_db`` only. No Webots, no
Postgres, no lane-keep. Scores are analytic heuristics so we can later
swap in a twin evaluation without changing the call shape.

    from src.design_agent import Mission, propose

    result = propose(Mission(track_length_m=17.4, max_cost_usd=800))
"""

from __future__ import annotations

import argparse
import sqlite3
from dataclasses import asdict, dataclass

from src.logger import logger
from src.parts_db import connect, get_part, list_motors, list_parts
from src.twin_energy import estimate_mission_energy, paper_energy

# Twin cruise is 0.44 m/s. Used when the profile does not publish a speed.
DEFAULT_CRUISE_M_S = 0.44
# Baseload (compute + sensors + idle drive electronics) when we have a Jetson-class SBC.
BASELOAD_W = 18.0
# Usable pack fraction — do not plan to the last watt-hour.
PACK_USABLE = 0.80
# 80 mm wheel diameter → 0.04 m radius. 0.08 was the diameter in metres.
DEFAULT_WHEEL_RADIUS_M = 0.04
# Cheapest build first, so a tie goes to the cheaper robot.
BOM_BUILD_ORDER = ("budget", "light", "heavy")
ENERGY_RESERVE = 0.20


@dataclass
class Mission:
    """Operator envelope. All limits are optional except length."""

    name: str = "mirrored S"
    track_length_m: float = 17.4  # mirrored double-S lane path (finish x=16.5 m)
    max_mass_g: float | None = None
    max_cost_usd: float | None = None
    min_speed_m_s: float | None = None
    max_minutes: float | None = None
    prefer_low_compute: bool = False
    drive_motors: int = 2
    wheels: int = 2
    cameras: int = 2
    # Used by the BoM build-out chooser. Defaults keep old propose() calls valid.
    payload_kg_max: float = 0.0
    payload_kg_avg: float | None = None
    shift_h: float | None = None
    distance_m: float | None = None
    trolley_tare_kg: float = 0.0
    speed_m_s: float | None = None


def propose(
    mission: Mission | None = None,
    *,
    conn: sqlite3.Connection | None = None,
    db_path=None,
    max_candidates: int = 4,
) -> dict:
    """Return a ranked list of candidate BOMs for ``mission``."""
    mission = mission or Mission()
    own = conn is None
    if own:
        conn = connect(db_path)
    try:
        combos = _assemble(conn, mission)
        scored = [_score(mission, bom) for bom in combos]
        scored.sort(key=lambda c: c["score"], reverse=True)
        picked = scored[: max(1, int(max_candidates))]
        selection = select_build(_request_from_mission(mission))
        return {
            "mission": asdict(mission),
            "candidates": picked,
            "considered": len(scored),
            "bom_candidates": bom_candidate_rows(),
            "build_select": selection,
        }
    finally:
        if own:
            conn.close()


def _first(conn: sqlite3.Connection, category: str) -> dict | None:
    rows = list_parts(conn, category=category)
    if not rows:
        return None
    return get_part(conn, rows[0]["id"])


def _assemble(conn: sqlite3.Connection, mission: Mission) -> list[dict]:
    motors = list_motors(conn)
    batteries = [get_part(conn, p["id"]) for p in list_parts(conn, category="battery")]
    cameras = [get_part(conn, p["id"]) for p in list_parts(conn, category="camera")]
    wheel = _first(conn, "wheel")
    compute = _first(conn, "compute")
    esc = _first(conn, "esc")
    imu = _first(conn, "sensor")
    batteries = [b for b in batteries if b]
    cameras = [c for c in cameras if c]
    if not motors or not batteries or not cameras:
        return []

    out = []
    for motor in motors:
        for batt in batteries:
            for cam in cameras:
                out.append(
                    {
                        "motor": motor,
                        "battery": batt,
                        "camera": cam,
                        "wheel": wheel,
                        "compute": compute,
                        "esc": esc,
                        "imu": imu,
                        "qty": {
                            "motor": mission.drive_motors,
                            "wheel": mission.wheels,
                            "camera": mission.cameras,
                        },
                    }
                )
    return out


def _cruise_m_s() -> float:
    """Profile cruise speed, not a hard-coded 0.44, when the profile has one."""
    try:
        from src.hardware_profile import profile_cruise_speed_m_s

        return float(profile_cruise_speed_m_s())
    except Exception:
        return DEFAULT_CRUISE_M_S


def _wheel_radius_m(wheel: dict | None) -> float:
    if not wheel:
        return DEFAULT_WHEEL_RADIUS_M
    name = f"{wheel.get('name') or ''} {wheel.get('notes') or ''}".lower()
    if "200" in name and "mm" in name:
        return 0.10
    if "90 mm" in name or "90mm" in name:
        return 0.045
    if "80 mm" in name or "80mm" in name:
        return 0.04
    return DEFAULT_WHEEL_RADIUS_M


def _line_item(part: dict | None, role: str, qty: int) -> dict | None:
    if not part:
        return None
    cost = part.get("cost_usd")
    mass = part.get("mass_g")
    return {
        "role": role,
        "qty": qty,
        "sku": part.get("sku"),
        "name": part.get("name"),
        "category": part.get("category"),
        "unit_cost_usd": cost,
        "unit_mass_g": mass,
        "line_cost_usd": None if cost is None else round(float(cost) * qty, 2),
        "line_mass_g": None if mass is None else round(float(mass) * qty, 1),
    }


def _sum_lines(items: list[dict]) -> tuple[float, float]:
    cost = sum(i["line_cost_usd"] or 0.0 for i in items)
    mass = sum(i["line_mass_g"] or 0.0 for i in items)
    return round(cost, 2), round(mass, 1)


def _required_speed(mission: Mission) -> float:
    if mission.max_minutes and mission.max_minutes > 0:
        return float(mission.track_length_m) / (float(mission.max_minutes) * 60.0)
    if mission.min_speed_m_s:
        return float(mission.min_speed_m_s)
    return _cruise_m_s()


def _motor_vmax(motor: dict, wheel_r: float) -> float | None:
    rpm = motor.get("no_load_rpm")
    if rpm is None:
        return None
    return float(rpm) * 2.0 * 3.141592653589793 * float(wheel_r) / 60.0


def _score(mission: Mission, raw: dict) -> dict:
    qty = raw["qty"]
    items = [
        _line_item(raw["motor"], "drive_motor", qty["motor"]),
        _line_item(raw["battery"], "battery", 1),
        _line_item(raw["camera"], "camera", qty["camera"]),
        _line_item(raw["wheel"], "wheel", qty["wheel"]),
        _line_item(raw["compute"], "compute", 1),
        _line_item(raw["esc"], "esc", 1),
        _line_item(raw["imu"], "imu", 1),
    ]
    items = [i for i in items if i]
    cost, mass = _sum_lines(items)

    batt = raw["battery"].get("battery") or {}
    pack_wh = batt.get("capacity_wh")
    motor = raw["motor"]
    cam = raw["camera"].get("camera") or {}
    wheel_r = _wheel_radius_m(raw["wheel"])
    v_req = _required_speed(mission)
    v_max = _motor_vmax(motor, wheel_r)
    cruise = _cruise_m_s()
    v_use = min(v_req, cruise) if v_max is None else min(v_req, cruise, max(0.05, 0.4 * v_max))
    if v_use <= 0:
        v_use = cruise
    time_s = float(mission.track_length_m) / v_use
    drive_w = float(motor.get("continuous_power_w") or 8.0) * qty["motor"]
    draw_w = drive_w + BASELOAD_W
    paper = paper_energy(mission.track_length_m, draw_w=draw_w, time_s=time_s)
    try:
        twin = estimate_mission_energy(
            mission.track_length_m,
            track="s",
            motor_cont_w=float(motor.get("continuous_power_w") or 8.0),
            motor_qty=qty["motor"],
        )
    except Exception:
        twin = None
    if twin is not None:
        energy_wh = float(twin["energy_wh"])
        energy_source = str(twin["source"])
        draw_w_used = float(twin["draw_w"])
        time_s_used = float(twin["time_s"])
    else:
        energy_wh = float(paper["energy_wh"])
        energy_source = "paper"
        draw_w_used = draw_w
        time_s_used = time_s
    usable = None if pack_wh is None else float(pack_wh) * PACK_USABLE
    energy_ok = usable is not None and usable >= energy_wh
    speed_ok = v_max is None or v_max >= v_req * 0.95

    notes = []
    mv = motor.get("voltage_v")
    bv = batt.get("voltage_v")
    if mv and bv and abs(float(bv) - float(mv)) > 2.0:
        notes.append(
            f"Voltage mismatch: {bv} V pack vs {mv} V motors — needs a DC-DC "
            "(current twin already assumes 48→12)."
        )
    elif mv and bv:
        notes.append(f"Voltage compatible: {bv} V pack ≈ {mv} V motor bus.")
    if not energy_ok:
        notes.append(
            f"Energy tight ({energy_source}): est {energy_wh:.2f} Wh vs usable "
            f"{usable if usable is None else f'{usable:.1f}'} Wh."
        )
    else:
        notes.append(
            f"Energy OK ({energy_source}): est {energy_wh:.2f} Wh, usable {usable:.1f} Wh"
            f" (paper was {paper['energy_wh']:.2f} Wh)."
        )
    if not speed_ok:
        notes.append(
            f"Speed short: motor free-run ~{v_max:.2f} m/s, need {v_req:.2f} m/s."
        )
    over_cost = mission.max_cost_usd is not None and cost > mission.max_cost_usd
    over_mass = mission.max_mass_g is not None and mass > mission.max_mass_g
    if over_cost:
        notes.append(f"Over cost cap ${mission.max_cost_usd:.0f} (BOM ${cost:.0f}).")
    if over_mass:
        notes.append(f"Over mass cap {mission.max_mass_g:.0f} g (BOM {mass:.0f} g).")

    compute_tax = float(cam.get("compute_cost") or 0.0)
    score = 0.0
    if energy_ok and speed_ok and not over_cost and not over_mass:
        score += 100.0
    if energy_ok:
        score += 20.0
    if speed_ok:
        score += 10.0
    if over_cost:
        score -= 40.0
    if over_mass:
        score -= 40.0
    if not energy_ok:
        score -= 50.0
    if not speed_ok:
        score -= 30.0
    score -= cost / 20.0
    score -= mass / 200.0
    if mission.prefer_low_compute:
        score -= compute_tax * 25.0
    else:
        score -= compute_tax * 5.0

    motor_sku = motor.get("sku")
    batt_sku = raw["battery"].get("sku")
    cam_sku = raw["camera"].get("sku")
    label = f"{motor_sku}+{batt_sku}+{cam_sku}"
    rationale = _rationale(
        energy_ok=energy_ok,
        speed_ok=speed_ok,
        over_cost=over_cost,
        over_mass=over_mass,
        cost=cost,
        mass=mass,
        prefer_low_compute=mission.prefer_low_compute,
        compute_tax=compute_tax,
        motor_sku=motor_sku,
        batt_sku=batt_sku,
    )
    return {
        "name": label,
        "score": round(score, 2),
        "feasible": bool(energy_ok and speed_ok and not over_cost and not over_mass),
        "bom": items,
        "totals": {
            "cost_usd": cost,
            "mass_g": mass,
            "energy_wh_est": round(energy_wh, 3),
            "energy_wh_paper": paper["energy_wh"],
            "energy_source": energy_source,
            "pack_wh": pack_wh,
            "usable_wh": None if usable is None else round(usable, 2),
            "energy_ok": energy_ok,
            "draw_w_est": round(draw_w_used, 1),
            "v_req_m_s": round(v_req, 3),
            "v_max_m_s": None if v_max is None else round(v_max, 3),
            "time_s_est": round(time_s_used, 1),
            "camera_compute_cost": compute_tax,
        },
        "notes": notes,
        "rationale": rationale,
    }


def _rationale(
    *,
    energy_ok: bool,
    speed_ok: bool,
    over_cost: bool,
    over_mass: bool,
    cost: float,
    mass: float,
    prefer_low_compute: bool,
    compute_tax: float,
    motor_sku: str | None,
    batt_sku: str | None,
) -> str:
    if over_cost or over_mass:
        return f"Breaks the envelope (${cost:.0f}, {mass:.0f} g) — keep only as a contrast."
    if not energy_ok:
        return f"{batt_sku} cannot cover the estimated mission energy with {motor_sku}."
    if not speed_ok:
        return f"{motor_sku} free-run is too slow for the required pace."
    extra = ""
    if prefer_low_compute:
        extra = f" Camera compute tax {compute_tax:.2f}."
    return (
        f"Fits the envelope at ${cost:.0f} / {mass:.0f} g using {motor_sku} "
        f"and {batt_sku}.{extra}"
    )


@dataclass
class BuildRequest:
    """Inputs for the deterministic build-out rule. Same inputs, same pick."""

    payload_kg_max: float = 0.0
    payload_kg_avg: float | None = None
    shift_h: float = 6.0
    distance_m: float = 3000.0
    speed_m_s: float = 0.44
    budget_usd: float | None = None
    trolley_tare_kg: float = 0.0
    reserve: float = ENERGY_RESERVE


def _request_from_mission(mission: Mission) -> BuildRequest:
    avg = mission.payload_kg_avg
    if avg is None:
        avg = mission.payload_kg_max
    return BuildRequest(
        payload_kg_max=float(mission.payload_kg_max or 0.0),
        payload_kg_avg=float(avg or 0.0),
        shift_h=6.0 if mission.shift_h is None else float(mission.shift_h),
        distance_m=float(
            mission.track_length_m if mission.distance_m is None else mission.distance_m
        ),
        speed_m_s=float(mission.speed_m_s or mission.min_speed_m_s or _cruise_m_s()),
        budget_usd=mission.max_cost_usd,
        trolley_tare_kg=float(mission.trolley_tare_kg or 0.0),
        reserve=ENERGY_RESERVE,
    )


def bom_candidate_rows(bom: dict | None = None) -> list[dict]:
    """The three build-outs, cheapest first, with the published totals."""
    try:
        from src.bom_profile import load_bom

        data = bom if bom is not None else load_bom()
    except Exception:
        return []
    rows = []
    for build_id in BOM_BUILD_ORDER:
        build = (data.get("builds") or {}).get(build_id)
        if not build:
            continue
        totals = build.get("totals") or {}
        rows.append(
            {
                "name": f"bom:{build_id}",
                "build": build_id,
                "label": build.get("label"),
                "price_usd_known": totals.get("price_usd_known"),
                "mass_kg_known": totals.get("mass_kg_known"),
                "pack_wh": totals.get("battery_wh"),
                "idle_w_known": totals.get("idle_w_known"),
                "price_missing_parts": list(totals.get("price_missing_parts") or []),
            }
        )
    return rows


def _part_by_number(parts: dict, part_number: str) -> dict | None:
    for part in parts.values():
        if str(part.get("part_number") or "") == part_number:
            return part
    return None


def _battery_on_build(build: dict, parts: dict) -> dict | None:
    for line in build.get("lines") or []:
        part = parts.get(line["part_id"])
        if part and part.get("category") == "battery":
            return part
    return None


def _with_battery(build: dict, parts: dict, new_part: dict) -> dict:
    """Copy a build with a different pack. Mass, price, and Wh follow the new part.

    Drive energy is recomputed by the caller from the new mass. Idle watts
    do not change: a battery has no idle draw in the BoM.
    """
    old = _battery_on_build(build, parts)
    cloned = json_copy(build)
    for line in cloned.get("lines") or []:
        part = parts.get(line["part_id"])
        if part and part.get("category") == "battery":
            line["part_id"] = new_part["id"]
    totals = dict(cloned.get("totals") or {})
    if old is not None and old.get("mass_g") is not None and new_part.get("mass_g") is not None:
        delta_kg = (float(new_part["mass_g"]) - float(old["mass_g"])) / 1000.0
        if totals.get("mass_kg_known") is not None:
            totals["mass_kg_known"] = round(float(totals["mass_kg_known"]) + delta_kg, 3)
    if (
        old is not None
        and old.get("price_usd") is not None
        and new_part.get("price_usd") is not None
        and totals.get("price_usd_known") is not None
    ):
        totals["price_usd_known"] = round(
            float(totals["price_usd_known"])
            - float(old["price_usd"])
            + float(new_part["price_usd"]),
            2,
        )
    if new_part.get("capacity_wh") is not None:
        totals["battery_wh"] = float(new_part["capacity_wh"])
    if new_part.get("max_continuous_discharge_a") is not None:
        totals["battery_max_continuous_a"] = float(new_part["max_continuous_discharge_a"])
    cloned["totals"] = totals
    cloned["battery_override"] = new_part.get("part_number")
    return cloned


def json_copy(obj):
    import json

    return json.loads(json.dumps(obj))


def _drive_terms_for(build: dict, parts: dict, speed_m_s: float, mass_kg: float) -> dict:
    from src.bom_profile import load_bom
    from src.twin_energy import brushed_drive_point

    motor = None
    wheel = None
    for line in build.get("lines") or []:
        part = parts.get(line["part_id"])
        if not part:
            continue
        if part.get("category") == "drive_motor" and motor is None:
            motor = part
        if part.get("category") == "wheel" and wheel is None:
            wheel = part
    assumptions = {}
    try:
        assumptions = load_bom().get("assumptions") or {}
    except Exception:
        assumptions = {}
    diameter = float((wheel or {}).get("diameter_mm") or 80.0)
    return brushed_drive_point(
        mass_kg=float(mass_kg),
        wheel_radius_m=diameter / 2000.0,
        speed_m_s=float(speed_m_s),
        voltage_v=float(motor.get("voltage_v") or 0.0) if motor else 0.0,
        no_load_rpm=float(motor.get("no_load_rpm") or 0.0) if motor else 0.0,
        no_load_current_a=float(motor.get("no_load_current_a") or 0.0) if motor else 0.0,
        stall_current_a=float(motor.get("stall_current_a") or 0.0) if motor else 0.0,
        stall_torque_nm=float(motor.get("stall_torque_nm") or 0.0) if motor else 0.0,
        crr=float(assumptions.get("crr") or 0.015),
        eta_driver=float(assumptions.get("driver_eff") or 0.92),
    )


def evaluate_build(build_id: str, build: dict, parts: dict, request: BuildRequest) -> dict:
    """Torque, speed, current, energy, budget. Unknown compute/camera watts fail energy."""
    from src.bom_profile import compute_camera_unknown_ids
    from src.twin_energy import shift_energy_wh

    totals = build.get("totals") or {}
    failed: list[str] = []
    flags: list[str] = []
    mass = totals.get("mass_kg_known")
    headroom = (build.get("torque_headroom") or {}).get("max_total_rolling_mass_kg_steady")
    if mass is None or headroom is None:
        failed.append("torque")
        m_total = None
    else:
        m_total = float(mass) + float(request.trolley_tare_kg) + float(request.payload_kg_max)
        if m_total > 0.8 * float(headroom):
            failed.append("torque")

    rows = list(build.get("energy") or [])
    free = None
    if rows and rows[0].get("motor_free_speed_m_s") is not None:
        free = float(rows[0]["motor_free_speed_m_s"])
    if free is None or float(request.speed_m_s) > 0.9 * free:
        failed.append("speed")

    motor = None
    driver = None
    battery = _battery_on_build(build, parts)
    for line in build.get("lines") or []:
        part = parts.get(line["part_id"])
        if not part:
            continue
        if part.get("category") == "drive_motor" and motor is None:
            motor = part
        if part.get("category") == "motor_driver" and driver is None:
            driver = part
    stall = None if motor is None else motor.get("stall_current_a")
    peak_a = None if battery is None else battery.get("peak_discharge_a")
    cont_ch = None if driver is None else driver.get("continuous_current_a_per_ch")
    if stall is None or peak_a is None or cont_ch is None:
        failed.append("current")
    else:
        # Two motors at stall, seen from the pack at duty 1.
        if 2.0 * float(stall) > float(peak_a) or float(stall) > float(cont_ch):
            failed.append("current")

    unknown = compute_camera_unknown_ids(build, parts)
    energy_wh = None
    reserve_wh = None
    pack_wh = None if battery is None else battery.get("capacity_wh")
    if pack_wh is None:
        pack_wh = totals.get("battery_wh")
    if pack_wh is not None:
        reserve_wh = round(float(pack_wh) * (1.0 - float(request.reserve)), 2)
    if unknown:
        # Do not treat the missing watts as zero and let the build pass.
        failed.append("energy")
        flags.append("unknown_draw=" + ",".join(unknown))
    elif totals.get("idle_w_known") is None or mass is None:
        failed.append("energy")
        flags.append("unknown_draw=idle_w")
    else:
        terms = _drive_terms_for(build, parts, request.speed_m_s, float(mass))
        payload = 0.0 if request.payload_kg_avg is None else float(request.payload_kg_avg)
        energy_wh = shift_energy_wh(
            idle_w=float(totals["idle_w_known"]),
            shift_h=float(request.shift_h),
            distance_m=float(request.distance_m),
            drive_wh_per_10ft=terms["drive_wh_per_10ft"],
            extra_wh_per_10ft_per_kg=terms["extra_wh_per_10ft_per_kg_payload"],
            payload_kg=payload,
        )
        if reserve_wh is None or energy_wh > reserve_wh:
            failed.append("energy")

    price = totals.get("price_usd_known")
    missing_price = list(totals.get("price_missing_parts") or [])
    if missing_price:
        flags.append("price_missing=" + ",".join(missing_price))
    if request.budget_usd is not None:
        if price is None or float(price) > float(request.budget_usd):
            failed.append("budget")

    return {
        "build": build_id,
        "pass": not failed,
        "failed": failed,
        "flags": flags,
        "E_wh": energy_wh,
        "reserve_wh": reserve_wh,
        "battery": None if battery is None else battery.get("part_number"),
        "upgraded": bool(build.get("battery_override")),
        "mass_kg": mass,
        "m_total_kg": m_total,
    }


def select_build(request: BuildRequest | None = None, *, bom: dict | None = None) -> dict:
    """Pick the cheapest build that passes every gate. No randomness.

    Order is budget, then light, then heavy. If none pass, each build is
    tried again with the next battery in its voltage family
    (24 V: BLF-2410A → BLF-2420A, 12 V: BLF-1220A → BLF-1240A).
    """
    from src.bom_profile import BATTERY_UPGRADE, load_bom, parts_by_id

    request = request or BuildRequest()
    data = bom if bom is not None else load_bom()
    parts = parts_by_id(data)
    builds = data.get("builds") or {}

    def _check(build_id: str, build: dict) -> dict:
        # Parts dict must see an upgraded battery id. Stock parts already do.
        return evaluate_build(build_id, build, parts, request)

    checked: dict[str, dict] = {}
    for build_id in BOM_BUILD_ORDER:
        build = builds.get(build_id)
        if build is None:
            checked[build_id] = {
                "build": build_id,
                "pass": False,
                "failed": ["missing"],
                "flags": [],
                "E_wh": None,
                "reserve_wh": None,
                "battery": None,
                "upgraded": False,
            }
            continue
        checked[build_id] = _check(build_id, build)

    winner = next((checked[b] for b in BOM_BUILD_ORDER if checked[b]["pass"]), None)
    if winner is None:
        for build_id in BOM_BUILD_ORDER:
            build = builds.get(build_id)
            if not build:
                continue
            battery = _battery_on_build(build, parts)
            number = None if battery is None else battery.get("part_number")
            nxt = BATTERY_UPGRADE.get(str(number or ""))
            if not nxt:
                continue
            new_part = _part_by_number(parts, nxt)
            if new_part is None:
                continue
            # The upgraded part must be findable by id inside evaluate_build.
            parts[new_part["id"]] = new_part
            upgraded = _with_battery(build, parts, new_part)
            gate = _check(build_id, upgraded)
            checked[build_id] = gate
            if gate["pass"]:
                winner = gate
                break

    def _token(gate: dict) -> str:
        return f"{gate['build']}:{'pass' if gate['pass'] else 'fail'}"

    tokens = ",".join(_token(checked[b]) for b in BOM_BUILD_ORDER if b in checked)
    if winner is None:
        chosen = "NO_FEASIBLE_BUILD"
        e_wh = ""
        reserve = ""
        status = "NO_FEASIBLE_BUILD"
    else:
        chosen = winner["build"]
        e_wh = "" if winner["E_wh"] is None else f"{winner['E_wh']:.2f}"
        reserve = "" if winner["reserve_wh"] is None else f"{winner['reserve_wh']:.2f}"
        status = chosen
    line = f"BUILD_SELECT chosen={chosen} gates={tokens} E_wh={e_wh} reserve_wh={reserve}"
    logger.info(line)
    return {
        "status": status,
        "chosen": None if winner is None else winner["build"],
        "battery": None if winner is None else winner.get("battery"),
        "upgraded": bool(winner and winner.get("upgraded")),
        "E_wh": None if winner is None else winner.get("E_wh"),
        "reserve_wh": None if winner is None else winner.get("reserve_wh"),
        "gates": {b: checked[b] for b in BOM_BUILD_ORDER if b in checked},
        "log": line,
    }


def format_report(result: dict) -> str:
    m = result["mission"]
    lines = [
        f"Mission: {m['name']}  {m['track_length_m']} m",
        f"Caps: cost={m['max_cost_usd']}  mass_g={m['max_mass_g']}  "
        f"max_minutes={m['max_minutes']}  prefer_low_compute={m['prefer_low_compute']}",
        f"Considered {result['considered']} assemblies, showing {len(result['candidates'])}.",
        "",
    ]
    for i, c in enumerate(result["candidates"], 1):
        t = c["totals"]
        flag = "OK" if c["feasible"] else "NO"
        lines.append(
            f"{i}. [{flag}] {c['name']}   score={c['score']}   "
            f"${t['cost_usd']}   {t['mass_g']} g   "
            f"{t['energy_wh_est']} Wh ({t.get('energy_source')}) / "
            f"paper {t.get('energy_wh_paper')} Wh / pack {t['pack_wh']} Wh"
        )
        lines.append(f"   {c['rationale']}")
        sku_bits = [f"{row['qty']}×{row['sku']}" for row in c["bom"]]
        lines.append("   BOM: " + ", ".join(sku_bits))
        lines.append("")
    sel = result.get("build_select")
    if sel:
        lines.append(
            f"Build-out: {sel.get('status')}  "
            f"E={sel.get('E_wh')} Wh  reserve={sel.get('reserve_wh')} Wh"
        )
        if sel.get("log"):
            lines.append(sel["log"])
        lines.append("")
    return "\n".join(lines)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Propose robot BOMs from the parts catalog")
    p.add_argument("--name", default="mirrored S")
    p.add_argument("--length", type=float, default=17.4, help="track length meters")
    p.add_argument("--max-cost", type=float, default=800.0)
    p.add_argument("--max-mass", type=float, default=6000.0, help="grams")
    p.add_argument("--minutes", type=float, default=5.0)
    p.add_argument("--prefer-low-compute", action="store_true")
    p.add_argument("--candidates", type=int, default=4)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    mission = Mission(
        name=args.name,
        track_length_m=args.length,
        max_cost_usd=args.max_cost,
        max_mass_g=args.max_mass,
        max_minutes=args.minutes,
        prefer_low_compute=args.prefer_low_compute,
    )
    result = propose(mission, max_candidates=args.candidates)
    print(format_report(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
