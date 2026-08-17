"""Thin design agent — assemble and score BOMs from the parts catalog.

North-star item 3, first cut. Reads ``src.parts_db`` only. No Webots, no
Postgres, no lane-keep. Scores are analytic heuristics so we can later
swap in a twin evaluation without changing the call shape.

    from src.design_agent import Mission, propose

    result = propose(Mission(track_length_m=25, max_cost_usd=800))
"""

from __future__ import annotations

import argparse
import sqlite3
from dataclasses import asdict, dataclass

from src.parts_db import connect, get_part, list_motors, list_parts
from src.twin_energy import estimate_mission_energy, paper_energy

# Twin cruise is 0.44 m/s. Used when the mission does not set a speed.
DEFAULT_CRUISE_M_S = 0.44
# Baseload (compute + sensors + idle drive electronics) when we have a Jetson-class SBC.
BASELOAD_W = 18.0
# Usable pack fraction — do not plan to the last watt-hour.
PACK_USABLE = 0.80
DEFAULT_WHEEL_RADIUS_M = 0.08


@dataclass
class Mission:
    """Operator envelope. All limits are optional except length."""

    name: str = "mirrored S"
    track_length_m: float = 25.0
    max_mass_g: float | None = None
    max_cost_usd: float | None = None
    min_speed_m_s: float | None = None
    max_minutes: float | None = None
    prefer_low_compute: bool = False
    drive_motors: int = 2
    wheels: int = 2
    cameras: int = 2


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
        return {
            "mission": asdict(mission),
            "candidates": picked,
            "considered": len(scored),
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


def _wheel_radius_m(wheel: dict | None) -> float:
    if not wheel:
        return DEFAULT_WHEEL_RADIUS_M
    name = f"{wheel.get('name') or ''} {wheel.get('notes') or ''}".lower()
    if "80 mm" in name or "80mm" in name or "r=0.08" in name:
        return 0.08
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
    return DEFAULT_CRUISE_M_S


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
    cruise = DEFAULT_CRUISE_M_S
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
    return "\n".join(lines)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Propose robot BOMs from the parts catalog")
    p.add_argument("--name", default="mirrored S")
    p.add_argument("--length", type=float, default=25.0, help="track length meters")
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
