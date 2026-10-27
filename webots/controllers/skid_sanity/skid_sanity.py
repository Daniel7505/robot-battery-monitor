"""Physics check for the four-wheel skid-steer proto.

Not imported by the apartment mapper. The robot drives itself and reads
its own contact points. A passing run sits level, keeps all four tires
on the floor, drives two metres straight, spins in place, and climbs a
vertical 12.7 mm lip.
"""

from __future__ import annotations

import math
import os

from controller import Supervisor

WHEEL_R = 0.08
SPEED = 0.22
LIP_X = float(os.environ.get("RBM_LIP_X", "3.0"))
LIP_Z = float(os.environ.get("RBM_LIP_Z", "0.0127"))
STRAIGHT_M = 2.0
SPIN_RAD = math.pi

# Soft contact may sink the hub by a millimetre or two. More than this is
# a tire floating or a bounding volume buried in the floor.
SINK_M = 0.004
LEVEL_RAD = 0.03

JOINTS = (
    ("rear_left", "LEFT_WHEEL_JOINT"),
    ("rear_right", "RIGHT_WHEEL_JOINT"),
    ("front_left", "FRONT_LEFT_WHEEL_JOINT"),
    ("front_right", "FRONT_RIGHT_WHEEL_JOINT"),
)
MOTORS = {
    "rear_left": "left_wheel",
    "rear_right": "right_wheel",
    "front_left": "front_left_wheel",
    "front_right": "front_right_wheel",
}


def _yaw(node) -> float:
    matrix = node.getOrientation()
    return math.atan2(float(matrix[3]), float(matrix[0]))


def _up_z(node) -> float:
    return float(node.getOrientation()[8])


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _median(values: list[float]) -> float:
    clean = sorted(value for value in values if value == value)
    if not clean:
        return float("nan")
    mid = len(clean) // 2
    if len(clean) % 2:
        return clean[mid]
    return 0.5 * (clean[mid - 1] + clean[mid])


def _endpoint(robot, def_name: str):
    joint = robot.getFromProtoDef(def_name)
    if joint is None:
        return None
    field = joint.getField("endPoint")
    if field is None:
        return None
    try:
        return field.getSFNode()
    except Exception:
        return None


def _partner(supervisor, node_id: int, cache: dict) -> str:
    if node_id in cache:
        return cache[node_id]
    name = ""
    node = supervisor.getFromId(int(node_id))
    if node is not None:
        field = node.getField("name")
        if field is not None:
            try:
                name = field.getSFString() or ""
            except Exception:
                name = ""
    cache[node_id] = name
    return name


def _floor_contact(supervisor, solid, cache: dict) -> tuple[int, float]:
    """Return (floor-contact count, lowest floor-contact z)."""
    if solid is None:
        return 0, float("nan")
    try:
        points = solid.getContactPoints()
    except Exception:
        return 0, float("nan")
    count = 0
    low = float("nan")
    for point in points:
        z = float(point.point[2])
        name = _partner(supervisor, point.node_id, cache).lower()
        if "floor" in name or z < 0.05:
            count += 1
            low = z if math.isnan(low) else min(low, z)
    return count, low


def _fmt(value: float, digits: int = 2) -> str:
    if value != value:
        return "nan"
    return f"{value:.{digits}f}"


def main() -> None:
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    node = robot.getSelf()
    folder = os.environ.get("RBM_SKID_DIR", "/tmp/skid_sanity")
    os.makedirs(folder, exist_ok=True)
    motors = {}
    for key, name in MOTORS.items():
        device = robot.getDevice(name)
        if device is None:
            print(f"SKID SANITY missing motor {name}", flush=True)
            robot.simulationQuit(1)
            return
        device.setPosition(float("inf"))
        device.setVelocity(0.0)
        motors[key] = device
    for name in ("torso_joint", "left_arm", "right_arm"):
        upper = robot.getDevice(name)
        if upper is None:
            continue
        upper.setPosition(0.0)
        upper.setVelocity(1.0)
    imu = robot.getDevice("imu")
    if imu is not None:
        imu.enable(timestep)
    wheels = {key: _endpoint(node, def_name) for key, def_name in JOINTS}
    missing = [key for key, solid in wheels.items() if solid is None]
    if missing:
        print(f"SKID SANITY missing wheel solids {missing}", flush=True)
        robot.simulationQuit(1)
        return
    for solid in wheels.values():
        solid.enableContactPointsTracking(timestep)
    cache: dict = {}
    omega = SPEED / WHEEL_R
    phase = "rest"
    phase_t = 0.0
    origin = None
    straight_pose = None
    spin_pose = None
    spin_yaw0 = 0.0
    spin_excursion = 0.0
    climb_t0 = None
    rest_samples: list[dict] = []
    climb_samples: list[dict] = []
    reasons: list[str] = []
    report: list[str] = []

    def command(left: float, right: float) -> None:
        motors["rear_left"].setVelocity(left)
        motors["front_left"].setVelocity(left)
        motors["rear_right"].setVelocity(right)
        motors["front_right"].setVelocity(right)

    def sample() -> dict:
        pos = node.getPosition()
        roll = pitch = float("nan")
        if imu is not None:
            try:
                rpy = imu.getRollPitchYaw()
                roll = float(rpy[0])
                pitch = float(rpy[1])
            except Exception:
                pass
        tires = {}
        for key, solid in wheels.items():
            centre = solid.getPosition()
            count, low = _floor_contact(robot, solid, cache)
            tires[key] = {
                "x": float(centre[0]),
                "y": float(centre[1]),
                "z": float(centre[2]),
                "n": count,
                "cz": low,
            }
        return {
            "x": float(pos[0]),
            "y": float(pos[1]),
            "z": float(pos[2]),
            "yaw": _yaw(node),
            "up": _up_z(node),
            "roll": roll,
            "pitch": pitch,
            "tires": tires,
        }

    def finish(now: float) -> None:
        command(0.0, 0.0)
        ok = not reasons
        line = "SKID SANITY result=" + ("PASS" if ok else "FAIL")
        report.append(line)
        if reasons:
            report.append("SKID SANITY fail " + "; ".join(reasons))
        text = "\n".join(report) + "\n"
        print(text, flush=True)
        with open(os.path.join(folder, "result.txt"), "w", encoding="utf-8") as handle:
            handle.write(text)
        print(f"SKID SANITY quit t={now:.2f}", flush=True)
        robot.simulationQuit(0 if ok else 2)

    def wheel_lines(title: str, rows: list[dict], surface_z: float) -> None:
        seated = surface_z + WHEEL_R
        report.append(
            f"SKID SANITY {title} body_z={_fmt(_median([row['z'] for row in rows]), 4)} "
            f"up_z={_fmt(_median([row['up'] for row in rows]), 5)} "
            f"roll_deg={_fmt(math.degrees(_median([row['roll'] for row in rows])))} "
            f"pitch_deg={_fmt(math.degrees(_median([row['pitch'] for row in rows])))}"
        )
        for key, _def in JOINTS:
            centres = [row["tires"][key]["z"] for row in rows]
            contacts = [row["tires"][key]["cz"] for row in rows]
            counts = [row["tires"][key]["n"] for row in rows]
            centre = _median(centres)
            contact = _median(contacts)
            touched = sum(1 for count in counts if count > 0) / max(1, len(counts))
            # Positive depth: the hub is below the seated height, or the
            # contact point is below the floor face.
            hub_mm = (seated - centre) * 1000.0
            face_mm = (surface_z - contact) * 1000.0 if contact == contact else float("nan")
            report.append(
                f"SKID SANITY {title} {key} center_z={centre:.5f} "
                f"contact_z={_fmt(contact, 5)} contacts={touched:.2f} "
                f"hub_depth_mm={hub_mm:.2f} contact_depth_mm={_fmt(face_mm)}"
            )
            if touched < 0.85:
                reasons.append(f"{title} {key} contact {touched:.2f}")
            if abs(centre - seated) > SINK_M:
                reasons.append(f"{title} {key} hub {hub_mm:.1f} mm")
            if contact != contact or contact < surface_z - SINK_M or contact > surface_z + SINK_M:
                reasons.append(f"{title} {key} contact depth {_fmt(face_mm)} mm")

    while robot.step(timestep) != -1:
        now = float(robot.getTime())
        if now > 50.0:
            reasons.append("ran long")
            finish(now)
            return
        snap = sample()
        if origin is None and now >= 0.2:
            origin = snap
        if phase == "rest":
            command(0.0, 0.0)
            if now >= 1.0:
                rest_samples.append(snap)
            if now >= 1.6 and len(rest_samples) > 10:
                wheel_lines("rest", rest_samples, 0.0)
                roll = _median([row["roll"] for row in rest_samples])
                pitch = _median([row["pitch"] for row in rest_samples])
                up = _median([row["up"] for row in rest_samples])
                zs = [row["z"] for row in rest_samples]
                spread = max(zs) - min(zs)
                if abs(roll) > LEVEL_RAD or abs(pitch) > LEVEL_RAD or up < 0.999:
                    reasons.append(
                        f"rest lean roll={math.degrees(roll):.2f} pitch={math.degrees(pitch):.2f} up={up:.4f}"
                    )
                if spread > 0.003:
                    reasons.append(f"rest bounce {spread * 1000.0:.1f} mm")
                phase = "straight"
                phase_t = now
                straight_pose = snap
            continue
        if phase == "straight":
            command(omega, omega)
            moved_x = snap["x"] - straight_pose["x"]
            moved_y = snap["y"] - straight_pose["y"]
            if moved_x >= STRAIGHT_M or now > phase_t + 16.0:
                command(0.0, 0.0)
                yaw = _wrap(snap["yaw"] - straight_pose["yaw"])
                report.append(
                    f"SKID SANITY straight forward={moved_x:.3f} lateral={moved_y:.3f} "
                    f"yaw_deg={math.degrees(yaw):.2f}"
                )
                if moved_x < STRAIGHT_M - 0.05:
                    reasons.append(f"straight only {moved_x:.2f} m")
                if abs(moved_y) > 0.05:
                    reasons.append(f"straight lateral {moved_y * 100.0:.1f} cm")
                if abs(yaw) > math.radians(5.0):
                    reasons.append(f"straight yaw {math.degrees(yaw):.1f} deg")
                phase = "spin_settle"
                phase_t = now
            continue
        if phase == "spin_settle":
            command(0.0, 0.0)
            if now >= phase_t + 0.4:
                phase = "spin"
                phase_t = now
                spin_pose = snap
                spin_yaw0 = snap["yaw"]
                spin_excursion = 0.0
            continue
        if phase == "spin":
            command(-omega, omega)
            travel = math.hypot(snap["x"] - spin_pose["x"], snap["y"] - spin_pose["y"])
            spin_excursion = max(spin_excursion, travel)
            turned = _wrap(snap["yaw"] - spin_yaw0)
            # Unwrap across the ±pi cut by integrating the short step.
            if not hasattr(main, "_spin_total"):
                main._spin_total = 0.0
                main._spin_prev = snap["yaw"]
            main._spin_total += _wrap(snap["yaw"] - main._spin_prev)
            main._spin_prev = snap["yaw"]
            if abs(main._spin_total) >= SPIN_RAD or now > phase_t + 8.0:
                command(0.0, 0.0)
                report.append(
                    f"SKID SANITY spin yaw_deg={math.degrees(main._spin_total):.2f} "
                    f"excursion_m={spin_excursion:.3f} net_m={travel:.3f}"
                )
                if abs(main._spin_total) < math.radians(160.0):
                    reasons.append(f"spin only {math.degrees(main._spin_total):.0f} deg")
                if spin_excursion > 0.08:
                    reasons.append(f"spin wandered {spin_excursion * 100.0:.1f} cm")
                phase = "climb_settle"
                phase_t = now
            continue
        if phase == "climb_settle":
            command(0.0, 0.0)
            if now >= phase_t + 0.4:
                phase = "climb"
                phase_t = now
                climb_t0 = now
            continue
        if phase == "climb":
            command(omega, omega)
            on_top = all(tire["z"] > LIP_Z + WHEEL_R - 0.006 and tire["x"] > LIP_X + 0.04 for tire in snap["tires"].values())
            if (on_top and snap["x"] >= LIP_X + 0.30) or now > phase_t + 18.0:
                command(0.0, 0.0)
                phase = "climb_done"
                phase_t = now
                climb_samples = []
            continue
        if phase == "climb_done":
            command(0.0, 0.0)
            climb_samples.append(snap)
            if now >= phase_t + 0.6:
                wheel_lines("climb", climb_samples, LIP_Z)
                elapsed = now - (climb_t0 or now)
                report.append(
                    f"SKID SANITY climb x={snap['x']:.3f} y={snap['y']:.3f} "
                    f"elapsed={elapsed:.2f}s lip_z={LIP_Z:.4f}"
                )
                if snap["x"] < LIP_X + 0.25:
                    reasons.append(f"climb stopped at x={snap['x']:.2f}")
                if snap["up"] < 0.995:
                    reasons.append(f"climb up_z={snap['up']:.4f}")
                finish(now)
                return
    reasons.append("simulation ended")
    finish(float(robot.getTime()))


if __name__ == "__main__":
    main()
