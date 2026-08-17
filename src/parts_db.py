"""Design-agent parts catalog (SQLite).

This is **not** the Postgres telemetry store (``src/database.py``).
Postgres holds live power history. This file is a mostly-read catalog of
purchasable hardware the future design agent will query.

Default path: ``<repo>/data/parts.db``. Schema is created on first connect.
Example SKUs are seeded only when the ``parts`` table is empty.
"""

from __future__ import annotations

import sqlite3
from datetime import datetime, timezone
from pathlib import Path

CATEGORIES = (
    "motor",
    "battery",
    "camera",
    "wheel",
    "structure",
    "esc",
    "sensor",
    "compute",
    "other",
)

_REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DB_PATH = _REPO_ROOT / "data" / "parts.db"

_SCHEMA = """
CREATE TABLE IF NOT EXISTS parts (
    id INTEGER PRIMARY KEY,
    sku TEXT UNIQUE,
    name TEXT NOT NULL,
    category TEXT NOT NULL,
    vendor TEXT,
    mass_g REAL,
    cost_usd REAL,
    notes TEXT,
    source_url TEXT,
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS motors (
    part_id INTEGER PRIMARY KEY REFERENCES parts(id) ON DELETE CASCADE,
    voltage_v REAL,
    gear_ratio REAL,
    continuous_torque_nm REAL,
    peak_torque_nm REAL,
    continuous_power_w REAL,
    peak_power_w REAL,
    no_load_rpm REAL,
    stall_current_a REAL,
    efficiency REAL
);

CREATE TABLE IF NOT EXISTS batteries (
    part_id INTEGER PRIMARY KEY REFERENCES parts(id) ON DELETE CASCADE,
    capacity_wh REAL,
    capacity_ah REAL,
    voltage_v REAL,
    max_continuous_a REAL,
    peak_a REAL,
    chemistry TEXT
);

CREATE TABLE IF NOT EXISTS cameras (
    part_id INTEGER PRIMARY KEY REFERENCES parts(id) ON DELETE CASCADE,
    width INTEGER,
    height INTEGER,
    fps_max INTEGER,
    fov_deg REAL,
    compute_cost REAL,
    interface TEXT
);

CREATE INDEX IF NOT EXISTS idx_parts_category ON parts(category);
CREATE INDEX IF NOT EXISTS idx_parts_cost ON parts(cost_usd);
CREATE INDEX IF NOT EXISTS idx_parts_mass ON parts(mass_g);
"""


def default_db_path() -> Path:
    return DEFAULT_DB_PATH


def connect(db_path: Path | str | None = None) -> sqlite3.Connection:
    """Open the catalog, create parent dir + schema, seed if empty."""
    path = Path(db_path) if db_path is not None else DEFAULT_DB_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(path))
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys = ON")
    init_schema(conn)
    if _parts_empty(conn):
        seed_examples(conn)
    return conn


def init_schema(conn: sqlite3.Connection) -> None:
    conn.executescript(_SCHEMA)
    conn.commit()


def _parts_empty(conn: sqlite3.Connection) -> bool:
    row = conn.execute("SELECT COUNT(*) AS n FROM parts").fetchone()
    return int(row["n"]) == 0


def _now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def add_part(
    conn: sqlite3.Connection,
    *,
    name: str,
    category: str,
    sku: str | None = None,
    vendor: str | None = None,
    mass_g: float | None = None,
    cost_usd: float | None = None,
    notes: str | None = None,
    source_url: str | None = None,
) -> int:
    if category not in CATEGORIES:
        raise ValueError(f"unknown category {category!r}; expected one of {CATEGORIES}")
    cur = conn.execute(
        """
        INSERT INTO parts (sku, name, category, vendor, mass_g, cost_usd, notes, source_url, created_at)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (sku, name, category, vendor, mass_g, cost_usd, notes, source_url, _now()),
    )
    conn.commit()
    return int(cur.lastrowid)


def add_motor(
    conn: sqlite3.Connection,
    part_id: int,
    *,
    voltage_v: float | None = None,
    gear_ratio: float | None = None,
    continuous_torque_nm: float | None = None,
    peak_torque_nm: float | None = None,
    continuous_power_w: float | None = None,
    peak_power_w: float | None = None,
    no_load_rpm: float | None = None,
    stall_current_a: float | None = None,
    efficiency: float | None = None,
) -> None:
    conn.execute(
        """
        INSERT INTO motors (
            part_id, voltage_v, gear_ratio, continuous_torque_nm, peak_torque_nm,
            continuous_power_w, peak_power_w, no_load_rpm, stall_current_a, efficiency
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            part_id,
            voltage_v,
            gear_ratio,
            continuous_torque_nm,
            peak_torque_nm,
            continuous_power_w,
            peak_power_w,
            no_load_rpm,
            stall_current_a,
            efficiency,
        ),
    )
    conn.commit()


def add_battery(
    conn: sqlite3.Connection,
    part_id: int,
    *,
    capacity_wh: float | None = None,
    capacity_ah: float | None = None,
    voltage_v: float | None = None,
    max_continuous_a: float | None = None,
    peak_a: float | None = None,
    chemistry: str | None = None,
) -> None:
    conn.execute(
        """
        INSERT INTO batteries (
            part_id, capacity_wh, capacity_ah, voltage_v,
            max_continuous_a, peak_a, chemistry
        ) VALUES (?, ?, ?, ?, ?, ?, ?)
        """,
        (
            part_id,
            capacity_wh,
            capacity_ah,
            voltage_v,
            max_continuous_a,
            peak_a,
            chemistry,
        ),
    )
    conn.commit()


def add_camera(
    conn: sqlite3.Connection,
    part_id: int,
    *,
    width: int | None = None,
    height: int | None = None,
    fps_max: int | None = None,
    fov_deg: float | None = None,
    compute_cost: float | None = None,
    interface: str | None = None,
) -> None:
    conn.execute(
        """
        INSERT INTO cameras (
            part_id, width, height, fps_max, fov_deg, compute_cost, interface
        ) VALUES (?, ?, ?, ?, ?, ?, ?)
        """,
        (part_id, width, height, fps_max, fov_deg, compute_cost, interface),
    )
    conn.commit()


def get_part(conn: sqlite3.Connection, part_id: int) -> dict | None:
    row = conn.execute("SELECT * FROM parts WHERE id = ?", (part_id,)).fetchone()
    if row is None:
        return None
    out = dict(row)
    cat = out["category"]
    if cat == "motor":
        spec = conn.execute("SELECT * FROM motors WHERE part_id = ?", (part_id,)).fetchone()
        out["motor"] = dict(spec) if spec else None
    elif cat == "battery":
        spec = conn.execute(
            "SELECT * FROM batteries WHERE part_id = ?", (part_id,)
        ).fetchone()
        out["battery"] = dict(spec) if spec else None
    elif cat == "camera":
        spec = conn.execute(
            "SELECT * FROM cameras WHERE part_id = ?", (part_id,)
        ).fetchone()
        out["camera"] = dict(spec) if spec else None
    return out


def list_parts(
    conn: sqlite3.Connection,
    *,
    category: str | None = None,
    max_cost_usd: float | None = None,
    max_mass_g: float | None = None,
) -> list[dict]:
    sql = "SELECT * FROM parts WHERE 1=1"
    args: list = []
    if category is not None:
        sql += " AND category = ?"
        args.append(category)
    if max_cost_usd is not None:
        sql += " AND cost_usd IS NOT NULL AND cost_usd <= ?"
        args.append(max_cost_usd)
    if max_mass_g is not None:
        sql += " AND mass_g IS NOT NULL AND mass_g <= ?"
        args.append(max_mass_g)
    sql += " ORDER BY category, cost_usd, name"
    return [dict(r) for r in conn.execute(sql, args)]


def list_motors(
    conn: sqlite3.Connection,
    *,
    max_cost_usd: float | None = None,
    max_mass_g: float | None = None,
) -> list[dict]:
    sql = """
        SELECT p.*, m.voltage_v, m.gear_ratio, m.continuous_torque_nm,
               m.peak_torque_nm, m.continuous_power_w, m.peak_power_w,
               m.no_load_rpm, m.stall_current_a, m.efficiency
        FROM parts p
        JOIN motors m ON m.part_id = p.id
        WHERE p.category = 'motor'
    """
    args: list = []
    if max_cost_usd is not None:
        sql += " AND p.cost_usd IS NOT NULL AND p.cost_usd <= ?"
        args.append(max_cost_usd)
    if max_mass_g is not None:
        sql += " AND p.mass_g IS NOT NULL AND p.mass_g <= ?"
        args.append(max_mass_g)
    sql += " ORDER BY p.cost_usd, p.name"
    return [dict(r) for r in conn.execute(sql, args)]


def seed_examples(conn: sqlite3.Connection) -> int:
    """Idempotent-enough seed: only call when the table is empty.

    SKUs come from the live wheeled BOM in
    ``config/hardware_profiles/butlerbot_wheeled.yaml`` plus a couple of
    cheaper / lighter catalog alternatives the design agent can compare.
    """
    n0 = int(conn.execute("SELECT COUNT(*) AS n FROM parts").fetchone()["n"])
    if n0:
        return 0

    pid = add_part(
        conn,
        sku="POLOLU-4753",
        name="Pololu 37D 50:1 12V metal gearmotor + 64 CPR encoder",
        category="motor",
        vendor="Pololu",
        mass_g=200,
        cost_usd=44.95,
        notes="Current twin drive. Stall 2.06 N·m / 5.5 A @ 12 V. Track cruise ~52 RPM.",
        source_url="https://www.pololu.com/product/4753",
    )
    add_motor(
        conn,
        pid,
        voltage_v=12,
        gear_ratio=50,
        continuous_torque_nm=0.22,
        peak_torque_nm=2.06,
        continuous_power_w=8.0,
        peak_power_w=66.0,
        no_load_rpm=200,
        stall_current_a=5.5,
        efficiency=0.51,
    )

    pid = add_part(
        conn,
        sku="POLOLU-4751",
        name="Pololu 37D 19:1 12V metal gearmotor + 64 CPR encoder",
        category="motor",
        vendor="Pololu",
        mass_g=195,
        cost_usd=44.95,
        notes="Faster / less torque sibling of #4753. Candidate if cruise rises.",
        source_url="https://www.pololu.com/product/4751",
    )
    add_motor(
        conn,
        pid,
        voltage_v=12,
        gear_ratio=19,
        continuous_torque_nm=0.09,
        peak_torque_nm=0.78,
        continuous_power_w=8.0,
        peak_power_w=66.0,
        no_load_rpm=530,
        stall_current_a=5.5,
        efficiency=0.50,
    )

    pid = add_part(
        conn,
        sku="AEGIS-48V-10AH",
        name="48V 10Ah NMC Li-ion pack (480 Wh class)",
        category="battery",
        vendor="generic-catalog",
        mass_g=2700,
        cost_usd=289.00,
        notes="Current twin pack class from butlerbot_wheeled.yaml. 2C cont / 3C peak.",
        source_url="https://www.aegisbattery.com/products/aegis-48v-10ah-lithium-ion-battery-pack-nmc-48v-lithium-battery",
    )
    add_battery(
        conn,
        pid,
        capacity_wh=480,
        capacity_ah=10,
        voltage_v=48,
        max_continuous_a=20,
        peak_a=30,
        chemistry="NMC Li-ion",
    )

    pid = add_part(
        conn,
        sku="LIPO-3S-5000",
        name="3S 11.1V 5000 mAh LiPo pack",
        category="battery",
        vendor="generic-hobby",
        mass_g=365,
        cost_usd=34.99,
        notes="Lighter cheaper pack for small-robot candidates. Not the current twin.",
        source_url="https://hobbyking.com/",
    )
    add_battery(
        conn,
        pid,
        capacity_wh=55.5,
        capacity_ah=5.0,
        voltage_v=11.1,
        max_continuous_a=125,
        peak_a=200,
        chemistry="LiPo",
    )

    pid = add_part(
        conn,
        sku="RPI-CAM-V2",
        name="Raspberry Pi Camera Module v2 (8 MP)",
        category="camera",
        vendor="Raspberry Pi",
        mass_g=3,
        cost_usd=25.00,
        notes="CSI. Fine for lane paint; compute_cost is a crude 0–1 tax (not watts).",
        source_url="https://www.raspberrypi.com/products/camera-module-v2/",
    )
    add_camera(
        conn,
        pid,
        width=3280,
        height=2464,
        fps_max=30,
        fov_deg=62,
        compute_cost=0.35,
        interface="CSI",
    )

    pid = add_part(
        conn,
        sku="ARDUCAM-USB-1080",
        name="Arducam 1080p USB camera module",
        category="camera",
        vendor="Arducam",
        mass_g=18,
        cost_usd=29.99,
        notes="USB UVC. Higher host tax than CSI. compute_cost 0.55 vs Pi Cam 0.35.",
        source_url="https://www.arducam.com/",
    )
    add_camera(
        conn,
        pid,
        width=1920,
        height=1080,
        fps_max=30,
        fov_deg=70,
        compute_cost=0.55,
        interface="USB",
    )

    add_part(
        conn,
        sku="ADAFRUIT-4754",
        name="Adafruit BNO085 9-DOF fusion IMU",
        category="sensor",
        vendor="Adafruit",
        mass_g=2.5,
        cost_usd=24.95,
        notes="Current twin IMU SKU (#4754). I2C 0x4A. 25.6×22.7×4.6 mm.",
        source_url="https://www.adafruit.com/product/4754",
    )
    add_part(
        conn,
        sku="SF-ROB-14451",
        name="SparkFun Dual TB6612FNG motor driver",
        category="esc",
        vendor="SparkFun",
        mass_g=4,
        cost_usd=5.50,
        notes="1.2 A cont / 3.2 A peak per channel. Current twin drive bridge.",
        source_url="https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-1a.html",
    )
    add_part(
        conn,
        sku="JETSON-ORIN-NANO-8",
        name="NVIDIA Jetson Orin Nano 8GB class module",
        category="compute",
        vendor="NVIDIA",
        mass_g=50,
        cost_usd=249.00,
        notes="Current twin compute class. ~7–15 W. Carrier mass extra.",
        source_url="https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/",
    )
    add_part(
        conn,
        sku="WHEEL-80-RUBBER",
        name="80 mm rubber robot wheel + hub",
        category="wheel",
        vendor="generic-catalog",
        mass_g=85,
        cost_usd=6.50,
        notes="Matches twin r=0.08 m. Catalog class mass from wheeled profile.",
        source_url="https://abra-electronics.com/robotics/robot-wheels/tire-80-black-8035mm-rubber-wheel.html",
    )

    return int(conn.execute("SELECT COUNT(*) AS n FROM parts").fetchone()["n"])


def _fmt(row: dict, keys: tuple[str, ...]) -> str:
    bits = []
    for k in keys:
        if k in row and row[k] is not None:
            bits.append(f"{k}={row[k]}")
    return "  ".join(bits)


def demo(db_path: Path | str | None = None) -> None:
    """Print a few queries the design agent will need."""
    conn = connect(db_path)
    path = Path(db_path) if db_path is not None else DEFAULT_DB_PATH
    print(f"catalog: {path}")
    print(f"parts: {len(list_parts(conn))}")
    print("\n-- motors under $50 and 250 g --")
    for row in list_motors(conn, max_cost_usd=50, max_mass_g=250):
        print(
            f"  {row['sku']}  {row['name'][:48]}  "
            f"${row['cost_usd']}  {row['mass_g']}g  "
            f"{row['voltage_v']}V  peak_τ={row['peak_torque_nm']} N·m"
        )
    print("\n-- cameras --")
    for row in list_parts(conn, category="camera"):
        full = get_part(conn, row["id"])
        cam = (full or {}).get("camera") or {}
        print(
            f"  {row['sku']}  ${row['cost_usd']}  {row['mass_g']}g  "
            f"{cam.get('width')}x{cam.get('height')}  "
            f"compute_cost={cam.get('compute_cost')}  {cam.get('interface')}"
        )
    print("\n-- all categories --")
    for row in list_parts(conn):
        print(f"  {row['category']:8}  {row['sku']:22}  ${row['cost_usd']}")
    conn.close()


if __name__ == "__main__":
    demo()
