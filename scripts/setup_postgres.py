#!/usr/bin/env python3
"""Initialize PostgreSQL databases and schema for the robot battery monitor."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import psycopg2
from src.database import init_db


def _admin_params(dbname: str = "robot_battery") -> dict:
    params = {
        "host": os.getenv("PGHOST", "localhost"),
        "port": int(os.getenv("PGPORT", "5432")),
        "dbname": dbname,
        "user": os.getenv("PGUSER", "robot"),
    }
    # No hard-coded default: set PGPASSWORD (or DATABASE_URL) — see .env.example
    password = os.getenv("PGPASSWORD") or os.getenv("POSTGRES_PASSWORD")
    if password:
        params["password"] = password
    return params


def ensure_database(name: str) -> None:
    conn = psycopg2.connect(**_admin_params("robot_battery"))
    conn.autocommit = True
    with conn.cursor() as cur:
        cur.execute("SELECT 1 FROM pg_database WHERE datname = %s", (name,))
        if not cur.fetchone():
            cur.execute(f'CREATE DATABASE "{name}"')
            print(f"Created database: {name}", flush=True)
    conn.close()


def main() -> None:
    print("==> PostgreSQL setup: ensuring databases exist...", flush=True)
    ensure_database("robot_battery")
    ensure_database("robot_battery_test")
    if not os.getenv("DATABASE_URL"):
        p = _admin_params("robot_battery")
        from urllib.parse import quote
        auth = quote(p["user"]) + (f":{quote(p['password'])}" if p.get("password") else "")
        os.environ["DATABASE_URL"] = (
            f"postgresql://{auth}@{p['host']}:{p['port']}/robot_battery"
        )
    print(
        "==> Creating schema (tables, indexes, analytics views)... "
        "(lock waits time out and stale retention deletes are cleared)",
        flush=True,
    )
    init_db()
    print("==> PostgreSQL setup complete.", flush=True)


if __name__ == "__main__":
    main()
