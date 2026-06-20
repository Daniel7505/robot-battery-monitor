#!/usr/bin/env python3
"""Initialize PostgreSQL databases and schema for the robot battery monitor."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import psycopg2
from src.database import init_db


def _admin_params(dbname: str = "robot_battery") -> dict:
    return {
        "host": os.getenv("PGHOST", "localhost"),
        "port": int(os.getenv("PGPORT", "5432")),
        "dbname": dbname,
        "user": os.getenv("PGUSER", "robot"),
        "password": os.getenv("PGPASSWORD", "robot"),
    }


def ensure_database(name: str) -> None:
    conn = psycopg2.connect(**_admin_params("robot_battery"))
    conn.autocommit = True
    with conn.cursor() as cur:
        cur.execute("SELECT 1 FROM pg_database WHERE datname = %s", (name,))
        if not cur.fetchone():
            cur.execute(f'CREATE DATABASE "{name}"')
            print(f"Created database: {name}")
    conn.close()


if __name__ == "__main__":
    ensure_database("robot_battery")
    ensure_database("robot_battery_test")
    os.environ.setdefault(
        "DATABASE_URL", "postgresql://robot:robot@localhost:5432/robot_battery"
    )
    init_db()
    print("PostgreSQL setup complete.")