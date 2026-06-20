#!/usr/bin/env python3
"""Block until PostgreSQL accepts connections."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import psycopg2

MAX_ATTEMPTS = int(os.getenv("DB_WAIT_ATTEMPTS", "30"))
SLEEP_SECONDS = float(os.getenv("DB_WAIT_INTERVAL", "2"))


def _params() -> dict:
    url = os.getenv("DATABASE_URL")
    if url:
        return {"dsn": url}
    return {
        "host": os.getenv("PGHOST", "localhost"),
        "port": int(os.getenv("PGPORT", "5432")),
        "dbname": os.getenv("PGDATABASE", "robot_battery"),
        "user": os.getenv("PGUSER", "robot"),
        "password": os.getenv("PGPASSWORD", "robot"),
    }


def wait_for_postgres() -> None:
    params = _params()
    host = params.get("host", params.get("dsn", "database"))
    print(f"Waiting for PostgreSQL at {host} (max {MAX_ATTEMPTS} attempts)...")

    for attempt in range(1, MAX_ATTEMPTS + 1):
        try:
            conn = psycopg2.connect(**params)
            conn.close()
            print("PostgreSQL is ready.")
            return
        except psycopg2.OperationalError as exc:
            print(f"  attempt {attempt}/{MAX_ATTEMPTS}: {exc}")
            time.sleep(SLEEP_SECONDS)

    raise SystemExit(f"PostgreSQL not ready after {MAX_ATTEMPTS} attempts")


if __name__ == "__main__":
    wait_for_postgres()