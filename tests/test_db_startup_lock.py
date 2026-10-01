"""Startup lock safety: init_db lock_timeout + stale retention cleanup, bounded purge.

Regression for: dashboard container hung forever in setup_postgres because an
orphaned ``DELETE FROM allocation_snapshots WHERE recorded_at < ...`` from a
killed container kept running and held locks that ``CREATE INDEX IF NOT
EXISTS`` in ``_SCHEMA_SQL`` waited on.
"""

from __future__ import annotations

import threading
import time
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
from unittest import mock

import psycopg2
import psycopg2.errors
import pytest

import src.analytics as analytics
import src.database as database
from src.database import db_cursor, get_db_connection


# ---------------------------------------------------------------------------
# Unit level (mocked cursor)
# ---------------------------------------------------------------------------

class _FakeCursor:
    def __init__(self, fail_schema_times: int):
        self.fail_schema_times = fail_schema_times
        self.executed: list[tuple] = []

    def execute(self, sql, params=None):
        self.executed.append((sql, params))
        if sql is database._SCHEMA_SQL and self.fail_schema_times > 0:
            self.fail_schema_times -= 1
            raise psycopg2.errors.LockNotAvailable(
                "canceling statement due to lock timeout"
            )


def _fake_db_cursor(cursor: _FakeCursor):
    @contextmanager
    def _cm():
        yield object(), cursor
    return _cm


@pytest.fixture
def no_sleep(monkeypatch):
    monkeypatch.setattr(database.time, "sleep", lambda *_a, **_k: None)


def test_init_db_sets_lock_timeout_before_schema(monkeypatch, no_sleep):
    cur = _FakeCursor(fail_schema_times=0)
    monkeypatch.setattr(database, "db_cursor", _fake_db_cursor(cur))
    views = mock.Mock()
    monkeypatch.setattr(analytics, "init_analytics_views", views)

    database.init_db()

    assert cur.executed[0] == ("SET lock_timeout = %s", (database.INIT_LOCK_TIMEOUT,))
    assert cur.executed[1][0] is database._SCHEMA_SQL
    views.assert_called_once_with(cur=cur)


def test_init_db_terminates_stale_retention_and_retries(monkeypatch, no_sleep):
    cur = _FakeCursor(fail_schema_times=1)
    monkeypatch.setattr(database, "db_cursor", _fake_db_cursor(cur))
    monkeypatch.setattr(analytics, "init_analytics_views", mock.Mock())
    term = mock.Mock(return_value=[4242])
    monkeypatch.setattr(database, "terminate_stale_retention_backends", term)

    database.init_db()

    term.assert_called_once_with()
    schema_runs = [e for e in cur.executed if e[0] is database._SCHEMA_SQL]
    assert len(schema_runs) == 2


def test_init_db_gives_up_after_max_attempts(monkeypatch, no_sleep):
    monkeypatch.setattr(database, "INIT_MAX_ATTEMPTS", 3)
    cur = _FakeCursor(fail_schema_times=99)
    monkeypatch.setattr(database, "db_cursor", _fake_db_cursor(cur))
    monkeypatch.setattr(analytics, "init_analytics_views", mock.Mock())
    term = mock.Mock(return_value=[])
    monkeypatch.setattr(database, "terminate_stale_retention_backends", term)

    with pytest.raises(psycopg2.errors.LockNotAvailable):
        database.init_db()
    # Cleanup attempted between tries, not after the final failure.
    assert term.call_count == 2


class _BatchCursor:
    def __init__(self, conn):
        self.conn = conn
        self.rowcount = -1

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def execute(self, sql, params=None):
        self.conn.statements.append((sql, params))
        if sql.startswith("DELETE"):
            self.rowcount = self.conn.rowcounts.pop(0)


class _BatchConn:
    def __init__(self, rowcounts, server_version=160000):
        self.rowcounts = list(rowcounts)
        self.statements: list[tuple] = []
        self.commits = 0
        self.server_version = server_version

    def cursor(self):
        return _BatchCursor(self)

    def commit(self):
        self.commits += 1


def test_delete_in_batches_loops_and_commits_each_batch():
    conn = _BatchConn([10, 10, 3])
    total = database._delete_older_than_in_batches(conn, "allocation_snapshots", "CUT", 10)
    assert total == 23
    assert conn.commits == 3
    sql, params = conn.statements[0]
    assert "LIMIT %s" in sql and "recorded_at < %s" in sql
    assert params == ("CUT", 10)


def test_delete_in_batches_rejects_unknown_table():
    with pytest.raises(ValueError):
        database._delete_older_than_in_batches(_BatchConn([0]), "pg_class; --", "CUT", 10)


def test_retention_session_is_bounded_and_tagged():
    conn = _BatchConn([])
    database._configure_retention_session(conn)
    sqls = [s for s, _ in conn.statements]
    assert "SET application_name = %s" in sqls
    assert "SET statement_timeout = %s" in sqls
    assert "SET lock_timeout = %s" in sqls
    assert "SET client_connection_check_interval = %s" in sqls

    old = _BatchConn([], server_version=130000)
    database._configure_retention_session(old)
    assert "SET client_connection_check_interval = %s" not in [s for s, _ in old.statements]


# ---------------------------------------------------------------------------
# DB level (real Postgres from CI service / TEST_DATABASE_URL)
# ---------------------------------------------------------------------------

def _insert_snapshot(cur, when):
    cur.execute(
        "INSERT INTO allocation_snapshots (recorded_at, task) VALUES (%s, 'test') RETURNING id",
        (when,),
    )
    return cur.fetchone()[0]


def _wait_for_activity(pattern: str, timeout_s: float = 10.0) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        with db_cursor() as (conn, cur):
            cur.execute(
                "SELECT pid FROM pg_stat_activity WHERE datname = current_database() "
                "AND pid <> pg_backend_pid() AND state = 'active' AND query LIKE %s",
                (pattern,),
            )
            row = cur.fetchone()
        if row:
            return row[0]
        time.sleep(0.1)
    raise AssertionError(f"backend matching {pattern!r} never became active")


def test_init_db_clears_orphaned_legacy_retention_delete(clean_database, monkeypatch):
    """Reproduces the hang: a long legacy DELETE holds locks; init_db must not wait forever."""
    monkeypatch.setattr(database, "INIT_LOCK_TIMEOUT", "500ms")
    monkeypatch.setattr(database, "INIT_RETRY_SLEEP_S", 0.2)

    with db_cursor() as (conn, cur):
        _insert_snapshot(cur, datetime.now(timezone.utc) - timedelta(days=40))

    result: dict = {}

    def _orphan():
        c = get_db_connection()
        try:
            with c.cursor() as cur:
                # Legacy statement shape; pg_sleep makes it "run for minutes".
                cur.execute(
                    "DELETE FROM allocation_snapshots WHERE recorded_at < now() "
                    "AND pg_sleep(60) IS NOT NULL"
                )
            c.commit()
            result["finished"] = True
        except psycopg2.Error as e:
            result["error"] = e
        finally:
            c.close()

    t = threading.Thread(target=_orphan, daemon=True)
    t.start()
    orphan_pid = _wait_for_activity("DELETE FROM allocation_snapshots WHERE recorded_at%")

    started = time.monotonic()
    database.init_db()
    elapsed = time.monotonic() - started

    t.join(timeout=10)
    assert not t.is_alive()
    assert "error" in result, "orphaned retention delete should have been terminated"
    assert elapsed < 20
    with db_cursor() as (conn, cur):
        cur.execute("SELECT count(*) FROM pg_stat_activity WHERE pid = %s", (orphan_pid,))
        assert cur.fetchone()[0] == 0


def test_init_db_does_not_kill_unrelated_lock_holders(clean_database, monkeypatch):
    monkeypatch.setattr(database, "INIT_LOCK_TIMEOUT", "300ms")
    monkeypatch.setattr(database, "INIT_MAX_ATTEMPTS", 2)
    monkeypatch.setattr(database, "INIT_RETRY_SLEEP_S", 0.1)

    other = get_db_connection()
    try:
        with other.cursor() as cur:
            cur.execute("SET application_name = 'someone-else'")
            cur.execute("LOCK TABLE allocation_snapshots IN ROW EXCLUSIVE MODE")
        with pytest.raises(psycopg2.errors.LockNotAvailable):
            database.init_db()
        # Still alive: narrow scope means we only touch retention deletes.
        with other.cursor() as cur:
            cur.execute("SELECT 1")
            assert cur.fetchone()[0] == 1
    finally:
        other.rollback()
        other.close()
    database.init_db()  # recovers once the lock is gone


def test_schema_has_fk_lookup_indexes():
    with db_cursor() as (conn, cur):
        cur.execute(
            "SELECT indexname FROM pg_indexes WHERE indexname IN "
            "('idx_channel_readings_snapshot_id', 'idx_energy_predictions_snapshot_id', "
            "'idx_allocation_snapshots_recorded_at')"
        )
        names = {r[0] for r in cur.fetchall()}
    assert names == {
        "idx_channel_readings_snapshot_id",
        "idx_energy_predictions_snapshot_id",
        "idx_allocation_snapshots_recorded_at",
    }


def test_archive_old_data_batched_deletes_all_old_rows(clean_database):
    old = datetime.now(timezone.utc) - timedelta(days=40)
    new = datetime.now(timezone.utc)
    with db_cursor() as (conn, cur):
        for i in range(11):
            sid = _insert_snapshot(cur, old)
            cur.execute(
                "INSERT INTO channel_readings (snapshot_id, channel_id, recorded_at, "
                "battery_level, power_draw_w) VALUES (%s, 'Legs', %s, 50, 5)",
                (sid, old),
            )
            cur.execute(
                "INSERT INTO energy_predictions (snapshot_id, recorded_at, task, battery_pct) "
                "VALUES (%s, %s, 'test', 50)",
                (sid, old),
            )
        keep = _insert_snapshot(cur, new)
        cur.execute(
            "INSERT INTO channel_readings (snapshot_id, channel_id, recorded_at, "
            "battery_level, power_draw_w) VALUES (%s, 'Arms', %s, 70, 8)",
            (keep, new),
        )

    database.archive_old_data(days=30, batch_rows=4)

    with db_cursor() as (conn, cur):
        cur.execute("SELECT count(*) FROM allocation_snapshots")
        assert cur.fetchone()[0] == 1
        cur.execute("SELECT channel_id, snapshot_id FROM channel_readings")
        assert cur.fetchall() == [("Arms", keep)]
        cur.execute("SELECT count(*) FROM energy_predictions")
        assert cur.fetchone()[0] == 0
