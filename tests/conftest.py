"""Shared pytest setup.

Repo docs
---------
``tests/test_repo_docs_pristine.py`` registers a session autouse fixture so
``docs/AGENT_JOURNAL.md`` and ``optimizer/KNOWLEDGE.md`` are not modified by
the test suite.

Postgres
--------
Most tests are pure Python. Tests that need the test database
(``robot_battery_test``) are marked ``db``: either explicitly with
``@pytest.mark.db`` or automatically because they use the
``clean_database`` fixture.

* Postgres reachable  -> every test runs (this is what CI does).
* Postgres missing    -> ``db`` tests are skipped with a clear reason and the
  rest still run, so a newcomer can run the suite without a database.
* ``RBM_REQUIRE_DB=1`` (set in CI) or ``CI=true`` -> a missing database is a
  hard error instead of a skip, so CI can never go green by skipping.

``pytest.ini`` sets ``--dist loadgroup``. With ``-n``, every ``db`` test is
pinned to one worker (``xdist_group``) because they share one database and
truncate it.

That grouping does not cover schema creation. ``init_db`` runs ``CREATE TABLE``
and drops and rebuilds the analytics views. Under ``-n`` the session fixture
used to call it on every worker, and PostgreSQL's ``CREATE TABLE IF NOT EXISTS``
is not safe when several sessions create the same missing table: the losers
die with ``pg_type_typname_nsp_index`` / ``pg_class_relname_nsp_index`` on
``allocation_snapshots``. The xdist controller therefore builds the schema
once in ``pytest_sessionstart``, before workers are spawned. Workers only
check that the database is reachable. A plain ``pytest`` without ``-n`` still
builds the schema in this process.

Pytest 9 reports ``item.nodeid`` from ``item._id``. pytest-xdist 3.8 still
writes the group onto ``item._nodeid``, which pytest 9 ignores, so the
``@db`` suffix never reached the scheduler and database tests ran on every
worker. On a worker we append that suffix ourselves.
"""
import os

import pytest
from src.config import Config
from src.database import get_db_connection, init_db, truncate_tables

_DB_UNAVAILABLE_REASON: str | None = None


def _test_database_url() -> str:
    return os.getenv(
        "TEST_DATABASE_URL",
        "postgresql://robot:robot@localhost:5432/robot_battery_test",
    )


def _db_required() -> bool:
    for name in ("RBM_REQUIRE_DB", "CI"):
        if os.getenv(name, "").strip().lower() in ("1", "true", "yes"):
            return True
    return False


def _is_xdist_worker(config) -> bool:
    return getattr(config, "workerinput", None) is not None


def _db_failure_summary(exc: BaseException) -> str:
    text = str(exc).strip()
    first_line = text.splitlines()[0] if text else type(exc).__name__
    return (
        f"Postgres test database not reachable ({first_line}). "
        "Start it to run DB tests (see README, setup B step 4)."
    )


@pytest.hookimpl(tryfirst=True)
def pytest_sessionstart(session):
    """Build the test schema before xdist spawns workers.

    xdist starts workers from its own ``pytest_sessionstart`` (``trylast``),
    so this ``tryfirst`` hook finishes first. Workers skip it: a second
    ``init_db`` would drop the analytics views while other workers are
    already running.
    """
    config = session.config
    if _is_xdist_worker(config):
        return
    os.environ["DATABASE_URL"] = _test_database_url()
    try:
        init_db()
    except Exception as exc:
        config._rbm_db_error = exc
        if _db_required():
            pytest.exit(_db_failure_summary(exc), returncode=1)
    else:
        config._rbm_db_error = None


def _probe_test_database() -> None:
    """Confirm Postgres is up and the controller finished schema init.

    Does not call ``init_db``. Workers that each rebuilt the schema raced
    on ``CREATE TABLE`` and then on ``DROP VIEW``.
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT to_regclass('public.allocation_snapshots')")
            relation = cur.fetchone()[0]
        conn.rollback()
    finally:
        conn.close()
    if relation is None:
        raise RuntimeError("allocation_snapshots is missing after schema init")


def _xdist_group_suffix(item) -> str | None:
    """Match pytest-xdist: sorted unique ``xdist_group`` names, joined by ``_``."""
    names: set[str] = set()
    for mark in item.iter_markers("xdist_group"):
        if mark.args:
            names.add(str(mark.args[0]))
        else:
            names.add(str(mark.kwargs.get("name", "default")))
    if not names:
        return None
    return "@" + "_".join(sorted(names))


def _ensure_xdist_group_nodeid(item) -> None:
    """Put the ``@group`` suffix on the node id xdist actually schedules.

    ``tryfirst`` adds the marker before xdist's own hook. That hook assigns
    ``item._nodeid``, which is the node id on pytest 8 and is ignored on
    pytest 9. When ``item._id`` exists, set it here so ``item.nodeid`` gains
    exactly one ``@db`` suffix. Pytest 8 is left to xdist.
    """
    suffix = _xdist_group_suffix(item)
    if suffix is None or item.nodeid.endswith(suffix):
        return
    if not hasattr(item, "_id"):
        return
    from _pytest.nodeid import NodeId

    item._id = NodeId.parse(item.nodeid + suffix)


@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config, items):
    """Auto-mark every test that uses ``clean_database`` as a ``db`` test.

    ``tryfirst`` so ``xdist_group`` is set before pytest-xdist copies the
    group name onto the node id (that is what keeps these tests on one
    worker under ``--dist loadgroup``).
    """
    for item in items:
        if "clean_database" in getattr(item, "fixturenames", ()):
            item.add_marker(pytest.mark.db)
        if item.get_closest_marker("db"):
            item.add_marker(pytest.mark.xdist_group("db"))
        if _is_xdist_worker(config):
            _ensure_xdist_group_nodeid(item)


@pytest.fixture(scope="session", autouse=True)
def _configure_test_database(request):
    global _DB_UNAVAILABLE_REASON
    os.environ["DATABASE_URL"] = _test_database_url()
    try:
        if _is_xdist_worker(request.config):
            _probe_test_database()
        else:
            error = getattr(request.config, "_rbm_db_error", None)
            if error is not None:
                _DB_UNAVAILABLE_REASON = _db_failure_summary(error)
    except Exception as exc:  # psycopg2.OperationalError and friends
        if _db_required():
            raise
        _DB_UNAVAILABLE_REASON = _db_failure_summary(exc)


@pytest.fixture(autouse=True)
def _skip_db_tests_without_postgres(request, _configure_test_database):
    if _DB_UNAVAILABLE_REASON and request.node.get_closest_marker("db"):
        pytest.skip(_DB_UNAVAILABLE_REASON)


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    if _DB_UNAVAILABLE_REASON:
        terminalreporter.write_line(
            "NOTE: Postgres was not reachable, so tests marked 'db' were skipped. "
            "CI runs them with a real database.",
            yellow=True,
        )


@pytest.fixture(scope="session")
def test_config():
    """Provides a loaded config instance for tests."""
    return Config()


@pytest.fixture
def clean_database():
    """Fresh PostgreSQL tables for each test."""
    if _DB_UNAVAILABLE_REASON:
        pytest.skip(_DB_UNAVAILABLE_REASON)
    truncate_tables()
    yield
    truncate_tables()
