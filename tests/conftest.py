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
"""
import os

import pytest
from src.config import Config
from src.database import init_db, truncate_tables

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


def pytest_collection_modifyitems(config, items):
    """Auto-mark every test that uses ``clean_database`` as a ``db`` test."""
    for item in items:
        if "clean_database" in getattr(item, "fixturenames", ()):
            item.add_marker(pytest.mark.db)


@pytest.fixture(scope="session", autouse=True)
def _configure_test_database():
    global _DB_UNAVAILABLE_REASON
    os.environ["DATABASE_URL"] = _test_database_url()
    try:
        init_db()
    except Exception as exc:  # psycopg2.OperationalError and friends
        if _db_required():
            raise
        first_line = str(exc).strip().splitlines()[0] if str(exc).strip() else type(exc).__name__
        _DB_UNAVAILABLE_REASON = (
            f"Postgres test database not reachable ({first_line}). "
            "Start it to run DB tests (see README, setup B step 4)."
        )


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
