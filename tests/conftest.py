import os
import pytest
from src.config import Config
from src.database import init_db, truncate_tables


def _test_database_url() -> str:
    return os.getenv(
        "TEST_DATABASE_URL",
        "postgresql://robot:robot@localhost:5432/robot_battery_test",
    )


@pytest.fixture(scope="session", autouse=True)
def _configure_test_database():
    os.environ["DATABASE_URL"] = _test_database_url()
    init_db()


@pytest.fixture(scope="session")
def test_config():
    """Provides a loaded config instance for tests."""
    return Config()


@pytest.fixture
def clean_database():
    """Fresh PostgreSQL tables for each test."""
    truncate_tables()
    yield
    truncate_tables()