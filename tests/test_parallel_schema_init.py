"""Schema init must happen once, in the xdist controller, not on every worker."""

import sys
from pathlib import Path

import pytest


def _conftest():
    for module in sys.modules.values():
        path = getattr(module, "__file__", None)
        if not path or not hasattr(module, "pytest_sessionstart"):
            continue
        file = Path(path)
        if file.name == "conftest.py" and file.parent.name == "tests":
            return module
    raise RuntimeError("tests/conftest.py is not loaded")


class _Config:
    def __init__(self, worker=False):
        if worker:
            self.workerinput = {"workerid": "gw0"}


class _Session:
    def __init__(self, worker=False):
        self.config = _Config(worker=worker)


def test_xdist_worker_does_not_build_the_schema(monkeypatch):
    conftest = _conftest()
    calls = []
    monkeypatch.setattr(conftest, "init_db", lambda: calls.append("init"))

    started = _Session(worker=True)
    conftest.pytest_sessionstart(started)

    assert calls == []
    assert not hasattr(started.config, "_rbm_db_error")


def test_controller_builds_the_schema(monkeypatch):
    conftest = _conftest()
    calls = []
    monkeypatch.setattr(conftest, "init_db", lambda: calls.append("init"))

    started = _Session(worker=False)
    conftest.pytest_sessionstart(started)

    assert calls == ["init"]
    assert started.config._rbm_db_error is None


def test_pytest9_nodeid_gets_one_db_suffix():
    from _pytest.nodeid import NodeId

    conftest = _conftest()

    class _Mark:
        args = ("db",)
        kwargs: dict = {}

    class _Item:
        def __init__(self):
            self._id = NodeId.parse("tests/test_basic.py::test_database_creates_table")

        @property
        def nodeid(self):
            return str(self._id)

        def iter_markers(self, name=None):
            return [_Mark()]

    item = _Item()
    conftest._ensure_xdist_group_nodeid(item)
    conftest._ensure_xdist_group_nodeid(item)
    assert item.nodeid == "tests/test_basic.py::test_database_creates_table@db"


def test_legacy_nodeid_is_left_for_xdist():
    conftest = _conftest()

    class _Mark:
        args = ("db",)
        kwargs: dict = {}

    class _Item:
        def __init__(self):
            self._nodeid = "tests/test_basic.py::test_database_creates_table"

        @property
        def nodeid(self):
            return self._nodeid

        def iter_markers(self, name=None):
            return [_Mark()]

    item = _Item()
    conftest._ensure_xdist_group_nodeid(item)
    assert item.nodeid == "tests/test_basic.py::test_database_creates_table"


def test_required_database_stops_the_session(monkeypatch):
    conftest = _conftest()

    def boom():
        raise RuntimeError("connection refused")

    monkeypatch.setattr(conftest, "init_db", boom)
    monkeypatch.setattr(conftest, "_db_required", lambda: False)

    skipped = _Session(worker=False)
    conftest.pytest_sessionstart(skipped)
    assert isinstance(skipped.config._rbm_db_error, RuntimeError)

    monkeypatch.setattr(conftest, "_db_required", lambda: True)
    with pytest.raises(pytest.exit.Exception) as excinfo:
        conftest.pytest_sessionstart(_Session(worker=False))
    assert excinfo.value.returncode == 1
    assert "connection refused" in excinfo.value.msg
