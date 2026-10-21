"""Repo docs that tests must not append to."""

from __future__ import annotations

from pathlib import Path

import pytest

from optimizer.loop import run_loop
from tests.test_optimizer import CONFIG, RunOutcome, ScriptedRunner, _layout, reference_then, summary_line

_REPO_ROOT = Path(__file__).resolve().parents[1]
_AGENT_JOURNAL = _REPO_ROOT / "docs" / "AGENT_JOURNAL.md"
_KNOWLEDGE = _REPO_ROOT / "optimizer" / "KNOWLEDGE.md"


@pytest.fixture(scope="session", autouse=True)
def _repo_learning_docs_unmodified():
    """Fail the session if the suite writes into tracked learning docs."""
    before = {path: (path.read_bytes() if path.is_file() else None) for path in (_AGENT_JOURNAL, _KNOWLEDGE)}
    yield
    for path, data in before.items():
        rel = path.relative_to(_REPO_ROOT)
        if data is None:
            assert not path.is_file(), f"{rel} was created during tests"
        else:
            assert path.read_bytes() == data, f"{rel} was modified during the test suite"


def test_run_loop_does_not_append_agent_journal_without_path(tmp_path):
    config, knowledge, run_dir = _layout(tmp_path)
    before = _AGENT_JOURNAL.read_bytes()
    runner = ScriptedRunner(
        reference_then(RunOutcome("ok", summary_line(err_pct="-27.0", ctrl_pivot_w_max_rad_s="0.60")))
    )
    run_loop(
        config_path=config,
        knowledge_path=knowledge,
        run_dir=run_dir,
        runner=runner,
        max_iterations=1,
        max_hours=8,
        lap_timeout_s=5,
        initial_log=summary_line(),
    )
    assert _AGENT_JOURNAL.read_bytes() == before


def test_repo_agent_journal_and_knowledge_unchanged():
    """Session autouse fixture ``_repo_learning_docs_unmodified`` enforces this."""
