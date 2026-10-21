"""Baseline consistency, picker guards, and run log isolation."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from optimizer.baseline import ctrl_settings_match, repeat_of_prior_lap
from optimizer.diagnose import diagnose
from optimizer.logbook import Logbook
from optimizer.loop import run_loop
from optimizer.evidence import measure_effects
from optimizer.picker import RulePicker, _reverts_confirmed_keep, _undoes_keep, Proposal
from tests.test_optimizer import lap_log
from optimizer.score import decide
from tests.test_optimizer import CONFIG, PARAMS, RunOutcome, ScriptedRunner, _layout, reference_then, summary_line


def test_decide_reason_names_kept_baseline():
    ref = diagnose(summary_line()).summary
    kept = diagnose(summary_line(err_pct="-18.0", ctrl_pivot_w_max_rad_s="0.60")).summary
    label = "kept baseline (pivot_w_max_rad_s=0.6)"
    decision = decide(kept, ref, baseline_label=label)
    assert label in decision.reason


def test_repeat_of_starting_settings_detected():
    ref = diagnose(summary_line()).summary
    again = diagnose(summary_line()).summary
    assert ctrl_settings_match(ref, again)
    label = repeat_of_prior_lap(
        again,
        starting_reference=ref,
        prior_laps=[("reference lap", ref)],
    )
    assert label == "starting settings (reference lap)"


def test_picker_blocks_undo_of_kept_pivot(tmp_path):
    history = [
        {
            "decision": "keep",
            "param": "controller.pivot_w_max_rad_s",
            "old": 0.9,
            "new": 0.6,
            "phase": "search",
        }
    ]
    params = dict(PARAMS)
    params["controller.pivot_w_max_rad_s"] = 0.6
    proposal = RulePicker().pick(diagnose(summary_line()), "", history, params)
    assert proposal is not None
    assert not (
        proposal.param == "controller.pivot_w_max_rad_s" and proposal.value == pytest.approx(0.9)
    )
    undo = Proposal(
        "controller.pivot_w_max_rad_s",
        0.9,
        "h",
        "e",
    )
    assert _undoes_keep(undo, {"param": "controller.pivot_w_max_rad_s", "old": 0.9, "new": 0.6})


def test_turn_delta_pp_uses_scoring_baseline_not_prior_trial():
    baseline = diagnose(
        lap_log(err_pct="-20.0", turn_errs=[-40.0, -40.0, -40.0], ctrl_pivot_w_max_rad_s="0.60")
    )
    after = diagnose(
        lap_log(err_pct="-18.0", turn_errs=[-35.0, -35.0, -35.0], ctrl_pivot_w_max_rad_s="0.60")
    )
    prior_trial = diagnose(
        lap_log(err_pct="-19.0", turn_errs=[-32.0, -32.0, -32.0], ctrl_pivot_w_max_rad_s="0.60")
    )
    effects = measure_effects(
        after,
        baseline.summary,
        baseline_diag=baseline,
        baseline_label="kept baseline (cruise=0.38)",
    )
    assert effects["turn_delta_pp"] == pytest.approx(5.0)
    wrong = measure_effects(after, baseline.summary, before=prior_trial)
    assert wrong["turn_delta_pp"] == pytest.approx(-3.0)


def test_picker_blocks_revert_toward_pre_keep_after_confirm():
    history = [
        {
            "phase": "confirm",
            "decision": "confirm-pass",
            "param": "controller.pivot_w_max_rad_s",
            "old": 0.9,
            "new": 0.6,
        },
        {
            "phase": "confirm",
            "decision": "confirm-pass",
            "param": "controller.cruise_m_s",
            "old": 0.44,
            "new": 0.38,
        },
    ]
    params = dict(PARAMS)
    params["controller.pivot_w_max_rad_s"] = 0.6
    params["controller.cruise_m_s"] = 0.38
    proposal = RulePicker().pick(diagnose(summary_line()), "", history, params)
    assert proposal is not None
    assert not (
        proposal.param == "controller.pivot_w_max_rad_s" and proposal.value == pytest.approx(0.9)
    )
    undo = Proposal("controller.pivot_w_max_rad_s", 0.9, "h", "e")
    assert _reverts_confirmed_keep(undo, history)


def test_logbook_begin_run_clears_prior_jsonl(tmp_path):
    book = Logbook(tmp_path / "runs")
    book.begin_run("run-a")
    book.append({"iteration": 1, "decision": "reference"})
    book.begin_run("run-b")
    book.append({"iteration": 0, "decision": "reference", "run_id": "run-b"})
    lines = (tmp_path / "runs" / "results.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1
    assert json.loads(lines[0])["run_id"] == "run-b"


