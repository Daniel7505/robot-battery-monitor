"""Optimizer logic that does not need Webots: diagnose, score, log, restore."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from optimizer.config_store import ConfigStore
from optimizer.diagnose import diagnose
from optimizer.logbook import Logbook
from optimizer.loop import recover_session, run_loop
from optimizer.picker import LlmPicker, RulePicker
from optimizer.runner import DryRunRunner, RunOutcome, launch_command
from optimizer.score import confirmation_ok, decide
from src.controller_limits import ALLOWLIST, KNOBS, AllowlistError, check_param, limits_from_mapping
from src.lane_keep import DEFAULT_K_STEER
from src.rowfit_control import (
    PIVOT_W_MAX,
    V_CRUISE_M_S,
    V_STRAIGHT_M_S,
    RowfitController,
    curve_speed,
)
from src.controller_limits import load_controller_limits

CONFIG = """\
# keep this comment
bom_build: light

# Rowfit speeds
controller:
  cruise_m_s: 0.44
  straight_m_s: 0.50
  pivot_w_max_rad_s: 0.9
  k_steer: 2.0
"""

PARAMS = {
    "bom_build": "light",
    "controller.cruise_m_s": 0.44,
    "controller.straight_m_s": 0.50,
    "controller.pivot_w_max_rad_s": 0.9,
    "controller.k_steer": 2.0,
}


def summary_line(**overrides) -> str:
    fields = {
        "build": "light",
        "duration_s": "662.0",
        "dist_m": "200.91",
        "turns": "13",
        "payload_kg": "0",
        "soc_start": "100.00",
        "soc_end": "97.68",
        "wh_model": "7.054",
        "wh_model_idle": "1.832",
        "wh_model_drive": "5.222",
        "wh_model_roll": "0.438",
        "wh_model_visc": "4.667",
        "wh_model_turn": "0.110",
        "wh_model_accel": "0.007",
        "wh_measured_drive_shaft": "0.866",
        "wh_measured_drive_elec": "3.742",
        "eta_avg": "0.232",
        "wh_measured": "5.556",
        "wh_measured_soc": "5.568",
        "err_pct": "-28.3",
    }
    fields.update({key: str(value) for key, value in overrides.items()})
    body = " ".join(f"{key}={value}" for key, value in fields.items())
    return f"MISSION_SUMMARY {body}"


def segment(heading: float, err: float, speed: float) -> str:
    return (
        "SEGMENT_ENERGY build=light dist_m=3.048 dt_s=10.00 "
        f"avg_speed={speed:.3f} heading_change_deg={heading:.1f} "
        f"err_pct={err:.1f} wh_model_drive=0.0800 wh_model_roll=0.0100 "
        "wh_model_visc=0.0600 wh_model_turn=0.0100 wh_model_accel=0.0000 "
        "wh_measured_drive_elec=0.0700 wh_measured_drive_shaft=0.0200"
    )


class ScriptedRunner:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = 0
        self.restarts = 0

    def restart_dashboard(self) -> str:
        self.restarts += 1
        return "restarted robot-battery-dashboard"

    def run_lap(self, *, params, timeout_s, world="warehouse", log_path=None):
        del params, timeout_s, world, log_path
        self.calls += 1
        item = self.outcomes.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


def _layout(tmp_path: Path) -> tuple[Path, Path, Path]:
    config = tmp_path / "config.yaml"
    config.write_text(CONFIG, encoding="utf-8")
    knowledge = tmp_path / "KNOWLEDGE.md"
    knowledge.write_text(Path("optimizer/KNOWLEDGE.md").read_text(encoding="utf-8"), encoding="utf-8")
    run_dir = tmp_path / "runs"
    return config, knowledge, run_dir


def test_diagnose_docker_line_and_energy_shares():
    line = (
        "2026-10-14 23:01:02,123 | INFO     | robot_monitor | "
        + summary_line(build="heavy", err_pct="-5.0")
    )
    diag = diagnose(line)
    assert diag.summary is not None
    assert diag.summary.build == "heavy"
    assert diag.summary.err_pct == pytest.approx(-5.0)
    assert diag.summary.wh_measured_drive_shaft == pytest.approx(0.866)
    assert diag.share("visc") == pytest.approx(4.667 / 5.222)
    assert any("viscous" in note or "visc" in note for note in diag.notes)


def test_diagnose_uses_the_last_mission_and_keeps_each_build():
    text = "\n".join(
        [
            summary_line(build="heavy", err_pct="-5.0", wh_measured_drive_shaft="0.866"),
            segment(5, -10, 0.30),
            segment(90, -40, 0.15),
            summary_line(),
        ]
    )
    diag = diagnose(text)
    assert diag.summary.build == "light"
    assert diag.summary.err_pct == pytest.approx(-28.3)
    assert diag.per_build["heavy"].err_pct == pytest.approx(-5.0)
    assert len(diag.segments) == 2
    assert diag.turn_err == pytest.approx(-40.0)
    assert diag.straight_err == pytest.approx(-10.0)


def test_score_keeps_a_closer_model_and_rolls_back_a_worse_one():
    ref = diagnose(summary_line()).summary
    better = diagnose(
        summary_line(err_pct="-18.0", wh_measured_drive_elec="3.400", wh_model_drive="4.146")
    ).summary
    worse = diagnose(summary_line(err_pct="-40.0", wh_measured_drive_elec="3.900")).summary
    short = diagnose(summary_line(dist_m="40.0")).summary
    missing = diagnose(summary_line(err_pct="na")).summary
    assert decide(better, ref).action == "keep"
    assert decide(worse, ref).action == "rollback"
    assert decide(short, ref).action == "invalid"
    assert decide(missing, ref).action == "invalid"
    assert decide(ref, None).action == "baseline"


def test_confirmation_passes_a_sane_other_build_and_fails_a_wild_one():
    provisional = diagnose(summary_line(err_pct="-18.0")).summary
    ok = diagnose(summary_line(build="heavy", err_pct="-5.0")).summary
    wild = diagnose(summary_line(build="heavy", err_pct="-55.0")).summary
    repeat = diagnose(summary_line(err_pct="-19.0")).summary
    drift = diagnose(summary_line(err_pct="-30.0")).summary
    passed, _why = confirmation_ok(provisional, ok, same_build=False)
    failed, _why = confirmation_ok(provisional, wild, same_build=False)
    same, _why = confirmation_ok(provisional, repeat, same_build=True)
    moved, _why = confirmation_ok(provisional, drift, same_build=True)
    assert passed and same
    assert not failed and not moved


def test_allowlist_rejects_the_energy_model_and_out_of_range_speeds():
    with pytest.raises(AllowlistError):
        check_param("src/twin_energy.py", 1)
    with pytest.raises(AllowlistError):
        check_param("controller.cruise_m_s", 2.0)
    assert "src/twin_energy.py" not in ALLOWLIST
    limits = limits_from_mapping({"controller": {"cruise_m_s": 9}})
    assert limits.cruise_m_s == pytest.approx(0.44)
    assert limits.warnings


def test_config_edit_keeps_comments_and_restores(tmp_path):
    config, _knowledge, _run_dir = _layout(tmp_path)
    store = ConfigStore(config)
    original = config.read_text(encoding="utf-8")
    store.apply_one("controller.straight_m_s", 0.48)
    edited = config.read_text(encoding="utf-8")
    assert "keep this comment" in edited
    assert "straight_m_s: 0.48" in edited
    assert "bom_build: light" in edited
    store.write_params(PARAMS)
    assert config.read_text(encoding="utf-8") == original


def test_logbook_is_append_only_and_redacts_secrets(tmp_path):
    book = Logbook(tmp_path / "runs")
    book.append({"iteration": 1, "phase": "search", "note": "ok", "decision": "keep"})
    book.append({"iteration": 2, "phase": "search", "note": "token=supersecret", "decision": "rollback"})
    lines = (tmp_path / "runs" / "results.jsonl").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert "supersecret" not in lines[1]
    assert "[REDACTED]" in lines[1]
    csv_text = (tmp_path / "runs" / "results.csv").read_text(encoding="utf-8")
    assert csv_text.count("\n") >= 3
    assert "supersecret" not in csv_text


def test_rule_picker_slows_a_visc_heavy_high_model():
    proposal = RulePicker().pick(diagnose(summary_line()), "", [], PARAMS)
    assert proposal is not None
    assert proposal.param == "controller.straight_m_s"
    assert proposal.value == pytest.approx(0.48)
    assert "v_slip" in proposal.hypothesis
    assert proposal.code_suggestions


def test_rule_picker_steps_turn_rate_when_turns_are_the_miss():
    text = "\n".join(
        [
            segment(5, -10, 0.30),
            segment(8, -12, 0.40),
            segment(6, -11, 0.22),
            segment(4, -9, 0.35),
            segment(90, -40, 0.15),
            segment(70, -36, 0.12),
            summary_line(),
        ]
    )
    proposal = RulePicker().pick(diagnose(text), "", [], PARAMS)
    assert proposal is not None
    assert proposal.param == "controller.pivot_w_max_rad_s"
    assert proposal.value == pytest.approx(0.8)


def test_rule_picker_does_not_repeat_a_rolled_back_value():
    history = [
        {
            "param": "controller.straight_m_s",
            "new": 0.48,
            "decision": "rollback",
            "phase": "search",
        }
    ]
    proposal = RulePicker().pick(diagnose(summary_line()), "", history, PARAMS)
    assert proposal is not None
    assert not (proposal.param == "controller.straight_m_s" and proposal.value == pytest.approx(0.48))
    assert proposal.param in ALLOWLIST


def test_llm_picker_rejects_a_file_edit_and_accepts_an_allowlisted_value(tmp_path):
    bad = tmp_path / "bad.py"
    bad.write_text(
        "import json,sys\n"
        "json.dump({'param':'src/twin_energy.py','value':1,"
        "'hypothesis':'h','expected_effect':'e','patch':'nope'}, sys.stdout)\n",
        encoding="utf-8",
    )
    rejected = LlmPicker(f'"{sys.executable}" "{bad}"').pick(
        diagnose(summary_line()), "notes", [], PARAMS
    )
    assert rejected is not None
    assert rejected.param in KNOBS
    assert "file edit" in rejected.note

    good = tmp_path / "good.py"
    good.write_text(
        "import json,sys\n"
        "json.dump({'param':'controller.cruise_m_s','value':0.42,"
        "'hypothesis':'slower cruise','expected_effect':'lower viscous Wh'}, sys.stdout)\n",
        encoding="utf-8",
    )
    accepted = LlmPicker(f'"{sys.executable}" "{good}"').pick(
        diagnose(summary_line()), "notes", [], PARAMS
    )
    assert accepted is not None
    assert accepted.param == "controller.cruise_m_s"
    assert accepted.value == pytest.approx(0.42)
    assert accepted.note == "picked by OPTIMIZER_LLM_CMD"


def test_loop_keeps_then_confirms_and_restores_config(tmp_path):
    config, knowledge, run_dir = _layout(tmp_path)
    better = summary_line(err_pct="-18.0", wh_measured_drive_elec="3.400", wh_model_drive="4.146")
    held = summary_line(build="heavy", err_pct="-5.0", wh_measured_drive_shaft="0.866")
    runner = ScriptedRunner([RunOutcome("ok", better), RunOutcome("ok", held)])
    report = run_loop(
        config_path=config,
        knowledge_path=knowledge,
        run_dir=run_dir,
        runner=runner,
        max_iterations=2,
        max_hours=8,
        lap_timeout_s=5,
        initial_log=summary_line(),
    )
    text = report.read_text(encoding="utf-8")
    assert "put back" in text
    assert "confirm-pass" in text
    assert "not applied" in text.lower() or "Suggestions" in text
    assert "keep this comment" in config.read_text(encoding="utf-8")
    assert load_controller_limits(config).straight_m_s == pytest.approx(0.50)
    rows = [
        json.loads(line)
        for line in (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [row["decision"] for row in rows] == ["keep", "confirm-pass"]
    assert "held-out" in rows[0]["postmortem"]["next_hypothesis"]
    assert "Predicted versus observed" in text
    learned = knowledge.read_text(encoding="utf-8")
    assert "## Known facts" in learned
    assert "Hypothesis:" in learned
    assert runner.restarts >= 1
    session = json.loads((run_dir / "session.json").read_text(encoding="utf-8"))
    assert session["in_progress"] is False


def test_loop_rolls_back_a_worse_lap_and_tries_something_else(tmp_path):
    config, knowledge, run_dir = _layout(tmp_path)
    worse = summary_line(err_pct="-40.0", wh_measured_drive_elec="4.200")
    still = summary_line(err_pct="-27.0", wh_measured_drive_elec="3.700", wh_model_drive="5.068")
    runner = ScriptedRunner([RunOutcome("ok", worse), RunOutcome("ok", still)])
    run_loop(
        config_path=config,
        knowledge_path=knowledge,
        run_dir=run_dir,
        runner=runner,
        max_iterations=2,
        max_hours=8,
        lap_timeout_s=5,
        initial_log=summary_line(),
    )
    rows = [
        json.loads(line)
        for line in (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert rows[0]["decision"] == "rollback"
    assert rows[1]["param"] != rows[0]["param"] or rows[1]["new"] != rows[0]["new"]
    assert load_controller_limits(config).straight_m_s == pytest.approx(0.50)


def test_loop_crash_is_retried_rolled_back_and_redacted(tmp_path):
    config, knowledge, run_dir = _layout(tmp_path)

    class Boom:
        def restart_dashboard(self):
            return "restarted robot-battery-dashboard"

        def run_lap(self, **kwargs):
            del kwargs
            raise RuntimeError("webots died token=supersecret")

    run_loop(
        config_path=config,
        knowledge_path=knowledge,
        run_dir=run_dir,
        runner=Boom(),
        max_iterations=1,
        max_hours=8,
        lap_timeout_s=5,
        initial_log=summary_line(),
    )
    raw = (run_dir / "results.jsonl").read_text(encoding="utf-8")
    learned = knowledge.read_text(encoding="utf-8")
    assert "supersecret" not in raw
    assert "supersecret" not in learned
    assert "[REDACTED]" in raw
    row = json.loads(raw.splitlines()[0])
    assert row["decision"] == "rollback"
    assert load_controller_limits(config).cruise_m_s == pytest.approx(0.44)


def test_loop_stops_on_the_wall_clock(tmp_path):
    config, knowledge, run_dir = _layout(tmp_path)

    class Clock:
        def __init__(self):
            self.t = 0.0

        def __call__(self):
            return self.t

    clock = Clock()

    class Advance(ScriptedRunner):
        def run_lap(self, **kwargs):
            clock.t += 3 * 3600
            return super().run_lap(**kwargs)

    runner = Advance([RunOutcome("ok", summary_line(err_pct="-40.0"))])
    report = run_loop(
        config_path=config,
        knowledge_path=knowledge,
        run_dir=run_dir,
        runner=runner,
        max_iterations=5,
        max_hours=1,
        lap_timeout_s=5,
        initial_log=summary_line(),
        monotonic=clock,
    )
    assert "wall-clock" in report.read_text(encoding="utf-8")
    assert runner.calls == 1


def test_recover_session_puts_a_crashed_edit_back(tmp_path):
    config, _knowledge, run_dir = _layout(tmp_path)
    store = ConfigStore(config)
    store.apply_one("controller.cruise_m_s", 0.40)
    run_dir.mkdir()
    (run_dir / "session.json").write_text(
        json.dumps({"in_progress": True, "snapshot": PARAMS}),
        encoding="utf-8",
    )
    recover_session(store, run_dir / "session.json")
    assert load_controller_limits(config).cruise_m_s == pytest.approx(0.44)


def test_launch_command_uses_the_existing_windows_launcher():
    argv = launch_command(Path("."), "warehouse")
    assert argv[0] == "powershell"
    assert argv[-1] == "warehouse"
    assert argv[-2] == "-World"
    assert argv[argv.index("-File") + 1].endswith("launch_webots_twin.ps1")


def test_dry_run_cli_writes_a_morning_report(tmp_path):
    from optimizer.__main__ import main

    config, knowledge, run_dir = _layout(tmp_path)
    initial = tmp_path / "last.log"
    initial.write_text(summary_line(), encoding="utf-8")
    code = main(
        [
            "--dry-run",
            "--max-iterations",
            "2",
            "--max-hours",
            "1",
            "--lap-timeout-min",
            "1",
            "--config",
            str(config),
            "--knowledge",
            str(knowledge),
            "--run-dir",
            str(run_dir),
            "--initial-log",
            str(initial),
        ]
    )
    assert code == 0
    report = (run_dir / "morning.md").read_text(encoding="utf-8")
    assert "dry-run" in report
    assert "put back" in report
    assert load_controller_limits(config).straight_m_s == pytest.approx(0.50)


def test_cli_refuses_a_real_lap_off_windows():
    from optimizer.__main__ import main

    if sys.platform.startswith("win"):
        pytest.skip("this check is for machines without Webots launch")
    assert main(["--max-iterations", "1", "--max-hours", "1", "--lap-timeout-min", "1"]) == 2


def test_repo_picker_does_not_embed_an_api_key():
    for path in Path("optimizer").rglob("*"):
        if path.suffix not in {".py", ".md", ".ps1"}:
            continue
        text = path.read_text(encoding="utf-8")
        assert "sk-" not in text
        assert "BEGIN PRIVATE" not in text
        assert "git commit" not in text


def test_shipped_limits_match_the_rowfit_constants():
    limits = load_controller_limits()
    assert limits.cruise_m_s == pytest.approx(V_CRUISE_M_S)
    assert limits.straight_m_s == pytest.approx(V_STRAIGHT_M_S)
    assert limits.pivot_w_max_rad_s == pytest.approx(PIVOT_W_MAX)
    assert limits.k_steer == pytest.approx(DEFAULT_K_STEER)
    assert limits.bom_build is None


def test_rowfit_override_does_not_move_the_module_defaults():
    ctl = RowfitController(v_cruise=0.36, v_straight=0.42, pivot_w_max=0.5)
    assert curve_speed(0.0) == pytest.approx(V_CRUISE_M_S)
    assert curve_speed(0.0, v_cruise=0.36) == pytest.approx(0.36)
    ctl.reset()
    assert ctl.v_cruise == pytest.approx(0.36)
    assert ctl.v_straight == pytest.approx(0.42)
    assert ctl.pivot_w_max == pytest.approx(0.5)


def test_dry_runner_is_not_a_measurement():
    outcome = DryRunRunner().run_lap(params=PARAMS, timeout_s=1)
    assert "MISSION_SUMMARY" in outcome.log_text
    assert "dry-run" in outcome.note


class _QuietDocker:
    returncode = 0
    stdout = "dashboard ready\n"
    stderr = ""


def test_reads_mission_summary_json_when_docker_is_quiet(tmp_path, monkeypatch):
    """Main writes the lap to mission_summary_*.json; docker often never sees it."""
    from datetime import datetime, timedelta

    from optimizer import runner

    monkeypatch.setenv("RBM_LOG_DIR", str(tmp_path))
    payload = {"source": "controller", "lines": [summary_line(build="heavy", err_pct="-5.0")]}
    (tmp_path / "mission_summary_lap.json").write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(runner.subprocess, "run", lambda *args, **kwargs: _QuietDocker())
    text, note = runner.read_dashboard_log(since_local=datetime.now() - timedelta(minutes=5))
    diag = diagnose(text)
    assert diag.summary is not None
    assert diag.summary.build == "heavy"
    assert diag.summary.err_pct == pytest.approx(-5.0)
    assert "mission_summary" in note


def test_ignores_a_mission_summary_json_from_before_the_lap(tmp_path, monkeypatch):
    import os
    from datetime import datetime, timedelta

    from optimizer import runner

    monkeypatch.setenv("RBM_LOG_DIR", str(tmp_path))
    path = tmp_path / "mission_summary_old.json"
    path.write_text(json.dumps({"lines": [summary_line(build="budget")]}), encoding="utf-8")
    old = datetime.now() - timedelta(hours=2)
    os.utime(path, (old.timestamp(), old.timestamp()))
    monkeypatch.setattr(runner.subprocess, "run", lambda *args, **kwargs: _QuietDocker())
    text, note = runner.read_dashboard_log(since_local=datetime.now() - timedelta(minutes=5))
    assert "MISSION_SUMMARY" not in text
    assert note == ""


def test_identical_metrics_score_one_and_say_the_change_may_not_have_landed():
    reference = diagnose(summary_line()).summary
    decision = decide(reference, reference)
    assert decision.action == "rollback"
    assert decision.score == pytest.approx(1.0)
    assert decision.score > 0.98
    assert "may not have taken effect" in decision.reason


def test_text_capturing_subprocesses_use_utf8():
    """Windows cp1252 turns a UTF-8 arrow (byte 0x90) into UnicodeDecodeError."""
    import ast

    offenders = []
    for root in (Path("optimizer"), Path("scripts")):
        for path in root.rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                func = node.func
                if not isinstance(func, ast.Attribute) or func.attr not in {"run", "Popen"}:
                    continue
                if not isinstance(func.value, ast.Name) or func.value.id != "subprocess":
                    continue
                keywords = {kw.arg: kw.value for kw in node.keywords if kw.arg}
                text = keywords.get("text") or keywords.get("universal_newlines")
                if not (isinstance(text, ast.Constant) and text.value is True):
                    continue
                encoding = keywords.get("encoding")
                errors = keywords.get("errors")
                ok_enc = isinstance(encoding, ast.Constant) and encoding.value == "utf-8"
                ok_err = isinstance(errors, ast.Constant) and errors.value == "replace"
                if not (ok_enc and ok_err):
                    offenders.append(f"{path}:{node.lineno} subprocess.{func.attr}")
    assert offenders == []


def test_picker_does_not_repeat_a_refuted_slowdown():
    knowledge = "Lesson: viscous loss did not move. [refuted controller.straight_m_s down]\n"
    proposal = RulePicker().pick(diagnose(summary_line()), knowledge, [], PARAMS)
    assert proposal is not None
    assert proposal.param == "controller.cruise_m_s"
    assert proposal.value == pytest.approx(0.42)


def test_flat_lap_is_inconclusive_and_the_next_knob_is_cruise(tmp_path):
    config, knowledge, run_dir = _layout(tmp_path)
    same = summary_line()
    moved = summary_line(err_pct="-26.0", wh_measured_drive_elec="3.600", duration_s="700")
    runner = ScriptedRunner([RunOutcome("ok", same), RunOutcome("ok", moved)])
    report = run_loop(
        config_path=config,
        knowledge_path=knowledge,
        run_dir=run_dir,
        runner=runner,
        max_iterations=2,
        max_hours=8,
        lap_timeout_s=5,
        initial_log=same,
    )
    rows = [
        json.loads(line)
        for line in (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert rows[0]["param"] == "controller.straight_m_s"
    assert rows[0]["new"] == pytest.approx(0.48)
    assert rows[0]["decision"] == "rollback"
    assert rows[0]["score"] == pytest.approx(1.0)
    assert "may not have taken effect" in rows[0]["reason"]
    post = rows[0]["postmortem"]
    assert post["flat"] is True
    assert post["verdict"] == "inconclusive"
    assert "CONTROLLER LIMITS" in post["mechanism"]
    assert post["tag"] == "[inconclusive controller.straight_m_s down]"
    assert "refuted" not in post["tag"]
    assert "cruise_m_s" in post["next_hypothesis"]
    assert rows[1]["param"] == "controller.cruise_m_s"
    morning = report.read_text(encoding="utf-8")
    assert "### Trial 1" in morning
    assert "Predicted versus observed" in morning
    assert "Lesson:" in morning
    assert "Next hypothesis:" in morning
    learned = knowledge.read_text(encoding="utf-8")
    assert "[inconclusive controller.straight_m_s down]" in learned
    assert "Next hypothesis:" in learned


def test_a_real_miss_refutes_slowing_the_straight(tmp_path):
    config, knowledge, run_dir = _layout(tmp_path)
    # err moves, lap time moves, viscous Wh stays put. Not a flat lap.
    missed = summary_line(err_pct="-29.5", duration_s="700")
    runner = ScriptedRunner([
        RunOutcome("ok", missed),
        RunOutcome("ok", summary_line(err_pct="-27.0", duration_s="640")),
    ])
    report = run_loop(
        config_path=config,
        knowledge_path=knowledge,
        run_dir=run_dir,
        runner=runner,
        max_iterations=2,
        max_hours=8,
        lap_timeout_s=5,
        initial_log=summary_line(),
    )
    rows = [
        json.loads(line)
        for line in (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    post = rows[0]["postmortem"]
    assert rows[0]["param"] == "controller.straight_m_s"
    assert rows[0]["decision"] == "rollback"
    assert post["flat"] is False
    assert post["verdict"] == "refuted"
    assert "did not cut viscous Wh enough" in post["mechanism"]
    assert "less speed-sensitive" in post["mechanism"]
    assert post["tag"] == "[refuted controller.straight_m_s down]"
    assert rows[1]["param"] == "controller.cruise_m_s"
    assert not (rows[1]["param"] == "controller.straight_m_s")
    morning = report.read_text(encoding="utf-8")
    assert "Lowering straight speed did not cut viscous Wh enough" in morning
    learned = knowledge.read_text(encoding="utf-8")
    assert "[refuted controller.straight_m_s down]" in learned


def test_rowfit_reset_keeps_the_track_and_the_speed_knobs():
    ctl = RowfitController(v_cruise=0.36, v_straight=0.42, pivot_w_max=0.5)
    track = object()
    ctl.set_track(track)
    ctl.reset()
    assert ctl.track is track
    assert ctl.v_cruise == pytest.approx(0.36)
    assert ctl.v_straight == pytest.approx(0.42)
    assert ctl.pivot_w_max == pytest.approx(0.5)
