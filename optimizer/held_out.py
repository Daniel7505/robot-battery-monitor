"""Why cross-build confirm is off, and what we run instead."""

from __future__ import annotations

# Oct 13 (main f00326c): heavy and light warehouse laps both reported
# wh_measured_drive_shaft ≈ 0.866 Wh because the Webots hinge/contact stack
# still used the shipped 0.08 m wheel. Only the twin energy model and
# (today) the body solid mass follow bom_build; catalog wheel radius is not
# on the contact cylinder (see src/webots_body.py).
CROSS_BUILD_HELD_OUT_ENABLED = False


def cross_build_confirm_disabled_reason() -> str:
    return (
        "Cross-build (heavy) held-out confirm is disabled. Measured shaft Wh still "
        "tracks the same 0.08 m contact cylinder and hinge physics as light, while "
        "MISSION_SUMMARY drive Wh/m follows the selected BoM energy model. A heavy "
        "lap therefore does not isolate the kept controller knobs on real robot "
        "physics. Re-enable after the sim applies BoM mass and catalog wheel radius "
        "on the contact cylinder (separate change; see docs/baselines and "
        "src/webots_body.py)."
    )


def confirm_strategy_note() -> str:
    return (
        "After a tentative keep, the loop runs two fresh laps on the fixed run body: "
        "once at the pre-keep knob value and once at the kept value. The keep is "
        "confirmed only if the kept lap beats the pre-keep lap by the evidence "
        "threshold (|err|, Wh/m, or turn segment). Each lap must carry a matching "
        "opt_run_id/opt_trial_id stamp and a new mission_summary JSON mtime."
    )
