# Agent journal

Plain-language notes appended after each overnight optimizer run. Structured lessons stay in `optimizer/KNOWLEDGE.md`.
## 2026-10-21 run_id=r20261021015621 (2026-10-21 01:56 UTC)

What was hard: launches stayed clean; stamps and summaries matched each trial.
What we learned: controller.pivot_w_max_rad_s → 0.6: |err| +10.30 pp vs baseline, Wh/m +9.1%; Confirmed controller.pivot_w_max_rad_s at 0.6 (was 0.9)..
Where we were wrong: nothing major flagged in prediction_miss or invalid rows.
Next time: From that lesson, try `controller.k_steer` at 2.5 rather than repeating the step that just finished. Model drive is 89% viscous damping and err_pct is -28.3%. Negative means the model is high. Viscous watt-hours grow with speed, while turn scrub on a gentle turn is min(mu*m*g/2, .
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) (controller reported controller.k_steer=2.2 but config had 2.5).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Invalid: controller reported controller.k_steer=2.2 but config had 2.5
Next time: Retry `controller.k_steer` at 2.5 once more before trying another knob or direction. The last lap was invalid (missing summary, limits, or body mismatch)..
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) ([redacted detail]).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Invalid: [redacted detail]
Next time: Retry `controller.pivot_w_max_rad_s` at 0.6 once more before trying another knob or direction. The last lap was invalid (missing summary, limits, or body mismatch)..
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: launches stayed clean; stamps and summaries matched each trial.
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: nothing major flagged in prediction_miss or invalid rows.
Next time: From that lesson, try `controller.pivot_w_max_rad_s` at 0.6 rather than repeating the step that just finished. Turn segments are further off than straights (turn err None, straight err None). Turn scrub force is min(mu*m*g/2, v_slip/FDS), so on a gentle turn mass barely matters a.
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: launches stayed clean; stamps and summaries matched each trial.
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: nothing major flagged in prediction_miss or invalid rows.
Next time: From that lesson, try `controller.cruise_m_s` at 0.38 rather than repeating the step that just finished. Model drive is 85% viscous damping and err_pct is -28.3%. Negative means the model is high. Viscous watt-hours grow with speed, while turn scrub on a gentle turn is min(mu*m*g.
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: launches stayed clean; stamps and summaries matched each trial.
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: nothing major flagged in prediction_miss or invalid rows.
Next time: From that lesson, try `controller.k_steer` at 2.5 rather than repeating the step that just finished. Model drive is 89% viscous damping and err_pct is -28.3%. Negative means the model is high. Viscous watt-hours grow with speed, while turn scrub on a gentle turn is min(mu*m*g/2, .
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) (controller reported controller.pivot_w_max_rad_s=0.6 but config had 0.9).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Invalid: controller reported controller.pivot_w_max_rad_s=0.6 but config had 0.9
Next time: Retry `controller.k_steer` at 2.5 once more before trying another knob or direction. The last lap was invalid (missing summary, limits, or body mismatch)..
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) (controller reported controller.cruise_m_s=0.44 but config had 0.38).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Predicted a smaller |err| miss vs starting settings (cruise_m_s=0.44, straight_m_s=0.5, pivot_w_max_rad_s=0.9, k_steer=2); |err| worsened by 1.2 pp instead. Invalid: controller reported controller.cruise_m_s=0.44 but config had 0.38
Next time: Retry `controller.cruise_m_s` at 0.38 once more before trying another knob or direction. The last lap was invalid (missing summary, limits, or body mismatch)..
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) (controller reported controller.cruise_m_s=0.4 but config had 0.44).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Invalid: controller reported controller.cruise_m_s=0.4 but config had 0.44
Next time: Retry `controller.pivot_w_max_rad_s` at 0.6 once more before trying another knob or direction. The last lap was invalid (missing summary, limits, or body mismatch)..
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) (lap build default does not match the fixed run body light; the reference lap must be on the same body).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Invalid: lap build default does not match the fixed run body light; the reference lap must be on the same body
Next time: Retry `controller.pivot_w_max_rad_s` at 0.6 once more before trying another knob or direction. The last lap was invalid (missing summary, limits, or body mismatch)..
## 2026-10-21 run_id=r20261021015622 (2026-10-21 01:56 UTC)

What was hard: launches stayed clean; stamps and summaries matched each trial.
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: nothing major flagged in prediction_miss or invalid rows.
Next time: Continue from the last kept baseline on the next run..
## 2026-10-21 run_id=r20261021015623 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) (lap had no MISSION_SUMMARY / did not finish; no metrics from this lap were scored).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Invalid: lap had no MISSION_SUMMARY / did not finish; no metrics from this lap were scored
Next time: From that lesson, try `controller.k_steer` at 2.5 rather than repeating the step that just finished. Model drive is 89% viscous damping and err_pct is -28.3%. Negative means the model is high. Viscous watt-hours grow with speed, while turn scrub on a gentle turn is min(mu*m*g/2, .
## 2026-10-21 run_id=r20261021015623 (2026-10-21 01:56 UTC)

What was hard: 1 invalid lap(s) (crash: retried after crash. launcher exited 1 before MISSION_SUMMARY launcher exited 1 before MISSION_SUMMARY).
What we learned: No new keeps; the run mostly held starting settings..
Where we were wrong: Invalid: crash: retried after crash. launcher exited 1 before MISSION_SUMMARY launcher exited 1 before MISSION_SUMMARY
Next time: Retry `controller.pivot_w_max_rad_s` at 0.6 once more before trying another knob or direction. The last lap was invalid (missing summary, limits, or body mismatch)..
