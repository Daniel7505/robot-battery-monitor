"""
Predictive energy estimation with short-horizon forecasting and mission awareness.
"""

from statistics import mean, pstdev

from src.mission_tasks import (
    TASK_PROFILES,
    LOCOMOTION_PHASES,
    anticipated_phases,
    predict_runtime,
    TICK_SECONDS,
)

HORIZON_30S = 30
HORIZON_60S = 60


class EnergyPredictor:
    """Forecasts draw, runtime, and mission feasibility with confidence scoring."""

    def __init__(self, window: int = 12):
        self._draw_history: list[float] = []
        self._window = window

    def update(self, draw_w: float) -> None:
        self._draw_history.append(draw_w)
        if len(self._draw_history) > self._window:
            self._draw_history.pop(0)

    def _ema_draw(self) -> float | None:
        if not self._draw_history:
            return None
        alpha = 0.35
        ema = self._draw_history[0]
        for v in self._draw_history[1:]:
            ema = alpha * v + (1 - alpha) * ema
        return round(ema, 1)

    def _trend_w_per_s(self) -> float:
        """Linear draw trend in W/s from recent samples."""
        n = len(self._draw_history)
        if n < 3:
            return 0.0
        span_s = (n - 1) * TICK_SECONDS
        delta = self._draw_history[-1] - self._draw_history[0]
        return round(delta / max(span_s, TICK_SECONDS), 3)

    def _spread(self) -> float:
        if len(self._draw_history) < 2:
            return 2.5
        return max(pstdev(self._draw_history), 1.5)

    def _confidence(
        self,
        predicted_draw: float,
        task_avg_draw: float,
        blend_progress: float,
        transition_uncertainty: float = 0.0,
    ) -> float:
        n = len(self._draw_history)
        base = 42.0 + min(n, 10) * 4.5

        if n >= 3:
            spread = pstdev(self._draw_history)
            base -= min(spread * 2.2, 18)
        else:
            base -= 8

        drift = abs(predicted_draw - task_avg_draw) / max(task_avg_draw, 1)
        base -= min(drift * 12, 10)
        base -= (1.0 - blend_progress) * 15
        base -= transition_uncertainty * 20

        return round(max(35.0, min(96.0, base)), 1)

    def _mission_blended_draw(
        self,
        task_id: str,
        task_remaining_s: int,
        blend_progress: float,
        current_draw_w: float,
        horizon_s: int,
    ) -> tuple[float, float]:
        """Blend current task draw with anticipated transitions within horizon."""
        profile = TASK_PROFILES.get(task_id)
        task_avg = profile.avg_draw_w if profile else current_draw_w or 30.0
        ema = self._ema_draw()
        base = ema if ema is not None else (current_draw_w or task_avg)
        near_term = round(0.60 * base + 0.40 * task_avg, 1)

        transition_uncertainty = 0.0
        if task_remaining_s <= horizon_s:
            transition_uncertainty = min(
                1.0, (horizon_s - max(task_remaining_s, 0)) / max(horizon_s, 1)
            )

        phases = anticipated_phases(task_id)
        if not phases or transition_uncertainty <= 0:
            return near_term, transition_uncertainty

        expected_next = sum(
            p["expected_draw_w"] * (p["probability_pct"] / 100.0) for p in phases
        )
        remain_frac = min(1.0, task_remaining_s / max(horizon_s, 1))
        blended = round(
            near_term * remain_frac + expected_next * (1.0 - remain_frac), 1
        )
        if blend_progress < 1.0:
            blended = round(blended * 0.85 + task_avg * 0.15, 1)
        return blended, transition_uncertainty

    def _horizon_point(
        self,
        t_s: int,
        current_draw_w: float,
        task_id: str,
        task_remaining_s: int,
        blend_progress: float,
    ) -> dict:
        trend = self._trend_w_per_s()
        spread = self._spread()
        base, trans_unc = self._mission_blended_draw(
            task_id, task_remaining_s, blend_progress, current_draw_w, t_s
        )
        projected = round(base + trend * t_s, 1)
        interval = round(spread * (1.0 + trans_unc * 0.8 + (t_s / 60.0) * 0.5), 1)
        return {
            "t_s": t_s,
            "draw_w": max(0.0, projected),
            "draw_low_w": round(max(0.0, projected - interval), 1),
            "draw_high_w": round(projected + interval, 1),
        }

    def _horizon_forecast(
        self,
        current_draw_w: float,
        task_id: str,
        task_remaining_s: int,
        blend_progress: float,
    ) -> dict:
        points = [
            self._horizon_point(t, current_draw_w, task_id, task_remaining_s, blend_progress)
            for t in (10, 20, 30, 40, 50, 60)
        ]
        at_30 = next(p for p in points if p["t_s"] == HORIZON_30S)
        at_60 = next(p for p in points if p["t_s"] == HORIZON_60S)
        avg_60 = round(mean(p["draw_w"] for p in points), 1)
        return {
            "horizon_points": points,
            "forecast_30s": at_30,
            "forecast_60s": at_60,
            "avg_draw_60s": avg_60,
        }

    def _locomotion_outlook(
        self, task_id: str, task_remaining_s: int, phases: list[dict]
    ) -> dict:
        current_phase = LOCOMOTION_PHASES.get(task_id, "unknown")
        current_label = TASK_PROFILES[task_id].label if task_id in TASK_PROFILES else task_id

        if task_remaining_s > HORIZON_60S or not phases:
            return {
                "current_phase": current_phase,
                "current_phase_label": current_label,
                "outlook": f"Stable {current_label.lower()} for next {HORIZON_60S}s",
                "transition_in_s": None,
                "likely_next_phase": None,
                "likely_next_label": None,
            }

        top = max(phases, key=lambda p: p["probability_pct"])
        return {
            "current_phase": current_phase,
            "current_phase_label": current_label,
            "outlook": (
                f"Locomotion shift likely in ~{task_remaining_s}s → "
                f"{top['task_label']} ({top['probability_pct']:.0f}%)"
            ),
            "transition_in_s": task_remaining_s,
            "likely_next_phase": top["locomotion_phase"],
            "likely_next_label": top["task_label"],
            "anticipated_phases": phases,
        }

    def _risk_level(
        self,
        confidence: float,
        mission_energy_ok: bool,
        battery_pct: float,
        forecast_60s: dict,
        transition_in_s: int | None,
        phases: list[dict],
    ) -> str:
        high_draw = forecast_60s["draw_high_w"] >= 65
        locomotion_ramp = False
        if phases and transition_in_s is not None and transition_in_s <= HORIZON_60S:
            top = max(phases, key=lambda p: p["probability_pct"])
            locomotion_ramp = top["locomotion_phase"] in (
                "active_locomotion",
                "manipulation_peak",
            )

        if not mission_energy_ok or battery_pct <= 12:
            return "critical"
        if confidence < 50 or (high_draw and locomotion_ramp and battery_pct < 25):
            return "high"
        if confidence < 70 or high_draw or locomotion_ramp or not mission_energy_ok:
            return "medium"
        return "low"

    def _improved_runtime(
        self,
        battery_pct: float,
        capacity_wh: float,
        horizon: dict,
        task_avg_draw: float,
    ) -> dict:
        energy_wh = round((battery_pct / 100.0) * capacity_wh, 1)
        avg_draw = max(horizon["avg_draw_60s"], 0.1)
        f60 = horizon["forecast_60s"]

        runtime_best = round((energy_wh / f60["draw_low_w"]) * 60, 1) if f60["draw_low_w"] > 0 else None
        runtime_worst = round((energy_wh / f60["draw_high_w"]) * 60, 1) if f60["draw_high_w"] > 0 else None
        runtime_expected = round((energy_wh / avg_draw) * 60, 1)

        blended_draw = round(0.55 * avg_draw + 0.45 * task_avg_draw, 1)
        runtime_blended = round((energy_wh / max(blended_draw, 0.1)) * 60, 1)

        return {
            "runtime_min_expected": runtime_expected,
            "runtime_min_low": runtime_worst,
            "runtime_min_high": runtime_best,
            "runtime_min_blended": runtime_blended,
            "forecast_avg_draw_w": avg_draw,
        }

    def forecast(
        self,
        battery_pct: float,
        capacity_wh: float,
        task_id: str,
        task_remaining_s: int,
        blend_progress: float = 1.0,
        current_draw_w: float = 0.0,
    ) -> dict:
        profile = TASK_PROFILES.get(task_id)
        task_avg = profile.avg_draw_w if profile else current_draw_w or 30.0

        ema = self._ema_draw()
        if ema is not None:
            predicted_draw = round(0.55 * ema + 0.45 * task_avg, 1)
        else:
            predicted_draw = round(task_avg if current_draw_w <= 0 else current_draw_w, 1)

        horizon = self._horizon_forecast(
            current_draw_w, task_id, task_remaining_s, blend_progress
        )
        phases = anticipated_phases(task_id)
        outlook = self._locomotion_outlook(task_id, task_remaining_s, phases)

        _, trans_unc = self._mission_blended_draw(
            task_id, task_remaining_s, blend_progress, current_draw_w, HORIZON_60S
        )
        confidence = self._confidence(predicted_draw, task_avg, blend_progress, trans_unc)
        runtime = predict_runtime(battery_pct, capacity_wh, predicted_draw, task_avg)
        improved = self._improved_runtime(battery_pct, capacity_wh, horizon, task_avg)

        energy_wh = runtime["energy_wh_remaining"]
        mission_energy_wh = round(task_avg * (task_remaining_s / 3600.0), 2)
        mission_energy_ok = energy_wh >= mission_energy_wh * 1.08

        horizon_energy_wh = round(
            horizon["avg_draw_60s"] * (HORIZON_60S / 3600.0), 2
        )
        mission_battery_at_end = round(
            battery_pct - (mission_energy_wh / capacity_wh) * 100, 2
        )
        battery_at_60s = round(
            battery_pct - (horizon_energy_wh / capacity_wh) * 100, 2
        )

        risk_level = self._risk_level(
            confidence,
            mission_energy_ok,
            battery_pct,
            horizon["forecast_60s"],
            outlook.get("transition_in_s"),
            phases,
        )

        return {
            "predicted_draw_w": predicted_draw,
            "predicted_runtime_min": improved["runtime_min_blended"],
            "mission_forecast_min": improved["runtime_min_expected"],
            "mission_energy_wh": mission_energy_wh,
            "mission_energy_ok": mission_energy_ok,
            "mission_battery_pct_at_end": mission_battery_at_end,
            "battery_pct_at_60s": battery_at_60s,
            "confidence_pct": confidence,
            "forecast_method": "ema_task_horizon_blend",
            "risk_level": risk_level,
            "draw_trend_w_per_s": self._trend_w_per_s(),
            "forecast_30s": horizon["forecast_30s"],
            "forecast_60s": horizon["forecast_60s"],
            "horizon_points": horizon["horizon_points"],
            "avg_draw_60s": horizon["avg_draw_60s"],
            "locomotion_outlook": outlook,
            "anticipated_phases": phases,
            **runtime,
            **improved,
        }