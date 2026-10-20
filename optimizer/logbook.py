"""Append-only trial log. CSV and JSONL grow; they are not rewritten."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

CSV_FIELDS = (
    "iteration",
    "utc",
    "phase",
    "param",
    "old",
    "new",
    "hypothesis",
    "expected_effect",
    "decision",
    "reason",
    "build",
    "err_pct",
    "wh_model_drive",
    "wh_measured_drive_elec",
    "wh_measured_drive_shaft",
    "wh_per_m",
    "dist_m",
    "duration_s",
    "score",
    "evidence_status",
    "err_gain_pp",
    "wpm_gain_rel",
    "turn_delta_pp",
    "turn_noise_band_pp",
    "note",
    "lesson",
    "mechanism",
    "next_hypothesis",
    "code_suggestions",
)

_SECRET = re.compile(
    r"(?i)\b(password|passwd|secret|token|api[_-]?key|authorization)\b(\s*[=:]\s*)(\S+)"
)


def redact(text: str) -> str:
    """Hide credential-shaped assignments. The value is not kept."""
    return _SECRET.sub(r"\1\2[REDACTED]", text)


def _clean(value):
    if isinstance(value, str):
        return redact(value)
    if isinstance(value, list):
        return [_clean(item) for item in value]
    if isinstance(value, dict):
        return {key: _clean(item) for key, item in value.items()}
    return value


class Logbook:
    def __init__(self, run_dir: Path | str):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl_path = self.run_dir / "results.jsonl"
        self.csv_path = self.run_dir / "results.csv"

    def append(self, record: dict) -> dict:
        clean = _clean(record)
        with self.jsonl_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(clean, sort_keys=True, default=str) + "\n")
        new_file = not self.csv_path.exists() or self.csv_path.stat().st_size == 0
        row = {key: clean.get(key, "") for key in CSV_FIELDS}
        suggestions = row.get("code_suggestions")
        if isinstance(suggestions, list):
            row["code_suggestions"] = " | ".join(str(item) for item in suggestions)
        with self.csv_path.open("a", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS, extrasaction="ignore")
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        return clean
