"""Edit only the allowlisted keys in config.yaml, and keep the comments."""

from __future__ import annotations

import re
from pathlib import Path

from src.controller_limits import (
    KNOBS,
    MAX_STRAIGHT_GAP_M_S,
    AllowlistError,
    check_param,
    check_set,
    load_controller_limits,
)


def _format(key: str, value) -> str:
    if key == "bom_build":
        return "" if value in (None, "") else str(value)
    if key == "controller.coast_enabled":
        if value in (None, "", True, "true", "1", 1):
            return "true"
        return "false"
    number = float(value)
    if key == "controller.pivot_w_max_rad_s" or key == "controller.k_steer":
        return f"{number:.1f}"
    return f"{number:.2f}"


def write_key(text: str, key: str, value) -> str:
    """Return ``text`` with one allowlisted key replaced. Comments stay put."""
    if key not in KNOBS:
        raise AllowlistError(f"refusing to write {key!r}")
    rendered = _format(key, value)
    if key == "bom_build":
        pattern = re.compile(r"^bom_build:.*$", re.M)
        line = "bom_build:" if rendered == "" else f"bom_build: {rendered}"
        if not pattern.search(text):
            raise AllowlistError("config.yaml has no bom_build key to edit")
        return pattern.sub(line, text, count=1)
    _section, name = key.split(".", 1)
    section = "controller"
    header = re.compile(rf"^{section}:\s*$", re.M)
    if not header.search(text):
        return text.rstrip() + f"\n{section}:\n  {name}: {rendered}\n"
    key_line = re.compile(rf"^(\s{{2}}{re.escape(name)}:\s*).*$", re.M)
    if key_line.search(text):
        return key_line.sub(rf"\g<1>{rendered}", text, count=1)
    return header.sub(rf"{section}:\n  {name}: {rendered}", text, count=1)


class ConfigStore:
    """Snapshot and restore the allowlisted keys. Other YAML is left alone."""

    def __init__(self, path: Path | str):
        self.path = Path(path)

    def read(self) -> dict:
        return load_controller_limits(self.path).as_params()

    def write_params(self, params: dict) -> None:
        merged = self.read()
        normalised = {}
        for key in KNOBS:
            if key not in params:
                continue
            value = check_param(key, params[key])
            normalised[key] = value
            merged[key] = "" if value is None else value
        cruise = merged.get("controller.cruise_m_s")
        straight = merged.get("controller.straight_m_s")
        if cruise is not None and straight is not None:
            c_f = float(cruise)
            s_f = float(straight)
            if s_f + 1e-9 < c_f:
                hi = float(KNOBS["controller.straight_m_s"].hi or c_f)
                merged["controller.straight_m_s"] = min(c_f + MAX_STRAIGHT_GAP_M_S, hi)
        check_set(merged)
        text = self.path.read_text(encoding="utf-8")
        for key, value in normalised.items():
            text = write_key(text, key, value)
        self.path.write_text(text, encoding="utf-8")

    def apply_one(self, key: str, value) -> None:
        current = self.read()
        current[key] = value
        self.write_params(current)
