"""Append-only learnings. The seeded 'Known facts' section is not rewritten."""

from __future__ import annotations

from pathlib import Path

_HEADING = "## Learnings"


def append_learning(path: Path | str, body: str) -> None:
    file = Path(path)
    if file.is_file():
        text = file.read_text(encoding="utf-8")
    else:
        text = "# Optimizer knowledge\n\n## Known facts\n\n" + _HEADING + "\n"
    if _HEADING not in text:
        text = text.rstrip() + "\n\n" + _HEADING + "\n"
    file.write_text(text.rstrip() + "\n\n" + body.strip() + "\n", encoding="utf-8")
