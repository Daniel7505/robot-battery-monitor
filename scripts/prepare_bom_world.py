#!/usr/bin/env python3
"""Print the world Webots should open for the selected BoM build.

Unset ``bom_build`` prints the path it was given. ``light``, ``heavy``, or
``budget`` prints a sibling ``<name>.bom-<build>.wbt`` whose ``ButlerBot``
instance sets ``bodyMass`` and ``bodyInertia`` for that build. The contact
cylinder and the drawn mesh stay 0.08 m. The shipped world file is not
modified.

    python scripts/prepare_bom_world.py webots/worlds/butlerbot_warehouse.wbt
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.webots_body import prepare_bom_world  # noqa: E402


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: prepare_bom_world.py WORLD.wbt", file=sys.stderr)
        return 2
    print(prepare_bom_world(argv[1]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
