"""Headless physics check for ButlerBotSkid. The world is deleted afterwards.

The lip is a vertical box 12.7 mm tall. There is no ramp. The controller
prints contact depth and writes /tmp/skid_sanity/result.txt.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.mapping.world_guard import require_skid_world
WEBOTS = ROOT / "webots"
WORLD = WEBOTS / "worlds" / "_skid_sanity.wbt"
OUT = Path("/tmp/skid_sanity")

# Low floor top is z=0. The lip box sits on that face: bottom z=0, top
# z=0.0127, near face at x=3. Both solids are static, so the shared face
# is not a dynamic contact. The tire meets a vertical 12.7 mm riser.
WORLD_TEXT = """#VRML_SIM R2025a utf8

EXTERNPROTO "../protos/ButlerBotSkid.proto"

WorldInfo {
  basicTimeStep 8
  ERP 0.6
  CFM 1e-9
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties {
      material1 "drive_wheel"
      material2 "laminate"
      coulombFriction [
        8
      ]
      forceDependentSlip [
        0.01
      ]
      frictionRotation 0 0
      bounce 0
      softCFM 1e-6
      softERP 0.7
    }
    ContactProperties {
      material1 "drive_wheel"
      material2 "default"
      coulombFriction [
        8
      ]
      forceDependentSlip [
        0.01
      ]
      bounce 0
      softCFM 1e-6
      softERP 0.7
    }
  ]
}

DEF FLOOR Solid {
  translation 2.5 0 -0.05
  name "floor"
  contactMaterial "laminate"
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.55 0.48 0.38
        roughness 0.8
      }
      geometry Box {
        size 8 4 0.1
      }
    }
  ]
  boundingObject Box {
    size 8 4 0.1
  }
}

DEF LIP Solid {
  translation 4.5 0 0.00635
  name "floor_lip"
  contactMaterial "laminate"
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.45 0.32 0.28
        roughness 0.9
      }
      geometry Box {
        size 3 3 0.0127
      }
    }
  ]
  boundingObject Box {
    size 3 3 0.0127
  }
}

ButlerBotSkid {
  translation 0.5 0 0
  rotation 0 0 1 0
  name "ButlerBot"
  controller "skid_sanity"
  controllerArgs [
  ]
  mappingSensors FALSE
}
"""


def main() -> int:
    require_skid_world(WORLD_TEXT, "skid physics world")
    OUT.mkdir(parents=True, exist_ok=True)
    WORLD.write_text(WORLD_TEXT, encoding="utf-8")
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT)
    env["RBM_SKID_DIR"] = str(OUT)
    env["RBM_LIP_X"] = "3.0"
    env["RBM_LIP_Z"] = "0.0127"
    log_path = OUT / "webots.log"
    cmd = [
        "xvfb-run",
        "-a",
        "webots",
        "--mode=fast",
        "--no-rendering",
        "--batch",
        "--stdout",
        "--stderr",
        "--port=1611",
        "worlds/_skid_sanity.wbt",
    ]
    try:
        with log_path.open("w", encoding="utf-8") as handle:
            proc = subprocess.run(cmd, cwd=str(WEBOTS), env=env, stdout=handle, stderr=subprocess.STDOUT)
        print(f"webots exit={proc.returncode} log={log_path}")
    finally:
        WORLD.unlink(missing_ok=True)
    result = OUT / "result.txt"
    if not result.is_file():
        print("no result file")
        text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.is_file() else ""
        print("\n".join(text.splitlines()[-40:]))
        return 1
    text = result.read_text(encoding="utf-8")
    print(text)
    return 0 if "result=PASS" in text else 1


if __name__ == "__main__":
    sys.exit(main())
