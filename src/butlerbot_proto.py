"""Shared ButlerBot PROTO used by every Webots world.

The 11 ``butlerbot*.wbt`` files used to paste the same Robot node. That node
now lives in ``webots/protos/ButlerBot.proto``. A world names it with
``EXTERNPROTO`` and a short ``ButlerBot { }`` instance.

Checked across those worlds before the split: translation, rotation, name,
controller, and controllerArgs do not differ. They are still PROTO fields so
a world can place the robot or point it at a controller without editing the
PROTO. ``bodyMass`` and ``bodyInertia`` are fields because a BoM build
rewrites the body solid (``src.webots_body``). Wheel mass, contact radius,
cameras, and the rest stay inside the PROTO.
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

PROTO_PATH = Path(__file__).resolve().parent.parent / "webots" / "protos" / "ButlerBot.proto"
EXTERNPROTO_LINE = 'EXTERNPROTO "../protos/ButlerBot.proto"'
# md5 of expand_robot() for a default instance (no trailing newline).
# Moves when the shared Robot body changes. The apartment-only close-range
# ring and floor camera sit in a procedural block that expand_robot strips
# unless the instance sets mappingSensors TRUE, so warehouse worlds still
# expand to this hash. Their controller does not enable oak_* either.
SHIPPED_ROBOT_MD5 = "80548bfff8795fe72cd0b98407345041"

_MAPPING_BEGIN = "%< if (fields.mappingSensors.value) { >%"
_MAPPING_END = "%< } >%"

_FLOOR_EXTERN = (
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/'
    'projects/objects/floors/protos/Floor.proto"\n'
)

INSTANCE_TEXT = (
    "ButlerBot {\n"
    "  # Spawn ON the start line (x=0), lane center y=0, facing +x (toward finish)\n"
    "  translation 0 0 0\n"
    "  rotation 0 0 1 0\n"
    '  name "ButlerBot"\n'
    '  controller "butlerbot_controller"\n'
    "  controllerArgs [\n"
    '    "--dashboard-url=http://127.0.0.1:5000"\n'
    "  ]\n"
    "}"
)

# Shipped values. Rendering these goes back to the original VRML text.
_SHIPPED_TRANSLATION = (0.0, 0.0, 0.0)
_SHIPPED_ROTATION = (0.0, 0.0, 1.0, 0.0)
_SHIPPED_NAME = "ButlerBot"
_SHIPPED_CONTROLLER = "butlerbot_controller"
_SHIPPED_ARGS = ("--dashboard-url=http://127.0.0.1:5000",)
_SHIPPED_MASS = 3.2
_SHIPPED_INERTIA = (0.055, 0.062, 0.078, 0.0, 0.0, 0.0)

_EXACT = {
    "translation": "translation 0 0 0",
    "rotation": "rotation 0 0 1 0",
    "name": 'name "ButlerBot"',
    "controller": 'controller "butlerbot_controller"',
    "controllerArgs": (
        "controllerArgs [\n"
        '    "--dashboard-url=http://127.0.0.1:5000"\n'
        "  ]"
    ),
    "bodyMass": "mass 3.2",
    "bodyInertia": (
        "inertiaMatrix [\n"
        "      0.055 0.062 0.078\n"
        "      0 0 0\n"
        "    ]"
    ),
}

_IS_TOKEN = {
    "translation": "translation IS translation",
    "rotation": "rotation IS rotation",
    "name": "name IS name",
    "controller": "controller IS controller",
    "controllerArgs": "controllerArgs IS controllerArgs",
    "bodyMass": "mass IS bodyMass",
    "bodyInertia": "inertiaMatrix IS bodyInertia",
}


def _fmt(value: float) -> str:
    return f"{float(value):.10g}"


def _close(left: float, right: float, tol: float = 1e-12) -> bool:
    return abs(float(left) - float(right)) <= tol


def _brace_end(text: str, open_at: int) -> int:
    """Index just past the ``}`` that closes the ``{`` at ``open_at``."""
    depth = 0
    for i in range(open_at, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i + 1
    raise ValueError("unclosed brace")


def _node_span(text: str, pattern: str) -> tuple[int, int] | None:
    match = re.search(pattern, text)
    if match is None:
        return None
    open_at = text.find("{", match.start())
    if open_at < 0:
        return None
    return match.start(), _brace_end(text, open_at)


def robot_block_to_proto(robot_block: str) -> str:
    """Turn the shipped Robot node into ``ButlerBot.proto`` text.

    ``robot_block`` is the exact ``Robot { ... }`` paste (no trailing newline).
    """
    if hashlib.md5(robot_block.encode("utf-8")).hexdigest() != SHIPPED_ROBOT_MD5:
        raise ValueError("robot block is not the shipped ButlerBot paste")
    body = robot_block
    swaps = (
        (
            "  translation 0 0 0\n"
            "  rotation 0 0 1 0\n"
            '  name "ButlerBot"\n',
            "  translation IS translation\n"
            "  rotation IS rotation\n"
            "  name IS name\n",
        ),
        ('  controller "butlerbot_controller"\n', "  controller IS controller\n"),
        (
            "  controllerArgs [\n"
            '    "--dashboard-url=http://127.0.0.1:5000"\n'
            "  ]\n",
            "  controllerArgs IS controllerArgs\n",
        ),
        ("    mass 3.2\n", "    mass IS bodyMass\n"),
        (
            "    inertiaMatrix [\n"
            "      0.055 0.062 0.078\n"
            "      0 0 0\n"
            "    ]\n",
            "    inertiaMatrix IS bodyInertia\n",
        ),
    )
    for old, new in swaps:
        count = body.count(old)
        if count != 1:
            raise ValueError(f"expected one occurrence, found {count}: {old!r}")
        body = body.replace(old, new, 1)
    header = (
        "#VRML_SIM R2025a utf8\n"
        "\n"
        "# ButlerBot, the wheeled robot shared by every world in webots/worlds.\n"
        "# Pose, name, and controller are fields because a world may set them.\n"
        "# The committed worlds all use the defaults below. bodyMass and\n"
        "# bodyInertia are fields because a BoM build rewrites the body solid\n"
        "# (src/webots_body.py). Cameras, wheels, and the other devices stay here.\n"
        "PROTO ButlerBot [\n"
        "  field SFVec3f translation 0 0 0\n"
        "  field SFRotation rotation 0 0 1 0\n"
        '  field SFString name "ButlerBot"\n'
        '  field SFString controller "butlerbot_controller"\n'
        '  field MFString controllerArgs [ "--dashboard-url=http://127.0.0.1:5000" ]\n'
        "  field SFFloat bodyMass 3.2\n"
        "  field MFVec3f bodyInertia [ 0.055 0.062 0.078 0 0 0 ]\n"
        "]\n"
        "{\n"
    )
    return header + body + "\n}\n"


def _proto_robot_body() -> str:
    text = PROTO_PATH.read_text(encoding="utf-8")
    span = _node_span(text, r"(?m)^Robot \{")
    if span is None:
        raise ValueError(f"no Robot node in {PROTO_PATH}")
    return text[span[0] : span[1]]


def _parse_numbers(text: str) -> list[float]:
    return [float(part) for part in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", text)]


def _unquote(token: str) -> str:
    token = token.strip().rstrip(",")
    if len(token) >= 2 and token[0] == '"' and token[-1] == '"':
        return token[1:-1]
    return token


def parse_instance_fields(block: str) -> dict:
    """Fields set on a ``ButlerBot { }`` instance. Absent fields are omitted."""
    span = _node_span(block, r"(?m)^ButlerBot \{")
    if span is None:
        if block.lstrip().startswith("ButlerBot"):
            span = (0, len(block))
        else:
            return {}
    inner = block[span[0] : span[1]]
    lines = inner.splitlines()[1:-1]
    fields: dict = {}
    i = 0
    while i < len(lines):
        raw = lines[i].strip()
        i += 1
        if not raw or raw.startswith("#"):
            continue
        if raw.endswith("[") and " " not in raw.rstrip("[").strip():
            name = raw[:-1].strip()
            chunk: list[str] = []
            while i < len(lines) and lines[i].strip() != "]":
                chunk.append(lines[i].strip())
                i += 1
            if i < len(lines) and lines[i].strip() == "]":
                i += 1
            fields[name] = chunk
            continue
        name, _, rest = raw.partition(" ")
        if rest.startswith("["):
            fields[name] = rest
        else:
            fields[name] = rest.strip()
    return _coerce_fields(fields)


def _coerce_fields(raw: dict) -> dict:
    out: dict = {}
    if "translation" in raw and isinstance(raw["translation"], str):
        nums = _parse_numbers(raw["translation"])
        if len(nums) >= 3:
            out["translation"] = tuple(nums[:3])
    if "rotation" in raw and isinstance(raw["rotation"], str):
        nums = _parse_numbers(raw["rotation"])
        if len(nums) >= 4:
            out["rotation"] = tuple(nums[:4])
    if "name" in raw:
        out["name"] = _unquote(str(raw["name"]))
    if "controller" in raw:
        out["controller"] = _unquote(str(raw["controller"]))
    if "controllerArgs" in raw:
        chunk = raw["controllerArgs"]
        if isinstance(chunk, str):
            parts = re.findall(r'"([^"]*)"', chunk)
        else:
            parts = [_unquote(part) for part in chunk if _unquote(part)]
        out["controllerArgs"] = tuple(parts)
    if "bodyMass" in raw:
        nums = _parse_numbers(str(raw["bodyMass"]))
        if nums:
            out["bodyMass"] = float(nums[0])
    if "bodyInertia" in raw:
        chunk = raw["bodyInertia"]
        text = chunk if isinstance(chunk, str) else " ".join(chunk)
        nums = _parse_numbers(text)
        if len(nums) >= 3:
            while len(nums) < 6:
                nums.append(0.0)
            out["bodyInertia"] = tuple(float(v) for v in nums[:6])
    if "mappingSensors" in raw:
        out["mappingSensors"] = str(raw["mappingSensors"]).strip().upper() == "TRUE"
    return out


def _render(field: str, value) -> str:
    if field == "translation":
        if all(_close(a, b) for a, b in zip(value, _SHIPPED_TRANSLATION)):
            return _EXACT["translation"]
        return "translation " + " ".join(_fmt(v) for v in value)
    if field == "rotation":
        if all(_close(a, b) for a, b in zip(value, _SHIPPED_ROTATION)):
            return _EXACT["rotation"]
        return "rotation " + " ".join(_fmt(v) for v in value)
    if field == "name":
        if value == _SHIPPED_NAME:
            return _EXACT["name"]
        return f'name "{value}"'
    if field == "controller":
        if value == _SHIPPED_CONTROLLER:
            return _EXACT["controller"]
        return f'controller "{value}"'
    if field == "controllerArgs":
        args = tuple(value)
        if args == _SHIPPED_ARGS:
            return _EXACT["controllerArgs"]
        inner = "\n".join(f'    "{arg}"' for arg in args)
        return f"controllerArgs [\n{inner}\n  ]"
    if field == "bodyMass":
        if _close(value, _SHIPPED_MASS):
            return _EXACT["bodyMass"]
        return f"mass {_fmt(value)}"
    if field == "bodyInertia":
        vals = tuple(value)
        if len(vals) >= 6 and all(_close(a, b) for a, b in zip(vals, _SHIPPED_INERTIA)):
            return _EXACT["bodyInertia"]
        diag = list(vals[:3]) + [0.0, 0.0, 0.0]
        if len(vals) >= 6:
            diag = list(vals[:6])
        return (
            "inertiaMatrix [\n"
            f"      {_fmt(diag[0])} {_fmt(diag[1])} {_fmt(diag[2])}\n"
            f"      {_fmt(diag[3])} {_fmt(diag[4])} {_fmt(diag[5])}\n"
            "    ]"
        )
    raise KeyError(field)


def expand_robot(overrides: dict | None = None) -> str:
    """The Robot node the PROTO expands to, with ``overrides`` applied."""
    body = _proto_robot_body()
    values = {
        "translation": _SHIPPED_TRANSLATION,
        "rotation": _SHIPPED_ROTATION,
        "name": _SHIPPED_NAME,
        "controller": _SHIPPED_CONTROLLER,
        "controllerArgs": _SHIPPED_ARGS,
        "bodyMass": _SHIPPED_MASS,
        "bodyInertia": _SHIPPED_INERTIA,
    }
    if overrides:
        values.update({key: overrides[key] for key in values if key in overrides})
        if "mappingSensors" in overrides:
            values["mappingSensors"] = bool(overrides["mappingSensors"])
    values.setdefault("mappingSensors", False)
    for field, token in _IS_TOKEN.items():
        rendered = _render(field, values[field])
        if token not in body:
            raise ValueError(f"PROTO is missing {token}")
        body = body.replace(token, rendered, 1)
    return _apply_mapping_sensors(body, bool(values.get("mappingSensors", False)))


def _apply_mapping_sensors(body: str, enabled: bool) -> str:
    """Keep or drop the apartment-only sensor block.

    The markers are Webots procedural PROTO lines. A default instance
    (every warehouse world) expands to the bytes from before that block
    existed. An instance with mappingSensors TRUE keeps the devices and
    drops the markers.
    """
    begin = body.find(_MAPPING_BEGIN)
    if begin < 0:
        return body
    end = body.find(_MAPPING_END, begin)
    if end < 0:
        raise ValueError("mappingSensors block is not closed")
    line_start = body.rfind("\n", 0, begin) + 1
    end_nl = body.find("\n", end)
    end_nl = len(body) if end_nl < 0 else end_nl + 1
    if not enabled:
        return body[:line_start] + body[end_nl:]
    begin_nl = body.find("\n", begin)
    end_line = body.rfind("\n", 0, end) + 1
    return body[:line_start] + body[begin_nl + 1 : end_line] + body[end_nl:]


def expand_world_text(text: str) -> str:
    """Replace a ``ButlerBot`` instance with the expanded Robot node.

    Text that already has a pasted Robot (or no ButlerBot) is returned
    unchanged. Expansion reads ``webots/protos/ButlerBot.proto`` from this
    repo, so a BoM sibling copied beside a temp world still resolves.
    """
    span = _node_span(text, r"(?m)^ButlerBot \{")
    if span is None:
        return text
    overrides = parse_instance_fields(text[span[0] : span[1]])
    return text[: span[0]] + expand_robot(overrides) + text[span[1] :]


def use_butlerbot_proto(text: str) -> str:
    """Emit the EXTERNPROTO line and the short instance.

    A pasted Robot whose md5 is the shipped block is replaced. A different
    Robot is left alone and reported, so a hand edit is not dropped quietly.
    """
    text = _insert_externproto(text)
    span = _node_span(text, r"(?m)^Robot \{")
    if span is None:
        return text
    block = text[span[0] : span[1]]
    digest = hashlib.md5(block.encode("utf-8")).hexdigest()
    if digest != SHIPPED_ROBOT_MD5:
        raise ValueError(
            f"Robot block md5 {digest} is not the shipped ButlerBot ({SHIPPED_ROBOT_MD5})"
        )
    return text[: span[0]] + INSTANCE_TEXT + text[span[1] :]


def _insert_externproto(text: str) -> str:
    if "ButlerBot.proto" in text:
        return text
    if _FLOOR_EXTERN in text:
        return text.replace(_FLOOR_EXTERN, _FLOOR_EXTERN + EXTERNPROTO_LINE + "\n", 1)
    header = "#VRML_SIM R2025a utf8\n"
    if text.startswith(header):
        return header + "\n" + EXTERNPROTO_LINE + "\n" + text[len(header) :]
    return EXTERNPROTO_LINE + "\n" + text


def patch_instance_body(
    text: str,
    *,
    mass_kg: float,
    inertia: tuple[float, float, float],
    comment: str,
) -> str | None:
    """Set ``bodyMass`` / ``bodyInertia`` on the ButlerBot instance.

    Returns None when ``text`` has no ButlerBot instance (the caller patches
    a pasted Robot instead). The comment sits next to the fields. A second
    patch replaces the previous comment and values, so inertia is not scaled
    twice.
    """
    span = _node_span(text, r"(?m)^ButlerBot \{")
    if span is None:
        return None
    lines = text[span[0] : span[1]].splitlines()
    kept: list[str] = []
    for line in lines:
        stripped = line.strip()
        if stripped == "}":
            continue
        if stripped.startswith("# BoM ") or stripped.startswith("bodyMass ") or stripped.startswith("bodyInertia "):
            continue
        kept.append(line)
    products = (0.0, 0.0, 0.0)
    iner = " ".join(_fmt(v) for v in (inertia[0], inertia[1], inertia[2], *products))
    for raw in comment.strip("\n").splitlines():
        kept.append("  # " + raw.strip().lstrip("#").strip())
    kept.append(f"  bodyMass {_fmt(mass_kg)}")
    kept.append(f"  bodyInertia [ {iner} ]")
    kept.append("}")
    return text[: span[0]] + "\n".join(kept) + text[span[1] :]
