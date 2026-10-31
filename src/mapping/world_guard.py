"""Refuse a mapping run that is not the four-wheel skid-steer.

The caster ``ButlerBot`` proto stays in the tree because the warehouse
worlds instance it. Apartment mapping must not. A world that names the
apartment mapper, or a seed world this checker is asked to approve, has
to instance ``ButlerBotSkid`` or the launch stops.
"""

from __future__ import annotations

import re

_SKID = re.compile(r"(?m)^[ \t]*ButlerBotSkid[ \t]*\{")
_CASTER = re.compile(r"(?m)^[ \t]*ButlerBot[ \t]*\{")
_MAPPING = 'controller "apartment_mapper"'


def world_uses_skid(text: str) -> bool:
    """True when the world instances ButlerBotSkid and not the caster robot."""
    return _SKID.search(text) is not None and _CASTER.search(text) is None


def require_skid_world(text: str, source: str) -> None:
    """Abort unless ``text`` is a ButlerBotSkid world."""
    if world_uses_skid(text):
        return
    raise SystemExit(
        f"ERROR: {source} does not use ButlerBotSkid. "
        "This run is the four-wheel skid-steer. The caster ButlerBot must not be launched."
    )


def require_mapping_world_is_skid(text: str, source: str) -> None:
    """Apartment mapping worlds must be ButlerBotSkid. Warehouse worlds are left alone."""
    if _MAPPING not in text:
        return
    require_skid_world(text, source)


def swap_caster_instance(text: str) -> str:
    """Point one caster robot instance at ButlerBotSkid. The proto file is not edited."""
    old_proto = 'EXTERNPROTO "../protos/ButlerBot.proto"\n'
    skid_proto = 'EXTERNPROTO "../protos/ButlerBotSkid.proto"\n'
    if old_proto in text and skid_proto not in text:
        text = text.replace(old_proto, old_proto + skid_proto, 1)
    return re.sub(r"(?m)^ButlerBot \{", "ButlerBotSkid {", text, count=1)
