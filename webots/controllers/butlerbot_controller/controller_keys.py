"""Keyboard teleop input. No physics."""
from __future__ import annotations

from controller import Keyboard

_teleop = None


def bind_teleop(mod) -> None:
    global _teleop
    _teleop = mod


KEY_W = ord("W")
KEY_A = ord("A")
KEY_S = ord("S")
KEY_D = ord("D")
KEY_R = ord("R")
KEY_SPACE = ord(" ")
KEY_I = ord("I")
KEY_J = ord("J")
KEY_K = ord("K")
KEY_L = ord("L")

# Arrow keys (Webots keyboard constants)
try:
    KEY_UP = Keyboard.KEY_UP
    KEY_LEFT = Keyboard.KEY_LEFT
    KEY_DOWN = Keyboard.KEY_DOWN
    KEY_RIGHT = Keyboard.KEY_RIGHT
except AttributeError:
    KEY_UP, KEY_LEFT, KEY_DOWN, KEY_RIGHT = 315, 314, 317, 316


_DRIVE_KEY_CODES = frozenset({
    KEY_W, KEY_A, KEY_S, KEY_D,
    KEY_I, KEY_J, KEY_K, KEY_L,
    KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT,
})


class KeyTracker:
    """Track held keys from Webots keyboard events (press/release stream)."""

    def __init__(self) -> None:
        self._active: set[int] = set()

    def poll(self, keyboard: Keyboard) -> tuple[set[int], set[int]]:
        """Return (currently held keys, keys newly pressed this step)."""
        pressed: set[int] = set()
        key = keyboard.getKey()
        while key != -1:
            if key > 0:
                self._active.add(key)
                pressed.add(key)
            else:
                self._active.discard(-key)
            key = keyboard.getKey()
        return set(self._active), pressed

    def active_keys(self) -> set[int]:
        return set(self._active)

    def cancel_drive_keys(self) -> None:
        """Drop held drive keys — Space stop should not fight still-held I/J/K/L."""
        self._active -= _DRIVE_KEY_CODES


def _expand_teleop_keys(keys: set[int]) -> set[int]:
    """Map IJKL / arrows onto WASD semantics for a single drive helper."""
    expanded = set(keys)
    if _teleop is not None:
        expanded = _teleop.normalize_key_set(keys)
    else:
        expanded = {k - 32 if ord("a") <= k <= ord("z") else k for k in keys}
    if KEY_UP in keys:
        expanded.add(KEY_W)
    if KEY_DOWN in keys:
        expanded.add(KEY_S)
    if KEY_LEFT in keys:
        expanded.add(KEY_A)
    if KEY_RIGHT in keys:
        expanded.add(KEY_D)
    if KEY_I in expanded:
        expanded.add(KEY_W)
    if KEY_K in expanded:
        expanded.add(KEY_S)
    if KEY_J in expanded:
        expanded.add(KEY_A)
    if KEY_L in expanded:
        expanded.add(KEY_D)
    return expanded


def _teleop_drive(keys: set[int]) -> tuple[float, float]:
    """Keyboard → (left_wheel_v, right_wheel_v) in rad/s."""
    expanded = _expand_teleop_keys(keys)
    if _teleop is not None:
        return _teleop.drive_from_key_set(
            expanded, key_w=KEY_W, key_a=KEY_A, key_s=KEY_S, key_d=KEY_D
        )
    left = right = 0.0
    if KEY_W in expanded:
        left += 5.5
        right += 5.5
    if KEY_S in expanded:
        left -= 4.2
        right -= 4.2
    if KEY_A in expanded:
        left -= 2.6
        right += 2.6
    if KEY_D in expanded:
        left += 2.6
        right -= 2.6
    return left, right
