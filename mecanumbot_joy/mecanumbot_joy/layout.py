"""
Controller layout normalisation: raw joy frame to named controls.

Pure functions only -- this module must not import ``rclpy``, so the whole
translation from a raw ``sensor_msgs/Joy`` frame to named controls can be
unit tested without a ROS graph.

The knowledge encoded here used to be hardcoded per-layout tables in
``mecanumbot_gui/robot/controller_pkg/controller_node.py``: two trigger
conventions, two D-Pad conventions, and two button-name dictionaries
selected by an ``if self._layout == 'xbox360'`` scattered through the
node.  It is now driven entirely by a profile document (see
:mod:`mecanumbot_joy.profile`), so supporting a new gamepad is a YAML
change rather than a code change.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Sequence

#: The four D-Pad pseudo-buttons, exposed under these names whichever
#: physical source (axes or buttons) the controller reports them on.
DPAD_NAMES = ("DPadUp", "DPadDown", "DPadLeft", "DPadRight")

#: A D-Pad axis must exceed this magnitude to count as pressed.  Matches
#: the 0.5 threshold the previous controller_node used.
DPAD_AXIS_THRESHOLD = 0.5


@dataclass
class ControlState:
    """
    A single joy frame resolved into profile-declared control names.

    ``axes`` maps an axis name (``left_x``, ``rt``, ...) to its normalised
    value: sticks stay in -1.0..1.0, triggers are mapped to 0.0..1.0.
    ``buttons`` maps a button name (``A``, ``START``, ``DPadUp``, ...) to
    0 or 1, with the D-Pad folded in regardless of how it is reported.
    """

    axes: Dict[str, float] = field(default_factory=dict)
    buttons: Dict[str, int] = field(default_factory=dict)

    def axis(self, name: str) -> float:
        """Return the named axis value, or 0.0 if the profile lacks it."""
        return self.axes.get(name, 0.0)

    def button(self, name: str) -> int:
        """Return the named button state, or 0 if the profile lacks it."""
        return self.buttons.get(name, 0)


def safe_axis(axes: Sequence[float], index) -> float:
    """
    Return ``axes[index]`` as a float, or 0.0 when out of range.

    A profile written for an 8-axis pad must not crash the node when a
    6-axis pad is plugged in; the missing control simply reads neutral.
    """
    try:
        return float(axes[index])
    except (IndexError, TypeError, ValueError):
        return 0.0


def safe_button(buttons: Sequence[int], index) -> int:
    """Return ``buttons[index]`` as an int, or 0 when out of range."""
    try:
        return int(buttons[index])
    except (IndexError, TypeError, ValueError):
        return 0


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into the inclusive ``[low, high]`` interval."""
    if value < low:
        return low
    if value > high:
        return high
    return value


def normalise_axis(raw: float, spec: Mapping) -> float:
    """
    Map a raw axis reading onto its profile-declared range.

    A spec carrying ``range: [rest, pressed]`` is an analogue trigger: the
    reading is mapped linearly so that ``rest`` becomes 0.0 and ``pressed``
    becomes 1.0.  This single formula subsumes both hardcoded conventions
    the old controller_node carried -- Xbox 360 via xpad rests at +1.0
    (``range: [1.0, -1.0]`` gives ``(1 - raw) / 2``) while most other pads
    rest at -1.0 (``range: [-1.0, 1.0]`` gives ``(raw + 1) / 2``).

    A spec without ``range`` is a stick and passes through, clamped to
    -1.0..1.0.
    """
    axis_range = spec.get("range") if isinstance(spec, Mapping) else None
    if not axis_range:
        return clamp(raw, -1.0, 1.0)

    rest, pressed = float(axis_range[0]), float(axis_range[1])
    if pressed == rest:
        return 0.0
    # The trailing `+ 0.0` normalises -0.0, which an inverted range
    # produces at rest and which reads confusingly in the GUI.
    return clamp((raw - rest) / (pressed - rest), 0.0, 1.0) + 0.0


def read_dpad(
    axes: Sequence[float],
    buttons: Sequence[int],
    spec: Mapping,
) -> Dict[str, bool]:
    """
    Resolve the D-Pad into a ``{DPadUp: bool, ...}`` map.

    ``spec['source']`` selects the convention: ``axes`` reads a pair of
    signed axes (positive X is left, positive Y is up -- the xpad
    convention), ``buttons`` reads four discrete button indices.  Both
    produce identical output, which is what lets the action layer above
    stay ignorant of the difference.
    """
    states = {name: False for name in DPAD_NAMES}
    if not isinstance(spec, Mapping):
        return states

    source = spec.get("source")
    if source == "axes":
        x = safe_axis(axes, spec.get("x_axis"))
        y = safe_axis(axes, spec.get("y_axis"))
        states["DPadLeft"] = x > DPAD_AXIS_THRESHOLD
        states["DPadRight"] = x < -DPAD_AXIS_THRESHOLD
        states["DPadUp"] = y > DPAD_AXIS_THRESHOLD
        states["DPadDown"] = y < -DPAD_AXIS_THRESHOLD
    elif source == "buttons":
        for name in DPAD_NAMES:
            index = spec.get(name[len("DPad"):].lower())
            if index is not None:
                states[name] = safe_button(buttons, index) == 1

    return states


def resolve_controls(
    axes: Sequence[float],
    buttons: Sequence[int],
    profile,
) -> ControlState:
    """
    Resolve one joy frame into the controls a profile declares.

    ``profile`` is a :class:`mecanumbot_joy.profile.Profile`, or any object
    exposing ``axes``, ``buttons`` and ``dpad`` mappings.
    """
    state = ControlState()

    for name, spec in profile.axes.items():
        state.axes[name] = normalise_axis(safe_axis(axes, spec.get("index")), spec)

    for name, index in profile.buttons.items():
        state.buttons[name] = safe_button(buttons, index)

    for name, pressed in read_dpad(axes, buttons, profile.dpad).items():
        state.buttons[name] = 1 if pressed else 0

    return state


def fingerprint(axes: Sequence[float], buttons: Sequence[int]) -> Dict[str, int]:
    """
    Return the ``{axes: n, buttons: n}`` shape used for auto-detection.

    This is the whole of the old ``_detect_layout`` heuristic: an Xbox 360
    pad on xpad reports exactly 8 axes and 11 buttons, and nothing else the
    lab owns collides with that.  Matching lives in
    :func:`mecanumbot_joy.profile.select_profile`.
    """
    return {"axes": len(axes), "buttons": len(buttons)}


def describe_controls(profile) -> List[str]:
    """
    Return human-readable lines describing a profile's control set.

    Logged once when a profile becomes active, replacing the old
    ``Button table: {...}`` dump -- the operator needs to know which
    physical control a name refers to when a binding misbehaves.
    """
    lines = []
    for name, spec in sorted(profile.axes.items()):
        kind = "trigger" if spec.get("range") else "stick"
        lines.append("  axis   {:<10} index {:<3} ({})".format(name, spec.get("index"), kind))
    for name, index in sorted(profile.buttons.items(), key=lambda item: item[1]):
        lines.append("  button {:<10} index {}".format(name, index))
    if profile.dpad:
        lines.append("  dpad   source {}".format(profile.dpad.get("source")))
    return lines
