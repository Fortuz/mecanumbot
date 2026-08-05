"""
Joystick profile documents: load, validate, normalise.

Pure Python -- no ``rclpy``, no ROS parameter server.  A profile is a YAML
document shipped in ``mecanumbot_description/config/joystick/``; this
module is the single authority on what a valid one looks like.

That single-authority property is deliberate and load bearing.  The web
GUI (``mecanumbot_web``) imports :func:`validate_document` before it will
write a file, and the joy node imports it before it will swap a profile
in.  Because both sides run the same validator, the GUI cannot save a
document that the node would then refuse to load.

Schema, in brief::

    profile:
      name: xbox360
      match: {axes: 8, buttons: 11}   # omit -> never auto-selected
      limits:    {max_lin_vel, max_ang_vel, deadzone, lin_step, ang_step,
                  publish_hz, joy_timeout, zero_hold_ticks}
      accessory: {neck: {min, max, default, step},
                  gripper: {min, front, max, default}}
      axes:      {left_x: {index: 0}, lt: {index: 2, range: [1.0, -1.0]}}
      dpad:      {source: axes, x_axis: 6, y_axis: 7}
      buttons:   {A: 0, B: 1, ...}
      bindings:  {drive: {control: axis, axis: left_y}, ...}
      led_presets: {attention: {fl: {mode: 6, color: 1}, ...}}

The two-level indirection between ``axes``/``buttons`` (what the pad has)
and ``bindings`` (what the robot does) is what makes a profile portable:
rebinding an action never touches an index, and swapping pads never
touches a semantic.
"""

import copy
import glob
import os
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional

import yaml

from .layout import DPAD_NAMES

#: Continuous actions.  Each consumes one axis and drives one velocity
#: component.
AXIS_ACTIONS = ("drive", "strafe", "turn")

#: Discrete actions.  Each consumes one button.
BUTTON_ACTIONS = (
    "neck_up",
    "neck_down",
    "neck_preset",
    "gripper_open",
    "gripper_close",
    "estop",
    "led_preset",
)

#: The complete, fixed action vocabulary.  A binding naming anything else
#: is a load error.  This replaces the previous system's arbitrary
#: nine-field action tuples with JSON placeholder substitution -- that
#: generality bought a mapping GUI, a SQLite schema and twelve custom ROS
#: services, and was only ever used to express these ten things.
VOCABULARY = AXIS_ACTIONS + BUTTON_ACTIONS

#: Valid ``trigger`` values for a button binding.
TRIGGERS = ("press", "hold")

DEFAULT_LIMITS = {
    # Preserved from mecanumbot_teleop/script/teleop_joystick.py.
    "max_lin_vel": 0.234,
    "max_ang_vel": 1.092,
    "deadzone": 0.05,
    # The old node declared smoothing parameters but never applied them
    # (make_simple_profile was commented out), so these are newly live.
    "lin_step": 0.02,
    "ang_step": 0.10,
    # Was 3.0 Hz, which is far too slow to steer by. Every *_step here and
    # in the accessory block is per publish tick, so this also sets the ramp
    # rates: at 20 Hz these steps took ~0.6 s to reach max_lin_vel, which
    # read as input lag. 50 Hz brings that to ~0.23 s.
    "publish_hz": 50.0,
    # No equivalent existed: the old node held the last velocity forever
    # if the pad was unplugged mid-drive.
    "joy_timeout": 0.5,
    "zero_hold_ticks": 12,
    "accessory_keepalive_hz": 1.0,
}

# Neck step scaled with publish_hz, so the hold-to-move sweep stays at the
# ~3 units/s it ran at when the tick was 20 Hz.
DEFAULT_NECK = {"min": 2.0, "max": 8.6, "default": 8.6, "step": 0.06}
DEFAULT_GRIPPER = {"min": 1.6, "front": 5.12, "max": 8.54, "default": 5.12}

#: LED corners, in the order ``SetLedStatus`` declares them.
LED_CORNERS = ("fl", "fr", "bl", "br")


class ProfileError(ValueError):
    """Raised when a profile document is structurally invalid."""


@dataclass
class Profile:
    """A validated, defaults-merged joystick profile."""

    name: str
    axes: Dict[str, dict] = field(default_factory=dict)
    buttons: Dict[str, int] = field(default_factory=dict)
    dpad: Dict[str, object] = field(default_factory=dict)
    bindings: Dict[str, dict] = field(default_factory=dict)
    limits: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_LIMITS))
    neck: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_NECK))
    gripper: Dict[str, float] = field(default_factory=lambda: dict(DEFAULT_GRIPPER))
    led_presets: Dict[str, dict] = field(default_factory=dict)
    match: Optional[Dict[str, int]] = None
    description: str = ""
    source_path: Optional[str] = None

    def control_names(self) -> Dict[str, List[str]]:
        """
        Return the axis and button names this profile declares.

        The web GUI uses this to populate its per-binding dropdowns, so a
        binding naming a control the pad does not have is impossible to
        express in the UI rather than merely rejected on save.
        """
        return {
            "axes": sorted(self.axes),
            "buttons": sorted(self.buttons) + list(DPAD_NAMES),
        }


#: YAML 1.1 words that parse as booleans rather than strings, so they
#: cannot be used as control or preset names without quoting.
YAML_BOOLEAN_WORDS = (
    "y", "yes", "n", "no", "true", "false", "on", "off")


def _is_number(value) -> bool:
    """Return True for a real number, excluding bool."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _check_names(mapping, where: str, errors: List[str]) -> bool:
    """
    Reject non-string keys, which YAML 1.1 boolean words silently become.

    Naming an LED preset ``off`` (or a button ``no``) produces the key
    ``False`` rather than the word, so the entry becomes unreferenceable
    by name.  Without this check the failure surfaces much later as a
    confusing sort error rather than as the quoting problem it is.
    """
    ok = True
    for key in mapping:
        if isinstance(key, str):
            continue
        ok = False
        hint = ""
        if isinstance(key, bool):
            hint = (" YAML reads words like {} as booleans; quote the name or "
                    "choose another.".format(", ".join(YAML_BOOLEAN_WORDS)))
        errors.append("{}: name {!r} is not a string.{}".format(where, key, hint))
    return ok


def _sorted_names(names) -> str:
    """Render a name set for an error message, tolerating odd key types."""
    return ", ".join(sorted(str(name) for name in names))


def _validate_limits(limits, errors: List[str]) -> None:
    """Append errors for a malformed ``limits`` block."""
    if not isinstance(limits, Mapping):
        errors.append("profile.limits must be a mapping")
        return
    for key, value in limits.items():
        if key not in DEFAULT_LIMITS:
            errors.append("profile.limits.{}: unknown limit".format(key))
        elif not _is_number(value):
            errors.append("profile.limits.{}: must be a number".format(key))
    for key in ("max_lin_vel", "max_ang_vel", "publish_hz"):
        if _is_number(limits.get(key, DEFAULT_LIMITS[key])) and \
                float(limits.get(key, DEFAULT_LIMITS[key])) <= 0.0:
            errors.append("profile.limits.{}: must be greater than zero".format(key))


def _validate_accessory(accessory, errors: List[str]) -> None:
    """Append errors for a malformed ``accessory`` block."""
    if accessory is None:
        return
    if not isinstance(accessory, Mapping):
        errors.append("profile.accessory must be a mapping")
        return

    neck = accessory.get("neck")
    if neck is not None:
        if not isinstance(neck, Mapping):
            errors.append("profile.accessory.neck must be a mapping")
        else:
            for key in ("min", "max", "default", "step"):
                if key in neck and not _is_number(neck[key]):
                    errors.append("profile.accessory.neck.{}: must be a number".format(key))
            low = neck.get("min", DEFAULT_NECK["min"])
            high = neck.get("max", DEFAULT_NECK["max"])
            if _is_number(low) and _is_number(high) and low >= high:
                errors.append("profile.accessory.neck: min must be less than max")

    gripper = accessory.get("gripper")
    if gripper is not None:
        if not isinstance(gripper, Mapping):
            errors.append("profile.accessory.gripper must be a mapping")
        else:
            for key in ("min", "front", "max", "default"):
                if key in gripper and not _is_number(gripper[key]):
                    errors.append("profile.accessory.gripper.{}: must be a number".format(key))
            low = gripper.get("min", DEFAULT_GRIPPER["min"])
            high = gripper.get("max", DEFAULT_GRIPPER["max"])
            if _is_number(low) and _is_number(high) and low >= high:
                errors.append("profile.accessory.gripper: min must be less than max")


def _validate_axes(axes, errors: List[str]) -> None:
    """Append errors for a malformed ``axes`` block."""
    if not isinstance(axes, Mapping) or not axes:
        errors.append("profile.axes must be a non-empty mapping of name -> {index: N}")
        return
    _check_names(axes, "profile.axes", errors)
    for name, spec in axes.items():
        if not isinstance(spec, Mapping):
            errors.append("profile.axes.{}: must be a mapping with an 'index'".format(name))
            continue
        index = spec.get("index")
        if not isinstance(index, int) or isinstance(index, bool) or index < 0:
            errors.append("profile.axes.{}.index: must be a non-negative integer".format(name))
        axis_range = spec.get("range")
        if axis_range is not None:
            if (not isinstance(axis_range, (list, tuple)) or len(axis_range) != 2
                    or not all(_is_number(v) for v in axis_range)):
                errors.append(
                    "profile.axes.{}.range: must be [rest, pressed]".format(name))
            elif axis_range[0] == axis_range[1]:
                errors.append(
                    "profile.axes.{}.range: rest and pressed must differ".format(name))


def _validate_buttons(buttons, errors: List[str]) -> None:
    """Append errors for a malformed ``buttons`` block."""
    if not isinstance(buttons, Mapping) or not buttons:
        errors.append("profile.buttons must be a non-empty mapping of name -> index")
        return
    _check_names(buttons, "profile.buttons", errors)
    for name, index in buttons.items():
        if not isinstance(index, int) or isinstance(index, bool) or index < 0:
            errors.append("profile.buttons.{}: must be a non-negative integer".format(name))
        if name in DPAD_NAMES:
            errors.append(
                "profile.buttons.{}: D-Pad names are reserved, declare them "
                "under profile.dpad".format(name))


def _validate_dpad(dpad, errors: List[str]) -> None:
    """Append errors for a malformed ``dpad`` block."""
    if dpad is None:
        return
    if not isinstance(dpad, Mapping):
        errors.append("profile.dpad must be a mapping")
        return

    source = dpad.get("source")
    if source == "axes":
        for key in ("x_axis", "y_axis"):
            value = dpad.get(key)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                errors.append("profile.dpad.{}: must be a non-negative integer".format(key))
    elif source == "buttons":
        for key in ("up", "down", "left", "right"):
            value = dpad.get(key)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                errors.append("profile.dpad.{}: must be a non-negative integer".format(key))
    else:
        errors.append("profile.dpad.source: must be 'axes' or 'buttons'")


def _validate_led_presets(presets, errors: List[str]) -> None:
    """Append errors for a malformed ``led_presets`` block."""
    if presets is None:
        return
    if not isinstance(presets, Mapping):
        errors.append("profile.led_presets must be a mapping")
        return
    _check_names(presets, "profile.led_presets", errors)
    for name, preset in presets.items():
        if not isinstance(preset, Mapping):
            errors.append("profile.led_presets.{}: must be a mapping".format(name))
            continue
        for corner in LED_CORNERS:
            entry = preset.get(corner)
            if not isinstance(entry, Mapping):
                errors.append(
                    "profile.led_presets.{}.{}: missing, expected "
                    "{{mode, color}}".format(name, corner))
                continue
            for key in ("mode", "color"):
                value = entry.get(key)
                if not isinstance(value, int) or isinstance(value, bool):
                    errors.append(
                        "profile.led_presets.{}.{}.{}: must be an "
                        "integer".format(name, corner, key))
                elif not -128 <= value <= 127:
                    errors.append(
                        "profile.led_presets.{}.{}.{}: outside int8 "
                        "range".format(name, corner, key))


def _validate_bindings(bindings, axes, buttons, dpad, presets, errors: List[str]) -> None:
    """Append errors for a malformed ``bindings`` block."""
    if not isinstance(bindings, Mapping) or not bindings:
        errors.append("profile.bindings must be a non-empty mapping")
        return

    known_axes = set(axes) if isinstance(axes, Mapping) else set()
    known_buttons = set(buttons) if isinstance(buttons, Mapping) else set()
    if isinstance(dpad, Mapping) and dpad.get("source") in ("axes", "buttons"):
        known_buttons |= set(DPAD_NAMES)
    known_presets = set(presets) if isinstance(presets, Mapping) else set()

    for action, spec in bindings.items():
        if action not in VOCABULARY:
            errors.append(
                "profile.bindings.{}: unknown action. Valid actions are: "
                "{}".format(action, ", ".join(VOCABULARY)))
            continue
        if not isinstance(spec, Mapping):
            errors.append("profile.bindings.{}: must be a mapping".format(action))
            continue

        control = spec.get("control")
        if action in AXIS_ACTIONS and control != "axis":
            errors.append(
                "profile.bindings.{}: is a continuous action and needs "
                "'control: axis'".format(action))
            continue
        if action in BUTTON_ACTIONS and control != "button":
            errors.append(
                "profile.bindings.{}: is a discrete action and needs "
                "'control: button'".format(action))
            continue

        if control == "axis":
            name = spec.get("axis")
            if name not in known_axes:
                errors.append(
                    "profile.bindings.{}.axis: '{}' is not declared under "
                    "profile.axes ({})".format(
                        action, name, _sorted_names(known_axes) or "none"))
            if "invert" in spec and not isinstance(spec["invert"], bool):
                errors.append("profile.bindings.{}.invert: must be true or false".format(action))
        else:
            name = spec.get("button")
            if name not in known_buttons:
                errors.append(
                    "profile.bindings.{}.button: '{}' is not declared under "
                    "profile.buttons ({})".format(
                        action, name, _sorted_names(known_buttons) or "none"))
            trigger = spec.get("trigger", "press")
            if trigger not in TRIGGERS:
                errors.append(
                    "profile.bindings.{}.trigger: must be one of {}".format(
                        action, ", ".join(TRIGGERS)))

        if action == "neck_preset" and not _is_number(spec.get("value")):
            errors.append("profile.bindings.neck_preset.value: must be a number")
        if action == "led_preset":
            preset = spec.get("preset")
            if preset not in known_presets:
                errors.append(
                    "profile.bindings.led_preset.preset: '{}' is not declared under "
                    "profile.led_presets ({})".format(
                        preset, _sorted_names(known_presets) or "none"))


def validate_document(document) -> List[str]:
    """
    Return a list of human-readable errors; empty means valid.

    Never raises, and never stops at the first problem -- the web GUI
    shows the whole list at once so a bad edit can be fixed in one pass.
    """
    errors: List[str] = []

    if not isinstance(document, Mapping):
        return ["document root must be a mapping with a 'profile' key"]

    profile = document.get("profile")
    if not isinstance(profile, Mapping):
        return ["document must have a top-level 'profile' mapping"]

    name = profile.get("name")
    if not isinstance(name, str) or not name.strip():
        errors.append("profile.name: must be a non-empty string")

    match = profile.get("match")
    if match is not None:
        if not isinstance(match, Mapping):
            errors.append("profile.match: must be a mapping or omitted")
        else:
            for key in ("axes", "buttons"):
                value = match.get(key)
                if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                    errors.append(
                        "profile.match.{}: must be a non-negative integer".format(key))

    _validate_limits(profile.get("limits", {}), errors)
    _validate_accessory(profile.get("accessory"), errors)
    _validate_axes(profile.get("axes"), errors)
    _validate_buttons(profile.get("buttons"), errors)
    _validate_dpad(profile.get("dpad"), errors)
    _validate_led_presets(profile.get("led_presets"), errors)
    _validate_bindings(
        profile.get("bindings"),
        profile.get("axes"),
        profile.get("buttons"),
        profile.get("dpad"),
        profile.get("led_presets"),
        errors,
    )

    return errors


def profile_from_document(document, source_path: Optional[str] = None) -> Profile:
    """
    Build a :class:`Profile` from a document, merging in defaults.

    Raises :class:`ProfileError` listing every problem if the document is
    invalid.
    """
    errors = validate_document(document)
    if errors:
        raise ProfileError("; ".join(errors))

    spec = document["profile"]

    limits = dict(DEFAULT_LIMITS)
    limits.update({k: float(v) for k, v in (spec.get("limits") or {}).items()})

    accessory = spec.get("accessory") or {}
    neck = dict(DEFAULT_NECK)
    neck.update({k: float(v) for k, v in (accessory.get("neck") or {}).items()})
    gripper = dict(DEFAULT_GRIPPER)
    gripper.update({k: float(v) for k, v in (accessory.get("gripper") or {}).items()})

    return Profile(
        name=spec["name"],
        axes=copy.deepcopy(dict(spec.get("axes") or {})),
        buttons={k: int(v) for k, v in (spec.get("buttons") or {}).items()},
        dpad=copy.deepcopy(dict(spec.get("dpad") or {})),
        bindings=copy.deepcopy(dict(spec.get("bindings") or {})),
        limits=limits,
        neck=neck,
        gripper=gripper,
        led_presets=copy.deepcopy(dict(spec.get("led_presets") or {})),
        match=copy.deepcopy(spec.get("match")),
        description=spec.get("description", ""),
        source_path=source_path,
    )


def load_document(path: str):
    """Read and parse a profile YAML file without validating it."""
    with open(path, "r") as handle:
        return yaml.safe_load(handle)


def load_profile(path: str) -> Profile:
    """Load, validate and normalise a single profile file."""
    try:
        document = load_document(path)
    except yaml.YAMLError as exc:
        raise ProfileError("{}: {}".format(os.path.basename(path), exc)) from exc

    try:
        return profile_from_document(document, source_path=path)
    except ProfileError as exc:
        raise ProfileError("{}: {}".format(os.path.basename(path), exc)) from exc


def load_profile_dir(directory: str):
    """
    Load every ``*.yaml`` in a directory.

    Returns ``(profiles, errors)`` where ``profiles`` maps file stem to
    :class:`Profile` and ``errors`` maps file stem to a message.  One
    broken file must not prevent the others from loading -- otherwise a
    typo in an unused profile takes the robot's joystick offline.
    """
    profiles: Dict[str, Profile] = {}
    errors: Dict[str, str] = {}

    for path in sorted(glob.glob(os.path.join(directory, "*.yaml"))):
        stem = os.path.splitext(os.path.basename(path))[0]
        try:
            profiles[stem] = load_profile(path)
        except (ProfileError, OSError) as exc:
            errors[stem] = str(exc)

    return profiles, errors


def select_profile(
    profiles: Mapping[str, Profile],
    axes_count: int,
    buttons_count: int,
    fallback: str = "generic",
) -> Optional[str]:
    """
    Pick a profile stem by controller fingerprint.

    An exact ``match`` on both counts wins.  Ties are broken by stem name
    so selection is deterministic.  A profile without ``match`` is never
    auto-selected, which is how ``generic.yaml`` stays reachable only as
    the explicit fallback.
    """
    for stem in sorted(profiles):
        match = profiles[stem].match
        if match and match.get("axes") == axes_count and \
                match.get("buttons") == buttons_count:
            return stem

    if fallback in profiles:
        return fallback
    return next(iter(sorted(profiles)), None)
