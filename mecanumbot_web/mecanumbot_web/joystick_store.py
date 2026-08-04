"""
Read, validate and rewrite joystick profile YAML files.

Pure Python -- no ``rclpy``, no Flask.

Validation is *not* implemented here.  It is delegated to
``mecanumbot_joy.profile.validate_document``, the same function the joy
node runs before it will load a profile.  Sharing one validator is what
guarantees this GUI cannot save a document the robot would then reject:
if the two ever drifted apart, the failure would show up as a joystick
that silently stops working after an apparently successful save.

Writes go through :mod:`mecanumbot_web.yaml_io`, which resolves the
share-directory symlink back to the source tree before renaming.  See
that module for why that matters.
"""

import os
from typing import Dict, List, Optional, Tuple

import yaml

from . import led_enums, yaml_io

try:
    from mecanumbot_joy.profile import (
        AXIS_ACTIONS,
        BUTTON_ACTIONS,
        DEFAULT_LIMITS,
        TRIGGERS,
        VOCABULARY,
        validate_document,
    )
    JOY_AVAILABLE = True
except ImportError:  # pragma: no cover - only when mecanumbot_joy is unbuilt
    AXIS_ACTIONS = ("drive", "strafe", "turn")
    BUTTON_ACTIONS = ()
    DEFAULT_LIMITS = {}
    TRIGGERS = ("press", "hold")
    VOCABULARY = AXIS_ACTIONS
    JOY_AVAILABLE = False

    def validate_document(document):
        """Fallback that refuses to validate rather than pretend to."""
        del document
        return ["mecanumbot_joy is not built, so profiles cannot be validated"]


#: Human-readable descriptions of each action, for the editor UI.
ACTION_DOCS = {
    "drive": "Forward / back (linear.x)",
    "strafe": "Left / right (linear.y)",
    "turn": "Rotate in place (angular.z)",
    "neck_up": "Tilt the neck up",
    "neck_down": "Tilt the neck down",
    "neck_preset": "Jump the neck to a fixed position",
    "gripper_open": "Open both grippers",
    "gripper_close": "Close both grippers",
    "estop": "Latch an emergency stop (press again, or clear from this page)",
    "led_preset": "Apply a named LED preset",
}

#: Units and meanings for the limits block, shown beside each field.
LIMIT_DOCS = {
    "max_lin_vel": ("m/s", "Full-stick linear speed"),
    "max_ang_vel": ("rad/s", "Full-stick rotation speed"),
    "deadzone": ("", "Stick movement below this is treated as centred"),
    "lin_step": ("m/s per tick", "Linear ramp rate; 0 disables ramping"),
    "ang_step": ("rad/s per tick", "Angular ramp rate; 0 disables ramping"),
    "publish_hz": ("Hz", "How often cmd_vel is published while driving"),
    "joy_timeout": ("s", "Silence from the pad before the wheels are stopped"),
    "zero_hold_ticks": ("ticks", "Zero commands sent after a stop, then silence"),
    "accessory_keepalive_hz": ("Hz", "Refresh rate for accessory positions"),
}


class JoystickStoreError(Exception):
    """Raised when a profile file cannot be read at all."""


def _profile_path(directory: str, stem: str) -> str:
    """Return the file path for a profile stem, rejecting path escapes."""
    if not stem or os.sep in stem or stem in (".", "..") or stem.startswith("."):
        raise JoystickStoreError("Invalid profile name '{}'".format(stem))
    return os.path.join(directory, "{}.yaml".format(stem))


def list_profiles(directory: str) -> List[dict]:
    """
    Summarise every profile in a directory, valid or not.

    Invalid profiles are listed with their errors rather than hidden, so
    a broken file is visible in the UI instead of silently missing.
    """
    if not directory or not os.path.isdir(directory):
        return []

    summaries = []
    for name in sorted(os.listdir(directory)):
        if not name.endswith(".yaml"):
            continue
        stem = os.path.splitext(name)[0]
        path = os.path.join(directory, name)

        entry = {
            "stem": stem,
            "path": path,
            "name": stem,
            "description": "",
            "match": None,
            "valid": False,
            "errors": [],
            "symlinked": yaml_io.is_symlinked_install(path),
        }

        try:
            with open(path, "r") as stream:
                document = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as exc:
            entry["errors"] = [str(exc)]
            summaries.append(entry)
            continue

        errors = validate_document(document)
        entry["errors"] = errors
        entry["valid"] = not errors
        spec = (document or {}).get("profile") or {}
        if isinstance(spec, dict):
            entry["name"] = spec.get("name", stem)
            entry["description"] = spec.get("description", "")
            entry["match"] = spec.get("match")

        summaries.append(entry)

    return summaries


def read_profile(directory: str, stem: str) -> dict:
    """Read one profile, returning its document, raw text and errors."""
    path = _profile_path(directory, stem)
    if not os.path.isfile(path):
        raise JoystickStoreError("No such profile '{}'".format(stem))

    raw = yaml_io.read_text(path)
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as exc:
        return {
            "stem": stem,
            "path": path,
            "document": None,
            "raw": raw,
            "errors": ["{} is not valid YAML: {}".format(os.path.basename(path), exc)],
            "controls": {"axes": [], "buttons": []},
            "symlinked": yaml_io.is_symlinked_install(path),
        }

    errors = validate_document(document)
    return {
        "stem": stem,
        "path": path,
        "document": document,
        "raw": raw,
        "errors": errors,
        "controls": controls_of(document),
        "symlinked": yaml_io.is_symlinked_install(path),
    }


def controls_of(document) -> Dict[str, List[str]]:
    """
    List the control names a document declares.

    Drives the editor's per-binding dropdowns.  Because the dropdown is
    built from this, a binding naming a control the pad does not have is
    not merely rejected on save -- it cannot be expressed in the UI.
    """
    spec = (document or {}).get("profile") or {}
    axes = sorted((spec.get("axes") or {}))
    buttons = sorted((spec.get("buttons") or {}))
    dpad = spec.get("dpad") or {}
    if isinstance(dpad, dict) and dpad.get("source") in ("axes", "buttons"):
        buttons = buttons + ["DPadUp", "DPadDown", "DPadLeft", "DPadRight"]
    return {"axes": axes, "buttons": buttons}


def editor_metadata() -> dict:
    """Return the static schema the editor page renders itself from."""
    return {
        "vocabulary": list(VOCABULARY),
        "axis_actions": list(AXIS_ACTIONS),
        "button_actions": list(BUTTON_ACTIONS),
        "triggers": list(TRIGGERS),
        "action_docs": ACTION_DOCS,
        "limit_docs": {key: list(value) for key, value in LIMIT_DOCS.items()},
        "limit_defaults": dict(DEFAULT_LIMITS),
        "colors": {str(k): v for k, v in led_enums.COLOR_NAMES.items()},
        "modes": {str(k): v for k, v in led_enums.MODE_NAMES.items()},
        "corners": list(led_enums.LED_CORNERS),
        "joy_available": JOY_AVAILABLE,
    }


# ── emitting ─────────────────────────────────────────────────────────────

def _scalar(value) -> str:
    """
    Render a value the way the shipped profiles write it.

    Lists are emitted in flow style rather than stringified.  Getting this
    wrong turns an axis ``range: [1.0, -1.0]`` into the *string*
    ``"[1.0, -1.0]"``, which parses back as a scalar and fails validation
    -- which is precisely what the re-validation in :func:`save_profile`
    exists to catch.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_scalar(item) for item in value) + "]"
    if isinstance(value, dict):
        return _flow(value)
    text = str(value)
    if text == "" or any(character in text for character in ":#{}[],&*!|>'\"%@`"):
        return '"{}"'.format(text.replace('"', '\\"'))
    return text


def _flow(mapping) -> str:
    """Render a small mapping in YAML flow style, as the profiles do."""
    return "{" + ", ".join(
        "{}: {}".format(key, _scalar(value)) for key, value in mapping.items()) + "}"


_HEADER = """\
# Joystick profile for mecanumbot_joy.
#
# `axes` / `buttons` / `dpad` describe the pad. `bindings` describes what
# the robot does, referring to controls by name. Rebinding an action never
# touches an index, and swapping pads never touches a semantic.
#
# Written by the robot's web GUI. Reload it into the running node with
#   ros2 service call /mecanumbot/mecanumbot_joy_node/reload_profile \\
#     std_srvs/srv/Trigger {}
# or press Reload on the joystick page.
"""


def dump(document) -> str:
    """
    Render a profile document back to YAML.

    Emitted by hand rather than through a YAML dumper so the layout stays
    diffable against the shipped profiles and the legends below stay
    attached to the values they explain.
    """
    spec = (document or {}).get("profile") or {}
    lines: List[str] = [_HEADER.rstrip(), "", "profile:"]

    lines.append("  name: {}".format(_scalar(spec.get("name", ""))))
    if spec.get("description"):
        lines.append("  description: {}".format(_scalar(spec["description"])))

    if spec.get("match"):
        lines.append("")
        lines.append("  # Auto-detection fingerprint. Omit to make this profile")
        lines.append("  # reachable only as an explicit choice or the fallback.")
        lines.append("  match:")
        for key in ("axes", "buttons"):
            if key in spec["match"]:
                lines.append("    {}: {}".format(key, _scalar(spec["match"][key])))

    limits = spec.get("limits") or {}
    if limits:
        lines.append("")
        lines.append("  limits:")
        key_width = max(len(key) for key in limits) + 1
        value_width = max(len(_scalar(value)) for value in limits.values())
        for key, value in limits.items():
            unit, doc = LIMIT_DOCS.get(key, ("", ""))
            entry = "    {:<{kw}} {:<{vw}}".format(
                key + ":", _scalar(value), kw=key_width, vw=value_width)
            if doc:
                entry += "  # {}{}".format(doc, " [{}]".format(unit) if unit else "")
            lines.append(entry.rstrip())

    accessory = spec.get("accessory") or {}
    if accessory:
        lines.append("")
        lines.append("  accessory:")
        for group in ("neck", "gripper"):
            if group in accessory:
                lines.append("    {}: {}".format(group, _flow(accessory[group])))

    axes = spec.get("axes") or {}
    if axes:
        lines.append("")
        lines.append("  # `range: [rest, pressed]` marks an analogue trigger and maps")
        lines.append("  # it onto 0.0 .. 1.0. Sticks need no range and stay -1.0 .. 1.0.")
        lines.append("  axes:")
        width = max(len(key) for key in axes)
        for key, value in axes.items():
            lines.append("    {:<{width}} {}".format(
                key + ":", _flow(value), width=width + 1))

    dpad = spec.get("dpad") or {}
    if dpad:
        lines.append("")
        lines.append("  dpad: {}".format(_flow(dpad)))

    buttons = spec.get("buttons") or {}
    if buttons:
        lines.append("")
        lines.append("  buttons:")
        width = max(len(key) for key in buttons)
        for key, value in buttons.items():
            lines.append("    {:<{width}} {}".format(
                key + ":", _scalar(value), width=width + 1))

    bindings = spec.get("bindings") or {}
    if bindings:
        lines.append("")
        lines.append("  bindings:")
        width = max(len(key) for key in bindings)
        for action in VOCABULARY:
            if action not in bindings:
                continue
            doc = ACTION_DOCS.get(action, "")
            lines.append("    {:<{width}} {}{}".format(
                action + ":", _flow(bindings[action]),
                "  # {}".format(doc) if doc else "", width=width + 1))
        for action in bindings:
            if action not in VOCABULARY:
                lines.append("    {:<{width}} {}".format(
                    action + ":", _flow(bindings[action]), width=width + 1))

    presets = spec.get("led_presets") or {}
    if presets:
        lines.append("")
        lines.append("  # mode:  {}".format(", ".join(
            "{} {}".format(k, v) for k, v in sorted(led_enums.MODE_NAMES.items()))))
        lines.append("  # color: {}".format(", ".join(
            "{} {}".format(k, v) for k, v in sorted(led_enums.COLOR_NAMES.items()))))
        lines.append("  led_presets:")
        for name, preset in presets.items():
            lines.append("    {}:   # {}".format(name, led_enums.describe_corners(preset)))
            for corner in led_enums.LED_CORNERS:
                if corner in preset:
                    lines.append("      {}: {}".format(corner, _flow(preset[corner])))

    lines.append("")
    return "\n".join(lines)


# ── saving ───────────────────────────────────────────────────────────────

def save_profile(
    directory: str,
    stem: str,
    document,
    backup_root: str = yaml_io.DEFAULT_BACKUP_ROOT,
) -> Tuple[bool, List[str], Optional[str], bool]:
    """
    Validate and write a profile.

    Returns ``(ok, errors, backup_path, created)``.  Nothing is written
    when validation fails, and the validator is the joy node's own -- see
    the module docstring.
    """
    path = _profile_path(directory, stem)
    created = not os.path.exists(path)

    errors = validate_document(document)
    if errors:
        return False, errors, None, created

    try:
        text = dump(document)
    except (TypeError, ValueError, KeyError) as exc:
        return False, ["Could not render the profile: {}".format(exc)], None, created

    # Re-validate what will actually hit disk, not just what was posted.
    reparsed = yaml.safe_load(text)
    errors = validate_document(reparsed)
    if errors:
        return False, ["Rendered profile failed validation: " + "; ".join(errors)], \
            None, created

    backup = yaml_io.make_backup(path, root=backup_root)
    yaml_io.atomic_write_text(path, text)
    return True, [], backup, created
