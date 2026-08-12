"""
Read, validate and rewrite the leading-behaviour constants YAML.

Pure Python -- no ``rclpy``, no Flask.

These files configure the thesis experiments, so the contract they have to
satisfy is set by what actually consumes them, not by what YAML permits.
Two consumers matter:

``ConstantParamsToBlackboard`` (``blackboard_managers.py``)
    Reads the file with a plain ``yaml.safe_load``, descends through
    ``bottom_up_tree_node`` -> ``ros__parameters``, then runs
    ``ast.literal_eval`` over every checkpoint, LED pattern and gesture
    entry.  Those entries are *Python dict literals stored as YAML
    strings*.  Emit one as a real YAML mapping and ``literal_eval`` gets a
    ``dict`` instead of a ``str`` and the tree throws during ``setup()``.
    That constraint drives the whole emitter below.

``LEDBehaviourSequence`` (``LED_behaviours.py``)
    Steps ``index`` up to ``len(patterns)`` and reads
    ``delays[index - 1]``, so a ``_times`` list shorter than its ``_seq``
    list raises ``IndexError`` mid-tick.  Hence that is a hard error here,
    while a *longer* ``_times`` is only a warning -- the surplus entries
    are simply never read, which is the current state of both shipped
    files.

A number's *type* is part of that contract, not a detail of how it is
written.  Several of these values are assigned straight into a ROS
message field, and those fields are strictly typed: ``parse_checkpoint``
does ``point.z = coordinates["Z"]`` into a ``float64``, so an ``int``
there is ``AssertionError: The 'z' field must be of type 'float'`` at
``setup()``, and ``parse_led`` does the mirror image into ``int8`` fields,
where a float is the error.  The browser cannot express the difference --
JSON sends ``0`` and ``0.0`` identically, and ``JSON.stringify`` drops the
decimal point off every whole number -- so the type is decided here, from
what the tree does with each key, rather than taken from the request.
See :data:`CHECKPOINT_AXES` and the constants beside it.

Note the ROS-parameter route is vestigial: the launch file passes these
files to nodes named ``doglike_leading_bt_node`` and friends while the
document is rooted at ``bottom_up_tree_node``, so ROS discards every key.
Only the ``yaml.safe_load`` path above is live, and it runs at ``setup()``
time -- an edit therefore applies on the next tree start, not live.
"""

import ast
import math
import textwrap
from typing import Dict, List, Optional, Tuple

import yaml

from . import flat_store, led_enums, yaml_io

#: The nesting every behaviour constants file uses, whichever tree loads it.
#: Mirrors ``blackboard_managers.YAML_ROOT_KEYS``.
ROOT_KEYS = ("bottom_up_tree_node", "ros__parameters")

#: Numeric parameters, with the units and meanings the source files
#: document in trailing comments.  ``Dog_max_wander_allowed`` is required
#: too but is written separately by ``_write_thresholds``.
SCALAR_PARAMS = (
    ("init_delay", "s", "Delay before the tree starts acting"),
    ("robot_closeness_threshold", "m",
     "Minimum approach distance; below this the robot risks hitting the subject"),
    ("robot_approach_distance", "m", "How far the robot advances in one step"),
    ("target_reached_threshold", "m",
     "Distance between target and subject that counts as arrived"),
    ("visibility_time_threshold", "s", "Age of the last subject fix before it is stale"),
    ("Dog_following_max_threshold", "m",
     "Largest distance difference that still counts as following"),
    ("Dog_max_wander_allowed", "count",
     "Wander events tolerated before the subject is considered lost"),
)

SCALAR_NAMES = tuple(name for name, _unit, _doc in SCALAR_PARAMS)

#: LED signalling scripts. Each needs a ``_seq`` and a ``_times`` list.
LED_SCRIPTS = (
    "LED_indicate_target",
    "LED_indicate_close_target",
    "LED_catch_attention",
    "LED_thank",
)

#: Accessory gesture scripts. Same ``_seq`` / ``_times`` shape.
GESTURE_SCRIPTS = (
    "Dog_indicate_target",
    "Dog_catch_attention",
    "Dog_thank",
)

#: LED numbering is shared with the joystick profiles; see led_enums.
LED_CORNERS = led_enums.LED_CORNERS
COLOR_NAMES = led_enums.COLOR_NAMES
MODE_NAMES = led_enums.MODE_NAMES

#: Accessory travel limits, for range warnings. Match the joystick profiles.
NECK_RANGE = (2.0, 8.6)
GRIPPER_RANGE = (1.6, 8.54)

# ── what type each value has to be written as ────────────────────────────
#
# Two things decide this, and neither is what the browser sent:
#
# * A value the tree assigns into a ROS message field has to match that
#   field, because the assignment is unchecked -- the message class does
#   the checking, by raising.
# * A value the tree casts itself (``float(params[key])`` for the
#   thresholds, ``int()`` or ``float()`` for the tunables) cannot break
#   either way, but is still written in the shape it is read in, so that
#   the file says what kind of quantity it holds.

#: Checkpoint axes. ``parse_checkpoint`` assigns each into a
#: ``geometry_msgs/Point``, whose fields are ``float64`` -- so a whole
#: number written as ``0`` rather than ``0.0`` crashes the tree at setup.
CHECKPOINT_AXES = ("X", "Y", "Z")

#: Accessory positions. ``parse_gesture`` casts these to ``float`` before
#: they reach ``AccessMotorCmd``, so an int survives; written as floats
#: anyway, since they are travel positions rather than counts.
GESTURE_FIELDS = ("n_pos", "gl_pos", "gr_pos")

#: LED corner fields, the mirror image of the checkpoints: ``parse_led``
#: assigns them uncast into ``SetLedStatus``, whose fields are ``int8``.
LED_FIELDS = ("mode", "color")

#: The range an ``int8`` message field accepts.
INT8_RANGE = (-128, 127)

#: Scalars that are counts rather than measurements. Everything else in
#: :data:`SCALAR_PARAMS` is a distance or a duration and is a float.
INTEGER_SCALARS = frozenset({"Dog_max_wander_allowed"})

#: Tunables the tree casts with ``int()``. Mirrors
#: ``behaviours/constants.py``'s ``INTEGER_TUNABLES``; a tunable this
#: module has never heard of keeps whichever shape the file gave it.
INTEGER_TUNABLES = frozenset({"turn_corrections", "recover_retries"})


def _script_keys() -> frozenset:
    """Return every key the structured editor renders as a sequence."""
    keys = {"Dog_checkpoints", "LED_start_setting"}
    for script in LED_SCRIPTS + GESTURE_SCRIPTS:
        keys.add("{}_seq".format(script))
        keys.add("{}_times".format(script))
    return frozenset(keys)


#: Keys this module gives a purpose-built editor to.  Everything else in
#: the file is a *tunable*: one of the turn speeds, timeouts and poses
#: that used to be constructor defaults in the behaviour library and now
#: live in the YAML so that a run is described by one file.
#:
#: Tunables are carried through unchanged unless they are edited.  That
#: is load bearing rather than tidy: this emitter rewrites the whole
#: document, so a key it does not know about is a key it would delete,
#: and the shipped files declare about thirty of them.
STRUCTURED_KEYS = frozenset(SCALAR_NAMES) | _script_keys()


class BehaviourStoreError(Exception):
    """Raised when a behaviour file cannot be read or parsed at all."""


# ── numbers: coercion and formatting ─────────────────────────────────────

def _is_number(value) -> bool:
    """Report whether a value is a real number, excluding bool."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def as_float(name: str, value) -> float:
    """Return ``value`` as a float, or raise naming the field."""
    if not _is_number(value):
        raise BehaviourStoreError("{}: must be a number, got {}".format(
            name, type(value).__name__))
    number = float(value)
    if not math.isfinite(number):
        raise BehaviourStoreError("{}: must be a finite number".format(name))
    return number


def as_int(name: str, value, limits: Optional[Tuple[int, int]] = None) -> int:
    """
    Return ``value`` as an int, or raise naming the field.

    A fractional value is refused rather than rounded: these are message
    fields and enum codes, so the nearest whole number is a different
    setting, not an approximation of the one asked for.
    """
    if not _is_number(value):
        raise BehaviourStoreError("{}: must be a whole number, got {}".format(
            name, type(value).__name__))
    if not math.isfinite(float(value)) or float(value) != int(value):
        raise BehaviourStoreError("{}: must be a whole number, got {}".format(
            name, value))
    number = int(value)
    if limits and not limits[0] <= number <= limits[1]:
        raise BehaviourStoreError("{}: {} is outside {}..{}".format(
            name, number, limits[0], limits[1]))
    return number


def _float_text(value: float) -> str:
    """Render a float so it reads back as a float, without float noise."""
    return repr(float(value))


def _int_text(value: int) -> str:
    """Render an int so it reads back as an int."""
    return str(int(value))


# ── literal entries: parse and emit ──────────────────────────────────────

def parse_entry(entry):
    """Parse one dict-literal string, exactly as the behaviour tree does."""
    if not isinstance(entry, str):
        raise BehaviourStoreError(
            "expected a quoted Python dict literal, got {}".format(type(entry).__name__))
    return ast.literal_eval(entry)


def _field(entry: Dict, key: str, field: str):
    """Return one field of a literal, or raise naming what is missing."""
    if not isinstance(entry, dict) or field not in entry:
        raise BehaviourStoreError("{}: missing '{}'".format(key, field))
    return entry[field]


def format_led(corners: Dict) -> str:
    """Emit an LED pattern as the quoted dict literal the tree expects."""
    parts = []
    for corner in LED_CORNERS:
        entry = _field(corners, "LED pattern", corner)
        values = [as_int("LED {}.{}".format(corner, field),
                         _field(entry, "LED " + corner, field), INT8_RANGE)
                  for field in LED_FIELDS]
        parts.append("'{}':{{'mode':{}, 'color':{}}}".format(
            corner, _int_text(values[0]), _int_text(values[1])))
    return "{" + ",".join(parts) + "}"


def format_gesture(positions: Dict) -> str:
    """Emit an accessory pose as the quoted dict literal the tree expects."""
    values = [_float_text(as_float("pose " + field,
                                   _field(positions, "pose", field)))
              for field in GESTURE_FIELDS]
    return "{{'n_pos':{},'gl_pos':{},'gr_pos':{}}}".format(*values)


def format_checkpoint(point: Dict) -> str:
    """
    Emit a route checkpoint as the quoted dict literal the tree expects.

    Always as floats: the tree puts these into a ``Point``, which refuses
    an int, and a whole coordinate arrives from the browser as one.
    """
    values = [_float_text(as_float("checkpoint " + axis,
                                   _field(point, "checkpoint", axis)))
              for axis in CHECKPOINT_AXES]
    return "{{'X':{}, 'Y':{}, 'Z':{}}}".format(*values)


#: Describe an LED pattern in words, for the regenerated comment.
describe_led = led_enums.describe_corners


def describe_gesture(positions: Dict) -> str:
    """Describe an accessory pose in words, for the regenerated comment."""
    try:
        neck = float(positions["n_pos"])
        left = float(positions["gl_pos"])
        right = float(positions["gr_pos"])
    except (KeyError, TypeError, ValueError):
        return "malformed"

    if neck >= 7.5:
        head = "look ahead"
    elif neck >= 5.5:
        head = "look level"
    else:
        head = "look down"

    if abs(left - right) < 0.25:
        grip = "grippers together"
    elif left > right:
        grip = "gripper open"
    else:
        grip = "gripper closed"

    return "{}, {}".format(grip, head)


# ── loading ──────────────────────────────────────────────────────────────

def load_params(path: str) -> Dict:
    """Read a behaviour file and return its ``ros__parameters`` mapping."""
    try:
        with open(path, "r") as stream:
            document = yaml.safe_load(stream)
    except OSError as exc:
        raise BehaviourStoreError("cannot read {}: {}".format(path, exc)) from exc
    except yaml.YAMLError as exc:
        raise BehaviourStoreError("{} is not valid YAML: {}".format(path, exc)) from exc

    for key in ROOT_KEYS:
        if not isinstance(document, dict) or key not in document:
            raise BehaviourStoreError(
                "{}: expected the document to be nested under {}".format(
                    path, " -> ".join(ROOT_KEYS)))
        document = document[key]

    if not isinstance(document, dict):
        raise BehaviourStoreError(
            "{}: {} is not a mapping".format(path, " -> ".join(ROOT_KEYS)))
    return document


def parse_params(params: Dict) -> Dict:
    """
    Turn raw params into the structured form the web page edits.

    Entries that fail to parse are reported rather than raised, so a file
    with one bad pattern still opens in the editor for repair.
    """
    structured = {
        "scalars": {},
        "tunables": [],
        "extras": {},
        "checkpoints": [],
        "led_start": None,
        "led_scripts": {},
        "gesture_scripts": {},
        "parse_errors": [],
    }

    for name in SCALAR_NAMES:
        if name in params:
            structured["scalars"][name] = params[name]

    for name, value in params.items():
        if name in STRUCTURED_KEYS:
            continue
        if _is_number(value) or isinstance(value, str):
            structured["tunables"].append({"name": name, "value": value})
        else:
            # Neither a tunable nor anything this editor renders: kept so
            # that saving cannot lose it.
            structured["extras"][name] = value

    for index, entry in enumerate(params.get("Dog_checkpoints") or ()):
        try:
            structured["checkpoints"].append(parse_entry(entry))
        except (BehaviourStoreError, ValueError, SyntaxError) as exc:
            structured["parse_errors"].append(
                "Dog_checkpoints[{}]: {}".format(index, exc))

    start = params.get("LED_start_setting") or ()
    if start:
        try:
            structured["led_start"] = parse_entry(start[0])
        except (BehaviourStoreError, ValueError, SyntaxError) as exc:
            structured["parse_errors"].append("LED_start_setting[0]: {}".format(exc))

    for group, scripts in (("led_scripts", LED_SCRIPTS),
                           ("gesture_scripts", GESTURE_SCRIPTS)):
        for script in scripts:
            sequence = []
            for index, entry in enumerate(params.get("{}_seq".format(script)) or ()):
                try:
                    sequence.append(parse_entry(entry))
                except (BehaviourStoreError, ValueError, SyntaxError) as exc:
                    structured["parse_errors"].append(
                        "{}_seq[{}]: {}".format(script, index, exc))
                    sequence.append(None)
            times = list(params.get("{}_times".format(script)) or ())
            structured[group][script] = {"seq": sequence, "times": times}

    return structured


def annotate_tunables(structured: Dict, path: str) -> Dict:
    """
    Attach each tunable's own documentation from the file it came from.

    The page shows a tunable next to the sentence in the file that says
    what it does, which is the only description of it anywhere -- these
    keys have no table in this module the way the thresholds do.
    """
    try:
        with open(path, "r") as stream:
            notes = annotations(stream.read())
    except OSError:
        notes = {}

    for entry in structured.get("tunables") or ():
        note = notes.get(entry["name"]) or {}
        entry["doc"] = note.get("doc", "")
        entry["section"] = note.get("section", "")
        entry["comment"] = note.get("comment", "")
    return structured


def params_from_structured(structured: Dict) -> Dict:
    """Rebuild the raw params mapping from the structured editor form."""
    params: Dict = {}

    for name in SCALAR_NAMES:
        if name in structured.get("scalars", {}):
            params[name] = structured["scalars"][name]

    for entry in structured.get("tunables") or ():
        params[entry["name"]] = entry["value"]

    for name, value in (structured.get("extras") or {}).items():
        params[name] = value

    params["Dog_checkpoints"] = [
        format_checkpoint(point) for point in structured.get("checkpoints") or ()]

    if structured.get("led_start") is not None:
        params["LED_start_setting"] = [format_led(structured["led_start"])]

    for script in LED_SCRIPTS:
        block = (structured.get("led_scripts") or {}).get(script)
        if block is None:
            continue
        params["{}_seq".format(script)] = [
            format_led(entry) for entry in block.get("seq") or ()]
        params["{}_times".format(script)] = list(block.get("times") or ())

    for script in GESTURE_SCRIPTS:
        block = (structured.get("gesture_scripts") or {}).get(script)
        if block is None:
            continue
        params["{}_seq".format(script)] = [
            format_gesture(entry) for entry in block.get("seq") or ()]
        params["{}_times".format(script)] = list(block.get("times") or ())

    return normalize(params)


def _renormalized(entries, formatter) -> List:
    """
    Rewrite each literal entry in the types the tree needs.

    Best effort by design: an entry that cannot be parsed, or whose
    fields cannot be typed at all, is handed back exactly as it came in
    so that :func:`validate` is the one place a problem is reported.
    """
    rewritten = []
    for entry in entries or ():
        try:
            rewritten.append(formatter(parse_entry(entry)))
        except (BehaviourStoreError, ValueError, SyntaxError, TypeError):
            rewritten.append(entry)
    return rewritten


def normalize(params: Dict) -> Dict:
    """
    Return ``params`` with every value written in the type the tree needs.

    This is what stops the editor writing a file it cannot itself load:
    the page sends whatever JSON makes of a number, and a coordinate the
    operator left at zero arrives as ``0``, which the tree cannot assign
    to ``Point.z``.  Rather than refuse the save, the type is restored
    from the key -- the operator typed a coordinate, not an integer.

    Never raises.  A value that cannot be typed is left exactly as found
    for :func:`validate` to report; nothing here decides what is a valid
    document.
    """
    if not isinstance(params, dict):
        return params

    result = dict(params)

    for name in SCALAR_NAMES:
        if name not in result:
            continue
        try:
            result[name] = (as_int(name, result[name])
                            if name in INTEGER_SCALARS
                            else as_float(name, result[name]))
        except BehaviourStoreError:
            pass

    for key, formatter in [("Dog_checkpoints", format_checkpoint),
                           ("LED_start_setting", format_led)]:
        if isinstance(result.get(key), list):
            result[key] = _renormalized(result[key], formatter)

    for group, formatter in ((LED_SCRIPTS, format_led),
                             (GESTURE_SCRIPTS, format_gesture)):
        for script in group:
            seq_key = "{}_seq".format(script)
            times_key = "{}_times".format(script)
            if isinstance(result.get(seq_key), list):
                result[seq_key] = _renormalized(result[seq_key], formatter)
            if isinstance(result.get(times_key), list):
                # Delays reach rclpy as Duration(seconds=float(...)), so an
                # int is harmless -- but a delay is a duration, and the file
                # should not have to be read twice to see that.
                result[times_key] = [
                    float(value) if _is_number(value) and math.isfinite(value)
                    else value
                    for value in result[times_key]]

    return result


# ── validation ───────────────────────────────────────────────────────────

def _validate_led_corners(parsed, label, errors, warnings):
    """Check one LED pattern's four corners, as ``parse_led`` will read it."""
    for corner in LED_CORNERS:
        entry_corner = parsed.get(corner) if isinstance(parsed, dict) else None
        if not isinstance(entry_corner, dict):
            errors.append("{}: missing corner '{}'".format(label, corner))
            continue
        for field in LED_FIELDS:
            value = entry_corner.get(field)
            if not isinstance(value, int) or isinstance(value, bool):
                # SetLedStatus' fields are int8 and parse_led assigns to
                # them uncast, so 4.0 is as fatal here as "four".
                errors.append("{}.{}.{}: must be an integer".format(
                    label, corner, field))
            elif not INT8_RANGE[0] <= value <= INT8_RANGE[1]:
                errors.append("{}.{}.{}: outside int8 range".format(
                    label, corner, field))
            elif field == "mode" and value not in MODE_NAMES:
                warnings.append("{}.{}.mode: {} is not a known mode".format(
                    label, corner, value))
            elif field == "color" and value not in COLOR_NAMES:
                warnings.append("{}.{}.color: {} is not a known colour".format(
                    label, corner, value))


def _validate_sequence(params, script, kind, errors, warnings):
    """Check one ``_seq`` / ``_times`` pair."""
    seq_key = "{}_seq".format(script)
    times_key = "{}_times".format(script)

    sequence = params.get(seq_key)
    times = params.get(times_key)

    if sequence is None:
        errors.append("{}: missing".format(seq_key))
        return
    if not isinstance(sequence, list) or not sequence:
        errors.append("{}: must be a non-empty list".format(seq_key))
        return
    if times is None:
        errors.append("{}: missing".format(times_key))
        return
    if not isinstance(times, list):
        errors.append("{}: must be a list".format(times_key))
        return

    for index, entry in enumerate(sequence):
        try:
            parsed = parse_entry(entry)
        except (BehaviourStoreError, ValueError, SyntaxError) as exc:
            errors.append("{}[{}]: not a parseable dict literal ({})".format(
                seq_key, index, exc))
            continue

        if not isinstance(parsed, dict):
            errors.append("{}[{}]: literal must be a dict".format(seq_key, index))
            continue

        if kind == "led":
            _validate_led_corners(
                parsed, "{}[{}]".format(seq_key, index), errors, warnings)
        else:
            for field, limits in (("n_pos", NECK_RANGE),
                                  ("gl_pos", GRIPPER_RANGE),
                                  ("gr_pos", GRIPPER_RANGE)):
                value = parsed.get(field)
                if not _is_number(value):
                    errors.append("{}[{}].{}: must be a number".format(
                        seq_key, index, field))
                elif not limits[0] <= float(value) <= limits[1]:
                    warnings.append(
                        "{}[{}].{}: {} is outside the travel range {}..{}".format(
                            seq_key, index, field, value, limits[0], limits[1]))

    for index, value in enumerate(times):
        if not _is_number(value):
            errors.append("{}[{}]: must be a number".format(times_key, index))
        elif float(value) < 0.0:
            errors.append("{}[{}]: must not be negative".format(times_key, index))

    # LEDBehaviourSequence.update reads delays[index - 1] with index rising
    # to len(patterns), so a short _times list is an IndexError at runtime.
    if len(times) < len(sequence):
        errors.append(
            "{}: has {} entries but {} has {} -- the behaviour reads one delay "
            "per pattern and would raise IndexError mid-tick".format(
                times_key, len(times), seq_key, len(sequence)))
    elif len(times) > len(sequence):
        warnings.append(
            "{}: has {} entries but {} has only {} -- the surplus delays are "
            "never read. Most likely some patterns were dropped.".format(
                times_key, len(times), seq_key, len(sequence)))


def validate(params: Dict) -> Tuple[List[str], List[str]]:
    """
    Return ``(errors, warnings)`` for a raw params mapping.

    Errors block a save; warnings do not.  Never raises, and never stops
    at the first problem -- the page shows the whole list at once.
    """
    errors: List[str] = []
    warnings: List[str] = []

    if not isinstance(params, dict):
        return ["parameters must be a mapping"], []

    for name in SCALAR_NAMES:
        if name not in params:
            errors.append("{}: missing".format(name))
        elif not _is_number(params[name]):
            errors.append("{}: must be a number".format(name))
        elif not math.isfinite(float(params[name])):
            errors.append("{}: must be a finite number".format(name))
        elif name in INTEGER_SCALARS and float(params[name]) != int(params[name]):
            errors.append("{}: must be a whole number -- it is a count".format(name))

    checkpoints = params.get("Dog_checkpoints")
    if not isinstance(checkpoints, list) or not checkpoints:
        errors.append(
            "Dog_checkpoints: must be a non-empty list; the tree reads [0] as the "
            "start position and [-1] as the target")
    else:
        for index, entry in enumerate(checkpoints):
            try:
                point = parse_entry(entry)
            except (BehaviourStoreError, ValueError, SyntaxError) as exc:
                errors.append("Dog_checkpoints[{}]: not a parseable dict literal "
                              "({})".format(index, exc))
                continue
            for axis in CHECKPOINT_AXES:
                value = point.get(axis) if isinstance(point, dict) else None
                if not _is_number(value):
                    errors.append("Dog_checkpoints[{}].{}: must be a number".format(
                        index, axis))
                elif not isinstance(value, float):
                    # parse_checkpoint assigns straight into a Point, whose
                    # fields are float64: `Z: 0` is AssertionError at setup,
                    # not a rounding difference. Saving from this editor
                    # rewrites it as 0.0.
                    errors.append(
                        "Dog_checkpoints[{}].{}: {} is a whole number written as an "
                        "integer; the tree assigns it to Point.{} (float64) and "
                        "would raise at setup. Save this file to rewrite it as "
                        "{}.".format(index, axis, value, axis.lower(),
                                     _float_text(value)))
        if len(checkpoints) == 1:
            warnings.append(
                "Dog_checkpoints: only one checkpoint, so the start and target "
                "positions are the same point")

    start = params.get("LED_start_setting")
    if not isinstance(start, list) or not start:
        errors.append(
            "LED_start_setting: must be a list with at least one entry; the tree "
            "reads [0]")
    else:
        try:
            parsed = parse_entry(start[0])
        except (BehaviourStoreError, ValueError, SyntaxError) as exc:
            errors.append("LED_start_setting[0]: not a parseable dict literal "
                          "({})".format(exc))
        else:
            # Checked exactly like a sequence entry: ConstantParamsToBlackboard
            # sends this one through parse_led too, at initialise().
            _validate_led_corners(parsed, "LED_start_setting[0]", errors, warnings)

    for script in LED_SCRIPTS:
        _validate_sequence(params, script, "led", errors, warnings)
    for script in GESTURE_SCRIPTS:
        _validate_sequence(params, script, "gesture", errors, warnings)

    return errors, warnings


# ── emitting ─────────────────────────────────────────────────────────────

_HEADER = """\
# Leading-behaviour constants.
#
# Loaded by ConstantParamsToBlackboard (blackboard_managers.py) with a plain
# yaml.safe_load, which descends bottom_up_tree_node -> ros__parameters and
# then runs ast.literal_eval over every quoted entry below. Those entries are
# Python dict literals stored as YAML strings and MUST stay quoted -- turning
# one into a real YAML mapping breaks the tree at setup().
#
# Changes take effect the next time a behaviour tree starts, not live.
#
# LED  mode:  1 wave right, 2 wave left, 3 pulse, 4 solid,
#             5 fast blink, 6 slow blink
# LED  color: 0 black, 1 white, 2 green, 3 red,
#             4 blue,  5 cyan,  6 pink,  7 yellow
# Pose n_pos: neck tilt {neck_lo} .. {neck_hi}   gl_pos/gr_pos: {grip_lo} .. {grip_hi}
#
# Each *_times list supplies one delay per *_seq pattern. It may be longer
# (surplus entries are ignored) but never shorter.
"""


def _emit_scalars(lines: List[str], params: Dict) -> None:
    """Write the numeric parameters with their documented meanings."""
    width = max(len(name) for name in SCALAR_NAMES)
    for name, unit, doc in SCALAR_PARAMS:
        if name not in params:
            continue
        value = params[name]
        text = (_int_text(as_int(name, value)) if name in INTEGER_SCALARS
                else _float_text(as_float(name, value)))
        lines.append("    {name:<{width}} {value:<8} # {doc} [{unit}]".format(
            name=name + ":", width=width + 1,
            value=text, doc=doc, unit=unit))


def annotations(text: str) -> Dict[str, Dict]:
    """
    Return the comments the file itself attaches to each scalar.

    The tunables carry the reasoning for their values -- which Nav2 limit
    a turn speed has to stay inside, why a timeout is short -- and none
    of it can be regenerated from the numbers the way the LED and gesture
    comments can.  So it is read off the file before a save and written
    back out around the same keys.
    """
    try:
        parsed = flat_store.parse(text)
    except flat_store.FlatStoreError:
        return {}
    return {entry["name"]: entry for entry in parsed["entries"]}


def _emit_annotation(lines: List[str], note: Dict, section: Optional[str]) -> Optional[str]:
    """Write one tunable's section banner and comment block. Return the section."""
    if note.get("section") and note["section"] != section:
        section = note["section"]
        rule = "-" * max(3, 68 - len(section))
        lines.append("")
        lines.append("    # ----- {} {}".format(section, rule))
    for line in textwrap.wrap(note.get("doc") or "", width=72):
        lines.append("    # {}".format(line))
    return section


def _tunable_text(name: str, value, note: Dict) -> str:
    """
    Render one tunable in the shape it had, not the shape JSON gave it.

    The tree casts every tunable itself -- ``int()`` for the counts,
    ``float()`` for the rest -- so neither type can break a run.  What a
    wrong type breaks is the file: ``turn_max_speed: 1`` reads as a count
    of something, and the browser writes exactly that whenever a speed is
    set to a whole number.  So the shape comes from the same place the
    comments do, the file being replaced, with the two known counts
    pinned and a measurement assumed when the file has nothing to say.
    """
    if isinstance(value, str):
        return value
    if name in INTEGER_TUNABLES:
        return _int_text(as_int(name, value))
    original = note.get("value")
    if _is_number(original) and isinstance(original, int):
        try:
            return _int_text(as_int(name, value))
        except BehaviourStoreError:
            pass
    return _float_text(as_float(name, value))


def _emit_tunables(lines: List[str], params: Dict, notes: Dict) -> None:
    """Write every scalar the structured editor does not own itself."""
    tunables = [(name, value) for name, value in params.items()
                if name not in STRUCTURED_KEYS and (
                    _is_number(value) or isinstance(value, str))]
    if not tunables:
        return

    lines.append("")
    lines.append("    # ======================================================"
                 "================")
    lines.append("    # Tunables. Every key below used to be a constructor "
                 "default or a module")
    lines.append("    # constant in the behaviour library. A file that omits "
                 "one keeps the")
    lines.append("    # packaged default in `behaviours/constants.py`.")
    lines.append("    #")
    lines.append("    # Angles are in degrees and reach the blackboard in "
                 "radians, under the")
    lines.append("    # same name without the `_deg` suffix.")
    lines.append("    # ======================================================"
                 "================")

    section = None
    for name, value in tunables:
        note = notes.get(name) or {}
        new_section = _emit_annotation(lines, note, section)
        if new_section != section:
            section = new_section
        comment = note.get("comment") or ""
        lines.append("    {}: {}{}".format(
            name, _tunable_text(name, value, note),
            "  # " + comment if comment else ""))


def _emit_extras(lines: List[str], params: Dict) -> None:
    """
    Write anything this editor neither owns nor recognises as a tunable.

    Nothing in the shipped files lands here.  It exists so that a key
    somebody adds by hand -- a new list, a nested mapping -- survives a
    save from the GUI rather than being deleted by an emitter that had
    never heard of it.
    """
    extras = [(name, value) for name, value in params.items()
              if name not in STRUCTURED_KEYS
              and not _is_number(value) and not isinstance(value, str)]
    if not extras:
        return

    lines.append("")
    lines.append("    # Kept as found: this editor does not render these.")
    for name, value in extras:
        dumped = yaml.safe_dump(value, default_flow_style=True).strip()
        if dumped.endswith("..."):
            dumped = dumped[:-3].strip()
        lines.append("    {}: {}".format(name, dumped))


def _emit_literal_list(lines: List[str], key: str, entries, comments=None) -> None:
    """Write a list of quoted dict literals, one per line, with comments."""
    lines.append("    {}:".format(key))
    for index, entry in enumerate(entries):
        comment = ""
        if comments and index < len(comments) and comments[index]:
            comment = "  # {}".format(comments[index])
        lines.append('      - "{}"{}'.format(entry, comment))


def _emit_times(lines: List[str], key: str, times, comments=None) -> None:
    """Write a delay list, annotated with what each delay is waiting on."""
    lines.append("    {}:".format(key))
    for index, value in enumerate(times):
        comment = ""
        if comments and index < len(comments) and comments[index]:
            comment = "  # {}".format(comments[index])
        elif comments:
            comment = "  # (surplus, never read)"
        lines.append("      - {}{}".format(
            _float_text(as_float("{}[{}]".format(key, index), value)), comment))


def dump(params: Dict, notes: Optional[Dict] = None) -> str:
    """
    Render a params mapping as a behaviour constants YAML document.

    Emits every literal entry by hand rather than through a YAML dumper,
    so the quoting the behaviour tree depends on cannot be lost, and
    self-checks the result with ``ast.literal_eval`` before returning.

    Comments are handled two ways, because the file has two kinds.  Those
    annotating an LED pattern or a gesture pose are regenerated from the
    values -- a deliberate upgrade, since several in the original files
    contradict what they annotate.  Those on the tunables are the
    reasoning behind a number and cannot be regenerated at all, so they
    are carried over from the file being replaced, via ``notes``.
    """
    lines: List[str] = []
    lines.append(_HEADER.format(
        neck_lo=NECK_RANGE[0], neck_hi=NECK_RANGE[1],
        grip_lo=GRIPPER_RANGE[0], grip_hi=GRIPPER_RANGE[1]).rstrip())
    lines.append("")
    lines.append("{}:".format(ROOT_KEYS[0]))
    lines.append("  {}:".format(ROOT_KEYS[1]))

    _emit_scalars(lines, params)
    _emit_tunables(lines, params, notes or {})
    _emit_extras(lines, params)

    checkpoints = params.get("Dog_checkpoints") or []
    if checkpoints:
        lines.append("")
        lines.append("    # Route. [0] is the start position, [-1] is the target.")
        comments = []
        for index in range(len(checkpoints)):
            if index == 0:
                comments.append("start")
            elif index == len(checkpoints) - 1:
                comments.append("target")
            else:
                comments.append("waypoint {}".format(index))
        _emit_literal_list(lines, "Dog_checkpoints", checkpoints, comments)

    start = params.get("LED_start_setting") or []
    if start:
        lines.append("")
        comments = []
        for entry in start:
            try:
                comments.append(describe_led(parse_entry(entry)))
            except (BehaviourStoreError, ValueError, SyntaxError):
                comments.append("")
        _emit_literal_list(lines, "LED_start_setting", start, comments)

    for group, scripts, describe in (
            ("led", LED_SCRIPTS, describe_led),
            ("gesture", GESTURE_SCRIPTS, describe_gesture)):
        for script in scripts:
            seq_key = "{}_seq".format(script)
            times_key = "{}_times".format(script)
            if seq_key not in params:
                continue

            sequence = params.get(seq_key) or []
            times = params.get(times_key) or []

            comments = []
            for entry in sequence:
                try:
                    comments.append(describe(parse_entry(entry)))
                except (BehaviourStoreError, ValueError, SyntaxError):
                    comments.append("")

            lines.append("")
            _emit_literal_list(lines, seq_key, sequence, comments)
            _emit_times(lines, times_key, times, comments)

    lines.append("")
    text = "\n".join(lines)

    _self_check(text, params)
    return text


def _self_check(text: str, params: Dict) -> None:
    """
    Reparse the emitted document and confirm it round-trips.

    Cheap insurance against the one failure mode that matters: a literal
    entry that stops being a string, or stops being ``literal_eval``-able,
    would not surface until a behaviour tree crashed during an experiment.
    """
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise BehaviourStoreError(
            "internal error: emitted document is not valid YAML ({})".format(exc))

    for key in ROOT_KEYS:
        document = (document or {}).get(key)
    if not isinstance(document, dict):
        raise BehaviourStoreError(
            "internal error: emitted document lost its {} nesting".format(
                " -> ".join(ROOT_KEYS)))

    missing = sorted(set(params) - set(document))
    if missing:
        # The emitter is a whitelist of shapes it knows how to write, so
        # this is the failure that matters: a key it did not recognise
        # would be gone from the file, and the tree would fall back to a
        # packaged default mid-experiment without saying so.
        raise BehaviourStoreError(
            "internal error: {} would be dropped from the file".format(
                ", ".join(missing)))

    literal_keys = [("Dog_checkpoints", "checkpoint"),
                    ("LED_start_setting", "led")]
    literal_keys += [("{}_seq".format(name), "led") for name in LED_SCRIPTS]
    literal_keys += [("{}_seq".format(name), "gesture") for name in GESTURE_SCRIPTS]

    for key, kind in literal_keys:
        original = params.get(key)
        if original is None:
            continue
        emitted = document.get(key)
        if not isinstance(emitted, list) or len(emitted) != len(original):
            raise BehaviourStoreError(
                "internal error: '{}' did not survive the round trip".format(key))
        for index, entry in enumerate(emitted):
            if not isinstance(entry, str):
                raise BehaviourStoreError(
                    "internal error: {}[{}] was emitted as a YAML mapping instead "
                    "of a quoted literal".format(key, index))
            try:
                parsed = ast.literal_eval(entry)
            except (ValueError, SyntaxError) as exc:
                raise BehaviourStoreError(
                    "internal error: {}[{}] is not literal_eval-able after "
                    "emission ({})".format(key, index, exc))
            _check_literal_types(parsed, kind, "{}[{}]".format(key, index))


def _check_literal_types(parsed, kind: str, label: str) -> None:
    """
    Confirm one emitted literal reads back as the types the tree assigns.

    The other half of the self-check, and the half that catches a whole
    number losing its decimal point: ``0`` and ``0.0`` are the same YAML
    document to every check above this one, and only one of them can be
    written into ``Point.z``.
    """
    if kind == "led":
        for corner in LED_CORNERS:
            entry = (parsed or {}).get(corner) or {}
            for field in LED_FIELDS:
                value = entry.get(field)
                if not isinstance(value, int) or isinstance(value, bool):
                    raise BehaviourStoreError(
                        "internal error: {}.{}.{} was emitted as {!r}, which the "
                        "tree cannot assign to an int8 field".format(
                            label, corner, field, value))
        return

    fields = CHECKPOINT_AXES if kind == "checkpoint" else GESTURE_FIELDS
    for field in fields:
        value = (parsed or {}).get(field)
        if not isinstance(value, float):
            raise BehaviourStoreError(
                "internal error: {}.{} was emitted as {!r}, which the tree cannot "
                "assign to a float64 field".format(label, field, value))


# ── saving ───────────────────────────────────────────────────────────────

def save(path: str, params: Dict, backup_root=yaml_io.DEFAULT_BACKUP_ROOT):
    """
    Validate and write a behaviour file, backing up the previous version.

    Returns ``(ok, errors, warnings, backup_path)``.  Nothing is written
    when ``errors`` is non-empty.

    Types are restored before validation, not after, so that a save also
    repairs a file that already carries the wrong ones -- which is how a
    document written before this editor typed its numbers gets fixed:
    open it and press save.
    """
    params = normalize(params)
    errors, warnings = validate(params)
    if errors:
        return False, errors, warnings, None

    # Read the outgoing file for the comments on its tunables, so the
    # reasoning written next to those numbers survives the rewrite.
    try:
        with open(path, "r") as stream:
            notes = annotations(stream.read())
    except OSError:
        notes = {}

    try:
        text = dump(params, notes)
    except BehaviourStoreError as exc:
        return False, [str(exc)], warnings, None

    backup = yaml_io.make_backup(path, root=backup_root)
    yaml_io.atomic_write_text(path, text)
    return True, [], warnings, backup
