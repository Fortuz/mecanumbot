"""
Edit a flat ``ros__parameters`` constants file without losing its comments.

Pure Python -- no ``rclpy``, no Flask.

The ostensive constants are the flat schema in this workspace: one block
of scalars and two short lists, no LED or gesture scripts.  That makes
them simple to edit and *hard to rewrite*, which is the opposite of the
leading constants next door.  There, the inline comments are derived from
the values, so :mod:`behaviour_store` regenerates the whole document and
fixes stale annotations on the way.  Here the comments are the reasoning
-- why a threshold is a ratio against body scale, what happens if it is
raised, which detector a field of view has to match -- and no emitter can
reconstruct them.

So this module never rewrites the document.  It replaces the value on the
one line that carries it, leaves every comment, blank line and section
header exactly where it was, and then reparses the result and checks that
precisely the intended keys changed.  Adding or removing a parameter is
not offered: the tree reads a fixed set of keys and would raise ``KeyError``
during ``setup()`` if one went missing, and a key it does not read would
be silently ignored.

The rules below are what the tree does with these values, not what YAML
permits: ``OstensiveParamsToBlackboard`` casts each parameter to a fixed
type, so changing one's type breaks the load, and every scalar here is a
magnitude, so a negative is always a mistake.
"""

import math
import os
import re
from typing import Dict, List, Optional, Tuple

import yaml

from . import yaml_io

#: The second key every constants file in the workspace nests under. The
#: first is the tree's ROS node name, which differs per package and is
#: read from the document rather than assumed.
PARAMS_KEY = "ros__parameters"

#: ``name: value`` at some indent, which is the only shape a parameter
#: takes in these files.  The gap before the colon is captured rather
#: than assumed away: the leading constants contain
#: ``robot_approach_distance : 1.0``, and putting the space back is
#: cheaper than explaining why an unrelated line moved.
PARAM_LINE = re.compile(
    r"^(?P<indent>\s+)(?P<name>[A-Za-z_][A-Za-z0-9_]*)(?P<gap>\s*):(?P<rest>.*)$")

#: ``# ----- Camera geometry -----`` section banners, used to group the
#: page the same way the file groups itself.
SECTION_LINE = re.compile(r"^\s*#\s*-{3,}\s*(?P<title>.*?)\s*-{3,}\s*$")

#: A string value safe to write back unquoted. Every string parameter in
#: this schema is a mode name; anything needing quoting is refused rather
#: than quoted, because the quoting rules are where a hand-written
#: emitter goes wrong.
PLAIN_STRING = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*$")

#: Neck travel, shared with the joystick profiles and the leading
#: constants. Positions outside it are a warning, not an error: the board
#: clamps, so the nod still runs, just not where it was meant to.
NECK_RANGE = (2.0, 8.6)

#: Values a parameter is allowed to take, where the tree only understands
#: a fixed set. Mirrors the modes ``IsBeingAddressed`` implements.
CHOICES = {
    "attention_signal_mode": ("raised_hand", "wave", "any"),
}

#: Parameters whose entries are neck positions, so the travel range applies.
NECK_LISTS = ("ack_neck_seq",)

#: Lists the tree steps through in step with each other.
PAIRED_LISTS = (("ack_neck_seq", "ack_neck_times"),)


class FlatStoreError(Exception):
    """Raised when a flat constants file cannot be read or edited."""


# ── reading ──────────────────────────────────────────────────────────────

def _kind(value) -> str:
    """Name the editable type of one parsed value."""
    if isinstance(value, bool):
        return "unsupported"
    if isinstance(value, int):
        return "integer"
    if isinstance(value, float):
        return "number"
    if isinstance(value, str):
        return "string"
    if isinstance(value, list) and value and all(
            isinstance(item, (int, float)) and not isinstance(item, bool)
            for item in value):
        return "list"
    return "unsupported"


def _comment_start(text: str) -> Optional[int]:
    """
    Return the index where an inline comment starts, or None.

    A ``#`` only opens a comment when it follows whitespace or opens the
    text, and not inside a quoted scalar -- the same rule YAML uses, kept
    here so an inline comment is preserved rather than swallowed into the
    value.
    """
    quote = ""
    for index, char in enumerate(text):
        if quote:
            if char == quote:
                quote = ""
            continue
        if char in "'\"":
            quote = char
        elif char == "#" and (index == 0 or text[index - 1] in " \t"):
            return index
    return None


def _split_value(rest: str) -> Tuple[str, str]:
    """Split the text after ``name:`` into its value and inline comment."""
    index = _comment_start(rest)
    if index is None:
        return rest, ""
    return rest[:index], rest[index:]


def read_text(path: str) -> str:
    """Read a constants file, reporting the path on failure."""
    try:
        with open(path, "r") as stream:
            return stream.read()
    except OSError as exc:
        raise FlatStoreError("cannot read {}: {}".format(path, exc)) from exc


def parse(text: str, path: str = "") -> Dict:
    """
    Return the parameters of a flat constants file, with their comments.

    Every entry carries the line it lives on, the comment block above it,
    the section banner it falls under and its inline comment, so the page
    can show the file's own reasoning beside each field rather than a
    bare list of names.
    """
    try:
        document = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise FlatStoreError("{} is not valid YAML: {}".format(
            path or "document", exc)) from exc

    if not isinstance(document, dict) or len(document) != 1:
        raise FlatStoreError(
            "{}: expected a single top-level node name".format(path or "document"))

    root_key = next(iter(document))
    params = document[root_key]
    if not isinstance(params, dict) or PARAMS_KEY not in params:
        raise FlatStoreError("{}: expected {} -> {}".format(
            path or "document", root_key, PARAMS_KEY))
    params = params[PARAMS_KEY]
    if not isinstance(params, dict):
        raise FlatStoreError("{}: {} is not a mapping".format(
            path or "document", PARAMS_KEY))

    entries = []
    section = ""
    doc_lines: List[str] = []
    inside = False

    for number, line in enumerate(text.splitlines()):
        stripped = line.strip()

        if not inside:
            inside = stripped.startswith(PARAMS_KEY + ":")
            continue

        if not stripped:
            doc_lines = []
            continue

        banner = SECTION_LINE.match(line)
        if banner:
            section = banner.group("title")
            doc_lines = []
            continue

        if stripped.startswith("#"):
            doc_lines.append(stripped.lstrip("#").strip())
            continue

        match = PARAM_LINE.match(line)
        if not match or match.group("name") not in params:
            doc_lines = []
            continue

        name = match.group("name")
        value = params[name]
        value_text, comment = _split_value(match.group("rest"))
        if not value_text.strip():
            # The value is written as a block below the key, so it is not
            # on this line and rewriting the line would strip it. Left to
            # the file; the page says so.
            doc_lines = []
            continue
        entries.append({
            "name": name,
            "value": value,
            "kind": _kind(value),
            "section": section,
            "doc": " ".join(doc_lines),
            "comment": comment.lstrip("# ").strip(),
            "line": number,
            "choices": list(CHOICES.get(name, ())),
        })
        doc_lines = []

    seen = {entry["name"] for entry in entries}
    for name in params:
        if name not in seen:
            # A key the scanner could not attach to a line: a block list,
            # or a nested mapping. It stays in the file untouched and is
            # reported so the page can say the file is only partly editable.
            entries.append({
                "name": name, "value": params[name], "kind": "unsupported",
                "section": "", "doc": "", "comment": "", "line": None,
                "choices": [],
            })

    return {"root_key": root_key, "params": params, "entries": entries}


def load(path: str) -> Dict:
    """Read and parse one flat constants file."""
    result = parse(read_text(path), path)
    result["path"] = path
    return result


# ── writing values ───────────────────────────────────────────────────────

def _format_number(value, as_float: bool) -> str:
    """Render one number the way the source files write them."""
    if as_float:
        return repr(float(value))
    return str(int(value))


def format_value(value, original) -> str:
    """Render a new value in the shape the original had."""
    kind = _kind(original)
    if kind == "integer":
        return _format_number(value, as_float=False)
    if kind == "number":
        return _format_number(value, as_float=True)
    if kind == "string":
        return str(value)
    if kind == "list":
        as_float = any(isinstance(item, float) for item in original)
        return "[{}]".format(", ".join(
            _format_number(item, as_float) for item in value))
    raise FlatStoreError("cannot write a value of that shape")


def coerce(name: str, value, original):
    """
    Return ``value`` as the type the tree will cast it to, or raise.

    The type is taken from the file rather than from the submitted JSON:
    a browser sends 2 and 2.0 identically, and a float parameter that
    silently became an int would change how the emitted file reads even
    though the tree casts it back.
    """
    kind = _kind(original)

    if kind == "integer":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise FlatStoreError("{}: must be a whole number".format(name))
        if float(value) != int(value):
            raise FlatStoreError("{}: must be a whole number".format(name))
        return int(value)

    if kind == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise FlatStoreError("{}: must be a number".format(name))
        if not math.isfinite(float(value)):
            raise FlatStoreError("{}: must be a finite number".format(name))
        return float(value)

    if kind == "string":
        text = str(value).strip()
        if not PLAIN_STRING.match(text):
            raise FlatStoreError(
                "{}: must be a plain word -- letters, digits, '_', '-' and '.'"
                .format(name))
        return text

    if kind == "list":
        if not isinstance(value, list) or not value:
            raise FlatStoreError("{}: must be a non-empty list".format(name))
        coerced = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise FlatStoreError("{}: every entry must be a number".format(name))
            if not math.isfinite(float(item)):
                raise FlatStoreError("{}: every entry must be finite".format(name))
            coerced.append(item)
        as_float = any(isinstance(item, float) for item in original)
        return [float(item) if as_float else int(item) for item in coerced]

    raise FlatStoreError("{}: this parameter is not editable here".format(name))


def apply_updates(text: str, updates: Dict, parsed: Optional[Dict] = None) -> str:
    """
    Return ``text`` with the given parameters set to new values.

    Only the value on each parameter's own line changes; everything else
    in the document -- comments, blank lines, section banners, key order
    -- is the same object it was.
    """
    parsed = parsed or parse(text)
    by_name = {entry["name"]: entry for entry in parsed["entries"]}
    lines = text.splitlines()
    keepends = text.endswith("\n")

    for name, value in (updates or {}).items():
        entry = by_name.get(name)
        if entry is None:
            raise FlatStoreError(
                "{}: not a parameter in this file. Parameters cannot be added "
                "here -- the tree reads a fixed set.".format(name))
        if entry["line"] is None or entry["kind"] == "unsupported":
            raise FlatStoreError(
                "{}: is not editable here; edit it in the file".format(name))

        coerced = coerce(name, value, entry["value"])
        line = lines[entry["line"]]
        match = PARAM_LINE.match(line)
        _value_text, comment = _split_value(match.group("rest"))
        lines[entry["line"]] = "{}{}{}: {}{}".format(
            match.group("indent"), name, match.group("gap"),
            format_value(coerced, entry["value"]),
            "  " + comment if comment else "")

    return "\n".join(lines) + ("\n" if keepends else "")


# ── validation ───────────────────────────────────────────────────────────

def validate(params: Dict) -> Tuple[List[str], List[str]]:
    """
    Return ``(errors, warnings)`` for a parsed parameter mapping.

    Checks what the tree needs, not what YAML allows.  Never raises and
    never stops at the first problem.
    """
    errors: List[str] = []
    warnings: List[str] = []

    for name, choices in CHOICES.items():
        if name in params and params[name] not in choices:
            errors.append("{}: must be one of {}".format(name, ", ".join(choices)))

    for name, value in params.items():
        if isinstance(value, bool):
            continue
        if isinstance(value, (int, float)) and float(value) < 0.0:
            # Every scalar in this schema is a magnitude: a duration, a
            # ratio against body scale, an angle deadband or a speed.
            errors.append("{}: must not be negative".format(name))

    for name in NECK_LISTS:
        for index, value in enumerate(params.get(name) or ()):
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                continue
            if not NECK_RANGE[0] <= float(value) <= NECK_RANGE[1]:
                warnings.append(
                    "{}[{}]: {} is outside the neck travel range {}..{}".format(
                        name, index, value, NECK_RANGE[0], NECK_RANGE[1]))

    for first, second in PAIRED_LISTS:
        one, two = params.get(first), params.get(second)
        if isinstance(one, list) and isinstance(two, list) and len(one) != len(two):
            # OstensiveParamsToBlackboard._check_nod warns about exactly
            # this and carries on, so it is a warning here too.
            warnings.append(
                "{} has {} entries but {} has {} -- the nod stops at the "
                "shorter one".format(first, len(one), second, len(two)))

    return errors, warnings


# ── saving ───────────────────────────────────────────────────────────────

def _self_check(text: str, before: Dict, updates: Dict) -> None:
    """
    Reparse the edited document and confirm exactly the right keys moved.

    The edit is textual, so this is what makes it safe: a regex that hit
    the wrong line, or an emitted value YAML reads differently, shows up
    here rather than in a tree that loads the wrong constants mid-trial.
    """
    after = parse(text)["params"]

    if set(after) != set(before):
        raise FlatStoreError("internal error: the edit changed which parameters "
                             "the file defines")

    for name, value in before.items():
        expected = updates[name] if name in updates else value
        if after[name] != expected:
            raise FlatStoreError(
                "internal error: {} was written as {!r} but reads back as "
                "{!r}".format(name, expected, after[name]))


def save(path: str, updates: Dict, backup_root=yaml_io.DEFAULT_BACKUP_ROOT):
    """
    Validate and write new values into a flat constants file.

    Returns ``(ok, errors, warnings, backup_path)``.  Nothing is written
    when ``errors`` is non-empty.
    """
    text = read_text(path)
    parsed = parse(text, path)

    coerced = {}
    errors: List[str] = []
    by_name = {entry["name"]: entry for entry in parsed["entries"]}
    for name, value in (updates or {}).items():
        entry = by_name.get(name)
        if entry is None:
            errors.append("{}: not a parameter in this file".format(name))
            continue
        try:
            coerced[name] = coerce(name, value, entry["value"])
        except FlatStoreError as exc:
            errors.append(str(exc))

    if errors:
        return False, errors, [], None

    merged = dict(parsed["params"], **coerced)
    errors, warnings = validate(merged)
    if errors:
        return False, errors, warnings, None

    try:
        edited = apply_updates(text, coerced, parsed)
        _self_check(edited, parsed["params"], coerced)
    except FlatStoreError as exc:
        return False, [str(exc)], warnings, None

    backup = yaml_io.make_backup(path, root=backup_root)
    yaml_io.atomic_write_text(path, edited)
    return True, [], warnings, backup


def describe(path: str) -> Dict:
    """Return one flat constants file as the page's editor model."""
    parsed = load(path)
    errors, warnings = validate(parsed["params"])
    return {
        "path": path,
        "name": os.path.basename(path),
        "root_key": parsed["root_key"],
        "entries": parsed["entries"],
        "errors": errors,
        "warnings": warnings,
    }
