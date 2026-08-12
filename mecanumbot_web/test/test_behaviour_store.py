"""
Tests for reading and rewriting the leading-behaviour constants.

The contract these enforce comes from what actually consumes the files:
``ConstantParamsToBlackboard`` runs ``ast.literal_eval`` over every quoted
entry, and ``LEDBehaviourSequence`` reads one delay per pattern.  Both are
in ``mecanumbot_leading_behaviour``.
"""

import ast
import copy
import glob
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import behaviour_store as bs  # noqa: E402
from mecanumbot_web import yaml_io  # noqa: E402

# .../src/mecanumbot/mecanumbot_web/test/ -> .../src/
SRC_DIR = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
CONFIG_DIR = os.path.join(
    SRC_DIR, "mecanumbot_behaviours", "mecanumbot_leading_behaviour", "config")

SHIPPED = sorted(glob.glob(os.path.join(CONFIG_DIR, "*.yaml")))

# The behaviour constants live in a different git repository, so the tests
# that read them skip cleanly when only this repo is checked out.
requires_behaviour_repo = pytest.mark.skipif(
    not SHIPPED, reason="mecanumbot_leading_behaviour is not checked out")

#: parametrize needs at least one value even when the repo is absent.
SHIPPED_OR_PLACEHOLDER = SHIPPED or [""]


def literal_lists(params):
    """Return the keys whose values are lists of dict literals."""
    keys = ["Dog_checkpoints", "LED_start_setting"]
    keys += ["{}_seq".format(name) for name in bs.LED_SCRIPTS + bs.GESTURE_SCRIPTS]
    return [key for key in keys if key in params]


# ── the shipped files ────────────────────────────────────────────────────

@requires_behaviour_repo
@pytest.mark.parametrize("path", SHIPPED_OR_PLACEHOLDER, ids=os.path.basename)
def test_shipped_file_loads_and_has_no_errors(path):
    """Both experiment configurations must be valid as shipped."""
    errors, _warnings = bs.validate(bs.load_params(path))
    assert errors == []


@requires_behaviour_repo
@pytest.mark.parametrize("path", SHIPPED_OR_PLACEHOLDER, ids=os.path.basename)
def test_shipped_file_round_trips_semantically(path):
    """
    Emit, reparse, and confirm every value survives.

    Compares the *parsed* literals rather than the raw strings, since the
    emitter deliberately normalises whitespace and regenerates comments.
    """
    original = bs.load_params(path)
    document = yaml.safe_load(bs.dump(original))
    for key in bs.ROOT_KEYS:
        document = document[key]

    assert set(document) == set(original)
    for key, value in original.items():
        if key in literal_lists(original):
            assert [ast.literal_eval(entry) for entry in document[key]] == \
                [ast.literal_eval(entry) for entry in value]
        else:
            assert document[key] == value


@requires_behaviour_repo
@pytest.mark.parametrize("path", SHIPPED_OR_PLACEHOLDER, ids=os.path.basename)
def test_emitted_entries_stay_literal_eval_able(path):
    """The single constraint the behaviour tree cannot tolerate breaking."""
    params = bs.load_params(path)
    document = yaml.safe_load(bs.dump(params))
    for key in bs.ROOT_KEYS:
        document = document[key]

    for key in literal_lists(params):
        for entry in document[key]:
            assert isinstance(entry, str), "{} became a YAML mapping".format(key)
            ast.literal_eval(entry)


@requires_behaviour_repo
def test_emitter_is_idempotent():
    """Saving an unchanged file twice produces identical bytes."""
    params = bs.load_params(SHIPPED[0])
    first = bs.dump(params)
    document = yaml.safe_load(first)
    for key in bs.ROOT_KEYS:
        document = document[key]
    assert bs.dump(document) == first


@requires_behaviour_repo
def test_known_length_mismatches_are_warnings_not_errors():
    """Both shipped files have surplus delays; that must not block saving."""
    params = bs.load_params(
        os.path.join(CONFIG_DIR, "behaviour_setting_constants.yaml"))
    errors, warnings = bs.validate(params)
    assert errors == []
    assert any("LED_indicate_target_times" in warning for warning in warnings)
    assert any("Dog_indicate_target_times" in warning for warning in warnings)


@requires_behaviour_repo
def test_structured_form_round_trips_through_the_editor(tmp_path):
    """What the page edits rebuilds into what the tree reads."""
    params = bs.load_params(SHIPPED[0])
    rebuilt = bs.params_from_structured(bs.parse_params(params))

    for key in literal_lists(params):
        assert [ast.literal_eval(entry) for entry in rebuilt[key]] == \
            [ast.literal_eval(entry) for entry in params[key]]
    for name in bs.SCALAR_NAMES:
        assert rebuilt[name] == params[name]


# ── synthetic documents ──────────────────────────────────────────────────

def minimal_params():
    """Build a smallest valid params mapping."""
    led = ("{'fl':{'mode':4, 'color':0},'fr':{'mode':4, 'color':0},"
           "'bl':{'mode':4, 'color':0},'br':{'mode':4, 'color':0}}")
    gesture = "{'n_pos':6.0,'gl_pos':5.12,'gr_pos':5.12}"

    params = {name: 1.0 for name in bs.SCALAR_NAMES}
    params["Dog_checkpoints"] = [
        "{'X':0.0, 'Y':0.0, 'Z':0.0}", "{'X':1.0, 'Y':1.0, 'Z':0.0}"]
    params["LED_start_setting"] = [led]
    for script in bs.LED_SCRIPTS:
        params["{}_seq".format(script)] = [led, led]
        params["{}_times".format(script)] = [1.0, 1.0]
    for script in bs.GESTURE_SCRIPTS:
        params["{}_seq".format(script)] = [gesture, gesture]
        params["{}_times".format(script)] = [0.5, 0.5]
    return params


def test_minimal_document_is_valid():
    """The fixture itself must be clean, or every test below is noise."""
    errors, warnings = bs.validate(minimal_params())
    assert errors == []
    assert warnings == []


def test_times_shorter_than_seq_is_a_hard_error():
    """
    Reject a delay list shorter than its pattern list.

    LEDBehaviourSequence reads delays[index-1] up to len(patterns).

    A short delay list is therefore an IndexError inside a behaviour tick,
    which is the worst possible time to discover it.
    """
    params = minimal_params()
    params["LED_thank_times"] = [1.0]
    errors, _warnings = bs.validate(params)
    assert any("IndexError" in error for error in errors)


def test_times_longer_than_seq_is_only_a_warning():
    """Surplus delays are never read, so they cannot break anything."""
    params = minimal_params()
    params["LED_thank_times"] = [1.0, 1.0, 1.0, 1.0]
    errors, warnings = bs.validate(params)
    assert errors == []
    assert any("never read" in warning for warning in warnings)


def test_empty_checkpoints_is_an_error():
    """The tree indexes [0] and [-1] unconditionally."""
    params = minimal_params()
    params["Dog_checkpoints"] = []
    assert any("Dog_checkpoints" in error for error in bs.validate(params)[0])


def test_single_checkpoint_is_a_warning():
    """One checkpoint works but makes start and target the same point."""
    params = minimal_params()
    params["Dog_checkpoints"] = ["{'X':0.0, 'Y':0.0, 'Z':0.0}"]
    errors, warnings = bs.validate(params)
    assert errors == []
    assert any("same point" in warning for warning in warnings)


def test_empty_led_start_setting_is_an_error():
    """The tree reads LED_start_setting[0]."""
    params = minimal_params()
    params["LED_start_setting"] = []
    assert any("LED_start_setting" in error for error in bs.validate(params)[0])


def test_missing_scalar_is_an_error():
    """Every threshold the tree reads must be present."""
    params = minimal_params()
    del params["robot_closeness_threshold"]
    assert any("robot_closeness_threshold" in error
               for error in bs.validate(params)[0])


def test_non_numeric_scalar_is_an_error():
    """_write_thresholds calls float() on each of these."""
    params = minimal_params()
    params["init_delay"] = "soon"
    assert any("init_delay" in error for error in bs.validate(params)[0])


def test_unparseable_literal_is_an_error():
    """A broken entry is caught here rather than at tree setup."""
    params = minimal_params()
    params["LED_thank_seq"] = ["{not valid python", "{'fl': 1}"]
    assert any("literal" in error for error in bs.validate(params)[0])


def test_missing_led_corner_is_an_error():
    """parse_led indexes all four corners."""
    params = minimal_params()
    params["LED_thank_seq"] = ["{'fl':{'mode':4, 'color':0}}", ]
    params["LED_thank_times"] = [1.0]
    assert any("corner" in error for error in bs.validate(params)[0])


def test_out_of_range_led_value_is_an_error():
    """Reject LED values outside the int8 range SetLedStatus carries."""
    params = minimal_params()
    params["LED_thank_seq"] = [
        "{'fl':{'mode':4, 'color':900},'fr':{'mode':4, 'color':0},"
        "'bl':{'mode':4, 'color':0},'br':{'mode':4, 'color':0}}"]
    params["LED_thank_times"] = [1.0]
    assert any("int8" in error for error in bs.validate(params)[0])


def test_unknown_led_mode_is_only_a_warning():
    """Unrecognised but in-range values are allowed through."""
    params = minimal_params()
    params["LED_thank_seq"] = [
        "{'fl':{'mode':9, 'color':0},'fr':{'mode':9, 'color':0},"
        "'bl':{'mode':9, 'color':0},'br':{'mode':9, 'color':0}}"]
    params["LED_thank_times"] = [1.0]
    errors, warnings = bs.validate(params)
    assert errors == []
    assert any("not a known mode" in warning for warning in warnings)


def test_gesture_outside_travel_is_a_warning():
    """A pose the hardware cannot reach is flagged but not blocked."""
    params = minimal_params()
    params["Dog_thank_seq"] = ["{'n_pos':99.0,'gl_pos':5.12,'gr_pos':5.12}"]
    params["Dog_thank_times"] = [0.5]
    errors, warnings = bs.validate(params)
    assert errors == []
    assert any("travel range" in warning for warning in warnings)


def test_negative_delay_is_an_error():
    """Time does not run backwards."""
    params = minimal_params()
    params["LED_thank_times"] = [-1.0, 1.0]
    assert any("negative" in error for error in bs.validate(params)[0])


def test_wrong_root_nesting_is_rejected(tmp_path):
    """The document must be nested exactly as the tree expects."""
    path = tmp_path / "wrong.yaml"
    path.write_text("some_other_node:\n  ros__parameters:\n    init_delay: 1.0\n")
    with pytest.raises(bs.BehaviourStoreError):
        bs.load_params(str(path))


def test_invalid_yaml_is_rejected(tmp_path):
    """A syntax error reports as such rather than as a missing key."""
    path = tmp_path / "broken.yaml"
    path.write_text("bottom_up_tree_node: [\n")
    with pytest.raises(bs.BehaviourStoreError):
        bs.load_params(str(path))


# ── saving ───────────────────────────────────────────────────────────────

def test_save_refuses_an_invalid_document(tmp_path):
    """Nothing is written when validation fails."""
    path = tmp_path / "constants.yaml"
    path.write_text("placeholder\n")

    params = minimal_params()
    params["LED_thank_times"] = [1.0]
    ok, errors, _warnings, backup = bs.save(
        str(path), params, backup_root=str(tmp_path / "b"))

    assert ok is False
    assert errors
    assert backup is None
    assert path.read_text() == "placeholder\n"


def test_save_writes_and_backs_up(tmp_path):
    """A valid save lands on disk with the old version preserved."""
    path = tmp_path / "constants.yaml"
    path.write_text("placeholder\n")

    ok, errors, _warnings, backup = bs.save(
        str(path), minimal_params(), backup_root=str(tmp_path / "b"))

    assert ok is True
    assert errors == []
    assert open(backup).read() == "placeholder\n"
    assert bs.load_params(str(path))["init_delay"] == 1.0


def test_save_through_a_symlink_keeps_the_symlink(tmp_path):
    """Behaviour files sit behind a two-hop install symlink chain."""
    source = tmp_path / "source.yaml"
    source.write_text("placeholder\n")
    link = tmp_path / "share.yaml"
    link.symlink_to(source)

    bs.save(str(link), minimal_params(), backup_root=str(tmp_path / "b"))
    assert os.path.islink(str(link))
    assert bs.load_params(str(source))["init_delay"] == 1.0


def test_validate_does_not_mutate(tmp_path):
    """Validation is side-effect free."""
    params = minimal_params()
    snapshot = copy.deepcopy(params)
    bs.validate(params)
    assert params == snapshot


# ── generated comments ───────────────────────────────────────────────────

def test_led_comments_are_regenerated_from_values():
    """
    Comments describe the numbers, not what the old file claimed.

    Several comments in the shipped files contradict their values --
    'color':6 is labelled "white" where 6 is pink -- so they are
    regenerated rather than preserved.
    """
    corners = {corner: {"mode": 6, "color": 6}
               for corner in ("fl", "fr", "bl", "br")}
    assert bs.describe_led(corners) == "pink slow blink"

    corners = {corner: {"mode": 5, "color": 2}
               for corner in ("fl", "fr", "bl", "br")}
    assert bs.describe_led(corners) == "green fast blink"


def test_mixed_corners_are_described_as_such():
    """A per-corner pattern is not misdescribed as uniform."""
    corners = {"fl": {"mode": 4, "color": 1}, "fr": {"mode": 4, "color": 0},
               "bl": {"mode": 4, "color": 0}, "br": {"mode": 4, "color": 0}}
    assert bs.describe_led(corners) == "mixed per corner"


def test_emitted_file_carries_the_legend():
    """The regenerated header documents the numbering it uses."""
    text = bs.dump(minimal_params())
    assert "slow blink" in text
    assert "literal_eval" in text
    assert "next time a behaviour tree starts" in text


def test_self_check_catches_a_broken_emission(monkeypatch):
    """
    If the emitter ever stops quoting, dump() must refuse.

    This guards the one failure that would otherwise surface as a crashed
    behaviour tree in the middle of an experiment.
    """
    def unquoted(lines, key, entries, comments=None):
        lines.append("    {}:".format(key))
        for entry in entries:
            lines.append("      - {}".format(entry))

    monkeypatch.setattr(bs, "_emit_literal_list", unquoted)
    with pytest.raises(bs.BehaviourStoreError):
        bs.dump(minimal_params())


def test_number_formatting_avoids_float_noise():
    """Values come back the way they went in."""
    assert bs._float_text(0.452756) == "0.452756"
    assert bs._float_text(0.0) == "0.0"
    assert bs._int_text(1) == "1"


# ── types ────────────────────────────────────────────────────────────────
#
# The browser cannot tell 0 from 0.0 -- JSON has one number type and
# JSON.stringify drops the decimal point -- while the tree assigns these
# values into strictly typed message fields. So every one of them is
# typed here, from the key, and the crash that motivated these tests is
# `AssertionError: The 'z' field must be of type 'float'` at setup().

def checkpoints_of(params):
    """Return the parsed checkpoint literals of a params mapping."""
    return [ast.literal_eval(entry) for entry in params["Dog_checkpoints"]]


def test_whole_coordinates_are_written_as_floats():
    """A checkpoint the browser sent as an int must not stay one."""
    params = minimal_params()
    params["Dog_checkpoints"] = ["{'X':1, 'Y':0, 'Z':0}"]

    text = bs.dump(bs.normalize(params))

    assert "'X':1.0, 'Y':0.0, 'Z':0.0" in text
    for axis, value in checkpoints_of(yaml.safe_load(text)
                                      ["bottom_up_tree_node"]
                                      ["ros__parameters"])[0].items():
        assert isinstance(value, float), axis


def test_structured_save_types_a_whole_coordinate(tmp_path):
    """
    The whole path from the page: 0 in, 0.0 on disk.

    This is the regression. `Add checkpoint` starts a row at zero, the
    page sends {'X': 0, 'Y': 0, 'Z': 0}, and every checkpoint in the file
    used to come back an int and stop the tree at setup().
    """
    path = tmp_path / "constants.yaml"
    path.write_text("placeholder\n")

    structured = bs.parse_params(minimal_params())
    structured["checkpoints"] = [{"X": 0, "Y": 0, "Z": 0},
                                 {"X": 2, "Y": -3, "Z": 0}]

    ok, errors, _warnings, _backup = bs.save(
        str(path), bs.params_from_structured(structured),
        backup_root=str(tmp_path / "b"))

    assert (ok, errors) == (True, [])
    for point in checkpoints_of(bs.load_params(str(path))):
        assert sorted(point) == ["X", "Y", "Z"]
        assert all(isinstance(value, float) for value in point.values())


def test_integer_coordinate_on_disk_is_reported():
    """A file already carrying the crash says so when it is opened."""
    params = minimal_params()
    params["Dog_checkpoints"] = ["{'X':0.0, 'Y':0.0, 'Z':0}"]

    errors, _warnings = bs.validate(params)

    assert any("Dog_checkpoints[0].Z" in error for error in errors)


def test_saving_repairs_an_integer_coordinate(tmp_path):
    """...and pressing save is what fixes it, rather than being blocked."""
    path = tmp_path / "constants.yaml"
    path.write_text("placeholder\n")
    params = minimal_params()
    params["Dog_checkpoints"] = ["{'X':0.0, 'Y':0.0, 'Z':0}"]

    ok, errors, _warnings, _backup = bs.save(
        str(path), params, backup_root=str(tmp_path / "b"))

    assert (ok, errors) == (True, [])
    assert bs.validate(bs.load_params(str(path)))[0] == []


def test_led_codes_stay_integers():
    """The mirror image: SetLedStatus' fields are int8, not float."""
    params = minimal_params()
    entry = ast.literal_eval(params["LED_thank_seq"][0])
    for corner in bs.LED_CORNERS:
        entry[corner]["mode"] = 4.0
        entry[corner]["color"] = 2.0

    text = bs.dump(bs.normalize({**params, "LED_thank_seq": [
        bs.format_led(entry), bs.format_led(entry)]}))

    emitted = yaml.safe_load(text)["bottom_up_tree_node"]["ros__parameters"]
    for corner, fields in ast.literal_eval(emitted["LED_thank_seq"][0]).items():
        assert all(isinstance(value, int) for value in fields.values()), corner


def test_fractional_led_code_is_refused():
    """4.5 is not a mode, and rounding it would pick a different one."""
    with pytest.raises(bs.BehaviourStoreError):
        bs.format_led({corner: {"mode": 4.5, "color": 2}
                       for corner in bs.LED_CORNERS})


def test_out_of_int8_range_led_code_is_refused():
    """The int8 bound belongs to the message, so the emitter holds it too."""
    with pytest.raises(bs.BehaviourStoreError):
        bs.format_led({corner: {"mode": 4, "color": 300}
                       for corner in bs.LED_CORNERS})


def test_a_cleared_field_is_refused_by_name():
    """An empty input arrives as null; the operator has to be told which."""
    with pytest.raises(bs.BehaviourStoreError) as excinfo:
        bs.format_checkpoint({"X": 1.0, "Y": None, "Z": 0.0})
    assert "Y" in str(excinfo.value)


def test_infinite_values_are_refused():
    """YAML would take 'inf' back as a string, so it never gets written."""
    with pytest.raises(bs.BehaviourStoreError):
        bs.format_checkpoint({"X": float("inf"), "Y": 0.0, "Z": 0.0})


def test_counts_stay_whole():
    """Dog_max_wander_allowed counts events; 1.5 of one is not a setting."""
    params = minimal_params()
    params["Dog_max_wander_allowed"] = 1.5
    errors, _warnings = bs.validate(params)
    assert any("Dog_max_wander_allowed" in error for error in errors)

    params["Dog_max_wander_allowed"] = 2.0
    emitted = yaml.safe_load(bs.dump(params))
    for key in bs.ROOT_KEYS:
        emitted = emitted[key]
    assert emitted["Dog_max_wander_allowed"] == 2
    assert isinstance(emitted["Dog_max_wander_allowed"], int)
    # ...while every other threshold is a measurement and keeps its point.
    assert isinstance(emitted["visibility_time_threshold"], float)


def test_a_measurement_keeps_its_decimal_point():
    """
    A speed set to a whole number is still a speed.

    The type of a tunable is taken from the file being replaced, the same
    place its comment comes from, so editing 0.6 to 1 leaves a float.
    """
    notes = {"turn_max_speed": {"value": 0.6, "doc": "", "section": "",
                                "comment": ""},
             "recover_retries": {"value": 3, "doc": "", "section": "",
                                 "comment": ""}}
    params = dict(minimal_params(), turn_max_speed=1, recover_retries=3)
    text = bs.dump(params, notes)

    assert "turn_max_speed: 1.0" in text
    assert "recover_retries: 3\n" in text


def test_self_check_catches_an_untyped_coordinate(monkeypatch):
    """
    dump() refuses to emit a checkpoint the tree could not load.

    The last line of defence, and the one that would have caught this:
    every check above it reads 0 and 0.0 as the same document.
    """
    monkeypatch.setattr(bs, "format_checkpoint",
                        lambda point: "{'X':0, 'Y':0, 'Z':0}")
    params = minimal_params()
    with pytest.raises(bs.BehaviourStoreError):
        bs.dump(bs.params_from_structured(bs.parse_params(params)))
