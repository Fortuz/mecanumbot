"""
Tests for joystick profile loading and validation.

The validator is shared with the web GUI, which refuses to save anything
it rejects.  These tests are therefore also the contract the GUI relies
on: an error here is an error the operator sees before a bad profile can
reach the robot.
"""

import copy
import glob
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_joy.profile import (  # noqa: E402
    DEFAULT_LIMITS,
    VOCABULARY,
    ProfileError,
    load_profile,
    load_profile_dir,
    profile_from_document,
    validate_document,
)

PROFILE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "mecanumbot_description", "config", "joystick")

SHIPPED = sorted(glob.glob(os.path.join(PROFILE_DIR, "*.yaml")))


def _document(stem="xbox360"):
    """Load a shipped profile as a mutable document."""
    with open(os.path.join(PROFILE_DIR, "{}.yaml".format(stem)), "r") as handle:
        return yaml.safe_load(handle)


# ── the shipped profiles ─────────────────────────────────────────────────

def test_shipped_profiles_exist():
    """The three documented profiles are actually present."""
    stems = {os.path.splitext(os.path.basename(path))[0] for path in SHIPPED}
    assert {"xbox360", "generic", "ps4"} <= stems


@pytest.mark.parametrize("path", SHIPPED, ids=lambda p: os.path.basename(p))
def test_shipped_profile_is_valid(path):
    """Every profile that ships must load without error."""
    profile = load_profile(path)
    assert profile.name


@pytest.mark.parametrize("path", SHIPPED, ids=lambda p: os.path.basename(p))
def test_shipped_profile_binds_estop(path):
    """An unbound e-stop on a driving profile would be a safety gap."""
    profile = load_profile(path)
    assert "estop" in profile.bindings


@pytest.mark.parametrize("path", SHIPPED, ids=lambda p: os.path.basename(p))
def test_shipped_profile_binds_all_three_axes(path):
    """Drive, strafe and turn must all be reachable."""
    profile = load_profile(path)
    for action in ("drive", "strafe", "turn"):
        assert action in profile.bindings


def test_load_profile_dir_reports_errors_without_losing_good_files(tmp_path):
    """One broken profile must not take the others offline."""
    for path in SHIPPED:
        (tmp_path / os.path.basename(path)).write_text(open(path).read())
    (tmp_path / "broken.yaml").write_text("profile: [this is not a mapping]\n")

    profiles, errors = load_profile_dir(str(tmp_path))
    assert "broken" in errors
    assert {"xbox360", "generic", "ps4"} <= set(profiles)


def test_defaults_are_merged_for_omitted_limits():
    """A profile need not spell out every limit."""
    document = _document()
    del document["profile"]["limits"]["publish_hz"]
    profile = profile_from_document(document)
    assert profile.limits["publish_hz"] == DEFAULT_LIMITS["publish_hz"]


# ── rejection ────────────────────────────────────────────────────────────

def test_binding_to_undeclared_control_is_rejected():
    """A binding may only name a control the pad actually has."""
    document = _document()
    document["profile"]["bindings"]["drive"]["axis"] = "nonexistent"
    errors = validate_document(document)
    assert any("nonexistent" in error for error in errors)


def test_unknown_action_is_rejected_and_lists_the_vocabulary():
    """The error has to tell the operator what is allowed instead."""
    document = _document()
    document["profile"]["bindings"]["fly"] = {"control": "button", "button": "A"}
    errors = validate_document(document)
    assert errors
    message = " ".join(errors)
    assert "fly" in message
    for action in VOCABULARY:
        assert action in message


def test_axis_action_bound_to_a_button_is_rejected():
    """Continuous actions need a continuous control."""
    document = _document()
    document["profile"]["bindings"]["drive"] = {"control": "button", "button": "A"}
    errors = validate_document(document)
    assert any("control: axis" in error for error in errors)


def test_button_action_bound_to_an_axis_is_rejected():
    """Discrete actions need a discrete control."""
    document = _document()
    document["profile"]["bindings"]["estop"] = {"control": "axis", "axis": "left_x"}
    errors = validate_document(document)
    assert any("control: button" in error for error in errors)


def test_led_preset_must_name_a_declared_preset():
    """A binding cannot reference an LED preset that does not exist."""
    document = _document()
    document["profile"]["bindings"]["led_preset"]["preset"] = "disco"
    errors = validate_document(document)
    assert any("disco" in error for error in errors)


def test_neck_preset_requires_a_numeric_value():
    """Jump-to-position needs a position."""
    document = _document()
    del document["profile"]["bindings"]["neck_preset"]["value"]
    errors = validate_document(document)
    assert any("neck_preset.value" in error for error in errors)


def test_bad_trigger_is_rejected():
    """Only press and hold exist."""
    document = _document()
    document["profile"]["bindings"]["gripper_open"]["trigger"] = "double_tap"
    errors = validate_document(document)
    assert any("trigger" in error for error in errors)


def test_malformed_axis_range_is_rejected():
    """A range must be exactly two distinct numbers."""
    document = _document()
    document["profile"]["axes"]["lt"]["range"] = [1.0]
    assert any("range" in error for error in validate_document(document))

    document = _document()
    document["profile"]["axes"]["lt"]["range"] = [1.0, 1.0]
    assert any("differ" in error for error in validate_document(document))


def test_dpad_names_are_reserved_as_buttons():
    """D-Pad names come from the dpad block, not the button table."""
    document = _document()
    document["profile"]["buttons"]["DPadUp"] = 20
    errors = validate_document(document)
    assert any("reserved" in error for error in errors)


def test_negative_indices_are_rejected():
    """An index has to be a real position in the joy arrays."""
    document = _document()
    document["profile"]["buttons"]["A"] = -1
    assert any("non-negative" in error for error in validate_document(document))


def test_led_preset_values_must_fit_int8():
    """Reject LED values outside the int8 range SetLedStatus carries."""
    document = _document()
    document["profile"]["led_presets"]["attention"]["fl"]["color"] = 999
    assert any("int8" in error for error in validate_document(document))


def test_missing_profile_root_is_rejected():
    """A document without a profile key is not a profile."""
    assert validate_document({"nope": {}})
    assert validate_document([])
    assert validate_document(None)


def test_zero_publish_hz_is_rejected():
    """A zero publish rate would divide by zero in the node's timer."""
    document = _document()
    document["profile"]["limits"]["publish_hz"] = 0.0
    assert any("greater than zero" in error for error in validate_document(document))


def test_profile_from_document_raises_with_every_problem():
    """Callers get the full list, not just the first failure."""
    document = _document()
    document["profile"]["bindings"]["drive"]["axis"] = "nope"
    document["profile"]["bindings"]["estop"]["button"] = "alsonope"
    with pytest.raises(ProfileError) as excinfo:
        profile_from_document(document)
    assert "nope" in str(excinfo.value)
    assert "alsonope" in str(excinfo.value)


def test_validation_does_not_mutate_the_document():
    """Validating must be free of side effects; the GUI validates on load."""
    document = _document()
    snapshot = copy.deepcopy(document)
    validate_document(document)
    assert document == snapshot


def test_yaml_boolean_word_as_a_preset_name_is_rejected_clearly():
    """
    `off:` parses as the boolean False, not the word.

    Naming an LED preset `off` silently produces the key `False`, so the
    preset becomes unreferenceable. The shipped profiles call it `blank`
    for exactly this reason; the validator has to explain the trap rather
    than crash sorting a mixed set of names.
    """
    document = _document()
    document["profile"]["led_presets"][False] = \
        document["profile"]["led_presets"]["blank"]
    errors = validate_document(document)
    assert any("booleans" in error for error in errors)


def test_yaml_boolean_word_as_a_button_name_is_rejected():
    """The same trap applies to control names."""
    document = _document()
    document["profile"]["buttons"][True] = 12
    assert any("not a string" in error for error in validate_document(document))
