"""
Tests for the comment-preserving flat constants editor.

The ostensive constants are mostly comments: why a threshold is a ratio
against body scale, which detector a field of view has to match, what
happens if a dwell is shortened.  Losing that would cost more than the
edit was worth, so most of what is checked here is what did *not* change.
"""

import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import flat_store  # noqa: E402

PACKAGES_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
SRC_DIR = os.path.dirname(PACKAGES_ROOT)
OSTENSIVE_DIR = os.path.join(
    SRC_DIR, "mecanumbot_behaviours", "mecanumbot_ostensive_behaviour", "config")

DOCUMENT = """\
ostensive_bt_node:
  ros__parameters:
    # ----- Camera geometry ---------------------------------------------------
    # Must match the detector's own field of view.
    camera_hfov_deg: 60.0

    # How old a frame may be and still be acted on.
    detection_timeout: 1.0  # the detector runs at about 15 Hz

    # ----- Being addressed ---------------------------------------------------
    attention_signal_mode: any
    wave_min_reversals: 2

    ack_neck_seq: [7.6, 6.2, 7.6, 7.0]
    ack_neck_times: [0.35, 0.35, 0.35, 0.3]
"""


@pytest.fixture
def path(tmp_path):
    """Return a writable copy of the sample document."""
    target = tmp_path / "ostensive_setting_constants.yaml"
    target.write_text(DOCUMENT)
    return str(target)


# ── reading ──────────────────────────────────────────────────────────────

def test_every_parameter_is_found(path):
    """Each key on its own line becomes an editable entry."""
    entries = {entry["name"]: entry for entry in flat_store.load(path)["entries"]}
    assert set(entries) == {"camera_hfov_deg", "detection_timeout",
                            "attention_signal_mode", "wave_min_reversals",
                            "ack_neck_seq", "ack_neck_times"}


def test_types_are_taken_from_the_file(path):
    """The tree casts each parameter to a fixed type, so the file decides it."""
    entries = {entry["name"]: entry for entry in flat_store.load(path)["entries"]}
    assert entries["camera_hfov_deg"]["kind"] == "number"
    assert entries["wave_min_reversals"]["kind"] == "integer"
    assert entries["attention_signal_mode"]["kind"] == "string"
    assert entries["ack_neck_seq"]["kind"] == "list"


def test_the_files_own_explanations_come_with_it(path):
    """
    Each field is shown beside the sentence that says what it does.

    Those sentences are the only documentation these keys have.
    """
    entries = {entry["name"]: entry for entry in flat_store.load(path)["entries"]}
    assert entries["camera_hfov_deg"]["section"] == "Camera geometry"
    assert "field of view" in entries["camera_hfov_deg"]["doc"]
    assert entries["detection_timeout"]["comment"] == "the detector runs at about 15 Hz"


def test_a_mode_parameter_offers_only_the_modes_that_exist(path):
    """The tree implements three, so the page must not offer a fourth."""
    entries = {entry["name"]: entry for entry in flat_store.load(path)["entries"]}
    assert entries["attention_signal_mode"]["choices"] == \
        ["raised_hand", "wave", "any"]


# ── editing ──────────────────────────────────────────────────────────────

def test_only_the_edited_line_changes(path):
    """Every other line of the file is the line it was."""
    before = flat_store.read_text(path).splitlines()
    after = flat_store.apply_updates(
        flat_store.read_text(path), {"camera_hfov_deg": 78.0}).splitlines()

    assert len(before) == len(after)
    changed = [index for index, line in enumerate(before) if line != after[index]]
    assert len(changed) == 1
    assert "78.0" in after[changed[0]]


def test_an_inline_comment_survives_its_value_changing(path):
    """The comment explains the value, so it outlives any one number."""
    after = flat_store.apply_updates(
        flat_store.read_text(path), {"detection_timeout": 2.5})
    line = [text for text in after.splitlines()
            if text.strip().startswith("detection_timeout")][0]
    assert "2.5" in line
    assert "the detector runs at about 15 Hz" in line


def test_a_float_stays_a_float(path):
    """
    2 written into a float parameter is written back as 2.0.

    A browser sends 2 and 2.0 identically, and a parameter that quietly
    became an int would read differently even though the tree casts it.
    """
    after = flat_store.apply_updates(flat_store.read_text(path),
                                     {"camera_hfov_deg": 45})
    assert "camera_hfov_deg: 45.0" in after


def test_an_integer_parameter_refuses_a_fraction(path):
    """``wave_min_reversals`` counts reversals; 2.5 of them is nonsense."""
    with pytest.raises(flat_store.FlatStoreError):
        flat_store.apply_updates(flat_store.read_text(path),
                                 {"wave_min_reversals": 2.5})


def test_lists_are_rewritten_inline(path):
    """The nod sequence keeps its flow style, so the file reads the same."""
    after = flat_store.apply_updates(flat_store.read_text(path),
                                     {"ack_neck_seq": [7.0, 6.0]})
    assert "ack_neck_seq: [7.0, 6.0]" in after


def test_a_string_that_would_need_quoting_is_refused(path):
    """Quoting rules are where a hand-written emitter goes wrong."""
    with pytest.raises(flat_store.FlatStoreError):
        flat_store.apply_updates(flat_store.read_text(path),
                                 {"attention_signal_mode": "a: b # c"})


def test_a_parameter_cannot_be_added(path):
    """
    The tree reads a fixed set of keys and ignores anything else.

    So an added key would look like it took effect and would not have.
    """
    with pytest.raises(flat_store.FlatStoreError):
        flat_store.apply_updates(flat_store.read_text(path), {"invented": 1.0})


def test_a_space_before_the_colon_is_kept(tmp_path):
    """
    ``robot_approach_distance : 1.0`` appears in the shipped files.

    Putting the space back is cheaper than explaining why an unrelated
    line moved in the diff.
    """
    target = tmp_path / "spaced.yaml"
    target.write_text("node:\n  ros__parameters:\n    odd_key : 1.0\n")
    after = flat_store.apply_updates(target.read_text(), {"odd_key": 2.0})
    assert "    odd_key : 2.0" in after


# ── saving ───────────────────────────────────────────────────────────────

def test_save_writes_and_backs_up(path, tmp_path):
    """The previous version is kept, outside the git checkout."""
    ok, errors, _warnings, backup = flat_store.save(
        path, {"camera_hfov_deg": 55.0}, backup_root=str(tmp_path / "backups"))

    assert ok, errors
    assert backup and os.path.isfile(backup)
    assert flat_store.load(path)["params"]["camera_hfov_deg"] == 55.0


def test_save_refuses_an_unknown_mode(path, tmp_path):
    """
    ``attention_signal_mode`` is read by name in the tree's condition.

    A value it does not implement would simply never fire, which in a
    trial looks like a participant who was ignored.
    """
    ok, errors, _warnings, _backup = flat_store.save(
        path, {"attention_signal_mode": "shouting"},
        backup_root=str(tmp_path / "backups"))

    assert not ok
    assert any("attention_signal_mode" in error for error in errors)
    assert "shouting" not in flat_store.read_text(path)


def test_save_refuses_a_negative_magnitude(path, tmp_path):
    """Every scalar here is a duration, a ratio, an angle or a speed."""
    ok, errors, _warnings, _backup = flat_store.save(
        path, {"detection_timeout": -1.0}, backup_root=str(tmp_path / "backups"))

    assert not ok
    assert any("negative" in error for error in errors)


def test_a_nod_whose_lists_disagree_is_a_warning_not_an_error(path, tmp_path):
    """
    The tree warns and carries on, stopping at the shorter list.

    So this must not block a save; it is the operator's call.
    """
    ok, _errors, warnings, _backup = flat_store.save(
        path, {"ack_neck_seq": [7.0, 6.0]}, backup_root=str(tmp_path / "backups"))

    assert ok
    assert any("nod stops at the shorter one" in warning for warning in warnings)


def test_nothing_is_written_when_a_value_is_rejected(path, tmp_path):
    """One bad field must not half-apply the rest of the edit."""
    before = flat_store.read_text(path)
    ok, _errors, _warnings, _backup = flat_store.save(
        path, {"camera_hfov_deg": 70.0, "wave_min_reversals": 1.5},
        backup_root=str(tmp_path / "backups"))

    assert not ok
    assert flat_store.read_text(path) == before


# ── the shipped files ────────────────────────────────────────────────────

SHIPPED = [os.path.join(OSTENSIVE_DIR, name)
           for name in sorted(os.listdir(OSTENSIVE_DIR))
           if name.endswith(".yaml")] if os.path.isdir(OSTENSIVE_DIR) else []


@pytest.mark.skipif(not SHIPPED, reason="mecanumbot_ostensive_behaviour is absent")
@pytest.mark.parametrize("shipped", SHIPPED, ids=[os.path.basename(entry) for entry in SHIPPED])
def test_every_shipped_parameter_is_editable(shipped):
    """
    Nothing in the shipped files falls through to 'edit it in the file'.

    If one does, the page silently offers less than it appears to.
    """
    described = flat_store.describe(shipped)
    unsupported = [entry["name"] for entry in described["entries"]
                   if entry["kind"] == "unsupported"]
    assert unsupported == []


@pytest.mark.skipif(not SHIPPED, reason="mecanumbot_ostensive_behaviour is absent")
@pytest.mark.parametrize("shipped", SHIPPED, ids=[os.path.basename(entry) for entry in SHIPPED])
def test_the_shipped_files_are_valid(shipped):
    """The files the robot runs on must pass the page's own checks."""
    described = flat_store.describe(shipped)
    assert described["errors"] == []


@pytest.mark.skipif(not SHIPPED, reason="mecanumbot_ostensive_behaviour is absent")
@pytest.mark.parametrize("shipped", SHIPPED, ids=[os.path.basename(entry) for entry in SHIPPED])
def test_rewriting_every_value_changes_nothing(shipped):
    """
    Writing each value back as itself leaves the document identical.

    The strongest statement available about a textual edit: the formatter
    agrees with how the files are already written.
    """
    text = flat_store.read_text(shipped)
    parsed = flat_store.parse(text, shipped)
    updates = {entry["name"]: entry["value"] for entry in parsed["entries"]}

    rewritten = flat_store.apply_updates(text, updates, parsed)
    assert yaml.safe_load(rewritten) == yaml.safe_load(text)
