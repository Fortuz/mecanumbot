"""
Tests for reading and rewriting joystick profiles.

Validation itself is tested in ``mecanumbot_joy``; what matters here is
that the store delegates to that same validator and that its emitter
survives a round trip.
"""

import os
import shutil
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import joystick_store as js  # noqa: E402
from mecanumbot_web import yaml_io  # noqa: E402

# .../src/mecanumbot/mecanumbot_web/test/ -> .../src/mecanumbot/
PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
SHIPPED_DIR = os.path.join(
    PKG_ROOT, "mecanumbot_description", "config", "joystick")

requires_joy = pytest.mark.skipif(
    not js.JOY_AVAILABLE, reason="mecanumbot_joy is not importable")


@pytest.fixture
def profiles(tmp_path):
    """Build a writable copy of the shipped profiles, plus a backup root."""
    directory = tmp_path / "joystick"
    directory.mkdir()
    for name in os.listdir(SHIPPED_DIR):
        if name.endswith(".yaml"):
            shutil.copy(os.path.join(SHIPPED_DIR, name), str(directory))
    return {"dir": str(directory), "backups": str(tmp_path / "backups")}


# ── listing and reading ──────────────────────────────────────────────────

@requires_joy
def test_lists_every_shipped_profile(profiles):
    """All three profiles are found and valid."""
    entries = {entry["stem"]: entry for entry in js.list_profiles(profiles["dir"])}
    assert set(entries) == {"xbox360", "generic", "ps4"}
    assert all(entry["valid"] for entry in entries.values())


@requires_joy
def test_invalid_profiles_are_listed_with_their_errors(profiles):
    """A broken file must be visible in the UI, not silently missing."""
    with open(os.path.join(profiles["dir"], "broken.yaml"), "w") as handle:
        handle.write("profile:\n  name: broken\n")

    entries = {entry["stem"]: entry for entry in js.list_profiles(profiles["dir"])}
    assert "broken" in entries
    assert entries["broken"]["valid"] is False
    assert entries["broken"]["errors"]


def test_listing_a_missing_directory_is_empty():
    """A misconfigured path is an empty list, not a crash."""
    assert js.list_profiles("/nonexistent/path") == []


@requires_joy
def test_read_reports_the_declared_controls(profiles):
    """The editor builds its dropdowns from this."""
    data = js.read_profile(profiles["dir"], "xbox360")
    assert "left_y" in data["controls"]["axes"]
    assert "BACK" in data["controls"]["buttons"]
    assert "DPadUp" in data["controls"]["buttons"]


@requires_joy
def test_controls_omit_dpad_when_the_profile_has_none(profiles):
    """A pad without a D-Pad must not offer D-Pad bindings."""
    document = js.read_profile(profiles["dir"], "xbox360")["document"]
    del document["profile"]["dpad"]
    assert "DPadUp" not in js.controls_of(document)["buttons"]


def test_path_traversal_is_rejected():
    """A profile name may not escape the configured directory."""
    for bad in ("../etc/passwd", "a/b", "", ".", ".."):
        with pytest.raises(js.JoystickStoreError):
            js.read_profile("/tmp", bad)


def test_reading_a_missing_profile_raises():
    """A clear error rather than an empty document."""
    with pytest.raises(js.JoystickStoreError):
        js.read_profile("/tmp", "definitely_not_here")


# ── round trip ───────────────────────────────────────────────────────────

@requires_joy
@pytest.mark.parametrize("stem", ["xbox360", "generic", "ps4"])
def test_save_round_trips_without_semantic_change(profiles, stem):
    """Loading and saving an untouched profile must change nothing."""
    before = js.read_profile(profiles["dir"], stem)["document"]
    ok, errors, _backup, _created = js.save_profile(
        profiles["dir"], stem, before, backup_root=profiles["backups"])
    assert ok, errors

    after = js.read_profile(profiles["dir"], stem)
    assert after["document"] == before
    assert after["errors"] == []


@requires_joy
def test_emitter_is_idempotent(profiles):
    """Two saves in a row produce byte-identical files."""
    document = js.read_profile(profiles["dir"], "xbox360")["document"]
    js.save_profile(profiles["dir"], "xbox360", document,
                    backup_root=profiles["backups"])
    first = js.read_profile(profiles["dir"], "xbox360")["raw"]
    js.save_profile(profiles["dir"], "xbox360", yaml.safe_load(first),
                    backup_root=profiles["backups"])
    assert js.read_profile(profiles["dir"], "xbox360")["raw"] == first


@requires_joy
def test_axis_range_survives_as_a_list(profiles):
    """
    A range must not be stringified.

    Rendering `[1.0, -1.0]` through a scalar formatter produces the string
    "[1.0, -1.0]", which reparses as a scalar and fails validation. This is
    the regression test for that.
    """
    document = js.read_profile(profiles["dir"], "xbox360")["document"]
    js.save_profile(profiles["dir"], "xbox360", document,
                    backup_root=profiles["backups"])

    reloaded = js.read_profile(profiles["dir"], "xbox360")["document"]
    axis_range = reloaded["profile"]["axes"]["lt"]["range"]
    assert isinstance(axis_range, list)
    assert axis_range == [1.0, -1.0]


@requires_joy
def test_emitted_file_keeps_the_legends(profiles):
    """Comments explaining the LED numbering are regenerated on save."""
    document = js.read_profile(profiles["dir"], "xbox360")["document"]
    text = js.dump(document)
    assert "slow blink" in text
    assert "rest, pressed" in text


# ── rejection ────────────────────────────────────────────────────────────

@requires_joy
def test_save_refuses_an_invalid_document(profiles):
    """Nothing is written when the shared validator rejects the edit."""
    document = js.read_profile(profiles["dir"], "xbox360")["document"]
    original = js.read_profile(profiles["dir"], "xbox360")["raw"]

    document["profile"]["bindings"]["drive"]["axis"] = "nonexistent"
    ok, errors, backup, _created = js.save_profile(
        profiles["dir"], "xbox360", document, backup_root=profiles["backups"])

    assert ok is False
    assert any("nonexistent" in error for error in errors)
    assert backup is None
    assert js.read_profile(profiles["dir"], "xbox360")["raw"] == original


@requires_joy
def test_save_reports_a_new_profile_as_needing_a_rebuild(profiles):
    """A file that exists only in source is invisible until rebuilt."""
    document = js.read_profile(profiles["dir"], "xbox360")["document"]
    document["profile"]["name"] = "custom"
    del document["profile"]["match"]

    ok, _errors, _backup, created = js.save_profile(
        profiles["dir"], "custom", document, backup_root=profiles["backups"])
    assert ok is True
    assert created is True


@requires_joy
def test_save_backs_up_the_previous_version(profiles):
    """The old profile is recoverable after an edit."""
    document = js.read_profile(profiles["dir"], "xbox360")["document"]
    document["profile"]["limits"]["max_lin_vel"] = 0.1

    _ok, _errors, backup, _created = js.save_profile(
        profiles["dir"], "xbox360", document, backup_root=profiles["backups"])

    assert backup is not None
    restored = yaml.safe_load(open(backup).read())
    assert restored["profile"]["limits"]["max_lin_vel"] == 0.234


@requires_joy
def test_save_through_a_symlink_keeps_the_symlink(tmp_path):
    """The share directory is symlinked; a save must not break that."""
    source_dir = tmp_path / "src"
    share_dir = tmp_path / "share"
    source_dir.mkdir()
    share_dir.mkdir()

    shutil.copy(os.path.join(SHIPPED_DIR, "xbox360.yaml"), str(source_dir))
    link = share_dir / "xbox360.yaml"
    link.symlink_to(source_dir / "xbox360.yaml")

    document = js.read_profile(str(share_dir), "xbox360")["document"]
    document["profile"]["limits"]["max_lin_vel"] = 0.15
    js.save_profile(str(share_dir), "xbox360", document,
                    backup_root=str(tmp_path / "b"))

    assert os.path.islink(str(link))
    written = yaml.safe_load(open(str(source_dir / "xbox360.yaml")).read())
    assert written["profile"]["limits"]["max_lin_vel"] == 0.15


@requires_joy
def test_symlink_status_is_surfaced(profiles, tmp_path):
    """The GUI warns when an edit would be lost on the next build."""
    assert js.read_profile(profiles["dir"], "xbox360")["symlinked"] is False


# ── editor metadata ──────────────────────────────────────────────────────

@requires_joy
def test_editor_metadata_covers_the_whole_vocabulary():
    """Every action the node understands is documented for the page."""
    metadata = js.editor_metadata()
    for action in metadata["vocabulary"]:
        assert action in metadata["action_docs"]
    assert set(metadata["axis_actions"]) <= set(metadata["vocabulary"])
    assert metadata["joy_available"] is True


@requires_joy
def test_editor_metadata_documents_every_limit():
    """No limit should appear in the editor without an explanation."""
    metadata = js.editor_metadata()
    for key in metadata["limit_defaults"]:
        assert key in metadata["limit_docs"]
