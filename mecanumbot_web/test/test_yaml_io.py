"""
Tests for symlink-safe atomic writes and the backup store.

The symlink test is the important one.  Under ``--symlink-install`` every
config file in a package's share directory is a symlink into the source
tree, and the naive atomic-write idiom replaces the symlink with a regular
file -- which silently decouples the installed copy from the checkout and
makes every later edit appear to vanish on rebuild.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import yaml_io  # noqa: E402


@pytest.fixture
def workspace(tmp_path):
    """Build a source file plus a symlink to it, mimicking a symlink-install."""
    source_dir = tmp_path / "src"
    share_dir = tmp_path / "share"
    source_dir.mkdir()
    share_dir.mkdir()

    source = source_dir / "profile.yaml"
    source.write_text("original: true\n")
    link = share_dir / "profile.yaml"
    link.symlink_to(source)

    return {"source": str(source), "link": str(link),
            "backups": str(tmp_path / "backups")}


# ── the symlink contract ─────────────────────────────────────────────────

def test_write_through_a_symlink_edits_the_source(workspace):
    """Saving via the share path must reach the checkout."""
    yaml_io.atomic_write_text(workspace["link"], "edited: true\n")
    assert open(workspace["source"]).read() == "edited: true\n"


def test_write_through_a_symlink_keeps_the_symlink(workspace):
    """
    The share path must still be a symlink afterwards.

    This is the regression test for the whole module: os.replace onto the
    link path instead of the resolved path would leave a regular file here
    and quietly break the install-to-source relationship for good.
    """
    yaml_io.atomic_write_text(workspace["link"], "edited: true\n")
    assert os.path.islink(workspace["link"])
    assert os.path.realpath(workspace["link"]) == \
        os.path.realpath(workspace["source"])


def test_write_leaves_no_temporary_files(workspace):
    """The temp file is renamed away, not left behind."""
    yaml_io.atomic_write_text(workspace["link"], "edited: true\n")
    leftovers = [name for name in os.listdir(os.path.dirname(workspace["source"]))
                 if name.endswith(".tmp")]
    assert leftovers == []


def test_atomic_write_returns_the_resolved_path(workspace):
    """Callers are told where the bytes actually landed."""
    written = yaml_io.atomic_write_text(workspace["link"], "x: 1\n")
    assert written == os.path.realpath(workspace["source"])


def test_write_preserves_permissions(workspace):
    """A mode-644 config stays mode-644."""
    os.chmod(workspace["source"], 0o640)
    yaml_io.atomic_write_text(workspace["link"], "x: 1\n")
    assert oct(os.stat(workspace["source"]).st_mode)[-3:] == "640"


def test_is_symlinked_install_detects_both_cases(workspace, tmp_path):
    """The GUI warns when a save would be lost on rebuild."""
    assert yaml_io.is_symlinked_install(workspace["link"])
    plain = tmp_path / "plain.yaml"
    plain.write_text("x: 1\n")
    assert not yaml_io.is_symlinked_install(str(plain))


def test_write_creates_a_new_file(tmp_path):
    """A brand new profile has no existing file to resolve."""
    target = str(tmp_path / "new.yaml")
    yaml_io.atomic_write_text(target, "fresh: true\n")
    assert open(target).read() == "fresh: true\n"


# ── backups ──────────────────────────────────────────────────────────────

def test_backup_copies_the_current_contents(workspace):
    """The previous version is recoverable after a save."""
    backup = yaml_io.make_backup(workspace["link"], root=workspace["backups"])
    yaml_io.atomic_write_text(workspace["link"], "edited: true\n")
    assert open(backup).read() == "original: true\n"


def test_backup_of_a_missing_file_is_none(tmp_path):
    """Creating a new file is not an error, it just has no backup."""
    assert yaml_io.make_backup(
        str(tmp_path / "absent.yaml"), root=str(tmp_path / "b")) is None


def test_backups_live_outside_the_source_tree(workspace):
    """Backups must not pollute the git repositories being edited."""
    yaml_io.make_backup(workspace["link"], root=workspace["backups"])
    source_dir = os.path.dirname(workspace["source"])
    assert os.listdir(source_dir) == ["profile.yaml"]


def test_repeated_backups_do_not_collide(workspace):
    """Two saves in the same second both survive."""
    for index in range(4):
        yaml_io.atomic_write_text(workspace["link"], "v: {}\n".format(index))
        yaml_io.make_backup(workspace["link"], root=workspace["backups"])
    assert len(yaml_io.list_backups(workspace["link"], workspace["backups"])) == 4


def test_backups_are_pruned_to_the_keep_limit(workspace):
    """Old backups do not accumulate forever."""
    for index in range(9):
        yaml_io.atomic_write_text(workspace["link"], "v: {}\n".format(index))
        yaml_io.make_backup(workspace["link"], keep=3, root=workspace["backups"])
    assert len(yaml_io.list_backups(workspace["link"], workspace["backups"])) == 3


def test_backups_are_listed_newest_first(workspace):
    """The GUI's dropdown shows the most recent restore point on top."""
    for index in range(3):
        yaml_io.atomic_write_text(workspace["link"], "v: {}\n".format(index))
        yaml_io.make_backup(workspace["link"], root=workspace["backups"])
    entries = yaml_io.list_backups(workspace["link"], workspace["backups"])
    assert entries == sorted(entries, key=lambda e: e["name"], reverse=True)


def test_restore_puts_the_contents_back(workspace):
    """Restoring recovers the old file and backs up the current one."""
    backup = yaml_io.make_backup(workspace["link"], root=workspace["backups"])
    yaml_io.atomic_write_text(workspace["link"], "edited: true\n")

    yaml_io.restore_backup(backup, workspace["link"], root=workspace["backups"])
    assert open(workspace["source"]).read() == "original: true\n"
    assert os.path.islink(workspace["link"])
    assert len(yaml_io.list_backups(workspace["link"], workspace["backups"])) == 2


def test_backup_directories_are_keyed_by_full_path(tmp_path):
    """Same basename in two packages must not share a backup directory."""
    first = tmp_path / "pkg_a" / "config.yaml"
    second = tmp_path / "pkg_b" / "config.yaml"
    for path in (first, second):
        path.parent.mkdir(parents=True)
        path.write_text("x: 1\n")

    root = str(tmp_path / "backups")
    assert yaml_io.backup_dir_for(str(first), root) != \
        yaml_io.backup_dir_for(str(second), root)


def test_listing_backups_for_an_unbacked_file_is_empty(tmp_path):
    """No backups yet is an empty list, not an error."""
    assert yaml_io.list_backups(str(tmp_path / "x.yaml"), str(tmp_path / "b")) == []


def test_restore_keeps_backups_in_the_configured_store(workspace):
    """
    Restoring must not scatter its own safety backup elsewhere.

    restore_backup is itself a write, so it takes a backup first. Without
    threading `root` through, that backup lands in the default store under
    the user's home instead of wherever the caller is keeping them.
    """
    backup = yaml_io.make_backup(workspace["link"], root=workspace["backups"])
    yaml_io.atomic_write_text(workspace["link"], "edited: true\n")
    yaml_io.restore_backup(backup, workspace["link"], root=workspace["backups"])

    entries = yaml_io.list_backups(workspace["link"], workspace["backups"])
    assert len(entries) == 2
    assert open(entries[0]["path"]).read() == "edited: true\n"


def test_backups_in_the_same_second_sort_chronologically(workspace):
    """Names must sort newest-first even without sub-second resolution."""
    contents = []
    for index in range(5):
        yaml_io.atomic_write_text(workspace["link"], "v: {}\n".format(index))
        yaml_io.make_backup(workspace["link"], root=workspace["backups"])
        contents.append("v: {}\n".format(index))

    entries = yaml_io.list_backups(workspace["link"], workspace["backups"])
    assert [open(entry["path"]).read() for entry in entries] == \
        list(reversed(contents))
