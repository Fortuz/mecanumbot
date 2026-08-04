"""
Symlink-safe, backed-up writes for config files inside a ROS share dir.

Pure Python -- no ``rclpy``, no Flask.

The whole point of this module is one easy mistake.  Under
``colcon build --symlink-install``, an ``ament_cmake`` package's
``install(DIRECTORY ...)`` does not copy its config files: it creates a
*symlink per file* pointing back into the source tree::

    install/mecanumbot_description/share/.../config/joystick/xbox360.yaml
      -> src/mecanumbot/mecanumbot_description/config/joystick/xbox360.yaml

``ament_python`` packages are worse -- two hops, install to build to
source.  Either way, ``get_package_share_directory()`` hands you a path
that is a symlink.

Writing through it correctly edits the source file.  But the obvious
atomic-write idiom::

    os.replace(tmp, share_path)          # WRONG

replaces the *symlink itself* with a regular file.  The source tree keeps
its old contents, the install tree silently stops tracking it, and every
later rebuild appears to lose the change.  Resolving the real path first
is the entire fix, and it is why every write in this package goes through
:func:`atomic_write_text`.

Backups live outside the source tree on purpose.  Every config file this
GUI edits sits in a git repository the user commits from, and dropping
``*.bak-2026-08-04T...`` files next to them would turn ``git status``
into noise.
"""

import errno
import os
import shutil
import stat
import tempfile
from datetime import datetime, timezone
from typing import List, Optional

#: Where backups accumulate, outside any git repository.
DEFAULT_BACKUP_ROOT = os.path.join(
    os.path.expanduser("~"), ".mecanumbot", "config_backups")

#: How many backups to keep per file before pruning the oldest.
KEEP_BACKUPS = 5

#: Suffix marking a backup file.
BACKUP_SUFFIX = ".bak"


def resolve_target(path: str) -> str:
    """
    Return the real file a (possibly symlinked) config path points at.

    Always call this before writing.  See the module docstring.
    """
    return os.path.realpath(path)


def _slug(real_path: str) -> str:
    """Turn an absolute path into a flat, unique, readable directory name."""
    return real_path.lstrip(os.sep).replace(os.sep, "__")


def backup_dir_for(path: str, root: str = DEFAULT_BACKUP_ROOT) -> str:
    """
    Return the backup directory for one config file.

    Keyed on the resolved path, so two files with the same basename in
    different packages never collide.
    """
    return os.path.join(root, _slug(resolve_target(path)))


def _timestamp() -> str:
    """Return a filesystem-safe UTC timestamp."""
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def list_backups(path: str, root: str = DEFAULT_BACKUP_ROOT) -> List[dict]:
    """List a file's backups, newest first."""
    directory = backup_dir_for(path, root)
    if not os.path.isdir(directory):
        return []

    entries = []
    for name in os.listdir(directory):
        if not name.endswith(BACKUP_SUFFIX):
            continue
        full = os.path.join(directory, name)
        try:
            modified = os.path.getmtime(full)
        except OSError:
            continue
        entries.append({
            "name": name,
            "path": full,
            "modified_utc": datetime.fromtimestamp(
                modified, timezone.utc).isoformat(timespec="seconds"),
            "size": os.path.getsize(full),
        })

    return sorted(entries, key=lambda entry: entry["name"], reverse=True)


def prune_backups(path: str, keep: int = KEEP_BACKUPS,
                  root: str = DEFAULT_BACKUP_ROOT) -> int:
    """Delete all but the ``keep`` newest backups. Returns how many went."""
    stale = list_backups(path, root)[keep:]
    for entry in stale:
        try:
            os.remove(entry["path"])
        except OSError:
            pass
    return len(stale)


def make_backup(path: str, keep: int = KEEP_BACKUPS,
                root: str = DEFAULT_BACKUP_ROOT) -> Optional[str]:
    """
    Copy a config file into the backup store. Returns the backup path.

    Returns ``None`` when the source does not exist yet, which is the
    normal case for a brand new profile.
    """
    real = resolve_target(path)
    if not os.path.isfile(real):
        return None

    directory = backup_dir_for(path, root)
    os.makedirs(directory, exist_ok=True)

    # The counter is always present and zero padded so that names sort
    # chronologically as plain strings, including for two saves inside the
    # same second. list_backups relies on that ordering.
    stamp = _timestamp()
    counter = 0
    destination = os.path.join(
        directory, "{}.{}-{:02d}{}".format(
            os.path.basename(real), stamp, counter, BACKUP_SUFFIX))
    while os.path.exists(destination):
        counter += 1
        destination = os.path.join(
            directory, "{}.{}-{:02d}{}".format(
                os.path.basename(real), stamp, counter, BACKUP_SUFFIX))

    shutil.copy2(real, destination)
    prune_backups(path, keep, root)
    return destination


def atomic_write_text(path: str, text: str) -> str:
    """
    Write ``text`` to the real file behind ``path``, atomically.

    Resolves symlinks first (see the module docstring), writes a temporary
    file in the same directory so the rename cannot cross a filesystem,
    fsyncs it, then renames over the target.  A reader either sees the old
    file or the new one, never a half-written one -- which matters because
    the joy node reloads these files on request and a truncated profile
    would take the joystick offline.

    Returns the real path written.
    """
    real = resolve_target(path)
    directory = os.path.dirname(real) or "."

    try:
        os.makedirs(directory, exist_ok=True)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            raise

    mode = None
    if os.path.isfile(real):
        mode = stat.S_IMODE(os.stat(real).st_mode)

    handle, temporary = tempfile.mkstemp(
        dir=directory, prefix=".{}.".format(os.path.basename(real)), suffix=".tmp")
    try:
        with os.fdopen(handle, "w") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        if mode is not None:
            os.chmod(temporary, mode)
        os.replace(temporary, real)
    except BaseException:
        try:
            os.remove(temporary)
        except OSError:
            pass
        raise

    return real


def restore_backup(
    backup_path: str,
    path: str,
    keep: int = KEEP_BACKUPS,
    root: str = DEFAULT_BACKUP_ROOT,
) -> str:
    """
    Restore a backup over a config file, backing up the current one first.

    ``root`` must be threaded through: restoring is itself a write, and
    its safety backup has to land in the same store the caller is using,
    not in the default one.
    """
    with open(backup_path, "r") as stream:
        text = stream.read()
    make_backup(path, keep=keep, root=root)
    return atomic_write_text(path, text)


def read_text(path: str) -> str:
    """Read a config file as text."""
    with open(path, "r") as stream:
        return stream.read()


def is_symlinked_install(path: str) -> bool:
    """
    Report whether ``path`` is a symlink into a source tree.

    Surfaced in the GUI so it is obvious whether a save edits the source
    checkout (symlink-install, the normal development setup) or only the
    installed copy (a plain ``colcon build``, where the edit is lost on
    the next build).
    """
    return os.path.islink(path) or resolve_target(path) != os.path.abspath(path)
