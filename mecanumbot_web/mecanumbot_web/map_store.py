"""
Which map the study navigation stack loads.

No ``rclpy`` and no Flask, because two very different things read it: the
maps page, and ``mecanumbot_bringup/nav2.launch.py``, which imports this
module so that the two cannot disagree about what a choice means.

**A map is a folder.**  ``maps/<name>/<name>.yaml`` under
``mecanumbot_description``'s share directory, the layout every launch file
has always assumed.  A folder that breaks it is listed with what is wrong
rather than hidden, because a map that silently does not appear is harder to
explain than one that says why it cannot be chosen.

**The Wi-Fi SSID is still the default.**  A choice made on the page is a pin
that overrides it, held in one small file outside the source tree, so it
survives a reboot and leaves ``git status`` alone.  With no pin, every launch
picks exactly what it picked before this page existed.

**A pin moves the map and nothing else.**  The behaviour constants are still
selected by SSID, and their routes are coordinates on one particular map, so
:func:`constants_follow_wifi` exists for the page to say so.
"""

import glob
import os
from typing import List, Optional, Tuple

import yaml

from .yaml_io import atomic_write_text

#: Wi-Fi SSID to map folder.
SSID_MAPS = {
    "MecanumNet": "AI_dept",
    "MecanumetoNet": "LED_exp",
}

#: The map of any other network.
DEFAULT_MAP = "AI_dept"

#: Where the page's choice lives, outside any git repository.
SELECTION_FILE = os.path.join(
    os.path.expanduser("~"), ".mecanumbot", "selected_map.yaml")

#: The launch's word for where a map came from.
SELECTED = "selected"
WIFI = "wifi"


class MapStoreError(Exception):
    """A map that cannot be chosen, in words the page can show."""


def ssid_map(ssid: Optional[str]) -> str:
    """Return the map folder a Wi-Fi network selects."""
    return SSID_MAPS.get(ssid, DEFAULT_MAP)


def keepout_file(maps_dir: str, name: str) -> str:
    """Return a map folder's keepout mask YAML, or '' if it has none."""
    # The folder name is matched literally, whatever characters it contains.
    folder = glob.escape(os.path.join(maps_dir, name))
    matches = sorted(glob.glob(os.path.join(folder, "*_keepout.yaml")))
    return matches[0] if matches else ""


def _valid_name(name) -> bool:
    """Return whether ``name`` is one path component, not a route elsewhere."""
    return (isinstance(name, str) and bool(name) and not name.startswith(".")
            and os.sep not in name)


def describe_map(maps_dir: str, name: str) -> dict:
    """Describe one folder: usable or not, and why, with what the page shows."""
    entry = {
        "name": name,
        "ok": False,
        "problem": "",
        "yaml": "",
        "resolution": None,
        "keepout": False,
    }
    if not _valid_name(name) or not maps_dir or \
            not os.path.isdir(os.path.join(maps_dir, name)):
        entry["problem"] = "No such map folder"
        return entry

    path = os.path.join(maps_dir, name, "{}.yaml".format(name))
    entry["keepout"] = bool(keepout_file(maps_dir, name))
    if not os.path.isfile(path):
        entry["problem"] = "No {}.yaml in this folder".format(name)
        return entry
    entry["yaml"] = path

    try:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
        entry["problem"] = "Cannot read {}.yaml: {}".format(name, exc)
        return entry

    image = document.get("image") if isinstance(document, dict) else None
    if not image:
        entry["problem"] = "{}.yaml names no image".format(name)
        return entry
    if not os.path.isfile(os.path.join(maps_dir, name, str(image))):
        entry["problem"] = "The image {} is missing".format(image)
        return entry

    entry["resolution"] = document.get("resolution")
    entry["ok"] = True
    return entry


def list_maps(maps_dir: str) -> List[dict]:
    """Describe every folder under the maps directory, sorted by name."""
    if not maps_dir or not os.path.isdir(maps_dir):
        return []
    return [describe_map(maps_dir, name) for name in sorted(os.listdir(maps_dir))
            if _valid_name(name) and os.path.isdir(os.path.join(maps_dir, name))]


def read_selection(path: str = SELECTION_FILE) -> Optional[str]:
    """Return the pinned folder name, or None when the SSID decides."""
    try:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, UnicodeDecodeError, yaml.YAMLError):
        return None
    name = document.get("map") if isinstance(document, dict) else None
    return name if isinstance(name, str) and name else None


def write_selection(name: Optional[str], path: str = SELECTION_FILE) -> None:
    """Pin a folder, or with None remove the pin and let the SSID decide."""
    if name is None:
        try:
            os.remove(path)
        except FileNotFoundError:
            pass
        return
    # yaml escapes anything non-ASCII, so a Hungarian folder name survives a
    # launch running under a different locale from the page.
    atomic_write_text(path, yaml.safe_dump({"map": name}))


def select(maps_dir: str, name: Optional[str], path: str = SELECTION_FILE) -> None:
    """Pin a folder after checking it can load; raise if it cannot."""
    if name is not None:
        entry = describe_map(maps_dir, name)
        if not entry["ok"]:
            raise MapStoreError("Cannot select '{}': {}".format(name, entry["problem"]))
    write_selection(name, path)


def resolve(ssid: Optional[str], maps_dir: str,
            path: str = SELECTION_FILE) -> Tuple[str, str, str]:
    """
    Return ``(map name, where it came from, note)`` for one launch.

    A pin whose folder has since gone falls back to the SSID's map, and the
    note says so: nav2 coming up on the room's usual map is recoverable, and
    a launch that aborts on a stale file takes the drivers down with it.
    """
    pinned = read_selection(path)
    if pinned is None:
        return ssid_map(ssid), WIFI, ""
    problem = describe_map(maps_dir, pinned)["problem"]
    if problem:
        return ssid_map(ssid), WIFI, "selected map '{}' ignored: {}".format(
            pinned, problem)
    return pinned, SELECTED, ""


def constants_follow_wifi(ssid: Optional[str], map_name: str) -> bool:
    """Return whether the SSID's constants were written for another map."""
    return map_name != ssid_map(ssid)


def name_from_yaml_path(path: Optional[str]) -> Optional[str]:
    """Return the folder of a map server's ``yaml_filename``, or None."""
    if not path:
        return None
    return os.path.basename(os.path.dirname(path)) or None
