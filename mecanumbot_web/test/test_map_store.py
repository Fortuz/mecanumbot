"""Tests for choosing the map, against real folders and the shipped maps."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import map_store  # noqa: E402

SHIPPED = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "mecanumbot_description", "maps")


def make_map(root, name, keepout=False):
    """Create a loadable map folder."""
    folder = root / name
    folder.mkdir()
    (folder / "{}.pgm".format(name)).write_text("P5")
    (folder / "{}.yaml".format(name)).write_text(
        "image: {}.pgm\nresolution: 0.05\n".format(name))
    if keepout:
        (folder / "{}_keepout.yaml".format(name)).write_text("image: k.pgm\n")


@pytest.fixture
def selection(tmp_path):
    """Return a selection file that does not exist yet."""
    return str(tmp_path / "state" / "selected_map.yaml")


def test_every_shipped_map_can_be_chosen():
    listed = map_store.list_maps(SHIPPED)
    assert {"AI_dept", "LED_exp", "ethodept_old"} <= {e["name"] for e in listed}
    assert [e["problem"] for e in listed if not e["ok"]] == []


def test_only_the_led_room_ships_a_keepout_mask():
    """The launch serves a mask for any map with one, so this pins the study."""
    with_mask = {e["name"] for e in map_store.list_maps(SHIPPED) if e["keepout"]}
    assert with_mask == {"LED_exp"}


@pytest.mark.parametrize("ssid,expected", [
    ("MecanumNet", "AI_dept"),
    ("MecanumetoNet", "LED_exp"),
    ("somewhere else", "AI_dept"),
    (None, "AI_dept"),
])
def test_with_nothing_chosen_the_wifi_decides_as_it_always_did(
        ssid, expected, selection):
    assert map_store.resolve(ssid, SHIPPED, selection) == (
        expected, map_store.WIFI, "")


def test_a_choice_overrides_the_wifi_and_clearing_it_gives_it_back(selection):
    map_store.select(SHIPPED, "LED_exp", selection)
    assert map_store.resolve("MecanumNet", SHIPPED, selection) == (
        "LED_exp", map_store.SELECTED, "")
    map_store.select(SHIPPED, None, selection)
    assert map_store.resolve("MecanumNet", SHIPPED, selection)[0] == "AI_dept"


def test_clearing_nothing_is_not_an_error(selection):
    map_store.select(SHIPPED, None, selection)
    assert map_store.read_selection(selection) is None


def test_a_hungarian_folder_name_survives_the_file(tmp_path, selection):
    make_map(tmp_path, "Egyéb_terem")
    map_store.select(str(tmp_path), "Egyéb_terem", selection)
    assert map_store.read_selection(selection) == "Egyéb_terem"
    with open(selection, "rb") as stream:
        assert stream.read().isascii()


def test_a_chosen_folder_that_has_gone_falls_back_and_says_so(tmp_path, selection):
    make_map(tmp_path, "room")
    map_store.select(str(tmp_path), "room", selection)
    (tmp_path / "room" / "room.yaml").unlink()
    name, source, note = map_store.resolve("MecanumetoNet", str(tmp_path), selection)
    assert (name, source) == ("LED_exp", map_store.WIFI)
    assert "room" in note


@pytest.mark.parametrize("name", ["nowhere", "../room", ".hidden", "", None, 7])
def test_a_bad_name_is_never_loadable(tmp_path, name):
    make_map(tmp_path, "room")
    assert map_store.describe_map(str(tmp_path), name)["ok"] is False


def test_a_refused_folder_changes_nothing(tmp_path, selection):
    make_map(tmp_path, "room")
    map_store.select(str(tmp_path), "room", selection)
    with pytest.raises(map_store.MapStoreError):
        map_store.select(str(tmp_path), "nowhere", selection)
    assert map_store.read_selection(selection) == "room"


def test_broken_folders_are_listed_with_what_is_wrong(tmp_path):
    (tmp_path / "empty").mkdir()
    (tmp_path / "no_image").mkdir()
    (tmp_path / "no_image" / "no_image.yaml").write_text("image: gone.pgm\n")
    (tmp_path / "bad_yaml").mkdir()
    (tmp_path / "bad_yaml" / "bad_yaml.yaml").write_text("image: [\n")
    (tmp_path / ".git").mkdir()
    (tmp_path / "stray.txt").write_text("x")

    problems = {e["name"]: e["problem"] for e in map_store.list_maps(str(tmp_path))}
    assert set(problems) == {"empty", "no_image", "bad_yaml"}
    assert "empty.yaml" in problems["empty"]
    assert "gone.pgm" in problems["no_image"]
    assert "Cannot read" in problems["bad_yaml"]


def test_a_garbled_selection_file_means_no_choice(tmp_path):
    path = tmp_path / "selected_map.yaml"
    path.write_text("map: [\n")
    assert map_store.read_selection(str(path)) is None


def test_keepout_is_found_in_a_folder_with_glob_characters(tmp_path):
    make_map(tmp_path, "room[1]", keepout=True)
    assert map_store.keepout_file(str(tmp_path), "room[1]").endswith(
        "room[1]_keepout.yaml")


def test_the_loaded_folder_comes_from_the_servers_path():
    path = "/x/share/mecanumbot_description/maps/LED_exp/LED_exp.yaml"
    assert map_store.name_from_yaml_path(path) == "LED_exp"
    assert map_store.name_from_yaml_path("") is None
