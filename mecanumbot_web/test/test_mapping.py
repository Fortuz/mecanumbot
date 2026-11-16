"""
Tests for the mapping session: naming, where a map is saved, and the sequence.

No ROS: SLAM is ``sleep``, the map saver is a script that writes the two
files ``map_saver_cli`` writes, and nav2 is a stub recording what it was
asked.  The install layout is rebuilt in a temporary directory -- a source
``maps/`` and an install ``maps/`` whose files are symlinks into it -- so
the symlink-install handling is tested on the real filesystem.
"""

import os
import sys
import time

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import map_store, mapping  # noqa: E402

FAKE_SAVER = '''
import sys
base = sys.argv[sys.argv.index("-f") + 1]
open(base + ".pgm", "wb").write(b"P5\\n2 2\\n255\\n" + bytes([0, 205, 254, 0]))
open(base + ".yaml", "w").write(
    "image: " + base + ".pgm\\nmode: trinary\\nresolution: 0.05\\n"
    "origin: [-1.0, -2.0, 0]\\nnegate: 0\\noccupied_thresh: 0.65\\nfree_thresh: 0.25\\n")
'''


def _make_map(folder, name):
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, name + ".pgm"), "wb") as handle:
        handle.write(b"P5\n1 1\n255\n\x00")
    with open(os.path.join(folder, name + ".yaml"), "w") as handle:
        handle.write("image: {}.pgm\nresolution: 0.05\n".format(name))


@pytest.fixture
def layout(tmp_path):
    """Build src/maps with one map, and install/maps linking to it."""
    source = tmp_path / "src" / "maps"
    install = tmp_path / "install" / "maps"
    _make_map(str(source / "AI_dept"), "AI_dept")
    os.makedirs(str(install / "AI_dept"))
    for entry in os.listdir(str(source / "AI_dept")):
        os.symlink(str(source / "AI_dept" / entry), str(install / "AI_dept" / entry))
    saver = tmp_path / "saver.py"
    saver.write_text(FAKE_SAVER)
    return {"source": str(source), "install": str(install),
            "saver": [sys.executable, str(saver)], "tmp": tmp_path}


class StubNav:
    """Records what the session asked of nav2."""

    def __init__(self, left=()):
        self.calls = []
        self.left = list(left)
        self.mapping_active = lambda: False

    def stop_stack(self):
        self.calls.append("stop_stack")
        return list(self.left)

    def restart(self):
        self.calls.append(("restart", self.mapping_active()))
        return {}

    def status(self):
        return {"busy": False}


def _session(layout, nav=None, live=None, behaviour=None):
    nav = nav if nav is not None else StubNav()
    session = mapping.MappingSession(
        nav=nav, maps_dir=layout["install"],
        live_nodes=lambda: list(live or []),
        behaviour_status=lambda: dict(behaviour or {}),
        command=["sleep", "30"], saver=layout["saver"],
        selection_file=str(layout["tmp"] / "selected_map.yaml"))
    nav.mapping_active = session.active
    return session, nav


def _wait(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


# ── names and files ──────────────────────────────────────────────────────

@pytest.mark.parametrize("name", ["AI_dept_2", "LED-exp", "folyosó", "lab3"])
def test_good_names(name):
    assert mapping.check_name("  " + name + " ") == name


@pytest.mark.parametrize("name", ["", "   ", "../evil", "a/b", ".hidden", "-x",
                                  "has space", None, "x" * 65])
def test_bad_names(name):
    with pytest.raises(mapping.MappingError):
        mapping.check_name(name)


def test_the_source_tree_is_found_through_the_install_links(layout):
    assert mapping.source_maps_dir(layout["install"]) == layout["source"]


def test_a_copied_install_has_no_source(tmp_path):
    _make_map(str(tmp_path / "maps" / "A"), "A")
    assert mapping.source_maps_dir(str(tmp_path / "maps")) is None


def test_the_yaml_is_made_portable_and_unknown_safe(tmp_path):
    path = tmp_path / "m.yaml"
    path.write_text("image: /tmp/abs/m.pgm\nresolution: 0.05\nfree_thresh: 0.25\n"
                    "origin: [-1.0, -2.0, 0]\n")
    mapping.fix_map_yaml(str(path), "m.pgm")
    document = yaml.safe_load(path.read_text())
    assert document["image"] == "m.pgm"
    assert document["free_thresh"] == pytest.approx(0.196)
    assert document["origin"] == [-1.0, -2.0, 0]
    # 205 grey is 0.19608 occupied, and must not count as free.
    assert (255 - 205) / 255.0 >= document["free_thresh"]


# ── the session ──────────────────────────────────────────────────────────

def test_a_session_saves_into_the_source_and_links_it_into_the_install(layout):
    session, nav = _session(layout)
    session.start()
    assert _wait(lambda: session.status()["can_save"])
    assert nav.calls == ["stop_stack"]

    saved = session.save("new_room")
    assert saved["in_source"] and saved["linked_into_install"]
    src_folder = os.path.join(layout["source"], "new_room")
    inst_folder = os.path.join(layout["install"], "new_room")
    assert sorted(os.listdir(src_folder)) == ["new_room.pgm", "new_room.yaml"]
    for entry in os.listdir(inst_folder):
        link = os.path.join(inst_folder, entry)
        assert os.path.islink(link)
        assert os.path.realpath(link) == os.path.join(src_folder, entry)
    # The Maps page can choose it straight away.
    assert map_store.describe_map(layout["install"], "new_room")["ok"]
    # No staging folder is left behind.
    assert not [n for n in os.listdir(layout["source"]) if n.startswith(".")]

    session.finish(restart_nav=True, load="new_room")
    assert _wait(lambda: session.status()["phase"] == mapping.FINISHED)
    assert not session.status()["process"]["running"]
    # nav2 was restarted after the session stopped owning navigation.
    assert nav.calls[-1] == ("restart", False)
    assert map_store.read_selection(str(layout["tmp"] / "selected_map.yaml")) == "new_room"


def test_an_existing_map_is_not_replaced_unless_asked(layout):
    session, _nav = _session(layout)
    session.start()
    assert _wait(lambda: session.status()["can_save"])
    with pytest.raises(mapping.MappingError, match="already exists"):
        session.save("AI_dept")
    # Still mapping after a refused save.
    assert session.status()["can_save"]
    session.save("AI_dept", overwrite=True)
    document = yaml.safe_load(open(os.path.join(layout["source"], "AI_dept", "AI_dept.yaml")))
    assert document["free_thresh"] == pytest.approx(0.196)
    session.shutdown()


def test_saving_needs_a_running_session(layout):
    session, _nav = _session(layout)
    with pytest.raises(mapping.MappingError, match="Start mapping"):
        session.save("x")


def test_a_running_tree_refuses_a_session(layout):
    session, nav = _session(layout, behaviour={"running": True, "label": "Leading"})
    with pytest.raises(mapping.MappingError, match="Leading"):
        session.start()
    assert nav.calls == []


def test_slam_already_on_the_graph_refuses_a_session(layout):
    session, _nav = _session(layout, live=["/slam_toolbox"])
    with pytest.raises(mapping.MappingError, match="SLAM is already running"):
        session.start()


def test_nav2_that_will_not_leave_fails_the_session_before_slam(layout):
    session, _nav = _session(layout, nav=StubNav(left=["amcl"]))
    session.start()
    assert _wait(lambda: session.status()["phase"] == mapping.FAILED)
    assert "amcl" in session.status()["error"]
    assert not session.status()["process"]["running"]


def test_the_session_reports_itself_active_to_nav2(layout):
    session, nav = _session(layout)
    session.start()
    assert _wait(lambda: session.status()["can_save"])
    assert nav.mapping_active() is True
    session.finish(restart_nav=False)
    assert _wait(lambda: session.status()["phase"] == mapping.FINISHED)
    assert nav.mapping_active() is False
    assert not any(isinstance(c, tuple) for c in nav.calls)
