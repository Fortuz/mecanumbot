"""
Tests for the catalog of startable behaviours.

The command a run produces is the thing worth testing here: it is what
tells the robot which experimental condition it is running, and the page
has no other way to say it.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import behaviour_catalog as bc  # noqa: E402


@pytest.fixture
def config_dir(tmp_path):
    """Return a config directory holding both leading constants files."""
    for name in ("behaviour_setting_constants.yaml",
                 "Eto_behaviour_setting_constants.yaml"):
        (tmp_path / name).write_text("bottom_up_tree_node:\n")
    return str(tmp_path)


# ── the catalog itself ───────────────────────────────────────────────────

def test_every_behaviour_has_a_key_and_a_target():
    """Each entry names something that can actually be run."""
    for spec in bc.CATALOG:
        assert spec.key and spec.package and spec.target
        assert spec.mode in ("launch", "run")
        assert spec.editor in ("leading", "flat")


def test_keys_are_unique():
    """Keys address behaviours in URLs, so a collision would misroute."""
    assert len(set(bc.keys())) == len(bc.keys())


def test_unknown_key_is_refused():
    """A key not in the catalog cannot be turned into a command."""
    with pytest.raises(bc.CatalogError):
        bc.get("no_such_behaviour")


# ── building the command ─────────────────────────────────────────────────

def test_leading_command_carries_the_condition(config_dir):
    """The condition is what selects the tree, so it must reach the launch."""
    command = bc.LEADING.build_command(
        {"condition": "LED", "params_file": "behaviour_setting_constants.yaml"},
        config_dir)
    assert command[:4] == ["ros2", "launch", "mecanumbot_leading_behaviour",
                           "launch_wifi_condition_sequence.launch.py"]
    assert "condition:=LED" in command


def test_constants_file_is_passed_as_both_arguments(config_dir):
    """
    ``params`` and ``yaml_path`` must name the same file.

    The launch file hands ``params`` to ROS and ``yaml_path`` to the tree's
    own loader, and only the second is ever read -- so setting one without
    the other would edit a file the tree never opens.
    """
    command = bc.LEADING.build_command(
        {"params_file": "Eto_behaviour_setting_constants.yaml"}, config_dir)
    path = os.path.join(config_dir, "Eto_behaviour_setting_constants.yaml")
    assert "params:={}".format(path) in command
    assert "yaml_path:={}".format(path) in command


def test_omitted_arguments_take_their_defaults(config_dir):
    """A page that sends nothing still starts the documented default run."""
    command = bc.LEADING.build_command({}, config_dir)
    assert "condition:=Doglike" in command
    assert "namespace:=mecanumbot" in command


def test_unknown_argument_is_refused(config_dir):
    """
    An argument the catalog does not know is an error, not a no-op.

    Dropping it silently would start a run that is not the one asked
    for, and these runs are experiment trials.
    """
    with pytest.raises(bc.CatalogError):
        bc.LEADING.build_command({"speed": "fast"}, config_dir)


def test_condition_outside_the_choices_is_refused(config_dir):
    """Only the three conditions the launch file implements are startable."""
    with pytest.raises(bc.CatalogError) as excinfo:
        bc.LEADING.build_command({"condition": "Doglike; rm -rf /"}, config_dir)
    assert "Condition" in str(excinfo.value)


def test_namespace_must_be_a_ros_name(config_dir):
    """A namespace is a name token, so nothing else can be smuggled in."""
    with pytest.raises(bc.CatalogError):
        bc.LEADING.build_command({"namespace": "mecanumbot && reboot"}, config_dir)


@pytest.mark.parametrize("attempt", [
    "../../../etc/passwd",
    "/etc/passwd",
    "nonexistent.yaml",
    "",
])
def test_only_files_in_the_config_directory_can_be_launched(config_dir, attempt):
    """
    The filename is matched against the directory listing, not sanitised.

    Which means traversal, absolute paths and simply-wrong names all fail
    the same way, without the check having to anticipate each of them.
    """
    with pytest.raises(bc.CatalogError):
        bc.LEADING.build_command({"params_file": attempt}, config_dir)


def test_demo_is_started_with_ros2_run(config_dir):
    """
    The demo package's launch file names executables it does not register.

    So the catalog starts the one executable that exists, and its
    ``--yaml_path`` comes before ``--ros-args`` because rclpy takes
    everything after that.
    """
    command = bc.DEMO.build_command({}, config_dir)
    assert command[:2] == ["ros2", "run"]
    assert command.index("--yaml_path") < command.index("--ros-args")
    assert "__ns:=/mecanumbot" in command


# ── the SSID mapping ─────────────────────────────────────────────────────

def test_ssid_picks_the_same_file_the_launch_file_would():
    """The page and the launch file have to agree about the room."""
    assert bc.LEADING.file_for_ssid("MecanumetoNet") == \
        "Eto_behaviour_setting_constants.yaml"
    assert bc.LEADING.file_for_ssid("MecanumNet") == \
        "behaviour_setting_constants.yaml"
    assert bc.OSTENSIVE.file_for_ssid("MecanumetoNet") == \
        "Eto_ostensive_setting_constants.yaml"


def test_an_unknown_ssid_falls_back():
    """An unrecognised network is the AI_dept default, as everywhere else."""
    assert bc.LEADING.file_for_ssid("SomeCafeWifi") == \
        "behaviour_setting_constants.yaml"
    assert bc.LEADING.file_for_ssid(None) == "behaviour_setting_constants.yaml"


# ── describing it for the page ───────────────────────────────────────────

def test_description_offers_the_files_that_exist(config_dir):
    """File options come from the directory, so a new file needs no restart."""
    described = bc.LEADING.describe(config_dir, "MecanumNet")
    files = [argument for argument in described["arguments"]
             if argument["name"] == "params_file"][0]
    assert files["options"] == sorted(os.listdir(config_dir))
    assert described["available"] is True


def test_a_missing_package_is_described_as_unavailable():
    """A behaviour whose package is not built is shown, and cannot be run."""
    described = bc.OSTENSIVE.describe("", None)
    assert described["available"] is False
    assert described["files"] == []


def test_every_behaviour_names_the_nodes_it_puts_on_the_graph():
    """Used to spot a tree somebody started from a terminal."""
    for spec in bc.CATALOG:
        assert spec.node_names
    assert "bottom_up_tree_node" in bc.all_node_names()


# ── the experiments added after the page was written ─────────────────────

def test_every_experiment_in_the_workspace_is_startable():
    """
    The page and a shell have to be able to start the same runs.

    Seek, fetch and autoslam existed for months before the page could start
    them, so a trial run from the browser and one run from a terminal were
    not the same set of experiments.
    """
    assert set(bc.keys()) == {
        "leading", "ostensive", "seek", "fetch", "autoslam", "demo"}


@pytest.mark.parametrize("spec,expected", [
    (bc.SEEK, ["ros2", "launch", "mecanumbot_seek", "launch_seek.launch.py"]),
    (bc.FETCH, ["ros2", "launch", "mecanumbot_fetch_behaviour",
                "launch_fetch.launch.py"]),
    (bc.AUTOSLAM, ["ros2", "launch", "mecanumbot_autoslam",
                   "launch_autoslam.launch.py"]),
])
def test_each_new_behaviour_starts_its_own_launch_file(spec, expected, tmp_path):
    """The command is the one the package's README documents."""
    (tmp_path / spec.default_file).write_text("x:\n")
    assert spec.build_command({}, str(tmp_path))[:4] == expected


def test_autoslam_passes_its_constants_once(tmp_path):
    """
    Autoslam takes `params` and not also `yaml_path`.

    Unlike the trees, this pass has no blackboard: its constants are ordinary
    ROS parameters, so there is no second path for a loader to open, and
    emitting one would be an argument its launch file does not declare.
    """
    (tmp_path / "autoslam_setting_constants.yaml").write_text("/**:\n")
    argv = bc.AUTOSLAM.build_command({}, str(tmp_path))
    assert len([part for part in argv if part.startswith("params")]) == 1
    assert not any(part.startswith("yaml_path") for part in argv)


def test_autoslam_offers_the_two_choices_that_can_ruin_a_pass(tmp_path):
    """`require_cloud` and `use_preflight` are both on by default."""
    (tmp_path / "autoslam_setting_constants.yaml").write_text("/**:\n")
    argv = bc.AUTOSLAM.build_command({}, str(tmp_path))
    assert "require_cloud:=true" in argv
    assert "use_preflight:=true" in argv


def test_autoslam_says_that_it_stops_other_behaviours():
    """
    The page must warn about it: a pass started here stops the tree next to it.

    That is the point of the preflight, but a person clicking Start on one row
    should not discover it by watching another row's tree die.
    """
    assert "SHUTTING DOWN" in bc.AUTOSLAM.note


def test_the_new_experiments_use_the_flat_editor():
    """Their constants are one block of scalars, like the ostensive file."""
    for spec in (bc.SEEK, bc.FETCH, bc.AUTOSLAM):
        assert spec.editor == "flat"


def test_autoslam_loads_one_file_whatever_the_ssid():
    """
    Unlike the trees, the pass has no room-specific constants file.

    Everything in it describes the lidar, the robot and the reconstruction
    rather than the building; only `max_duration` is room-dependent.
    """
    for ssid in ("MecanumNet", "MecanumetoNet", None):
        assert bc.AUTOSLAM.file_for_ssid(ssid) == "autoslam_setting_constants.yaml"
