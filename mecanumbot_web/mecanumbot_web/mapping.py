"""
Make a new map from the page: stop nav2, run SLAM, save, bring nav2 back.

No ``rclpy`` and no Flask, so the whole sequence runs in a test against
real child processes, the same as :mod:`.nav_stack`.

**Mapping is ``mecanumbot_bringup mapping.launch.py``** -- slam_toolbox in
mapping mode plus the scan-grid node it reads -- the same two nodes the
simulator's ``mode:=mapping`` starts.  (The bringup file called
``slam_gmapping.launch.py`` actually starts Cartographer; neither gmapping
nor Cartographer is what the workspace maps with.)

**The study nav2 stack has to go first.**  AMCL publishes ``map -> odom``
and slam_toolbox publishes it too; two parents of one frame make the TF
tree flicker between them and the new map smears.  The map server's
``/map`` would also be the topic the saver records.  So a session stops
nav2 with :class:`~.nav_stack.NavStackRestarter`'s own stop, and ends by
asking it to relaunch.

**A map is saved where every launch file looks for it**:
``maps/<name>/<name>.yaml`` beside ``<name>.pgm`` in ``mecanumbot_description``.
The installed ``maps/`` is a symlink install of the source folder, file by
file, so a new folder is written into the *source* tree (where git sees it
and a rebuild keeps it) and linked into the share directory the same way
colcon would, so the Maps page and nav2 can load it straight away.  When
the install is a copy, the source cannot be found from it and the folder is
written into the share directory alone, which the page says.

**The saved YAML is rewritten in two places.**  ``image:`` is made the bare
file name, so the folder can be moved; and ``free_thresh`` is 0.196, as in
``AI_dept.yaml``: the saver writes unexplored cells as grey 205, which is
0.196 occupied, and the default 0.25 reads that as *free* -- the planner
then routes straight out of the mapped area.
"""

import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Callable, Iterable, List, Optional, Sequence

import yaml

from . import behaviour_runner, map_store, nav_stack
from .yaml_io import atomic_write_text

#: How a session maps.
COMMAND = ("ros2", "launch", "mecanumbot_bringup", "mapping.launch.py")

#: How a map is saved. ``-f`` and the base path are appended.
SAVER = ("ros2", "run", "nav2_map_server", "map_saver_cli")

#: What the saver is told, after ``--ros-args``.
SAVER_PARAMETERS = (
    "-p", "save_map_timeout:=10.0",
    "-p", "free_thresh_default:=0.196",
    "-p", "occupied_thresh_default:=0.65",
    # slam_toolbox latches the map, so a late subscriber still gets it.
    "-p", "map_subscribe_transient_local:=true",
)

#: The saver's own wait for a map is 10 s; this is the whole run.
SAVE_TIMEOUT = 40.0

FREE_THRESH = 0.196

#: Node names that are slam_toolbox's or the T1 pass's.
SLAM_NODES = ("slam_toolbox", "autoslam_node")

#: A map name: one folder, letters (any script), digits, ``_`` and ``-``.
NAME_PATTERN = re.compile(r"^[^\W][\w\-]*$", re.UNICODE)

IDLE = "idle"
STOPPING_NAV = "stopping_nav"
STARTING = "starting"
MAPPING = "mapping"
SAVING = "saving"
FINISHING = "finishing"
FINISHED = "finished"
FAILED = "failed"


class MappingError(Exception):
    """A mapping request that cannot be carried out, in the page's words."""


def check_name(name) -> str:
    """Return ``name`` stripped, or raise if it cannot be a map folder."""
    name = name.strip() if isinstance(name, str) else ""
    if not name:
        raise MappingError("Give the map a name.")
    if len(name) > 64 or not NAME_PATTERN.match(name):
        raise MappingError(
            "'{}' cannot be a map folder: use letters, digits, '_' and '-', "
            "starting with a letter or digit.".format(name))
    return name


def source_maps_dir(maps_dir: str) -> Optional[str]:
    """
    Return the source ``maps/`` the installed one links to, or None.

    Found by following any file of an existing map: under a symlink install
    each is a link into ``src/.../mecanumbot_description/maps/<map>/``.
    """
    if not maps_dir or not os.path.isdir(maps_dir):
        return None
    for name in sorted(os.listdir(maps_dir)):
        folder = os.path.join(maps_dir, name)
        if not os.path.isdir(folder):
            continue
        for entry in sorted(os.listdir(folder)):
            path = os.path.join(folder, entry)
            if os.path.islink(path):
                source = os.path.dirname(os.path.dirname(os.path.realpath(path)))
                if os.path.isdir(source) and \
                        os.path.realpath(source) != os.path.realpath(maps_dir):
                    return source
    return None


def fix_map_yaml(path: str, image: str) -> None:
    """Point the YAML at a bare image name and set the unknown-safe threshold."""
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    document["image"] = image
    document["free_thresh"] = FREE_THRESH
    header = ("# Saved from the web GUI's Maps page on {}. free_thresh is 0.196, "
              "not 0.25, so the\n# grey of unexplored cells reads as unknown "
              "rather than free.\n").format(time.strftime("%Y-%m-%d %H:%M"))
    atomic_write_text(path, header + yaml.safe_dump(
        document, sort_keys=False, default_flow_style=None))


def install_map(staging: str, name: str, maps_dir: str,
                overwrite: bool = False) -> dict:
    """
    Move a saved ``<name>.yaml`` + ``<name>.pgm`` into the map folders.

    Returns where it went.  Raises if the folder exists and ``overwrite``
    is not set, so a name typed twice cannot replace a study map.
    """
    files = ["{}.yaml".format(name), "{}.pgm".format(name)]
    for entry in files:
        if not os.path.isfile(os.path.join(staging, entry)):
            raise MappingError("The saver did not write {}.".format(entry))

    source_root = source_maps_dir(maps_dir)
    roots = [root for root in (source_root, maps_dir) if root]
    for root in roots:
        target = os.path.join(root, name)
        if os.path.exists(target) and not overwrite:
            raise MappingError(
                "A map called '{}' already exists ({}). Choose another name, or "
                "tick 'replace' to overwrite it.".format(name, target))

    primary = os.path.join(source_root or maps_dir, name)
    os.makedirs(primary, exist_ok=True)
    for entry in files:
        os.replace(os.path.join(staging, entry), os.path.join(primary, entry))

    linked = False
    if source_root:
        # The same shape a symlink install gives every other map.
        installed = os.path.join(maps_dir, name)
        os.makedirs(installed, exist_ok=True)
        for entry in files:
            link = os.path.join(installed, entry)
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(os.path.join(primary, entry), link)
        linked = True

    return {
        "name": name,
        "folder": primary,
        "in_source": bool(source_root),
        "linked_into_install": linked,
        "yaml": os.path.join(primary, files[0]),
    }


class MappingSession:
    """One mapping session at a time: start, save any number of times, finish."""

    def __init__(self, nav, maps_dir: str,
                 live_nodes: Callable[[], Iterable[str]],
                 behaviour_status: Optional[Callable[[], dict]] = None,
                 tree_nodes: Sequence[str] = (),
                 command: Sequence[str] = COMMAND,
                 saver: Sequence[str] = SAVER,
                 selection_file: str = map_store.SELECTION_FILE,
                 logger=None):
        """Build a session over the nav2 restarter and the maps directory."""
        self._nav = nav
        self._maps_dir = maps_dir
        self._live_nodes = live_nodes
        self._behaviour_status = behaviour_status or (lambda: {})
        self._tree_nodes = tuple(tree_nodes)
        self._command = list(command)
        self._saver = list(saver)
        self._selection_file = selection_file
        self._logger = logger

        #: The SLAM launch; its console is what the page shows.
        self.runner = behaviour_runner.BehaviourRunner(
            logger=logger, noun="mapping session")

        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._phase = IDLE
        self._message = ""
        self._error = ""
        self._saved: List[dict] = []
        self._closed = False

    # ── state ────────────────────────────────────────────────────────────

    def active(self) -> bool:
        """Report whether a session owns navigation right now."""
        with self._lock:
            return self._phase in (STOPPING_NAV, STARTING, MAPPING, SAVING, FINISHING)

    def _busy(self) -> bool:
        """Report whether a background step is running. Call under the lock."""
        return self._thread is not None and self._thread.is_alive()

    def _set(self, phase: str, message: str) -> None:
        """Record progress, and say it in the console too."""
        with self._lock:
            self._phase = phase
            self._message = message
        self.runner.note(message)

    def _fail(self, error: str) -> None:
        """Record a step that did not complete."""
        with self._lock:
            self._phase = FAILED
            self._message = "Mapping did not complete"
            self._error = error
        self.runner.note("failed: {}".format(error))
        if self._logger:
            self._logger.error("Mapping failed: {}".format(error))

    def _background(self, target, *args) -> None:
        """Run one step on its own thread. Call under the lock."""
        self._thread = threading.Thread(
            target=target, args=args, daemon=True, name="mecanumbot_web_mapping")
        self._thread.start()

    # ── starting ─────────────────────────────────────────────────────────

    def blockers(self) -> List[str]:
        """Return the reasons a session must not start now, in words."""
        names = {nav_stack.bare(name) for name in self._live_nodes()}
        behaviour = self._behaviour_status() or {}
        reasons = []
        slam = [name for name in SLAM_NODES if name in names]
        if slam:
            reasons.append(
                "SLAM is already running ({}) -- a T1 pass, or mapping started "
                "from a terminal. Stop it first.".format(", ".join(slam)))
        if behaviour.get("running"):
            reasons.append(
                "{} is running from the behaviour page. Stop it first: mapping "
                "takes nav2 away from it.".format(behaviour.get("label") or "A tree"))
        else:
            trees = [nav_stack.bare(name) for name in behaviour.get("graph_nodes") or ()]
            trees += [name for name in self._tree_nodes if name in names
                      and name not in trees]
            if trees:
                reasons.append("A behaviour tree is on the graph ({}). Stop it "
                               "first.".format(", ".join(trees)))
        return reasons

    def start(self) -> dict:
        """Stop nav2 and launch SLAM, in the background. Raises if refused."""
        reasons = self.blockers()
        with self._lock:
            if self._closed:
                raise MappingError("The web node is shutting down.")
            if self._busy() or self._phase in (MAPPING,):
                raise MappingError("A mapping session is already running.")
            if self._nav is not None and self._nav.status().get("busy"):
                raise MappingError("A navigation restart is in progress; wait for it.")
            if reasons:
                raise MappingError(" ".join(reasons))
            self._phase = STOPPING_NAV
            self._message = "Stopping the navigation stack"
            self._error = ""
            self._saved = []
            self._background(self._run_start)
        return self.status()

    def _run_start(self) -> None:
        """Stop nav2, wait for its names to go, launch SLAM."""
        try:
            self.runner.note("--- mapping requested ---")
            if self._nav is not None:
                left = self._nav.stop_stack()
                if left:
                    self._fail(
                        "The navigation stack is still on the ROS graph ({}). "
                        "Nothing on this robot runs it any more, so it is most "
                        "likely on another machine; stop it there.".format(
                            ", ".join(left)))
                    return
            with self._lock:
                if self._closed:
                    return
                self._phase = STARTING
                self._message = "Launching mapping.launch.py"
                self.runner.start_command(
                    "mapping", "Mapping session", self._command, clear_log=False)
            self._set(MAPPING,
                      "Mapping. Drive the robot slowly round the room (joystick or "
                      "the Teleop page), closing loops; save when the map is complete.")
        except Exception as exc:  # the page must learn why, whatever it was
            self._fail(str(exc))

    # ── saving ───────────────────────────────────────────────────────────

    def save(self, name, overwrite: bool = False) -> dict:
        """
        Save the map being built as ``maps/<name>/``. Blocks until written.

        Allowed only while mapping.  Mapping carries on afterwards, so a
        map can be saved, looked at, and saved again under another name.
        """
        name = check_name(name)
        with self._lock:
            if self._phase != MAPPING or not self.runner.status()["running"]:
                raise MappingError("Start mapping first: there is no map to save.")
            self._phase = SAVING
            self._message = "Saving '{}'".format(name)
        self.runner.note("--- saving {} ---".format(name))
        try:
            result = self._save(name, overwrite)
        except Exception as exc:
            with self._lock:
                self._phase = MAPPING
                self._message = "Mapping (the last save failed)"
            self.runner.note("save failed: {}".format(exc))
            raise MappingError(str(exc)) from exc
        with self._lock:
            self._saved.append(result)
            self._phase = MAPPING
            self._message = "Saved '{}'. Still mapping.".format(name)
        self.runner.note("saved {} to {}".format(name, result["folder"]))
        return result

    def _save(self, name: str, overwrite: bool) -> dict:
        """Run the saver into a staging folder, fix the YAML, install it."""
        # Staging beside the maps, so the final move is a rename on one disk.
        root = source_maps_dir(self._maps_dir) or self._maps_dir
        staging = tempfile.mkdtemp(prefix=".saving-", dir=root)
        try:
            command = self._saver + ["-f", os.path.join(staging, name),
                                     "--ros-args"] + list(SAVER_PARAMETERS)
            self.runner.note("$ {}".format(" ".join(command)))
            try:
                completed = subprocess.run(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, text=True, timeout=SAVE_TIMEOUT)
            except subprocess.TimeoutExpired as exc:
                raise MappingError("The map saver did not finish in {:.0f} s.".format(
                    SAVE_TIMEOUT)) from exc
            except OSError as exc:
                raise MappingError("Could not run the map saver: {}".format(exc)) from exc
            for line in (completed.stdout or "").splitlines():
                self.runner.note(line)
            if completed.returncode != 0:
                raise MappingError(
                    "The map saver failed (exit code {}); is slam_toolbox publishing "
                    "/map yet? See the console.".format(completed.returncode))
            yaml_path = os.path.join(staging, "{}.yaml".format(name))
            if not os.path.isfile(yaml_path):
                raise MappingError("The map saver wrote no {}.yaml.".format(name))
            fix_map_yaml(yaml_path, "{}.pgm".format(name))
            return install_map(staging, name, self._maps_dir, overwrite)
        finally:
            shutil.rmtree(staging, ignore_errors=True)

    # ── finishing ────────────────────────────────────────────────────────

    def finish(self, restart_nav: bool = True, load: Optional[str] = None) -> dict:
        """
        Stop SLAM, optionally select a map, and relaunch nav2, in the background.

        ``load`` is a saved map to select first, so nav2 comes up on it.
        """
        if load is not None:
            entry = map_store.describe_map(self._maps_dir, load)
            if not entry["ok"]:
                raise MappingError("Cannot load '{}': {}".format(load, entry["problem"]))
        with self._lock:
            if self._busy():
                raise MappingError("Wait for the current step to finish.")
            if not self.runner.status()["running"] and self._phase not in (FAILED,):
                raise MappingError("No mapping session is running.")
            self._phase = FINISHING
            self._message = "Stopping SLAM"
            self._error = ""
            self._background(self._run_finish, restart_nav, load)
        return self.status()

    def _run_finish(self, restart_nav: bool, load: Optional[str]) -> None:
        """Stop the SLAM launch, wait for its names, hand back to nav2."""
        try:
            if self.runner.status()["running"]:
                self.runner.stop()
            deadline = time.monotonic() + nav_stack.GRAPH_CLEAR_TIMEOUT
            while time.monotonic() < deadline:
                names = {nav_stack.bare(n) for n in self._live_nodes()}
                if "slam_toolbox" not in names:
                    break
                time.sleep(nav_stack.POLL_PERIOD)
            if load is not None:
                map_store.select(self._maps_dir, load, self._selection_file)
                self.runner.note("selected '{}' for nav2".format(load))
            if restart_nav and self._nav is not None:
                with self._lock:
                    self._phase = FINISHED  # so the restart is not refused for us
                self._nav.restart()
                self._set(FINISHED, "Mapping stopped; navigation is restarting{}."
                          .format(" on '{}'".format(load) if load else ""))
            else:
                self._set(FINISHED, "Mapping stopped. Navigation was not restarted.")
        except Exception as exc:
            self._fail(str(exc))

    # ── status and shutdown ──────────────────────────────────────────────

    def status(self) -> dict:
        """Return the session's progress, what it saved, and its process."""
        with self._lock:
            status = {
                "phase": self._phase,
                "busy": self._busy(),
                "active": self._phase in (STOPPING_NAV, STARTING, MAPPING,
                                          SAVING, FINISHING),
                "message": self._message,
                "error": self._error,
                "saved": list(self._saved),
            }
        status["process"] = self.runner.status()
        status["can_save"] = status["phase"] == MAPPING and status["process"]["running"]
        return status

    def shutdown(self) -> None:
        """Stop SLAM, for the node's own shutdown path."""
        with self._lock:
            self._closed = True
        self.runner.shutdown()
