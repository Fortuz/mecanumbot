"""
Restart the study navigation stack from the page.

No ``rclpy`` and no Flask.  The graph reaches this module as a function
returning node names, so the whole sequence runs in a test against real
child processes and a scripted graph.

**A restart is a relaunch, not a lifecycle reset.**  ``manage_nodes``
RESET then STARTUP would recover a stack whose bringup aborted, but not
one whose process is gone -- and the process being gone is the normal
case: a T1 pass's preflight signals the composed ``nav2_container`` away,
because autoslam registers the same node names and a merely finalized
node still holds its name.  So a restart stops whatever study nav2 is
running, waits for it to leave the graph, and launches
``mecanumbot_bringup/nav2.launch.py`` as a child of this node.

**Stopping is by process, matched on the ``__node:=`` remap.**  The base
launch starts nav2, so its processes belong to that ``ros2 launch``, not
to this node.  Every node ``bringup_launch.py`` starts is given a name,
and a named node carries ``__node:=<name>`` on its command line -- the
container as ``__node:=nav2_container``, the keepout servers likewise.
That token is compared whole, never searched for, so a launch file that
merely mentions ``amcl`` is not matched.  Unlike the autoslam preflight,
the process *group* is not excluded: this node was started by the same
``ros2 launch`` as the nav2 it is replacing, so it shares their group, and
excluding the group would exclude exactly the processes to stop.  Only
this process is skipped, and each target is signalled by pid.

**The graph, not the process table, decides when to start.**  A new
``lifecycle_manager_navigation`` configures nodes by name, and if an old
``controller_server`` still answers to that name the bringup aborts and
takes the new servers down with it.  A stack running on another machine
(``launch_external`` can start one) has no local process to signal at
all.  Either way the old names are still on the graph, so the restart
refuses to launch over them and says which they are.

**Some things refuse a restart outright.**  A behaviour tree is sending
nav2 goals, and restarting under it breaks its action clients mid-trial;
a T1 pass runs its own nav2 under the same names, which a restart would
kill.  Both are reasons to stop, stated, rather than things to tidy away.
"""

import os
import signal
import threading
import time
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from . import behaviour_runner

#: How the stack is started. The launch file re-reads the SSID, so a restart
#: after moving rooms picks up the other room's map.
COMMAND = ("ros2", "launch", "mecanumbot_bringup", "nav2.launch.py")

LOCALIZATION = ("lifecycle_manager_localization", "map_server", "amcl")
NAVIGATION = (
    "lifecycle_manager_navigation", "controller_server", "smoother_server",
    "planner_server", "behavior_server", "bt_navigator", "waypoint_follower",
    "velocity_smoother",
)
#: Only started on the SSID whose map has a keepout mask.
KEEPOUT = (
    "lifecycle_manager_keepout_zone", "keepout_filter_mask_server",
    "keepout_costmap_filter_info_server",
)
#: The composed form: with ``use_composition`` (the bringup default) every
#: LOCALIZATION and NAVIGATION node lives inside this one process.
CONTAINER = ("nav2_container",)

#: Every node name the study stack registers.
STACK_NODES = LOCALIZATION + NAVIGATION + KEEPOUT + CONTAINER

#: What must be on the graph for the stack to count as up.
REQUIRED_NODES = LOCALIZATION + NAVIGATION

#: The managers asked whether their nodes are active.
MANAGERS = ("lifecycle_manager_localization", "lifecycle_manager_navigation")

#: A T1 pass owns navigation while any of these is up.
T1_NODES = ("slam_toolbox", "autoslam_node")

#: Grace periods for the processes being replaced. nav2 takes longer than a
#: behaviour tree to deactivate cleanly, so SIGINT gets longer here.
SIGINT_GRACE = 10.0
SIGTERM_GRACE = 3.0

#: How long the old names get to leave the graph after their processes exit.
GRAPH_CLEAR_TIMEOUT = 15.0

POLL_PERIOD = 0.2

IDLE = "idle"
STOPPING = "stopping"
WAITING = "waiting"
STARTING = "starting"
STARTED = "started"
FAILED = "failed"


def bare(node_name: str) -> str:
    """Return a node name without its namespace."""
    return str(node_name).rsplit("/", 1)[-1]


def describe(live_nodes: Iterable[str],
             active: Optional[Dict[str, Optional[bool]]] = None) -> dict:
    """
    Summarise the stack from the graph and the managers' own answers.

    ``active`` maps a manager name to its ``is_active`` answer, or None
    when it did not answer.  The state is one word, in order of what it
    takes to be true:

    ``down``      none of the stack's nodes is on the graph
    ``t1``        a T1 pass is up, so whatever nav2 there is is its own
    ``partial``   some required node is missing
    ``inactive``  every node is there, but a manager says they are not active
                  -- the shape of a bringup that aborted
    ``unknown``   every node is there, but a manager did not answer
    ``active``    every node is there and both managers say so
    """
    names = {bare(name) for name in live_nodes}
    active = dict(active or {})
    present = [name for name in STACK_NODES if name in names]
    missing = [name for name in REQUIRED_NODES if name not in names]
    t1 = [name for name in T1_NODES if name in names]
    managers = {name: active.get(name) for name in MANAGERS if name in names}

    if t1:
        state = "t1"
    elif not present:
        state = "down"
    elif missing:
        state = "partial"
    elif any(answer is False for answer in managers.values()):
        state = "inactive"
    elif any(answer is None for answer in managers.values()):
        state = "unknown"
    else:
        state = "active"

    return {
        "state": state,
        "present": present,
        "missing": missing,
        "managers": managers,
        "t1_nodes": t1,
        "keepout": any(name in names for name in KEEPOUT),
    }


def blockers(live_nodes: Iterable[str], behaviour: Optional[dict] = None,
             tree_nodes: Sequence[str] = ()) -> List[str]:
    """
    Return the reasons a restart must not happen now, in words.

    ``behaviour`` is the behaviour page's run status: whether it is running
    a tree, and ``graph_nodes``, any tree somebody started elsewhere.
    """
    names = {bare(name) for name in live_nodes}
    behaviour = behaviour or {}
    reasons = []

    t1 = [name for name in T1_NODES if name in names]
    if t1:
        reasons.append(
            "A T1 exploration pass is running ({}). It runs its own nav2 under "
            "the same node names, so a restart would stop that instead. Stop "
            "the pass first.".format(", ".join(t1)))

    if behaviour.get("running"):
        reasons.append(
            "{} is running from the behaviour page and is sending nav2 goals; "
            "restarting nav2 under it would break the trial. Stop it first."
            .format(behaviour.get("label") or "A behaviour tree"))
    else:
        trees = [bare(name) for name in behaviour.get("graph_nodes") or ()]
        trees += [name for name in tree_nodes if name in names
                  and name not in trees]
        if trees:
            reasons.append(
                "A behaviour tree started outside this page is on the graph "
                "({}). Stop it before restarting nav2.".format(", ".join(trees)))
    return reasons


# ── processes ────────────────────────────────────────────────────────────────

def local_processes() -> List[Tuple[int, List[str]]]:
    """Return ``(pid, cmdline)`` for every readable process on this machine."""
    processes = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/{}/cmdline".format(entry), "rb") as handle:
                parts = handle.read().decode(errors="replace").split("\0")
        except OSError:
            continue
        parts = [part for part in parts if part]
        if parts:
            processes.append((int(entry), parts))
    return processes


def matching_processes(processes: Iterable[Tuple[int, List[str]]],
                       node_names: Sequence[str] = STACK_NODES,
                       skip_pids: Iterable[int] = ()) -> List[Tuple[int, str]]:
    """
    Return ``(pid, node name)`` for each process that runs a stack node.

    Matched on a whole ``__node:=<name>`` argument and nothing looser.
    """
    tokens = {"__node:={}".format(name): name for name in node_names}
    skip = set(skip_pids)
    hits = []
    for pid, cmdline in processes:
        if pid in skip:
            continue
        for argument in cmdline:
            if argument in tokens:
                hits.append((pid, tokens[argument]))
                break
    return hits


def alive(pid: int) -> bool:
    """
    Return whether a process is still running.

    A zombie is not: it has exited and only waits for its parent to reap
    it, which ``ros2 launch`` does in its own time.
    """
    try:
        with open("/proc/{}/stat".format(pid), "r") as handle:
            stat = handle.read()
    except OSError:
        return False
    # The command name is parenthesised and may contain spaces; the state
    # is the first field after the closing parenthesis.
    return stat[stat.rfind(")") + 2:stat.rfind(")") + 3] not in ("Z", "X", "")


class NavStackRestarter:
    """Stops the study nav2 stack wherever it came from, and relaunches it."""

    def __init__(self, live_nodes: Callable[[], Iterable[str]],
                 behaviour_status: Optional[Callable[[], dict]] = None,
                 tree_nodes: Sequence[str] = (),
                 command: Sequence[str] = COMMAND,
                 list_processes: Callable[[], Iterable] = local_processes,
                 logger=None,
                 sigint_grace: float = SIGINT_GRACE,
                 sigterm_grace: float = SIGTERM_GRACE,
                 graph_timeout: float = GRAPH_CLEAR_TIMEOUT):
        """Build a restarter reading the graph through ``live_nodes``."""
        self._live_nodes = live_nodes
        self._behaviour_status = behaviour_status or (lambda: {})
        self._tree_nodes = tuple(tree_nodes)
        self._command = list(command)
        self._list_processes = list_processes
        self._logger = logger
        self._sigint_grace = float(sigint_grace)
        self._sigterm_grace = float(sigterm_grace)
        self._graph_timeout = float(graph_timeout)

        #: The launched stack. Its console is what the page shows.
        self.runner = behaviour_runner.BehaviourRunner(
            logger=logger, noun="navigation stack")

        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._phase = IDLE
        self._message = ""
        self._error = ""
        self._requested_at: Optional[float] = None
        self._closed = False

    # ── restarting ───────────────────────────────────────────────────────

    def restart(self) -> dict:
        """
        Begin a restart in the background and return at once.

        Raises :class:`~.behaviour_runner.RunnerError` when a restart is
        already under way or something on the graph forbids one.
        """
        live = list(self._live_nodes())
        reasons = blockers(live, self._behaviour_status(), self._tree_nodes)
        with self._lock:
            if self._closed:
                raise behaviour_runner.RunnerError("The web node is shutting down.")
            if self._thread is not None and self._thread.is_alive():
                raise behaviour_runner.RunnerError(
                    "A navigation restart is already in progress.")
            if reasons:
                raise behaviour_runner.RunnerError(" ".join(reasons))
            self._phase = STOPPING
            self._message = "Stopping the running navigation stack"
            self._error = ""
            self._requested_at = time.time()
            self._thread = threading.Thread(
                target=self._run, daemon=True, name="mecanumbot_web_nav_restart")
            self._thread.start()
        return self.status()

    def _run(self) -> None:
        """Stop, wait for the graph, start. Runs on the restart thread."""
        try:
            self.runner.note("--- navigation restart requested ---")
            self._stop_everything()

            self._set(WAITING, "Waiting for the old nodes to leave the ROS graph")
            left = self._wait_for_graph()
            if left:
                self._fail(
                    "Still on the ROS graph after {:.0f} s: {}. Nothing on this "
                    "robot runs them any more, so they are most likely on another "
                    "machine (launch_external can start nav2). Stop them there; "
                    "starting a second stack under the same names would abort its "
                    "bringup.".format(self._graph_timeout, ", ".join(left)))
                return

            # Under the lock that shutdown() takes, so a node shutting down
            # either sees this child and stops it, or prevents it.
            with self._lock:
                if self._closed:
                    return
                self._phase = STARTING
                self._message = "Launching nav2.launch.py"
                self.runner.start_command(
                    "nav2", "Navigation stack", self._command, clear_log=False)
            self._set(STARTED,
                      "Launched. The lifecycle managers bring the servers up over "
                      "the next few seconds; AMCL starts from its initial pose, so "
                      "set the pose estimate again.")
        except Exception as exc:  # the page must learn why, whatever it was
            self._fail(str(exc))

    def _stop_everything(self) -> None:
        """Stop this node's own launch of the stack, then anyone else's."""
        if self.runner.status()["running"]:
            self.runner.note("stopping the navigation stack this page started")
            self.runner.stop(self._sigint_grace, self._sigterm_grace)

        hits = matching_processes(self._list_processes(),
                                  skip_pids=(os.getpid(),))
        if not hits:
            self.runner.note("no other local navigation processes to stop")
            return
        self.runner.note("stopping {}".format(", ".join(
            "{} (pid {})".format(name, pid) for pid, name in hits)))
        survivors = self._signal_until_gone([pid for pid, _name in hits])
        if survivors:
            raise behaviour_runner.RunnerError(
                "Could not stop pids {} even with SIGKILL.".format(
                    ", ".join(str(pid) for pid in survivors)))

    def _signal_until_gone(self, pids: List[int]) -> List[int]:
        """SIGINT, then SIGTERM, then SIGKILL; return whatever survived."""
        for sig, grace in ((signal.SIGINT, self._sigint_grace),
                           (signal.SIGTERM, self._sigterm_grace),
                           (signal.SIGKILL, self._sigterm_grace)):
            remaining = [pid for pid in pids if alive(pid)]
            if not remaining:
                return []
            if sig != signal.SIGINT:
                self.runner.note("{} did not exit; sending {}".format(
                    ", ".join(str(pid) for pid in remaining), sig.name))
            for pid in remaining:
                try:
                    os.kill(pid, sig)
                except (OSError, ProcessLookupError):
                    pass
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline and any(alive(p) for p in remaining):
                time.sleep(POLL_PERIOD)
        return [pid for pid in pids if alive(pid)]

    def _wait_for_graph(self) -> List[str]:
        """Wait for every stack name to leave the graph; return any that stay."""
        deadline = time.monotonic() + self._graph_timeout
        while True:
            names = {bare(name) for name in self._live_nodes()}
            left = [name for name in STACK_NODES if name in names]
            if not left or time.monotonic() >= deadline:
                return left
            time.sleep(POLL_PERIOD)

    def _set(self, phase: str, message: str) -> None:
        """Record progress, and say it in the console too."""
        with self._lock:
            self._phase = phase
            self._message = message
        self.runner.note(message)

    def _fail(self, error: str) -> None:
        """Record a restart that did not complete."""
        with self._lock:
            self._phase = FAILED
            self._message = "The restart did not complete"
            self._error = error
        self.runner.note("restart failed: {}".format(error))
        if self._logger:
            self._logger.error("Navigation restart failed: {}".format(error))

    # ── status and shutdown ──────────────────────────────────────────────

    def status(self) -> dict:
        """Return the restart's progress and the launched stack's status."""
        with self._lock:
            busy = self._thread is not None and self._thread.is_alive()
            status = {
                "phase": self._phase,
                "busy": busy,
                "message": self._message,
                "error": self._error,
                "requested_at": self._requested_at,
            }
        status["process"] = self.runner.status()
        return status

    def wait(self, timeout: Optional[float] = None) -> bool:
        """Wait for a restart in progress; return whether it finished."""
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout)
        return not thread.is_alive()

    def shutdown(self) -> None:
        """
        Stop the stack this node launched, for the node's own shutdown path.

        The child has its own session, so a Ctrl-C of the base launch does
        not reach it; without this it would outlive the drivers it navigates.
        A stack the base launch started is that launch's to stop, and is
        left alone.
        """
        with self._lock:
            self._closed = True
        self.wait(1.0)
        self.runner.shutdown()
