"""
Tests for restarting the study navigation stack.

The restart is exercised against real child processes standing in for nav2
servers, and a graph scripted from whether those processes are alive. The
process list handed to the restarter is **only this test's own children**:
the module signals whatever matches, and a test run on a machine with a real
nav2 up must not be the thing that stops it.
"""

import os
import subprocess
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import behaviour_runner, nav_stack  # noqa: E402

#: A stand-in for a nav2 server: runs until signalled.
SERVER = "import time; time.sleep(60)"

#: A server that ignores SIGINT and SIGTERM, the way a wedged one would.
WEDGED = ("import signal, time\n"
          "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
          "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
          "time.sleep(60)\n")

#: What the restarter launches in place of ``ros2 launch``.
LAUNCH = [sys.executable, "-u", "-c",
          "import time; print('nav2 up', flush=True); time.sleep(60)"]

EVERY_REQUIRED = ["/" + name for name in nav_stack.REQUIRED_NODES]


def wait_for(predicate, timeout=10.0):
    """Wait for a condition, returning whether it came true in time."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


class FakeServers:
    """Child processes carrying a ``__node:=`` remap, like launched nav2 nodes."""

    def __init__(self):
        """Start with none."""
        self.children = []

    def start(self, name, script=SERVER):
        """Start one stand-in server registered under ``name``."""
        argv = [sys.executable, "-c", script, "--ros-args", "-r",
                "__node:={}".format(name)]
        child = subprocess.Popen(argv)
        self.children.append((child, name, argv))
        return child

    def processes(self):
        """Return the process list the restarter may act on: these, and only these."""
        return [(child.pid, argv) for child, _name, argv in self.children
                if nav_stack.alive(child.pid)]

    def graph(self):
        """Return a graph holding every server still running."""
        return ["/" + name for child, name, _argv in self.children
                if nav_stack.alive(child.pid)]

    def close(self):
        """Kill and reap everything, whatever a test left behind."""
        for child, _name, _argv in self.children:
            if child.poll() is None:
                child.kill()
            child.wait()


@pytest.fixture
def servers():
    """Return a set of stand-in servers that is always cleaned up."""
    made = FakeServers()
    yield made
    made.close()


def restarter_for(servers, **kwargs):
    """Build a restarter that sees only ``servers``, with short grace periods."""
    options = dict(live_nodes=servers.graph, list_processes=servers.processes,
                   command=LAUNCH, sigint_grace=2.0, sigterm_grace=1.0,
                   graph_timeout=2.0)
    options.update(kwargs)
    return nav_stack.NavStackRestarter(**options)


# ── describing the stack ────────────────────────────────────────────────────

def test_nothing_on_the_graph_is_down():
    assert nav_stack.describe(["/mecanumbot/mecanumbot_io_node"])["state"] == "down"


def test_every_node_with_active_managers_is_active():
    active = {name: True for name in nav_stack.MANAGERS}
    state = nav_stack.describe(EVERY_REQUIRED, active)
    assert state["state"] == "active"
    assert state["missing"] == []


def test_a_bringup_that_aborted_reads_inactive_not_active():
    """Every node registered, nothing serving: presence alone would say fine."""
    active = {"lifecycle_manager_localization": True,
              "lifecycle_manager_navigation": False}
    assert nav_stack.describe(EVERY_REQUIRED, active)["state"] == "inactive"


def test_a_manager_that_does_not_answer_is_unknown():
    active = {"lifecycle_manager_localization": True,
              "lifecycle_manager_navigation": None}
    assert nav_stack.describe(EVERY_REQUIRED, active)["state"] == "unknown"


def test_a_stack_missing_amcl_is_partial_and_says_so():
    live = [name for name in EVERY_REQUIRED if name != "/amcl"]
    state = nav_stack.describe(live, {name: True for name in nav_stack.MANAGERS})
    assert state["state"] == "partial"
    assert state["missing"] == ["amcl"]


def test_a_t1_pass_is_reported_as_such_rather_than_partial():
    """Autoslam's nav2 has no AMCL, and is not a broken study stack."""
    live = ["/slam_toolbox", "/controller_server", "/bt_navigator"]
    assert nav_stack.describe(live)["state"] == "t1"


def test_names_match_in_any_namespace():
    live = ["/mecanumbot" + name for name in EVERY_REQUIRED]
    active = {name: True for name in nav_stack.MANAGERS}
    assert nav_stack.describe(live, active)["state"] == "active"


# ── what refuses a restart ──────────────────────────────────────────────────

def test_nothing_running_blocks_nothing():
    assert nav_stack.blockers(EVERY_REQUIRED, {"running": False}) == []


def test_a_t1_pass_blocks_a_restart():
    reasons = nav_stack.blockers(["/autoslam_node"], {})
    assert len(reasons) == 1 and "T1" in reasons[0]


def test_a_tree_from_the_behaviour_page_blocks_a_restart():
    reasons = nav_stack.blockers([], {"running": True, "label": "Leading"})
    assert len(reasons) == 1 and reasons[0].startswith("Leading is running")


def test_a_tree_started_elsewhere_blocks_a_restart():
    reasons = nav_stack.blockers(
        ["/mecanumbot/seek_bt_node"], {"running": False},
        tree_nodes=("seek_bt_node",))
    assert len(reasons) == 1 and "seek_bt_node" in reasons[0]


# ── matching processes ──────────────────────────────────────────────────────

def test_a_process_is_matched_on_its_whole_node_remap():
    processes = [
        (10, ["/opt/ros/humble/lib/nav2_amcl/amcl", "--ros-args",
              "-r", "__node:=amcl"]),
        (11, ["component_container_isolated", "--ros-args",
              "-r", "__node:=nav2_container"]),
    ]
    assert nav_stack.matching_processes(processes) == [
        (10, "amcl"), (11, "nav2_container")]


def test_a_launch_that_only_mentions_a_node_is_not_matched():
    """Killing the base launch because its arguments name amcl would stop the drivers."""
    processes = [
        (20, ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
              "mecanumbot_bringup", "launch_mecanumbot_base.launch.py"]),
        (21, ["python3", "-c", "print('__node:=amcl is a remap')"]),
        (22, ["node", "--ros-args", "-r", "__node:=amcl_helper"]),
        (23, ["node", "--ros-args", "-r", "__node:=mecanumbot_web_node"]),
    ]
    assert nav_stack.matching_processes(processes) == []


def test_this_process_is_never_matched():
    processes = [(os.getpid(), ["x", "-r", "__node:=amcl"])]
    assert nav_stack.matching_processes(processes, skip_pids=(os.getpid(),)) == []


def test_a_zombie_counts_as_gone():
    """ros2 launch reaps its children in its own time; exited is what matters."""
    child = subprocess.Popen([sys.executable, "-c", "pass"])
    try:
        # Not reaped until child.wait() below, so it is a zombie meanwhile.
        assert wait_for(lambda: not nav_stack.alive(child.pid))
        assert os.path.exists("/proc/{}".format(child.pid))
    finally:
        child.wait()


# ── restarting ──────────────────────────────────────────────────────────────

def test_a_restart_stops_the_old_stack_and_launches_the_new_one(servers):
    old_amcl = servers.start("amcl")
    old_container = servers.start("nav2_container")
    assert wait_for(lambda: len(servers.graph()) == 2)
    restarter = restarter_for(servers)
    try:
        assert restarter.restart()["busy"]
        assert restarter.wait(15.0)

        status = restarter.status()
        assert status["phase"] == nav_stack.STARTED, status["error"]
        assert status["process"]["running"]
        assert not nav_stack.alive(old_amcl.pid)
        assert not nav_stack.alive(old_container.pid)
        assert wait_for(lambda: any(
            "nav2 up" in line for line in restarter.runner.log()["lines"]))
    finally:
        restarter.shutdown()


def test_the_console_says_what_was_stopped_before_what_started(servers):
    servers.start("amcl")
    restarter = restarter_for(servers)
    try:
        restarter.restart()
        assert restarter.wait(15.0)
        lines = restarter.runner.log()["lines"]
        stopped = next(i for i, line in enumerate(lines) if "amcl (pid" in line)
        launched = next(i for i, line in enumerate(lines) if line.startswith("$ "))
        assert stopped < launched
    finally:
        restarter.shutdown()


def test_a_wedged_server_is_escalated_to_sigkill(servers):
    wedged = servers.start("controller_server", WEDGED)
    time.sleep(0.5)       # let it install its handlers before it is signalled
    restarter = restarter_for(servers, sigint_grace=0.5, sigterm_grace=0.5)
    try:
        restarter.restart()
        assert restarter.wait(15.0)
        assert restarter.status()["phase"] == nav_stack.STARTED
        assert not nav_stack.alive(wedged.pid)
        assert any("SIGKILL" in line for line in restarter.runner.log()["lines"])
    finally:
        restarter.shutdown()


def test_names_that_stay_on_the_graph_prevent_the_launch(servers):
    """A stack on another machine has no local process, and must not be launched over."""
    restarter = restarter_for(servers, live_nodes=lambda: ["/bt_navigator"],
                              graph_timeout=0.5)
    restarter.restart()
    assert restarter.wait(15.0)
    status = restarter.status()
    assert status["phase"] == nav_stack.FAILED
    assert "bt_navigator" in status["error"]
    assert not status["process"]["running"]


def test_a_blocked_restart_raises_and_touches_nothing(servers):
    amcl = servers.start("amcl")
    restarter = restarter_for(
        servers, behaviour_status=lambda: {"running": True, "label": "Fetch"})
    with pytest.raises(behaviour_runner.RunnerError, match="Fetch is running"):
        restarter.restart()
    time.sleep(0.3)
    assert nav_stack.alive(amcl.pid)
    assert restarter.status()["phase"] == nav_stack.IDLE


def test_a_second_restart_during_the_first_is_refused(servers):
    servers.start("amcl", WEDGED)
    time.sleep(0.5)
    restarter = restarter_for(servers, sigint_grace=1.5, sigterm_grace=0.5)
    try:
        restarter.restart()
        with pytest.raises(behaviour_runner.RunnerError, match="in progress"):
            restarter.restart()
        assert restarter.wait(15.0)
    finally:
        restarter.shutdown()


def test_restarting_a_stack_this_page_launched_replaces_it(servers):
    restarter = restarter_for(servers)
    try:
        restarter.restart()
        assert restarter.wait(15.0)
        first = restarter.status()["process"]["pid"]

        restarter.restart()
        assert restarter.wait(15.0)
        second = restarter.status()["process"]
        assert second["running"] and second["pid"] != first
        assert not nav_stack.alive(first)
    finally:
        restarter.shutdown()


def test_shutdown_stops_the_stack_this_page_launched(servers):
    restarter = restarter_for(servers)
    restarter.restart()
    assert restarter.wait(15.0)
    pid = restarter.status()["process"]["pid"]
    restarter.shutdown()
    assert wait_for(lambda: not nav_stack.alive(pid))


def test_no_restart_starts_after_shutdown(servers):
    restarter = restarter_for(servers)
    restarter.shutdown()
    with pytest.raises(behaviour_runner.RunnerError, match="shutting down"):
        restarter.restart()
