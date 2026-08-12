"""
Tests for the behaviour process supervisor.

Real child processes, not mocks: what this module is for is starting one
and being sure it is gone afterwards, and a mocked ``Popen`` would prove
nothing about either.  The children are short Python scripts rather than
``ros2 launch``, injected by standing a fake spec in for the catalog's.
"""

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import behaviour_runner  # noqa: E402

#: Prints, then waits to be stopped.
CHATTY = "import time; print('tree started', flush=True); time.sleep(30)"

#: Ignores SIGINT the way a wedged tree would, so the escalation to
#: SIGTERM and SIGKILL is exercised rather than assumed.
STUBBORN = ("import signal, time\n"
            "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
            "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            "print('wedged', flush=True)\n"
            "time.sleep(60)\n")

#: Exits immediately with a failure, the way a launch missing a package does.
FAILS = "import sys; print('no such package', flush=True); sys.exit(2)"


class FakeSpec:
    """A catalog entry whose command is a local script."""

    def __init__(self, script, key="fake", label="Fake behaviour"):
        """Describe a run of one short Python program."""
        self.key = key
        self.label = label
        self.script = script

    def build_command(self, values, config_dir):
        """Return the argv for the script, ignoring the arguments."""
        del values, config_dir
        return [sys.executable, "-u", "-c", self.script]

    def defaults(self, ssid=None):
        """Return no arguments; this spec takes none."""
        del ssid
        return {}


@pytest.fixture
def runner(monkeypatch):
    """Return a runner whose catalog lookups run a harmless local script."""
    made = behaviour_runner.BehaviourRunner(config_dirs={"fake": ""})
    monkeypatch.setattr(behaviour_runner.behaviour_catalog, "config_dir_for",
                        lambda key, dirs: "")
    yield made
    made.shutdown()


def use(runner, monkeypatch, script):
    """Point the runner's catalog at one script."""
    monkeypatch.setattr(behaviour_runner.behaviour_catalog, "get",
                        lambda key: FakeSpec(script))


def wait_for(predicate, timeout=10.0):
    """Wait for a condition, returning whether it came true in time."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


# ── starting and stopping ────────────────────────────────────────────────

def test_start_reports_a_running_process(runner, monkeypatch):
    """A started behaviour is running, with a pid to prove it."""
    use(runner, monkeypatch, CHATTY)
    status = runner.start("fake", {})
    assert status["running"] is True
    assert status["pid"] > 0
    assert status["behaviour"] == "fake"


def test_the_child_output_reaches_the_log(runner, monkeypatch):
    """The console is what says whether a tree found its prerequisites."""
    use(runner, monkeypatch, CHATTY)
    runner.start("fake", {})
    assert wait_for(lambda: any("tree started" in line
                                for line in runner.log()["lines"]))


def test_the_command_is_the_first_log_line(runner, monkeypatch):
    """What was run has to be visible, not just what it printed."""
    use(runner, monkeypatch, CHATTY)
    runner.start("fake", {})
    assert runner.log()["lines"][0].startswith("$ ")


def test_stop_ends_the_process(runner, monkeypatch):
    """After stop, nothing is running and the process is really gone."""
    use(runner, monkeypatch, CHATTY)
    pid = runner.start("fake", {})["pid"]
    status = runner.stop()

    assert status["running"] is False
    assert not wait_for(lambda: _alive(pid), timeout=0.5)


def test_a_wedged_tree_is_escalated_to_a_kill(runner, monkeypatch):
    """
    A tree that ignores SIGINT and SIGTERM still has to let go.

    It is holding /cmd_vel and /goal_pose while it does, so being polite
    indefinitely is worse than killing it.
    """
    use(runner, monkeypatch, STUBBORN)
    pid = runner.start("fake", {})["pid"]
    status = runner.stop(sigint_grace=0.4, sigterm_grace=0.4)

    assert status["running"] is False
    assert not _alive(pid)


def test_starting_twice_is_refused(runner, monkeypatch):
    """
    Two trees would drive the robot at once, so the second is an error.

    Not a queue: the operator asked for a run *now*, and starting it after
    the current trial ends would be a surprise, not a convenience.
    """
    use(runner, monkeypatch, CHATTY)
    runner.start("fake", {})
    with pytest.raises(behaviour_runner.RunnerError):
        runner.start("fake", {})


def test_stopping_nothing_is_an_error(runner):
    """Nothing is running, so there is nothing to report as stopped."""
    with pytest.raises(behaviour_runner.RunnerError):
        runner.stop()


def test_an_unknown_behaviour_is_refused(runner):
    """The key is checked against the real catalog before anything spawns."""
    with pytest.raises(behaviour_runner.RunnerError):
        runner.start("not_a_behaviour", {})


def test_a_command_that_cannot_run_is_reported(runner, monkeypatch):
    """A missing binary is a message, not a traceback out of a Flask route."""
    monkeypatch.setattr(
        behaviour_runner.behaviour_catalog, "get",
        lambda key: FakeSpec("", key="fake"))
    monkeypatch.setattr(FakeSpec, "build_command",
                        lambda self, values, config_dir: ["/nonexistent/ros2"])
    with pytest.raises(behaviour_runner.RunnerError):
        runner.start("fake", {})


# ── what happened afterwards ─────────────────────────────────────────────

def test_a_failed_run_keeps_its_exit_code(runner, monkeypatch):
    """
    A tree that exits on its own is not running, and says why it went.

    This is the common case in practice: nav2 is not up, the tree's setup
    times out, and the process is gone before the page next polls.
    """
    use(runner, monkeypatch, FAILS)
    runner.start("fake", {})
    assert wait_for(lambda: runner.status()["running"] is False)

    status = runner.status()
    assert status["returncode"] == 2
    assert "code 2" in status["exit_note"]
    assert any("no such package" in line for line in runner.log()["lines"])


def test_the_log_is_incremental(runner, monkeypatch):
    """The page polls for what it has not seen, not for the whole log."""
    use(runner, monkeypatch, CHATTY)
    runner.start("fake", {})
    assert wait_for(lambda: len(runner.log()["lines"]) >= 2)

    first = runner.log()
    assert runner.log(first["last_seq"])["lines"] == []


def test_dropped_lines_are_counted(runner, monkeypatch):
    """A gap in the log is reported rather than silently closed."""
    small = behaviour_runner.BehaviourRunner(config_dirs={}, log_lines=3)
    for index in range(10):
        small._record("line {}".format(index))

    result = small.log(after=2)
    assert result["dropped"] == 5
    assert result["lines"] == ["line 7", "line 8", "line 9"]


def test_shutdown_is_safe_when_nothing_runs(runner):
    """The node's shutdown path must never raise on the way out."""
    runner.shutdown()


def test_shutdown_stops_a_running_behaviour(runner, monkeypatch):
    """
    A tree started here must not outlive the stack it was driving.

    The child gets its own session, so nothing else will stop it: the web
    node's own shutdown is the only thing between a Ctrl-C and a tree
    still publishing goals at an empty graph.
    """
    use(runner, monkeypatch, CHATTY)
    pid = runner.start("fake", {})["pid"]
    runner.shutdown()
    assert not _alive(pid)


def _alive(pid):
    """Report whether a process still exists."""
    try:
        os.kill(pid, 0)
    except (OSError, ProcessLookupError):
        return False
    return True
