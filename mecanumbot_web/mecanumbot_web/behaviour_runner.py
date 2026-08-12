"""
Start and stop one behaviour tree, and keep its console output.

No ``rclpy`` and no Flask: this is a small process supervisor, and the
node only owns it so that it dies with the robot stack.

**One run at a time, deliberately.**  Every tree in the catalog publishes
``/goal_pose`` and ``/cmd_vel``, so a second one started while the first
is up does not run alongside it -- the two fight over the robot, and an
experiment trial is quietly invalid rather than obviously broken.  Asking
to start while something is running is therefore an error, not a queue.

**Stopping means SIGINT to the process group.**  ``ros2 launch`` responds
to SIGINT by shutting its nodes down properly; killing it outright leaves
the tree's own process orphaned, still holding the ROS graph and still
driving.  The child gets its own session (``start_new_session=True``) so
that one signal reaches the whole tree of processes, and so that a Ctrl-C
aimed at the web node does not race this shutdown.  SIGTERM and then
SIGKILL follow only if it does not go.

**The output is a ring buffer, not a file.**  Behaviour trees are chatty
and this runs for the length of a trial, so the last few hundred lines
are kept and the rest is dropped.  Lines are numbered so the page can
poll for what it has not seen, and the numbering makes a gap visible
rather than silently closing it.
"""

import os
import signal
import subprocess
import threading
import time
from collections import deque
from typing import Dict, List, Optional

from . import behaviour_catalog

#: Console lines kept for the page. A trial's worth of py_trees logging
#: is far more than this; the tail is what a live status view needs.
LOG_LINES = 400

#: How long a stopped run gets to exit on SIGINT before SIGTERM, and then
#: on SIGTERM before SIGKILL.  ``ros2 launch`` shuts down well within the
#: first of these when its nodes are healthy.
SIGINT_GRACE = 8.0
SIGTERM_GRACE = 3.0


class RunnerError(Exception):
    """Raised when a run cannot be started or stopped as asked."""


class BehaviourRunner:
    """Supervises at most one behaviour tree process."""

    def __init__(self, config_dirs: Optional[Dict[str, str]] = None,
                 log_lines: int = LOG_LINES, logger=None):
        """Build a runner over the config directories the behaviours use."""
        self._config_dirs = dict(config_dirs or {})
        self._logger = logger
        # Reentrant: ``start`` records the command line into the log
        # while it still holds the lock over the process handle.
        self._lock = threading.RLock()

        self._process: Optional[subprocess.Popen] = None
        self._reader: Optional[threading.Thread] = None
        self._lines = deque(maxlen=int(log_lines))
        self._next_seq = 1

        self._key: Optional[str] = None
        self._label: str = ""
        self._values: Dict[str, str] = {}
        self._command: List[str] = []
        self._started_at: Optional[float] = None
        self._started_monotonic: Optional[float] = None
        self._finished_at: Optional[float] = None
        self._returncode: Optional[int] = None
        self._stopping = False

    # ── starting ─────────────────────────────────────────────────────────

    def config_dir(self, key: str) -> str:
        """Return the config directory of one behaviour."""
        return behaviour_catalog.config_dir_for(key, self._config_dirs)

    def start(self, key: str, values: Optional[Dict] = None) -> dict:
        """
        Start one behaviour, validating every argument first.

        Returns the status of the new run.  Raises :class:`RunnerError`
        if something is already running, or the arguments do not describe
        a run this catalog can express.
        """
        try:
            spec = behaviour_catalog.get(key)
            command = spec.build_command(values or {}, self.config_dir(key))
        except behaviour_catalog.CatalogError as exc:
            raise RunnerError(str(exc)) from exc

        with self._lock:
            self._reap()
            if self._process is not None:
                raise RunnerError(
                    "{} is already running. Stop it before starting another "
                    "behaviour -- two trees would drive the robot at once."
                    .format(self._label or self._key))

            environment = dict(os.environ)
            # Child stdout is a pipe, not a terminal, so Python would
            # block-buffer it and the page would show nothing until the
            # run ended.
            environment["PYTHONUNBUFFERED"] = "1"

            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    env=environment,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
            except OSError as exc:
                raise RunnerError("Could not start {}: {}".format(
                    command[0], exc)) from exc

            self._process = process
            self._key = spec.key
            self._label = spec.label
            self._values = dict(spec.defaults(), **(values or {}))
            self._command = list(command)
            self._started_at = time.time()
            self._started_monotonic = time.monotonic()
            self._finished_at = None
            self._returncode = None
            self._stopping = False

            self._lines.clear()
            self._record("$ {}".format(" ".join(command)))

            self._reader = threading.Thread(
                target=self._read_output, args=(process,),
                daemon=True, name="mecanumbot_web_behaviour_log")
            self._reader.start()

            if self._logger:
                self._logger.info("Started {}: {}".format(
                    spec.label, " ".join(command)))
            return self._status_locked()

    # ── stopping ─────────────────────────────────────────────────────────

    def stop(self, sigint_grace: float = SIGINT_GRACE,
             sigterm_grace: float = SIGTERM_GRACE) -> dict:
        """
        Stop the running behaviour, escalating only if it will not go.

        Returns the status once the process is gone.  Raises
        :class:`RunnerError` if nothing is running.
        """
        with self._lock:
            self._reap()
            process = self._process
            if process is None:
                raise RunnerError("No behaviour is running.")
            self._stopping = True
            label = self._label

        self._record("--- stopping {} ---".format(label))
        for sig, grace in ((signal.SIGINT, sigint_grace),
                           (signal.SIGTERM, sigterm_grace)):
            if self._signal(process, sig) and self._wait(process, grace):
                break
        else:
            # Nothing left to be polite about: SIGINT and SIGTERM both
            # went unanswered, so the tree is wedged and still owns the
            # robot's command topics.
            self._signal(process, signal.SIGKILL)
            self._wait(process, sigterm_grace)

        with self._lock:
            self._reap()
            self._stopping = False
            return self._status_locked()

    def shutdown(self) -> None:
        """Stop any running behaviour, for the node's own shutdown path."""
        try:
            self.stop()
        except RunnerError:
            pass
        except Exception as exc:  # pragma: no cover - shutdown must not raise
            if self._logger:
                self._logger.warn("Behaviour shutdown failed: {}".format(exc))

    def _signal(self, process, sig) -> bool:
        """Send one signal to the child's whole process group."""
        try:
            os.killpg(os.getpgid(process.pid), sig)
            return True
        except (OSError, ProcessLookupError):
            # Already gone, or never had a group of its own.
            try:
                process.send_signal(sig)
                return True
            except (OSError, ProcessLookupError):
                return False

    @staticmethod
    def _wait(process, timeout: float) -> bool:
        """Wait for the child, reporting whether it exited in time."""
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False

    # ── output ───────────────────────────────────────────────────────────

    def _read_output(self, process) -> None:
        """Drain the child's console into the ring buffer until it ends."""
        stream = process.stdout
        try:
            for line in iter(stream.readline, ""):
                self._record(line.rstrip("\n"))
        except (ValueError, OSError):  # pragma: no cover - pipe torn down
            pass
        finally:
            try:
                stream.close()
            except OSError:  # pragma: no cover
                pass
        process.wait()
        self._record("--- exited with code {} ---".format(process.returncode))

    def _record(self, text: str) -> None:
        """Append one numbered line to the ring buffer."""
        with self._lock:
            self._lines.append((self._next_seq, text))
            self._next_seq += 1

    def log(self, after: int = 0) -> dict:
        """
        Return the retained lines numbered above ``after``.

        ``dropped`` says how many lines fell out of the ring buffer
        unseen, so the page can say so rather than presenting a
        discontinuous log as a continuous one.
        """
        with self._lock:
            lines = [(seq, text) for seq, text in self._lines if seq > after]
            oldest = self._lines[0][0] if self._lines else self._next_seq
            dropped = max(0, oldest - 1 - after) if after else 0
            return {
                "lines": [text for _seq, text in lines],
                "last_seq": lines[-1][0] if lines else max(after, self._next_seq - 1),
                "dropped": dropped,
            }

    # ── status ───────────────────────────────────────────────────────────

    def _reap(self) -> None:
        """Clear the process handle once it has exited. Call under the lock."""
        process = self._process
        if process is None or process.poll() is None:
            return
        self._returncode = process.returncode
        self._finished_at = self._finished_at or time.time()
        self._process = None

    def status(self) -> dict:
        """Return what is running, or what the last run did."""
        with self._lock:
            self._reap()
            return self._status_locked()

    def _status_locked(self) -> dict:
        """Build the status dict. Call under the lock."""
        running = self._process is not None
        if self._started_monotonic is None:
            uptime = None
        elif running:
            uptime = round(time.monotonic() - self._started_monotonic, 1)
        elif self._started_at and self._finished_at:
            uptime = round(self._finished_at - self._started_at, 1)
        else:
            uptime = None

        return {
            "running": running,
            "stopping": self._stopping,
            "behaviour": self._key,
            "label": self._label,
            "arguments": dict(self._values),
            "command": list(self._command),
            "command_text": " ".join(self._command),
            "pid": self._process.pid if running else None,
            "started_at": self._started_at,
            "finished_at": self._finished_at,
            "uptime_s": uptime,
            "returncode": None if running else self._returncode,
            "exit_note": self._exit_note(),
            "last_seq": self._next_seq - 1,
        }

    def _exit_note(self) -> str:
        """Describe how the last run ended, in words."""
        if self._process is not None or self._key is None:
            return ""
        code = self._returncode
        if code is None:
            return ""
        if code == 0:
            return "The behaviour exited normally."
        if code < 0:
            return "The behaviour was stopped by signal {}.".format(-code)
        if code == 130:
            return "The behaviour was interrupted."
        return ("The behaviour exited with code {} -- check the log above for "
                "what it was missing.".format(code))
