# mecanumbot_web

The robot-hosted web GUI. A normal rclpy node that also serves Flask, reachable
at `http://<robot>:8080` once the onboard stack is up.

It replaces a Flask app that ran in a Docker container **on the operator PC**,
paired with two ROS packages on the robot, sharing a SQLite schema that existed
in two copies — one per machine. That arrangement is what produced the "host
settings vs robot settings" split throughout the old UI: a
`destination: host | robot | both` parameter on six API routes, a
`from_robot_db` flag on `ApplyMapping`, and two side-by-side host/robot mapping
panels. None of it described anything about the robot; it only arbitrated
between two files. With the GUI on the robot there is one copy — the YAML in
the packages that own it — so the distinction has nothing left to describe.

There is no login and no database.

## Pages

| Path | Function |
| --- | --- |
| `/joystick` | Edit and reload `mecanumbot_joy`'s profile, with a live input readout. |
| `/diagnostics` | What the LEDs are showing, how the robot is moving, and the measured publish rate of every important topic against nominal. |
| `/behaviour` | Choose a behaviour tree, edit the constants it will load, and run it. |

`/` redirects to `/diagnostics`.

### Joystick

Bindings are rendered as the ten fixed vocabulary rows, each with a dropdown
populated from the controls *that profile declares* — so a binding naming a
control the pad does not have cannot be expressed, rather than merely being
rejected on save. Validation is `mecanumbot_joy.profile.validate_document`, the
same function the node runs, so the GUI cannot save something the robot would
refuse to load.

The live readout polls `/joy` at 5 Hz and shows which axis or button index
moved. That is the fastest way to build a profile for a pad nobody has mapped
before, and it is what the old GUI's static controller PNG never provided.

### Diagnostics

Answers "is each node keeping up?". Nothing in the workspace measured this
before — the old GUI had presence/absence watchdogs that could tell you a topic
had gone silent, but not that it had quietly dropped from 100 Hz to 12 Hz.

Subscriptions are created with `raw=True`, so payloads are never deserialised
and monitoring a 100 Hz `OpenCRState` costs almost nothing on the Orin. Rates
come from a sliding window over *receipt* times: several monitored topics carry
no header at all, and the question being asked is whether the pipeline is
delivering to a subscriber.

| Status | Meaning |
| --- | --- |
| `OK` | Within tolerance of nominal. |
| `SLOW` | Running but below tolerance — usually CPU contention or a blocked callback. |
| `FAST` | Above tolerance. Informational; usually means the nominal figure is stale. |
| `IDLE` | Silent, but silence is this topic's resting state and it is still wired up. |
| `DEAD` | Nothing on the graph holds the topic up, or nothing received recently enough to measure. |

The nominal table is `config/diagnostics_topics.yaml`; every rate in it is read
from the node that publishes it, with the source noted per entry. Topics with
`nominal_hz: 0` are event-driven and judged on staleness alone, so an idle
`/cmd_vel` never reads SLOW.

#### Idle is not dead

Several topics are *supposed* to be silent most of the time, and grading those
red would train the operator to ignore the colour that means something is
broken. `idle_when_silent: true` in the table marks them, and `presence` says
which end of the topic proves it is still wired up:

| Topic | Silent because | Alive because |
| --- | --- | --- |
| `/cmd_vel`, `/cmd_accessory_pos` | nothing is driving the robot | the io_node is still **subscribed** — nothing publishes these at rest, so the listener is the only evidence the link is intact |
| `people_fusion`, `subject_pose` | nobody is in view (`mecanumbot_locate_detections` only publishes a non-empty array; `subject_pose` publishes nothing until a subject is first seen) | the detector is **publishing** it at all |

So `DEAD` on `/cmd_vel` means a command would go nowhere, and `DEAD` on
`people_fusion` means the detector is down — both real faults, neither of them
"the robot is standing still" or "the room is empty". The table shows both
endpoint counts, dimming whichever one is not deciding.

Three traps the implementation handles: monitor subscriptions are BEST_EFFORT
(`scan` and the compressed camera topics are published that way, and a RELIABLE
subscription silently never matches one, so a healthy topic would read DEAD);
discovery re-runs on a timer, because this node starts alongside everything it
monitors and a one-shot lookup would miss all of it; and the subscriber count
discounts this node's own monitoring subscription — it subscribes to everything
in the table, so a raw `count_subscribers` would never be zero and a
subscriber-judged topic could never read DEAD.

#### Robot state: LEDs and movement

Rates answer "is every node keeping up?", which is not the same question as
"what is the robot doing". Two panels above the topic table answer the second
one, from the same 1 Hz poll — `/api/diagnostics` carries `led` and `motion`
alongside `topics`, so the page makes no extra requests.

**LEDs.** Four corner swatches in physical layout, each showing the colour and
the mode, and *animating the way the strip does* — pulse breathes, the two
blink modes blink at their own speeds, the wave modes sweep. There is no LED
state topic anywhere in the workspace; the Arduino Nano is only ever asked, so
this comes from the `get_led_status` service, which is a serial round trip on
the same link the behaviour trees use to set the LEDs. It is therefore polled
on a slow timer (`led_poll_period`, 2 s) into a cache the page reads, and can
lag a change by a second or two.

Three details are deliberate. The poll is `call_async` with a done callback,
not a blocking wait — it runs on an executor timer, and blocking there would
stall a callback group the rest of the node shares. A poll still outstanding
suppresses the next one, so a silent Arduino cannot queue requests behind
itself. And the all `-1` response the LED node returns for a bad checksum or a
silent board is treated as a failed read: it is reported as an error and the
last good reading stays on screen, rather than being rendered as a state the
strips can be in.

**Movement.** Commanded velocity from `/cmd_vel` beside measured velocity from
odometry, each classified into words. Showing both is the point: a command with
no measurement under it is a drivetrain fault, and it is invisible to
everything else on this page because both topics stay perfectly healthy
throughout. The panel calls that case out in words.

Classification lives in `motion.py`, with no rclpy in it. Mecanum drive is why
it is not a one-liner — `vy` is a real degree of freedom rather than
structurally zero, so "forward" and "strafing left" hold simultaneously during
a diagonal and the state is built from up to three components. Deadbands
(0.02 m/s, 0.05 rad/s) are set by what odometry noise looks like on a
stationary robot, so a parked robot does not read as creeping. The dial shows
direction of travel with the robot facing up, and a pure spin leaves it
unpointed because a spin has no direction of travel.

Odometry runs at 50 Hz and the page reads once a second, so that subscription
is `raw=True` like the monitor's and the bytes are deserialised at most every
0.2 s — the same reasoning as the rate monitor, taken one step further because
here the payload is actually wanted. Set `odom_topic` to an empty string to
drop the subscription entirely; the panel then says it is showing commanded
velocity only.

### Behaviour

One page for a whole trial: pick the tree, set the hyperparameters, edit the
constants it will load, start it, watch its console, stop it.

| Behaviour | Started as | Hyperparameters |
| --- | --- | --- |
| Leading | `ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py` | **condition** (`Doglike` / `Control` / `LED`), constants file, namespace |
| Ostensive | `ros2 launch mecanumbot_ostensive_behaviour launch_ostensive.launch.py` | constants file, namespace |
| Demo: wander between people | `ros2 run mecanumbot_demo_behaviours wander_between_people_node` | constants file, namespace |

The catalog is `behaviour_catalog.py`, and every entry is the command the
workspace documents for that tree — so a run started here and a run typed at a
terminal are the same run. The demo is the one exception to "use the launch
file", and for a stated reason: its launch file names `*_demo_bt_node`
executables its `setup.py` does not register, so launching it fails.

The constants file is not a separate control. The file the run will load *is*
the file the editor below is editing, chosen once, defaulting to whatever the
current SSID selects.

Only what a hyperparameter's own spec allows can be sent: a condition must be
one of the three the launch file implements, a namespace must be a single ROS
name token, and a constants file must be a name that exists in that behaviour's
own config directory — matched against the directory listing rather than
sanitised, so traversal, absolute paths and typos all fail identically.
`build_command` returns an argv list and nothing reaches a shell.

**One tree at a time.** Every tree publishes `/goal_pose` and `/cmd_vel`, so a
second one does not run alongside the first — they fight over the robot and the
trial is quietly invalid rather than obviously broken. Starting while something
runs is an error, and the page also watches the ROS graph for a tree somebody
started from a terminal, which its own bookkeeping cannot see.

Stopping sends SIGINT to the child's process group, because `ros2 launch` shuts
its nodes down properly on SIGINT and killing it outright would orphan the tree
— still on the graph, still driving. SIGTERM and then SIGKILL follow only if it
will not go. The child gets its own session, which is also why `web_node`'s
shutdown path stops it explicitly: nothing else would.

The console is a 400-line ring buffer, polled incrementally and numbered so a
gap is reported rather than silently closed. It is where a missing prerequisite
shows up, since a tree whose `setup()` cannot find nav2 exits before the page
next polls.

#### Editing the constants

**Edits take effect the next time a behaviour tree starts, not live.** Every
tree reads its file once, in its params behaviour's `setup()`. The page says so,
and warns before starting a run with unsaved edits on screen.

Two schemas, two editors, for a reason:

- **Leading and demo** (`behaviour_store.py`) — thresholds, the route, and the
  LED and gesture sequences, whose inline comments are *derived from the values*
  and are regenerated on save. Plus the **tunables**: the turn profile, timeouts
  and poses that used to be constructor defaults in the behaviour library. Those
  carry the reasoning for their numbers in prose, which cannot be regenerated,
  so it is read off the outgoing file and written back around the same keys.
- **Ostensive** (`flat_store.py`) — a flat block of scalars whose comments *are*
  the documentation. Nothing is rewritten there: only the value on each
  parameter's own line changes, then the result is reparsed and checked that
  exactly the intended keys moved. Parameters cannot be added or removed, since
  the tree reads a fixed set.

The emitter's `_self_check` refuses to write a document that has lost a key.
That is not defensive dressing: before the tunables were carried through, a save
from this page deleted about thirty of them, and a tree that lost one would fall
back to a packaged default mid-experiment without saying anything.

Validation follows what actually consumes each file:

- A `_times` list **shorter** than its `_seq` list is a hard error —
  `LEDBehaviourSequence` reads `delays[index - 1]` up to `len(patterns)`, so it
  would raise `IndexError` mid-tick, during an experiment.
- A `_times` list **longer** is a warning. Both shipped files are currently in
  that state. The page offers a one-click trim but does not apply it silently:
  seven delays against five patterns most likely means two patterns were lost,
  not that the delays are wrong, and that call belongs to the operator.

Entries are Python dict literals stored as YAML strings, and the emitter writes
them by hand rather than through a YAML dumper — turning one into a real YAML
mapping breaks `ast.literal_eval` and the tree throws at setup. `dump()`
self-checks by reparsing before returning.

**A number's type is part of that contract.** The tree assigns several of these
values straight into ROS message fields, which are strictly typed:
`parse_checkpoint` puts `X`/`Y`/`Z` into a `geometry_msgs/Point` (`float64`), so
`'Z':0` is `AssertionError: The 'z' field must be of type 'float'` at `setup()`,
and `parse_led` does the mirror image into `SetLedStatus`' `int8` fields, where
a float is the error. The browser cannot express the difference — JSON has one
number type and `JSON.stringify` drops the decimal point off every whole number
— so `behaviour_store` decides the type from the key rather than from the
request: coordinates, poses and delays are always written as floats, LED codes
and counts as whole numbers, and a tunable keeps whichever shape the file it
replaces gave it. A value that fits no type (an emptied field arriving as
`null`, a fractional LED mode) is refused by name instead of being rounded.
`_self_check` verifies the types of the emitted literals too, since every other
check reads `0` and `0.0` as the same document.

A file that already carries the wrong type says so when it is opened, and
**saving repairs it** — types are restored before validation, so a document
written by an older version of this editor is fixed by loading it and pressing
Save.

Inline comments on the LED and gesture entries are **regenerated from the
values** rather than preserved, which is an upgrade: several comments in the
shipped files contradict what they annotate (`'color':6` is labelled "white"
where 6 is pink, `'color':2` is labelled "yellow" where 2 is green). The
tunables' comments are carried over instead, because nothing can regenerate a
sentence about why a turn speed has to stay inside Nav2's `max_vel_theta`.

## Saving config safely

Under `--symlink-install`, a package's installed config files are symlinks back
into the source tree. Writing through them correctly edits the checkout — but
only if the path is `realpath()`-resolved first. The obvious atomic-write idiom
`os.replace(tmp, share_path)` replaces the *symlink itself* with a regular file
and silently decouples install from source, making every later edit appear to
vanish on rebuild. `yaml_io.atomic_write_text` resolves first; there is a
regression test asserting the symlink survives a save.

Every write is backed up first, to `~/.mecanumbot/config_backups/` — outside
the source tree, because every file this GUI edits sits in a git repository the
user commits from. Five backups are kept per file and are restorable from the
GUI.

## Design

Pure modules with no rclpy: `rate_monitor.py`, `motion.py`, `joystick_store.py`,
`behaviour_store.py`, `flat_store.py`, `behaviour_catalog.py`,
`behaviour_runner.py`, `yaml_io.py`, `led_enums.py`. Then `ros_bridge.py`
(the node), `app.py` (Flask routes) and `web_node.py` (the entry point).

`behaviour_runner.py` spawns processes but touches no ROS API, so the Flask app
owns it and the node's only involvement is stopping it on shutdown.

The palette is cyan and magenta, defined once as custom properties in
`base.html` and themed for both `prefers-color-scheme` settings. It carries the
chrome only — nav, focus rings, bars, panel headings. The four rate statuses
stay green / amber / rose, with only their temperature pulled toward the theme:
an operator reads those at a glance and hue is the whole signal.

The executor owns the main thread and Flask runs in a daemon thread — the
opposite of the GUI this replaces, which suited a container whose only job was
to serve HTTP but made a poor ROS node, because SIGINT never reached the
executor and `ros2 launch` could not shut it down cleanly.

Service calls arrive on Flask threads, so the clients sit in a
`ReentrantCallbackGroup` under a `MultiThreadedExecutor` and block on an event
rather than spinning; under the default callback group the future would never
complete.

## Parameters

| Parameter | Default |
| --- | --- |
| `host` / `port` | `0.0.0.0` / `8080` |
| `joystick_config_dir` | `mecanumbot_description/config/joystick` |
| `behaviour_config_dir` | `mecanumbot_leading_behaviour/config` |
| `ostensive_config_dir` | `mecanumbot_ostensive_behaviour/config` |
| `demo_config_dir` | `mecanumbot_demo_behaviours/config` |
| `diagnostics_config` | `mecanumbot_web/config/diagnostics_topics.yaml` |
| `joy_node` / `joy_topic` | `/mecanumbot/mecanumbot_joy_node` / `/mecanumbot/joy` |
| `backup_root` | `~/.mecanumbot/config_backups` |
| `led_service` | `get_led_status` |
| `led_poll_period` | `2.0` s — `0` disables LED reads |
| `cmd_vel_topic` | `/cmd_vel` |
| `odom_topic` | `odom` — empty disables measured motion |

The three config directories are resolved leniently: a behaviour whose package
is not in the checkout is shown on the page as unavailable, and cannot be edited
or started, rather than the GUI failing to come up.

`led_service` and `odom_topic` are **relative** names, so they resolve inside
this node's namespace alongside the LED service node and
`mecanumbot_sensorproc_node`. `cmd_vel_topic` is absolute because the base
launch remaps `cmd_vel` out of the namespace so Nav2 and the joy node meet on
one topic. `led_poll_period` and `odom_topic` are also launch arguments of
`web.launch.py`.

## Dependencies

**Flask is not installed by `rosdep`.** On the robot:

```bash
sudo apt install python3-flask
```

Jammy ships Flask 2.0.1, so avoid APIs added in 2.2 or later.

## Running

```bash
# with the rest of the onboard stack (use_web:=false to disable)
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py

# alone
ros2 launch mecanumbot_web web.launch.py web_port:=8080
```

## Tests

```bash
colcon test --packages-select mecanumbot_web && colcon test-result --verbose
```

224 tests, none needing a ROS graph; the 47 route tests skip cleanly when Flask
is absent. Note that `pytest.importorskip` is deliberately **not** used for
that — raising `Skipped` at module level aborts collection for the whole
session under the pytest version Humble ships, which would silently reduce the
suite to one test instead of skipping one file.

`test_behaviour_runner.py` starts **real child processes** — short Python
scripts, not `ros2 launch`, injected by standing a fake spec in for the
catalog's. What that module is for is starting a process and being sure it is
gone afterwards, and a mocked `Popen` would prove neither. One of them ignores
SIGINT and SIGTERM, so the escalation to SIGKILL is exercised rather than
assumed.
