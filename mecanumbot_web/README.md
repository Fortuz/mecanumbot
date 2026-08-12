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
| `/behaviour` | Edit the leading-behaviour constants YAML. |

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

Edits `behaviour_setting_constants.yaml` / `Eto_behaviour_setting_constants.yaml`
in `mecanumbot_leading_behaviour`, and marks which one the current Wi-Fi SSID
selects.

**Edits take effect the next time a behaviour tree starts, not live.** The tree
reads the file once, in `ConstantParamsToBlackboard.setup()`. The page says so.

Validation follows what actually consumes the file:

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

Inline comments are **regenerated from the values** rather than preserved,
which is an upgrade: several comments in the shipped files contradict what they
annotate (`'color':6` is labelled "white" where 6 is pink, `'color':2` is
labelled "yellow" where 2 is green).

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
`behaviour_store.py`, `yaml_io.py`, `led_enums.py`. Then `ros_bridge.py`
(the node), `app.py` (Flask routes) and `web_node.py` (the entry point).

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
| `diagnostics_config` | `mecanumbot_web/config/diagnostics_topics.yaml` |
| `joy_node` / `joy_topic` | `/mecanumbot/mecanumbot_joy_node` / `/mecanumbot/joy` |
| `backup_root` | `~/.mecanumbot/config_backups` |
| `led_service` | `get_led_status` |
| `led_poll_period` | `2.0` s — `0` disables LED reads |
| `cmd_vel_topic` | `/cmd_vel` |
| `odom_topic` | `odom` — empty disables measured motion |

`behaviour_config_dir` is resolved leniently: if `mecanumbot_leading_behaviour`
is not in the checkout the behaviour page reports nothing to edit rather than
the GUI failing to start.

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

148 tests, none needing a ROS graph; the 35 route tests skip cleanly when Flask
is absent. Note that `pytest.importorskip` is deliberately **not** used for
that — raising `Skipped` at module level aborts collection for the whole
session under the pytest version Humble ships, which would silently reduce the
suite to one test instead of skipping one file.
