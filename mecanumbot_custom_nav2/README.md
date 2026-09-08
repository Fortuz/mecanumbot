# mecanumbot_custom_nav2

Two things: **autonomous exploration** (the T1 pass of the Deep3R seeking
system) and **the robot's end of the 2D/3D comparison**.

The explorer drives the robot around a place it has never seen, building a 2D
map with slam_toolbox while the cluster builds a 3D point cloud from the same
drive, and decides for itself when the scan is finished.

The agreement handler takes the server's verdict on how those two disagree and
turns it into things the robot acts on — a nav2 keepout mask, a revisit list and
markers. It runs in **both** phases, because a table the lidar cannot see is
still there in T2 when the robot is fetching something across the room.

The name says `nav2` because that is what it steers. It replaces no part of the
navigation stack: every translation goes through `NavigateToPose`, and the nav2
tuning for this phase is a **parameter file in `mecanumbot_description`**
(`param/mecanumbot_exploration_nav2.yaml`), not code. What could be done in YAML
was done in YAML; this package is the part that could not be — a frontier
detector.

```text
                        ┌──────────────── mecanumbot_map_agreement ───────────────┐
  deep3r/map_agreement ─┤  AgreementModel  ──┬──► deep3r/keepout_mask  ──► nav2   │
        (the server)    │                    ├──► deep3r/costmap_filter_info      │
                        │                    ├──► deep3r/revisit_regions ──┐      │
                        │                    └──► deep3r/agreement_markers │      │
                        └────────────────────────────────────────────────┼─┘
                                                                          │
  /map (slam_toolbox) ──┐                                                 │
  map -> base_link (TF) ─┼─► RRT frontier detection ──► cluster ──► score ┤
  deep3r/map_agreement ──┘                                                ▼
                                                          nav2 NavigateToPose
                                                                          │
                    /exploration/finished ◄── exit criteria ◄─────────────┘
```

## Why a sampling-based detector

The straightforward frontier detector scans every cell of the occupancy grid for
the free/unknown boundary, and its cost grows with the area of the map. On the
Orin Nano, at the rate an explorer needs, that is real CPU the perception stack
wants back.

An RRT finds frontiers as a side effect of growing: the tree is biased towards
unexplored regions by construction, so its branches run into the unknown on
their own, and the cost of a detection cycle is set by the iteration budget
rather than by the size of the map. The algorithm is Umari & Mukhopadhyay's
(2017), with the two trees it specifies:

| Tree | Rooted | Reset | Finds |
| --- | --- | --- | --- |
| global | where exploration started | never (except after a loop closure) | frontiers anywhere, including rooms the robot has left |
| local | at the robot, every cycle | on every detection | frontiers near the robot — the ones worth driving to next |

Neither tree is a plan. A detected point is a *place worth going*; getting there
is nav2's problem, and the straight-line steps are only how the tree grows.

## Nodes

### `mecanumbot_frontier_explorer_node`

Registers as `mecanumbot_frontier_explorer`. **The node name has to match the
root key of `config/frontier_explorer.yaml`** — renaming it silently drops every
parameter, the same trap `mecanumbot_sensorprocess_smart` documents.

#### Subscribers

| Topic | Type | Function |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | The map being built. A *shrinking* known area is read as a loop closure: slam_toolbox re-rasterises the whole grid, so both RRTs are thrown away and regrown. |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Robot pose — only with `pose_source: amcl`. |
| `/mecanumbot/deep3r/map_agreement` | `mecanumbot_msgs/MapCloudAgreement` | The server's comparison verdict. Feeds the exit criteria and supplies revisit goals. |

With `pose_source: tf` (the default) the pose is read from the `map ->
mecanumbot/base_link` transform instead. **T1 runs under slam_toolbox, which
publishes no `/amcl_pose`**, so `tf` is the setting this node exists for; `amcl`
is for running it against a saved map.

#### Publishers

| Topic | Type | Function |
| --- | --- | --- |
| `exploration/finished` | `std_msgs/Bool` | Latched. `false` at start-up, `true` once T1 is over. This is how T1 hands over — whatever starts T2 waits for the latch, not for a wall-clock guess. |
| `exploration/state` | `std_msgs/String` | One line per cycle: frontier count, distance driven, current goal source, and every exit criterion with its reason. The first thing to look at when the robot is not moving. |
| `exploration/frontiers` | `visualization_msgs/MarkerArray` | Every scored frontier (green) and the chosen goal (red). Seeing the frontiers the robot *rejected* is most of the debugging. |

#### Actions

`/navigate_to_pose` (`nav2_msgs/NavigateToPose`) — one goal in flight at a time,
cancelled and re-decided on timeout. A goal nav2 gives up on is retired rather
than re-proposed: a frontier the planner cannot reach is behind something, and
the detector will happily suggest it again for ever.

The full parameter table is `config/frontier_explorer.yaml`, which documents
every constant where it is set rather than duplicating it here.

### `mecanumbot_map_agreement_node`

Registers as `mecanumbot_map_agreement`. Consumes the server's comparison
verdict and produces what the rest of the robot acts on. **Runs in both phases**
— it is started by `autoslam.launch.py` for T1 and by
`mecanumbot_seek/launch_seek.launch.py` for T2 (`use_agreement:=false` when one
is already running).

#### Subscribers

| Topic | Type | Function |
| --- | --- | --- |
| `/mecanumbot/deep3r/map_agreement` | `mecanumbot_msgs/MapCloudAgreement` | The verdict: where the two sources disagree, what kind of disagreement, and how tall the structure is. |
| `/map` | `nav_msgs/OccupancyGrid` | Only its geometry. The mask must have the same shape, origin and resolution as the map nav2 plans on, or every keepout lands somewhere else and nothing reports an error. |

The robot's position is read from TF (`map → mecanumbot/base_link`) rather than
from any goal outcome, so a region counts as looked at however the robot got
there — an explorer goal, the seek tree, or somebody driving it with the
joystick. It works unchanged under slam_toolbox in T1 and AMCL in T2.

#### Publishers

| Topic | Type | Function |
| --- | --- | --- |
| `deep3r/keepout_mask` | `nav_msgs/OccupancyGrid` | Latched. Lethal cells where the cloud sees structure the robot would hit. |
| `deep3r/costmap_filter_info` | `nav2_msgs/CostmapFilterInfo` | Latched. Lets `nav2_costmap_2d::KeepoutFilter` consume the mask directly — no new costmap plugin, no patch to nav2. |
| `deep3r/revisit_regions` | `geometry_msgs/PoseArray` | Regions still worth looking at, best first, with the structure height in `z`. The explorer interleaves these with its frontier goals. |
| `deep3r/agreement_markers` | `visualization_msgs/MarkerArray` | Colour-coded by kind and drawn at their real height, so a table top and a step on the floor look different. An invisible keepout that stops the robot in an apparently empty corridor is otherwise unexplainable. |

### The decision is about height

This is the whole content of `agreement.py`, and it follows from one fact: **the
lidar is a single horizontal plane.** It cannot see a table top, a low step, a
cable tray or an overhang, and the 2D map says "free" for all of them — because
at the lidar's height they *are* free. The cloud sees the volume. So a
`cloud_only` region is usually not noise in the reconstruction; it is the 2D map
being wrong in the one way it is systematically able to be wrong.

Whether that matters depends on where the structure sits in the robot's vertical
envelope:

| Height | Meaning | Action |
| --- | --- | --- |
| above `robot_height` (0.45 m) | an overhang | ignore — the robot drives under it |
| `clearance` … `robot_height` | the robot would hit it | **keepout** |
| below `clearance` (0.03 m) | a cable, a threshold strip | ignore — the robot rolls over it |
| **unknown** | the server saw structure and could not measure it | **keepout** |

An unknown height is treated as blocking on purpose. Assuming an overhang is the
one reading that lets the robot drive into something.

`lidar_height` (0.20 m) does not change the decision. It splits the keepouts into
the ones the lidar *could* have seen and missed — worth investigating as a sensor
or alignment problem — and the ones it structurally cannot see, which are worth
nothing but the keepout. That count is in the per-verdict log line.

### It only ever adds obstacles

A `map_only` region — the lidar seeing something the camera could not
reconstruct, which is what glass and thin dark panels look like — subtracts
nothing. The lidar is the one to believe there, and the map already has it. The
reconstruction is not allowed to talk the robot into driving somewhere the lidar
says it cannot.

A verdict is a **snapshot, not a diff**: regions accumulate and a keepout is
dropped only after `forget_after` seconds without the server mentioning it.
Dropping one the instant it left a verdict would make the costmap flicker at the
comparison rate, and a flickering costmap is one nav2 replans through.

### Which nav2 parameters carry it

| Phase | File | Filter |
| --- | --- | --- |
| T1 | `mecanumbot_exploration_nav2.yaml` | `keepout_filter` → `deep3r/costmap_filter_info` |
| T2 | `mecanumbot_seek_nav2.yaml` | `deep3r_filter` → same topic |
| study runs | `mecanumbot_custom_nav2*.yaml` | **untouched** |

The study files are deliberately left alone: the leading and ostensive
experiments run on them, and adding a filter to their configuration — even an
inert one — is a change to apparatus those trials are calibrated against. Select
the T2 file with the `NAV2_PARAMS_FILE` environment variable, which
`launch_mecanumbot_base.launch.py` now honours:

```bash
NAV2_PARAMS_FILE=$(ros2 pkg prefix mecanumbot_description)/share/mecanumbot_description/param/mecanumbot_seek_nav2.yaml \
  ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py
```

With the handler not running, no `costmap_filter_info` is published, the filter
stays inactive, and both files behave exactly like the ones they were derived
from.

## The SEEKING circuit does not steer this package

`mecanumbot_seek` models Panksepp's SEEKING circuit and its README describes an
`undirected` phase — the appetitive system before anything particular is wanted,
which is what T1 is meant to be. **Nothing in this package reads it.** The
frontier score, the revisit interleave and the exit rule are all this package's
own, and no `SeekingState` crosses between the two.

That is a live design question rather than an oversight, and it is recorded in
`mecanumbot_seek/README.md`: what an undirected circuit is entitled to change
during exploration — whether falling expectancy should widen the frontier search
the way it widens T2's rings, whether extinction should be able to end T1
alongside the criteria below — is a question about how far the model reaches,
and wiring it up would answer it by accident. Until it is answered, T1 is
exploration with the circuit running beside it, not exploration the circuit
drives.

## When is T1 finished?

This is the question the package exists to answer, and the answer is
`exit_criteria.py`. Ending T1 has two halves that are easy to confuse — *the
robot has nowhere left to go* and *the reconstruction has stopped improving* —
and they do not fall due at the same time.

| # | Criterion | Met when | Why it is not enough on its own |
| --- | --- | --- | --- |
| 1 | `FRONTIERS` | no frontier worth driving to for `frontier_quiet_time` | exact but brittle: one sliver of unknown behind a glass door keeps it false for ever |
| 2 | `GAIN` | newly-observed cells per **metre driven** below `min_cells_per_metre` | robust but never quite reaches zero |
| 3 | `STABLE` | known area grew < `max_map_growth` over the window, and no loop closure within `loop_closure_settle` | a map that has stopped growing may still have unexplored rooms |
| 4 | `CLOUD` | the server says coverage ≥ `min_grid_coverage`, ≤ `max_uncertain_regions` complaints left, and the cloud has stopped growing | says nothing about whether the robot can still get anywhere |
| 5 | `BUDGET` | time, distance or battery ran out | not a quality criterion at all |

```text
finished = (FRONTIERS or GAIN) and STABLE and CLOUD,  or  BUDGET
```

**1 and 2 are alternatives** because each catches what the other misses.

**3 is a conjunct, not an alternative.** slam_toolbox re-rasterises the whole
occupancy grid after a loop closure, and a fresh crop of frontiers appears a
second later. Exiting into that is how a run ends with half a map, so the rule
refuses to finish within `loop_closure_settle` of one.

**4 is a conjunct because it is the whole point.** The 2D map is the *means*;
the cloud is the *product*. A complete 2D map over a cloud that covers one wall
is a failed T1 that looks exactly like a successful one, and nothing in the 2D
data can tell you so. Set `require_cloud: false` for a mapping-only dry run —
which is the right thing without a server and the wrong thing during a trial.

**5 overrides everything** because an unattended run has to stop. A flat battery
mid-reconstruction loses the whole session: the cloud lives on the server
against a `map_id` that will not survive the robot coming back.

`Verdict.complete` distinguishes the two kinds of ending, and it matters for the
record — *"T1 ended because the reconstruction stopped improving"* and *"T1 ran
out of time"* are different runs and must not be summarised into the same word.

### Two measurement decisions worth knowing about

**The gain rate is per metre driven, not per second.** A robot that has stopped
— waiting for a plan, stuck against a chair, or parked because there is
genuinely nothing left — fills in no cells either way, and a per-second rate
cannot tell those apart. Per metre, a stationary robot contributes no evidence
and the criterion returns *no opinion* rather than *finished*.

**A loop closure is detected as the known area shrinking**, which is the only
thing that makes it go down. There is no slam_toolbox topic that announces one.

## Feeding back to the reconstruction

The diagram this package implements has an arrow from `Compare` back into
`Deep3r`, and the question behind it is whether the robot can improve the point
cloud rather than only consume it. It can, on three channels of increasing
commitment. **This package implements the first**; the other two are recorded
here because the choice between them is a design decision, not an oversight.

**(a) View planning — implemented.** The comparison finds regions where the 2D
map is confident and the cloud is thin, absent, or disagrees — the *"non-full
lines"*. `MapCloudAgreement.uncertain_points` carries them in the map frame, the
handler decides which are worth a look, and the explorer drives to them: one
region for every `uncertain_every` frontier goals. More frames from a better
viewpoint is the only feedback a feed-forward reconstruction like CUT3R can
actually use, and it needs nothing new on the server. Interleaved rather than
prioritised, because an explorer that only services the server's complaints
stops exploring.

**(a′) The reverse direction — also implemented, and the bigger win.** The
comparison does not only tell the robot where to point its camera; it tells it
**where not to drive**. A `cloud_only` region inside the robot's vertical
envelope is a table leg, a step or a cable tray the lidar plane passed over, and
the handler makes it lethal in the costmap. This is the reconstruction improving
the robot's *navigation* rather than the robot improving the reconstruction, and
it is the payoff that does not depend on anything changing on the server.

**(b) Scale and drift anchoring — the highest-value one, not built.** The
`mecanumbot_deep3r` client already logs the ratio between the cloud's near depth
and the LiDAR's forward range, which is the cheapest check that CUT3R's metric
scale is actually metric. Promoting that to a published estimate, and letting
the server apply a per-session scale and a rigid alignment onto the map frame,
is genuine feedback into the reconstruction and does not touch the model. This
is where the next work should go.

**(c) Conditioning the model on the 2D occupancy — not for now.** Feeding the
floor plan into CUT3R's inference as a prior is a research project of its own.

### Which source is more reliable?

They are reliable about **different things**, and the architecture depends on
not collapsing them into one answer.

The **2D LiDAR map** is better in *geometry and global consistency*: metric by
construction, loop-closed, and it does not drift. But it is one horizontal slice
at the LiDAR's height, blind above and below it, and it cannot tell a table from
a wall.

The **3D cloud** is better in *coverage and semantics*: it sees the whole volume
and carries colour, which is what an object can be located in. But CUT3R anchors
its world frame on the first frame of a `map_id` and drifts independently of
odometry, and its scale is only approximately metric — `mecanumbot_deep3r`'s
README says so and gives the check.

So the split the system is built on is: **the 2D map is the frame of record for
navigation, and the cloud is the frame of record for what is where.** The
comparison is not an adjudication between them; it is the transform plus a
disagreement map. That is why `MapCloudAgreement` carries its points in the
*map* frame — the server does the aligning, and the robot never has to reason in
a frame that drifts under it.

The height axis is where that split pays. The 2D map has no z at all, so from
the lidar alone a mug on a table and a mug on the floor behind a chair are the
same fact: "something is at (x, y) and the robot cannot get to it". Add a height
and they become two different problems with two different answers — drive round
it, or go and tell somebody. `mecanumbot_seek`'s alert behaviour is built on
exactly that, and it is the clearest single reason the reconstruction earns its
place rather than being a nice picture.

## Starting T1

T1 spans **two machines**, and the order matters: the robot dials a tunnel that
must already have a live server behind it. This is the canonical sequence —
`mecanumbot_deep3r` and `mecanumbot_seek` point here rather than repeating it.

### 1. The cluster, from `nipg1`

```bash
cd ~/mecanumbot_repos/RoboCamStreamProcessing
salloc --no-shell --gres=gpu:1 -c 8 --mem=24G -t 08:00:00
srun --jobid=<id> --overlap ./scripts/run_deep3r_bridged.sh
```

Allocate **from `nipg1`** — Slurm does not work from the `nipg36` container,
which cannot resolve the compute-node names. The job can land on any node with a
free GPU; prefer Ampere or newer (nipg10's 3090s are sm_86 and much faster than
nipg36's TITAN RTX). `run_deep3r_bridged.sh` starts the server and only then
opens the reverse tunnel to `nipg1:5555`, so a live tunnel implies a live server
rather than a socket in front of nothing.

No `--phase` argument: `config/server.yaml` already starts in `t1`. Model load
plus warm-up is about 25 s — wait for the bind line before starting the robot.

### 2. The robot — one terminal

```bash
./netcheck.sh          # first: can the robot reach the rendezvous at all?
ros2 launch mecanumbot_custom_nav2 t1.launch.py
```

`t1.launch.py` starts the drivers, the Deep3R client and this package's
exploration stack in that order, with delays so the logs read in the order the
subsystems came up. One Ctrl-C stops all of it. Useful arguments:

```bash
ros2 launch mecanumbot_custom_nav2 t1.launch.py require_cloud:=false use_deep3r:=false
ros2 launch mecanumbot_custom_nav2 t1.launch.py deep3r_delay:=15.0 explorer_delay:=25.0
```

The trade is that three stacks share one terminal. When something is wrong and
the noise is in the way, use the three-terminal form below instead — it is the
same three launch files and the same ordering.

### 2b. The robot — three terminals

```bash
# drivers and camera, but NOT nav2 -- autoslam brings its own
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py use_nav2:=false
ros2 launch mecanumbot_deep3r deep3r.launch.py        # needs the tunnel up
ros2 launch mecanumbot_custom_nav2 autoslam.launch.py
```

`use_nav2:=false` is not optional. The base launch otherwise starts nav2 against
the *study* parameters with AMCL and a saved map; that stack and this one both
publish `map -> odom` and both serve `navigate_to_pose`.

> **`use_nav2` did not exist until 2026-09-08.** It was documented here, in this
> package's launch file and in the workspace `CLAUDE.md`, but was never declared
> in `launch_mecanumbot_base.launch.py` — nav2 came up unconditionally in both
> SSID branches, and passing the argument did nothing but emit a warning. Every
> T1 run started before that date had two nav2 stacks and two things publishing
> `map -> odom`. `t1.launch.py` passes it for you, so this is one fewer thing to
> get right by hand.

`deep3r.launch.py` defaults to `enable_map_loop:=true`, which is what sends the
pose, the grid and the scan **up** and republishes the server's verdict back
down. Without it the node is a frame pump, the server has nothing to place the
cloud against, and the `CLOUD` criterion below can never be satisfied — so T1
would never finish, with nothing in the logs pointing at why.

### 3. Watching it

```bash
ros2 topic hz   /mecanumbot/deep3r/points          # ~6 Hz on a 3090
ros2 topic echo /mecanumbot/deep3r/map_agreement   # the server's verdict
ros2 topic echo /mecanumbot/exploration/state      # every criterion, with its reason
ros2 topic echo /mecanumbot/exploration/finished   # latches true when T1 is over

# what the comparison is telling it
ros2 topic echo /mecanumbot/deep3r/revisit_regions
ros2 topic hz   /mecanumbot/deep3r/keepout_mask
#   ... and add the MarkerArray /mecanumbot/deep3r/agreement_markers in RViz:
#   red cylinders are keepouts, amber ones places to go and look
```

**The number to read first is `agreement`.** In a mapped room it should be
clearly above zero within a minute. Near zero means the reconstruction is not
placed where the map is — a wrong `compare.camera_z`, a wrong mount, or a pose
in the wrong frame — and none of those announce themselves any other way. T1
will now refuse to finish on it rather than handing T2 a cloud that does not
line up with the room.

### Without a server

```bash
ros2 launch mecanumbot_custom_nav2 autoslam.launch.py require_cloud:=false
```

Mapping only, and the exit criteria fall back to the 2D ones. The right thing
for a dry run and the wrong thing during a trial: it drops `placed`, which is
the test that catches a failed T1 that looks exactly like a successful one.

### Handing over to T2

Nothing to do by hand any more. The explorer latches
`/mecanumbot/exploration/finished`, `mecanumbot_deep3r` sees the latch and sends
the server a `phase` message, and the server's decision stage starts looking for
the target. That link did not exist before 2026-09-08, and without it the two
ends spent the rest of the run in different phases.

Saving the map is still deliberately manual — the map T2 localizes against is
worth looking at before it is written over:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/Documents/mecanumbot_ws/src/mecanumbot/mecanumbot_description/maps/AI_dept/AI_dept
```

Then T2, which is `mecanumbot_seek` — see its README:

```bash
ros2 topic pub --once /mecanumbot/seek/request std_msgs/String \
  "{data: 'the red mug on the desk'}"     # free text, not a class label
ros2 launch mecanumbot_seek launch_seek.launch.py
```

> **Not yet run against the real robot and server.** As of 2026-09-08 the loop
> is green on tests and builds only. The first live run is where the camera
> height and the camera mount get checked, and `agreement` is the number that
> tells you whether they are right.

## The nav2 parameter file

`mecanumbot_description/param/mecanumbot_exploration_nav2.yaml`, derived from
`mecanumbot_custom_nav2_no_keepout.yaml`. Four differences, all of them things
exploration forces and none of them needing code:

1. **No localisation stack.** slam_toolbox owns `map -> odom`; the `amcl`,
   `map_server` and `lifecycle_manager_localization` blocks are gone. Nav2 comes
   up through `nav2_bringup/navigation_launch.py`, which starts navigation and
   nothing else — running AMCL as well gives two things estimating the same
   transform.
2. **No static layer in the global costmap.** `/map` is being built and
   republished whole on every update, so the obstacle layer is the only source.
3. **A generous planner tolerance (0.75 m).** Every goal is a frontier — a point
   on the boundary of the unknown, which is exactly where NavFn is least willing
   to terminate a path. At the study file's 0.3 m a large fraction of frontier
   goals are outright planning failures.
4. **Stock nav2 recovery behaviours.** The study runs replace them because half
   a minute of spinning and reversing mid-trial looks absurd; during exploration
   nobody is watching and a spin is free LiDAR coverage.

Everything else — footprint, inflation, DWB tuning, the velocity smoother — is
deliberately identical, so a path driven during exploration is driven the same
way as a path during a trial.

## Tests

```bash
cd src/mecanumbot/mecanumbot_custom_nav2
PYTHONPATH=. python3 -m pytest test/ -q -p no:launch_testing \
  --ignore=test/test_flake8.py --ignore=test/test_copyright.py --ignore=test/test_pep257.py
```

158 tests, all pure Python — no ROS graph, no map server, no GPU. Use
`/usr/bin/python3`, not the conda one.

| File | Covers |
| --- | --- |
| `test_occupancy.py` | world↔cell conversion, the three occupancy classes, the disc-masked information gain, Bresenham, and the frontier-cell definition — including that a **wall** between free and unknown is not a frontier, which is the mistake that makes a robot drive at a wall for ever |
| `test_rrt.py` | that no tree edge ever crosses a wall, that a closed room yields no frontiers at any seed, that a doorway is found, that the local tree re-roots and the global one does not |
| `test_frontiers.py` | clustering, revalidation, and every term of the score — including that hysteresis holds a goal against an equal rival but not against a much better one |
| `test_exit_criteria.py` | each criterion alone and the composite rule, written around the failures that look like successes |
| `test_agreement.py` | every height band exactly, that an unknown height is cautious, that `map_only` never subtracts, that regions survive a verdict omitting them, and that a re-report does not send the robot back to a region it already looked at |

`test_flake8` fails here as it does everywhere in this workspace — the installed
flake8 cannot load its own `pycodestyle` plugin. `python3 -m ament_flake8.main .`
and `ament_pep257` both report clean.
