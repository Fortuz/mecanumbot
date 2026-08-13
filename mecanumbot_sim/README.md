# mecanumbot_sim

Simulation package for the mecanumbot. It provides two interchangeable simulator
backends, the scenario-driven actors they populate the world with, and the
evaluation nodes that score a run.

Everything simulation-related lives here. `mecanumbot_bringup` keeps exactly one
launch file, `sim.launch.py`, which is the entry point for every simulated run;
this package holds the nodes, models, worlds and scenarios it starts.

## The two backends

Both backends present the same interface the real `mecanumbot_io_node` presents:
they publish `opencr_state` and `scan`, and they consume `cmd_vel` and
`cmd_accessory_pos`. Everything downstream — `mecanumbot_sensorproc_node`, the
DR-SPAAM detector, the evaluators, the behaviour trees — is therefore identical
in simulation and on the robot, and identical between the two simulators.

| Backend | Executable | Needs | Status |
| --- | --- | --- | --- |
| MuJoCo | `mecanumbot_sim_mujoco_io_node` | `mujoco` pip package | Works; the long-standing backend. |
| Gazebo Sim | `mecanumbot_sim_gz_io_node` | `ros-humble-ros-gz` | **Never executed.** Written against Gazebo Fortress but not run — see [Gazebo status](#gazebo-status). |

Shared unit conversions live in `kinematics.py`: mecanum inverse kinematics,
the OpenCR velocity-tick scaling and the neck/grabber servo scaling. Both
backends import it, so a scenario cannot behave differently between them because
of a constant that drifted in one copy.

## Node overview

| Node | Executable | Role |
| --- | --- | --- |
| `mecanumbot_sim_io_node` | `mecanumbot_sim_mujoco_io_node` / `mecanumbot_sim_gz_io_node` | The simulated robot. Both register the same ROS node name, so only one runs at a time. |
| `mecanumbot_sim_oracle_subject_node` | `mecanumbot_sim_oracle_subject_node` | Republishes ground-truth subject pose as `subject_pose`, standing in for the detector. |
| `mecanumbot_sim_detection_evaluator_node` | same | Scores perception against ground truth. |
| `mecanumbot_sim_behavior_evaluator_node` | same | Scores robot motion against ground truth. |
| `mecanumbot_sim_visualization_node` | same | RViz markers for actors, detections and evaluator verdicts. |
| `mecanumbot_sim_detector_debug_node` | same | Detection-to-truth association markers, for scan/detector alignment. |
| `mecanumbot_sim_nav_shim_node` | same | Stands in for Nav2 so behaviour trees can run without the full navigation stack. |

## Node: the simulated robot (both backends)

### Publishers

| Topic | Data type | Function |
| --- | --- | --- |
| `opencr_state` | `mecanumbot_msgs/msg/OpenCRState` | Wheel command/measured velocities and positions, accessory positions, battery voltage and IMU — the same frame the real board emits. |
| `scan` | `sensor_msgs/msg/LaserScan` | 240-point 360° scan, stamped in `<namespace>/base_scan`. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time. MuJoCo publishes it directly; on Gazebo it comes from the bridge. |
| `/sim/actors` | `mecanumbot_msgs/msg/SimActorArray` | Ground-truth pose and twist of every scenario actor, in `map`. |
| `/sim/subject_pose_ground_truth` | `geometry_msgs/msg/PoseStamped` | Ground-truth pose of the actor flagged `is_subject`. |

### Subscribers

| Topic | Data type | Processing |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Mecanum inverse kinematics to four wheel tick commands, clamped to ±300 as the firmware does. |
| `cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Neck and grabber targets, scaled to servo ticks. |

### Parameters shared by both backends

| Parameter | Default | Function |
| --- | --- | --- |
| `scenario_path` | `''` | Scenario YAML. Empty means an empty arena with no actors. |
| `sim_rate_hz` | `100.0` | Rate of the `opencr_state` / actor update loop. |
| `battery_voltage` | `12.1` | Constant reported as the main battery voltage. |
| `robot_params.wheel.radius` | `0.0325` | Wheel radius in metres. |
| `robot_params.wheel.separation_x` | `0.129` | Longitudinal wheel separation. |
| `robot_params.wheel.separation_y` | `0.300` | Lateral wheel separation. |
| `robot_params.wheel.vel_tick` | `0.229` | RPM per velocity tick. |
| `robot_params.accessory.neck_default` | `850` | Neck servo ticks at startup. |
| `robot_params.accessory.grabber_default` | `512` | Grabber servo ticks at startup. |

### MuJoCo-only parameters

| Parameter | Default | Function |
| --- | --- | --- |
| `model_path` | `mecanumbot_sim/mujoco/mecanumbot.xml` | MJCF model. |
| `publish_clock` | `true` | Publish `/clock`. This backend is the clock source, so it runs with `use_sim_time:=false` itself. |
| `show_viewer` | `true` | Open the passive MuJoCo viewer. |
| `scan_rate_hz` | `8.0` | Lidar rate. Rays are cast with `mj_ray`. |
| `scan_count` | `240` | Rays per scan. |
| `scan_range_min` / `scan_range_max` | `0.02` / `8.0` | Range limits in metres. |
| `scan_angle_min` / `scan_angle_max` | `-π` / `π` | Angular span. |

### Gazebo-only parameters

| Parameter | Default | Function |
| --- | --- | --- |
| `model_name` | `mecanumbot` | Name the robot is spawned under; decides the `/model/<name>/…` bridge topics. |
| `velocity_frame` | `body` | Frame Gazebo's VelocityControl reads `cmd_vel` in. See [Gazebo status](#gazebo-status). |
| `gz_scan_topic` | `/scan_raw` | Bridged lidar topic, re-stamped and republished as `scan`. |
| `gz_imu_topic` | `/imu_raw` | Bridged IMU topic, folded into `opencr_state`. |
| `gz_joint_state_topic` | `/model/mecanumbot/joint_state` | Bridged joint states, used for accessory position feedback. |

## Gazebo status

**The Gazebo backend has never been run.** Gazebo is not installed on the
development machine this was written on (`gz`, `ign`, `gazebo`, `ros_gz` and
`gazebo_ros_pkgs` are all absent), so the models, world, bridge and node are
written but unverified. The MuJoCo backend is unaffected and remains the one that
is known to work.

To try it:

```bash
sudo apt install ros-humble-ros-gz
ros2 launch mecanumbot_bringup sim.launch.py backend:=
```

Three things to check on that first run:

1. **`velocity_frame`.** Gazebo's `VelocityControl` system is documented
   ambiguously about whether the twist it receives is body- or world-framed. The
   default here is `body`, the ROS `cmd_vel` convention, passed through
   untouched. If the robot drives along fixed world axes instead of following its
   own heading, relaunch with `velocity_frame:=world` and the node will rotate
   the twist by the current yaw first.
2. **Plugin names.** The SDF files use the Ignition namespace
   (`ignition-gazebo-…` / `ignition::gazebo::systems::…`), which is what Fortress
   expects and what Garden still accepts. On Harmonic or newer, replace those
   with `gz-sim-…` and `gz::sim::systems::…`.
3. **Bridge type names.** For the same reason the bridge specs say
   `ignition.msgs.*`. Newer `ros_gz_bridge` wants `gz.msgs.*`.

### Fidelity: what the Gazebo backend does and does not simulate

Real, raycast against actual world geometry:

- The `gpu_lidar` scan. This is what the perception stack consumes, so it is the
  part that matters most.
- The IMU: orientation, angular velocity and linear acceleration from physics.

Kinematic rather than dynamic:

- **Base motion** is driven by `VelocityControl`, not by wheel friction, and the
  wheels are frictionless. There is no wheel slip and no traction limit.
- **Wheel telemetry** in `opencr_state` is reported from the commanded ticks,
  with wheel position integrated over time.

That is not a shortcut taken only here: the MuJoCo model also uses plain cylinder
wheels and applies mecanum kinematics at the command level. **Neither backend
simulates mecanum roller contact**, so neither can show you wheel slip, and a
strafe command is executed exactly rather than approximately. If wheel-level
fidelity ever matters, that is a change to both backends, not a Gazebo bug.

## Scenarios

A scenario is a YAML file listing actors and their motion. `config/sim_scenarios/`
ships nine of them, covering wall/human discrimination, a stopping subject, a
crossing wrong human, and patrol motion.

```yaml
scenario:
  name: single_human_follow
  actors:
    - id: 0
      name: subject
      kind: human
      body_name: sim_human_0     # must be one of SUPPORTED_ACTOR_BODIES
      is_subject: true
      visible_to_lidar: true
      pose: {x: 1.2, y: 0.0, z: 0.0, yaw: 0.0}
      motion:
        mode: patrol             # static | patrol
        speed: 0.35
        loop: true
        start_delay: 2.0
        waypoints:
          - {x: 1.2, y: 1.2, z: 0.0}
```

`body_name` must be one of `sim_human_0`, `sim_human_1`, `sim_wall_panel_0`,
`sim_wall_panel_1` — the loader rejects anything else, because on MuJoCo those
are the mocap bodies that exist in the MJCF. On Gazebo the same names select
which model directory is spawned (`sim_human_0` → `models/sim_human/`).

Motion is dead-reckoned by `SimActorRuntime` on both backends, so
`/sim/actors` is ground truth by construction rather than a physics readback.

## Evaluation nodes

`mecanumbot_sim_detection_evaluator_node` publishes
`mecanumbot_msgs/msg/SimDetectionEvaluation` on `/sim/detection_evaluation`:
whether the subject is tracked, whether a wall was locked onto as a person,
whether the wrong human was picked, nearest-actor identity and distance.

| Parameter | Default | Function |
| --- | --- | --- |
| `subject_match_radius` | `0.55` | Distance within which a detection counts as the subject. |
| `wall_false_positive_radius` | `0.55` | Distance within which a detection counts as a wall false positive. |
| `require_raw_detection` | `true` | Set `false` in truth-twin mode, where the oracle supplies the track and there are no raw detections to require. |
| `evaluation_rate_hz` | `4.0` | Verdict rate. |

`mecanumbot_sim_behavior_evaluator_node` publishes
`mecanumbot_msgs/msg/SimBehaviorEvaluation` on `/sim/behavior_evaluation`:
unsafe motion, proximity violations, wall proximity, wrong target, stale
detections, commanded speed and the relevant distances.

| Parameter | Default | Function |
| --- | --- | --- |
| `moving_speed_threshold` | `0.03` | Speed above which the robot counts as moving. |
| `human_stop_distance` | `0.45` | Closer than this to a human counts as a proximity violation. |
| `wall_stop_distance` | `0.35` | Same for walls. |
| `follow_min_distance` / `follow_max_distance` | `0.55` / `2.00` | Band the robot should hold behind the subject. |

## Node: mecanumbot_sim_nav_shim_node

Stands in for Nav2 so a behaviour tree can be exercised without bringing up
localisation, costmaps and planners. It publishes `/amcl_pose` from odometry and
serves Nav2's two navigation actions for real, driving each goal with a
proportional controller on `/cmd_vel`:

| Action | Behaviour |
| --- | --- |
| `navigate_to_pose` | Drives to the goal pose and takes up its heading. |
| `navigate_through_poses` | Drives the run of waypoints in order, passing each within `waypoint_tolerance` without stopping and finishing on the last one properly. Feedback carries `number_of_poses_remaining`, which is what a waypoint run is followed by. |

They are rclpy action servers, so goal ids, results, cancellation and the
`_action/status` topics the trees watch all come from rclpy rather than being
imitated. `/goal_pose` is still accepted the way Nav2's own navigator accepts
it — the pose is forwarded into `navigate_to_pose` rather than driven
separately, so there is one path through the node however a goal arrives.

| Parameter | Default | Function |
| --- | --- | --- |
| `max_linear_speed` | `0.16` | Forward speed cap. |
| `max_lateral_speed` | `0.10` | Strafe speed cap. |
| `max_angular_speed` | `0.75` | Yaw rate cap. |
| `linear_gain` / `angular_gain` | `0.7` / `1.6` | Proportional gains. |
| `position_tolerance` / `yaw_tolerance` | `0.08` / `0.12` | Goal acceptance thresholds. |
| `waypoint_tolerance` | `0.35` | How near counts as having passed a waypoint on the way through a route. |
| `navigate_to_pose_action` / `navigate_through_poses_action` | `navigate_to_pose` / `navigate_through_poses` | Action names to serve. |
| `enable_dummy_led_service` | `true` | Serve `set_led_status` so the LED tree does not block on a robot-only service. |

The node holds `/cmd_vel` only while it is driving, and stops the robot once
when a goal ends. Nav2 does not sit on the topic while it is idle and neither
may this: the trees turn in place by publishing `/cmd_vel` themselves, and a
shim repeating zeroes at 20 Hz cancels every one of those turns out.

This is a shim, not a navigator: it drives straight at each pose and does not
avoid obstacles. It exists to test tree logic, not navigation.

## Running

Everything goes through the one bringup launch file:

```bash
# MuJoCo, drivers only
ros2 launch mecanumbot_bringup sim.launch.py

# full perception pipeline with the real DR-SPAAM detector and evaluators
ros2 launch mecanumbot_bringup sim.launch.py mode:=perception

# behaviour tree against oracle tracking, no detector error in the loop
ros2 launch mecanumbot_bringup sim.launch.py mode:=behaviour condition:=Doglike

# pick a scenario by name from config/sim_scenarios (or give an absolute path)
ros2 launch mecanumbot_bringup sim.launch.py mode:=perception scenario:=two_humans_wall_discrimination

# same thing on Gazebo
ros2 launch mecanumbot_bringup sim.launch.py backend:=gazebo mode:=perception
```

Watching the verdicts:

```bash
ros2 topic echo /sim/detection_evaluation
ros2 topic echo /sim/behavior_evaluation
```

## Build and test

```bash
colcon build --symlink-install --packages-select mecanumbot_sim
source install/setup.bash
colcon test --packages-select mecanumbot_sim
colcon test-result --verbose
```

`test/` holds the workspace's only real unit tests alongside the usual lint
scaffolding: `test_sim_scenarios.py`, `test_sim_behavior_evaluator.py` and
`test_sim_detection_evaluator.py`. They are pure Python and need no ROS graph, so
they run in well under a second and are the fastest feedback loop here.

## File functions

| Path | Function |
| --- | --- |
| `mecanumbot_sim/kinematics.py` | Mecanum IK and OpenCR tick conversions shared by both backends. |
| `mecanumbot_sim/mujoco_io_node.py` | MuJoCo backend. |
| `mecanumbot_sim/gz_io_node.py` | Gazebo Sim backend and the ROS side of the bridge. |
| `mecanumbot_sim/sim_scenarios.py` | Scenario YAML loader and the supported actor body list. |
| `mecanumbot_sim/sim_actor_runtime.py` | Waypoint-following actor motion, shared by both backends. |
| `mecanumbot_sim/oracle_subject_node.py` | Ground truth republished as `subject_pose`. |
| `mecanumbot_sim/detection_evaluator_node.py` | Perception verdicts. |
| `mecanumbot_sim/behavior_evaluator_node.py` | Motion verdicts. |
| `mecanumbot_sim/visualization_node.py` | RViz markers for actors, detections and verdicts. |
| `mecanumbot_sim/detector_debug_node.py` | Detection-to-truth association markers. |
| `mecanumbot_sim/nav_shim_node.py` | Nav2 stand-in. |
| `launch/gz_backend.launch.py` | Gazebo server, model spawning and bridge. Included by the bringup launch, not run directly. |
| `config/sim_scenarios/*.yaml` | The nine shipped scenarios. |
| `mujoco/mecanumbot.xml` | MJCF model and arena. |
| `worlds/mecanumbot_arena.sdf` | Gazebo arena, geometrically identical to the MJCF one. |
| `models/mecanumbot/` | Gazebo robot model. |
| `models/sim_human/`, `models/sim_wall_panel/` | Gazebo scenario actor models. |

The two arenas are kept geometrically identical on purpose — same 10×10 m floor,
same 4×4 m walled box, same two obstacles — so a scenario can be replayed on
either backend and the results compared.

## Dependencies

ROS dependencies are declared in `package.xml`. The simulators themselves are not
both required: install whichever backend you intend to use.

- MuJoCo backend: `pip install mujoco` (developed against 3.6.0).
- Gazebo backend: `sudo apt install ros-humble-ros-gz`.

`package.xml` lists `ros_gz_sim` and `ros_gz_bridge` unconditionally, so
`rosdep install` will pull Gazebo in even if you only want MuJoCo.
