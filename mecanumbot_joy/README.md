# mecanumbot_joy

The robot's joystick node. Reads `/joy`, applies a YAML mapping profile, and
publishes `/cmd_vel`, `/cmd_accessory_pos` and LED service calls.

This is the **only** `/joy` consumer on the robot. It replaces two earlier
stacks that both read the same topic:

- `mecanumbot_teleop`'s `mecanumbot_joystick`, which ran on the operator PC and
  hardcoded which button did what — only the *indices* were parameters.
- `mecanumbot_gui`'s `controller_node` + `mapping_listener` pair, which routed
  input through custom event messages into a SQLite-backed action interpreter
  mirrored across two machines.

## Design

Everything that decides anything lives in three modules that import **nothing
from rclpy**:

| File | Function |
| --- | --- |
| `layout.py` | One joy frame to named controls: axis normalisation, both trigger conventions, both D-Pad conventions. |
| `profile.py` | Load, validate and normalise a profile document. The single authority on what a valid profile looks like. |
| `actions.py` | The fixed action vocabulary, the press/hold state machine, velocity ramping, the e-stop latch. |
| `joy_node.py` | Thin rclpy adapter: subscribe, publish, serve, own the clock. |

That split is what makes the behaviour unit-testable without a ROS graph — the
same reason `mecanumbot_sim`'s `kinematics.py` is separate from its nodes. Run
`colcon test --packages-select mecanumbot_joy` for 73 tests that need neither a
gamepad nor a robot.

`mecanumbot_web` imports `profile.validate_document` rather than reimplementing
it, so the web GUI cannot save a profile this node would then refuse to load.

## Profiles

Profiles live in `mecanumbot_description/config/joystick/` — `xbox360.yaml`,
`generic.yaml`, `ps4.yaml` — and are edited from the robot's web GUI at
`http://<robot>:8080/joystick`.

A profile has two levels. `axes` / `buttons` / `dpad` describe **the pad**;
`bindings` describes **what the robot does**, referring to controls by name.
Rebinding an action never touches an index, and swapping pads never touches a
semantic. See any shipped profile for the full annotated schema.

### Action vocabulary

Exactly ten actions. A binding naming anything else is a load-time error whose
message lists the valid set.

| Action | Kind | Effect |
| --- | --- | --- |
| `drive` | axis | `linear.x` |
| `strafe` | axis | `linear.y` |
| `turn` | axis | `angular.z` |
| `neck_up` / `neck_down` | button | Ramp the neck while held |
| `neck_preset` | button | Jump the neck to a fixed position |
| `gripper_open` / `gripper_close` | button | Move both grippers, mirrored |
| `estop` | button | Latch an emergency stop |
| `led_preset` | button | Apply a named LED preset |

This replaces the previous system's arbitrary nine-field action tuples with
JSON placeholder substitution. That generality bought a mapping GUI, a SQLite
schema and twelve custom ROS services, and was only ever used to express these
ten things.

## Interfaces

| Direction | Name | Type |
| --- | --- | --- |
| sub | `joy` | `sensor_msgs/Joy` |
| pub | `cmd_vel` | `geometry_msgs/Twist` |
| pub | `cmd_accessory_pos` | `mecanumbot_msgs/AccessMotorCmd` |
| pub | `~/active_profile` | `std_msgs/String` (transient local) |
| pub | `~/estop` | `std_msgs/Bool` (transient local) |
| client | `/mecanumbot/set_led_status` | `mecanumbot_msgs/SetLedStatus` |
| service | `~/reload_profile` | `std_srvs/Trigger` |
| service | `~/clear_estop` | `std_srvs/Trigger` |

Runtime control uses **only standard interfaces** — `std_srvs/Trigger`,
`rcl_interfaces/SetParameters`, `std_msgs` — so consolidating the joystick
stack shrank `mecanumbot_msgs` rather than growing it.

## Parameters

| Parameter | Default | Function |
| --- | --- | --- |
| `profile` | `auto` | Profile stem, or `auto` to fingerprint the pad on the first `/joy` message. Settable at runtime. |
| `profile_dir` | `mecanumbot_description/config/joystick` | Where profiles are loaded from. |

Everything else — speeds, deadzone, ramp rates, publish rate, accessory travel —
lives in the profile, not in ROS parameters, so it can be edited and version
controlled as one document.

## Running

```bash
# with the rest of the onboard stack
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py

# alone
ros2 launch mecanumbot_joy joy_teleop.launch.py

# pick a profile explicitly
ros2 launch mecanumbot_joy joy_teleop.launch.py joystick_profile:=ps4

# switch at runtime
ros2 param set /mecanumbot/mecanumbot_joy_node profile generic

# re-read the file after editing it
ros2 service call /mecanumbot/mecanumbot_joy_node/reload_profile std_srvs/srv/Trigger {}

# release a latched e-stop
ros2 service call /mecanumbot/mecanumbot_joy_node/clear_estop std_srvs/srv/Trigger {}
```

Reload is parse-validate-then-swap: a bad file is rejected with the validation
errors in the `Trigger` response and the previous profile stays live. A reload
never leaves the robot with no bindings.

Testing without a gamepad:

```bash
ros2 topic pub -r 20 /mecanumbot/joy sensor_msgs/msg/Joy \
  "{axes: [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0,0,0,0,0,0,0,0,0,0,0]}"
```

## Behaviour differences from the old teleop node

Deliberate changes, not accidents:

- **Y/A ramp the neck instead of jumping it.** The old node jumped to MAX/MID.
  `neck_preset` keeps a jump-to on a separate button.
- **`cmd_vel` publishes at 20 Hz while driving, then goes quiet**, instead of
  republishing unconditionally at 3 Hz. Accessory positions publish on change
  plus a 1 Hz keepalive, since they are positions rather than velocities.
- **Ramping actually happens.** The old node declared `smoothing.*` parameters
  but the ramp call was commented out, so they did nothing.
- **A `joy_timeout` watchdog stops the wheels** if the pad goes silent. The old
  node held the last commanded velocity indefinitely.
- **Deadzone is applied once.** The old launch file set it on `joy_node` *and*
  again in the teleop node, so the effective value was neither.
- **The output tuple is rebuilt each tick.** The old node aliased its output
  list to its target list, so clamping the output clamped the target too and the
  limit became sticky. `test_actions.py` has a regression test for it.
