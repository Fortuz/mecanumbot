# mecanumbot_monitor

ROS 2 utility nodes that generate deterministic command patterns for drivetrain and accessory testing.

## Node: `wheel_monitor`

### Publishers
| Topic | Data type | Function |
|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/Twist` | Publishes a timed velocity ramp on linear `x` to exercise base command path and low-level command transmission behavior. |

### Behaviour
- Alternates between idle and publishing phases in a fixed loop count.
- Designed for repeatable drivetrain command testing and logging.

## Node: `accessory_monitor`

### Publishers
| Topic | Data type | Function |
|---|---|---|
| `cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Publishes staged accessory position commands (camera/neck and grippers) for repeatable actuation testing. |

### Behavior
- Cycles through predefined accessory poses (camera up/down, gripper open/close) with idle periods between bursts.
- Useful for validating command path timing in `mecanumbot_io_node`.