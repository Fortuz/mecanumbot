# mecanumbot_monitor

ROS 2 utility nodes that generate deterministic command patterns for drivetrain and accessory testing.

| Executable | Function |
| --- | --- |
| `wheel_monitor` | Drivetrain command pattern (below). |
| `accessory_monitor` | Neck/gripper command pattern (below). |
| `rosbag_stepper` (alias `playback_stepper`) | Steps `ros2 bag play` in 3 s chunks on ENTER through `/rosbag2_player/pause` and `/rosbag2_player/resume`. |
| `metric_eval` (alias `metric_logger`) | Logs RViz `/goal_pose` clicks (transformed into `mecanumbot/base_scan`) as ground truth against `/mecanumbot/dets` to `ground_truth.csv` and `predictions.csv` in the working directory. |

`launch/rosbag_monitor_eval.launch.py` starts `rosbag_stepper` and `metric_eval` together.

## Node: `wheel_monitor`

### Publishers

| Topic     | Data type                 | Function                                                                                                                 |
| --------- | ------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Publishes a timed velocity ramp on linear `x` to exercise base command path and low-level command transmission behavior. |

### Behaviour

- Alternates between idle and publishing phases in a fixed loop count (6 loops on a 0.5 s timer, the speed rising by 0.01 m/s per tick).
- Designed for repeatable drivetrain command testing and logging.

## Node: `accessory_monitor`

### Publishers

| Topic               | Data type                            | Function                                                                                                  |
| ------------------- | ------------------------------------ | --------------------------------------------------------------------------------------------------------- |
| `cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Publishes staged accessory position commands (camera/neck and grippers) for repeatable actuation testing. |

### Behavior

- Cycles through predefined accessory poses (camera up/down, gripper open/close) with idle periods between bursts.
- Useful for validating command path timing in `mecanumbot_io_node`.
