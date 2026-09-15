# mecanumbot_core

ROS 2 nodes for low-level communication with OpenCR and conversion of raw board data into standard ROS sensor/state topics.

Runs on the robot's NVIDIA Jetson Orin Nano. The nodes read `/proc/device-tree/model` at import time and enable the Jetson-only parts (INA219 onboard power monitoring) when a Jetson is detected, so the same code still starts on a development machine with those features skipped.

## Node: `mecanumbot_io_node`

### Publishers

| Topic          | Data type                         | Function                                                                                               |
| -------------- | --------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `opencr_state` | `mecanumbot_msgs/msg/OpenCRState` | Publishes decoded OpenCR packet content (wheel velocities, positions, currents, errors, IMU, battery). |

### Subscribers

| Topic               | Data type                            | Processing                                                                                                                      |
| ------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `cmd_vel`           | `geometry_msgs/msg/Twist`            | Converts commanded base velocity to four wheel commands with mecanum kinematics and clamps them into the current command set. Subscribed at **depth 1** — a velocity stream only ever wants its newest sample, and a deeper queue shows up as the wheels obeying commands the operator gave a moment ago. |
| `cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Converts neck/gripper targets to board units into the current command set.                                                       |

### Parameters

| Parameter          | Default | Function                                                                              |
| ------------------ | ------- | --------------------------------------------------------------------------------------- |
| `dev_params.tx_hz` | `50.0`  | Rate at which the current command set is written to the board. See *Command transmission*. |
| `robot_params.wheel.max_cmd_ticks` | `300` | Ceiling on a single wheel command, in board ticks. Also the lever for peak current draw: four wheels accelerating to this at once is the worst case the supply sees. |

### Holding the wheels when the bus goes silent

The board reports `err_* = -1` when it could not read a wheel, which means every
other wheel field in the packet is a stale sample rather than a reading (see
`Mecanumbot_OpenCR/README.md` in the microcontroller repo). While that is true the
tx timer sends zero wheel velocities: commanding the wheels while the board cannot
hear them is driving blind, and on this robot a silent bus has meant a sagging
supply. It recovers on its own as soon as reads come back — there is no latch and
nothing to reset.

Only the wheels are held. Accessory positions pass through unchanged, because
zeroing a *position* command would slam the neck to 0, and the zeroed stop frame on
shutdown always goes out.

A latched Input Voltage Error (`err_* & 1`) is warned about but **not** acted on.
The motors keep driving with it set, the flag stays until they are power-cycled, and
stopping a trial on a latched flag would cost more than it saves.

### Command transmission

Neither subscription writes to the serial port. Both only update `cmd_outputs`; a dedicated tx timer packs the whole 7-short frame and writes it at `dev_params.tx_hz`.

This split matters for latency. The frame is absolute state, so sending the latest snapshot at a fixed rate carries exactly the same information as sending one frame per message — but writing from the callbacks made the achievable serial rate a hard cap on the accepted `cmd_vel` rate. With a 100 ms sleep in that path the node could only retire ~10 messages/s against a 20 Hz publisher, so the subscription queue stayed saturated and the wheels ran roughly a second behind the stick. Keep the write off the callback thread.

The timer stays silent until the first command arrives, so merely starting the node does not drive the accessories to their defaults.

### Behavior

- Opens a serial link to OpenCR (`/dev/opencr` by default on Linux, `COM3` on Windows) and runs a background reader thread. The `/dev/opencr` symlink comes from the udev rules in `mecanumbot_description/udev/`.
- On shutdown, `close_serial` cancels the tx timer, forces a zeroed stop frame, then joins the reader and closes the port.
- Uses CRC8-CCITT and payload plausibility checks before accepting inbound packets.
- Contains a disabled GPIO timing/debug signal (pin 27). It was driven by `wiringpi` on the earlier Raspberry Pi build; `wiringpi` is not available on the Jetson Orin Nano and the calls are commented out, so nothing is toggled at runtime.

## Node: `mecanumbot_sensorproc_node`

### Publishers

| Topic             | Data type                            | Function                                                                                       |
| ----------------- | ------------------------------------ | ---------------------------------------------------------------------------------------------- |
| `odom`            | `nav_msgs/msg/Odometry`              | Publishes base odometry computed from wheel velocities (or IMU orientation when enabled).      |
| `imu`             | `sensor_msgs/msg/Imu`                | Publishes IMU orientation, angular velocity, and linear acceleration from OpenCR state.        |
| `joint_states`    | `sensor_msgs/msg/JointState`         | Publishes wheel/accessory joint state for visualization and robot state publisher consumption. |
| `cr_battery_state`   | `sensor_msgs/msg/BatteryState`    | Publishes main battery voltage and estimated charge percentage/capacity state, as reported by OpenCR. |
| `orin_battery_state` | `sensor_msgs/msg/BatteryState`    | **Jetson only.** Publishes bus voltage and current read from an INA219 over I2C. Created only when the device tree reports a Jetson, and published only when the sensor was found on the bus. |
| `has_object`      | `std_msgs/msg/Bool`                  | Publishes whether the gripper distance sensor reports a held object.                           |
| `/tf` (broadcast) | `geometry_msgs/msg/TransformStamped` | Broadcasts `odom -> base_footprint` transform from integrated odometry pose.                   |

### Subscribers

| Topic          | Data type                         | Processing                                                                                                  |
| -------------- | --------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `opencr_state` | `mecanumbot_msgs/msg/OpenCRState` | Stores the latest board state and uses it in periodic processing to produce odom/imu/joint/battery outputs. |

### Behavior

- INA219 access uses `adafruit_ina219` over `busio.I2C(board.SCL, board.SDA)`. If the sensor is missing or the bus is not accessible, the node logs once and keeps running without `orin_battery_state`.

## Node: `mecanumbot_battery_alert`

Watches battery voltages and raises a visible alert when one drops below the threshold (`battery_threshold`, default `9.7` V).

The alarm needs `ALERT_AFTER_SAMPLES` (5) **consecutive** readings below the
threshold, and clears the LEDs when the voltage recovers. It clears them to off
rather than restoring whatever a trial had set — the node does not know that, and
the alarm itself already overwrites it. Worth remembering for the LED condition of
the leading experiment: a low battery will take the lights over mid-trial.

| Topic / Service      | Direction  | Function                                                                                                    |
| -------------------- | ---------- | ------------------------------------------------------------------------------------------------------------- |
| `cr_battery_state`   | subscribe  | OpenCR main battery voltage.                                                                                |
| `orin_battery_state` | subscribe  | **Jetson only.** Onboard computer rail voltage from the INA219.                                             |
| `cmd_accessory_pos`  | publish    | Nods the neck back and forth while the alert is active.                                                     |
| `set_led_status`     | service call | Sets all four LED panels to fast-blinking red while the alert is active.                                   |

## Node: `mecanumbot_scan_grid_node`

Republishes the lidar scan on a **fixed angular grid**, for slam_toolbox. Started by
the launch files that start slam_toolbox: `mecanumbot_bringup/mapping.launch.py`
(and so the simulator's `mapping` mode) and `mecanumbot_autoslam/launch_autoslam.launch.py`.

| Topic | Direction | Function |
| --- | --- | --- |
| `/mecanumbot/scan` (`input_topic`) | subscribe | The LD08 driver's scan. |
| `/mecanumbot/scan_grid` (`output_topic`) | publish | The same revolution on `bins` sectors (default 200, 1.8°) from angle 0. Same stamp and frame. |

**Why it exists.** The LD08 publishes every revolution with its own `angle_min`,
`angle_max` and `angle_increment`, and with 201 to 206 readings. slam_toolbox (Karto)
builds its laser model from the **first** scan it receives and drops every later
scan with a different reading count. Which scan comes first changes per launch, so a
mapping run kept between half its scans and almost none. Measured on the robot on
2026-09-15: about one launch in seven kept 2%, and slam_toolbox's map was still
0 × 0 after five minutes. Nav2's global costmap takes its size from that map, so it
fell back to a 5 × 5 m square at the origin and logged `Robot is out of bounds of
the costmap!`.

**What it does to the data.** Each sector keeps its *nearest* valid return, and a
sector with no valid return is `inf`. The sectors are wider than the driver's
~1.76° spacing, so a revolution leaves none empty. On 224 live scans every output had
200 readings, and exactly as many sectors were empty (11.7%) as raw readings were
no-returns. Costmaps, perception and the Deep3R client still read the driver's own
topic. The grid function is `scan_grid.py`, tested in `test/test_scan_grid.py`
(`PYTHONPATH=. python3 -m pytest test/test_scan_grid.py`, no ROS).
