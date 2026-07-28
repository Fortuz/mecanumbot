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
| `cmd_vel`           | `geometry_msgs/msg/Twist`            | Converts commanded base velocity to four wheel commands with mecanum kinematics, clamps values, serializes and sends to OpenCR. |
| `cmd_accessory_pos` | `mecanumbot_msgs/msg/AccessMotorCmd` | Converts neck/gripper targets to board units and sends command packet to OpenCR.                                                |

### Behavior

- Opens a serial link to OpenCR (`/dev/opencr` by default on Linux, `COM3` on Windows) and runs a background reader thread. The `/dev/opencr` symlink comes from the udev rules in `mecanumbot_description/udev/`.
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

| Topic / Service      | Direction  | Function                                                                                                    |
| -------------------- | ---------- | ------------------------------------------------------------------------------------------------------------- |
| `cr_battery_state`   | subscribe  | OpenCR main battery voltage.                                                                                |
| `orin_battery_state` | subscribe  | **Jetson only.** Onboard computer rail voltage from the INA219.                                             |
| `cmd_accessory_pos`  | publish    | Nods the neck back and forth while the alert is active.                                                     |
| `set_led_status`     | service call | Sets all four LED panels to fast-blinking red while the alert is active.                                   |
