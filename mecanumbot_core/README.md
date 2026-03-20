# mecanumbot_core

ROS 2 nodes for low-level communication with OpenCR and conversion of raw board data into standard ROS sensor/state topics.

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

- Opens a serial link to OpenCR (`/dev/ttyACM0` by default on Linux) and runs a background reader thread.
- Uses CRC8-CCITT and payload plausibility checks before accepting inbound packets.
- Toggles GPIO pin 27 via `wiringpi` on outgoing command activity (timing/debug signal for command transmission).

## Node: `mecanumbot_sensorproc_node`

### Publishers

| Topic             | Data type                            | Function                                                                                       |
| ----------------- | ------------------------------------ | ---------------------------------------------------------------------------------------------- |
| `odom`            | `nav_msgs/msg/Odometry`              | Publishes base odometry computed from wheel velocities (or IMU orientation when enabled).      |
| `imu`             | `sensor_msgs/msg/Imu`                | Publishes IMU orientation, angular velocity, and linear acceleration from OpenCR state.        |
| `joint_states`    | `sensor_msgs/msg/JointState`         | Publishes wheel/accessory joint state for visualization and robot state publisher consumption. |
| `battery_state`   | `sensor_msgs/msg/BatteryState`       | Publishes battery voltage and estimated charge percentage/capacity state.                      |
| `/tf` (broadcast) | `geometry_msgs/msg/TransformStamped` | Broadcasts `odom -> base_footprint` transform from integrated odometry pose.                   |

### Subscribers

| Topic          | Data type                         | Processing                                                                                                  |
| -------------- | --------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `opencr_state` | `mecanumbot_msgs/msg/OpenCRState` | Stores the latest board state and uses it in periodic processing to produce odom/imu/joint/battery outputs. |
