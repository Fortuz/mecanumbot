# Mecanumbot Monitor package

**Function:** monitor and publish diagnostic state for robot wheels and accessories.

---

## Nodes:

#### wheel_monitor (from `wheel_monitor.py`)

##### **Function:**
Runs a timed loop and publishes `geometry_msgs/Twist` velocity commands on `cmd_vel` to exercise the drivetrain for the GPIO testing of serial command execution in mecanumbot_io_node.

##### **Publishes:**

* `cmd_vel` (`geometry_msgs/Twist`) — periodically publishes increasing linear x velocity values in a looped sequence.

---

#### accessory_monitor (from `accessory_monitor.py`)

##### **Function:**
Runs a timed loop and publishes `mecanumbot_msgs/AccessMotorCmd` commands on `cmd_accessory_pos` to exercise accessory actuators (camera/neck and gripper)  for the GPIO testing of serial command execution in mecanumbot_io_node.

##### **Publishes:**

* `cmd_accessory_pos` (`mecanumbot_msgs/AccessMotorCmd`) — cyclically publishes position commands for `n_pos`, `gl_pos`, and `gr_pos` to drive the accessory mechanisms through a test sequence.

---

