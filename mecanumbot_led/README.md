# Mecanumbot LED package

**Function:** control the robot’s status LEDs via ROS.

---

## Nodes:

#### mecanumbot_led_control_node:

##### **Function:**
Listens for LED command messages and updates the OpenCR board LEDs accordingly.

##### **Subscribers:**

* ***cmd_led*** (or similar): LED command message (e.g., color, blink pattern, on/off) for the robot’s status LEDs.

##### **Publishers:**

* (none by default; the node acts on incoming LED commands)

---// filepath: /home/csenge/Documents/mecanumbot_ws/src/mecanumbot/mecanumbot_led/README.md
# Mecanumbot LED package

**Function:** control the robot’s LED panels via ROS service.

---

## Nodes:

#### mecanumbot_led_control_node:

##### **Service protocol (from `led_control_node.py`):**

The node exposes two ROS services to control the robot's LEDs

* ***set_led_status*** (service type mecanumbot_msgs/srv/SetLedStatus): sends an LED command packet to the Arduino Nano over serial.

* ***get_led_status***(service type mecanumbot_msgs/srv/GetLedStatus): requests the current LED state from the Arduino Nano and returns it.


##### **Service protocol syntax**

ros2 service call get_led_status mecanumbot_msgs/GetLedStatus

ros2 service call /set_led_status mecanumbot_msgs/srv/SetLedStatus "{fl_mode: FLM, fl_color: FLC, fr_mode: FLM, fr_color: FRC, bl_mode: BLM, bl_color: BLC, br_mode: BRM, br_color: BRC}"

##### **Setting options**

| Modes | Value |
|:--- |:---:|
| WAVE_RIGHT | 1 |
| WAVE_LEFT  | 2 |
| PULSE      | 3 |
| SOLID      | 4 |

| Colors | Value |
|:--- |:---:|
| BLACK   | 0 |
| WHITE   | 1 |
| GREEN   | 2 |
| RED     | 3 |
| BLUE    | 4 |
| CYAN    | 5 |
| PINK    | 6 |
| YELLOW  | 7 |
---