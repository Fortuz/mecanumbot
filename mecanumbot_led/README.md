# mecanumbot_led

ROS 2 LED control service node for the robot LED controller (Arduino Nano over serial).

## Node: `mecanumbot_led_service_node`

### Services handled

| Service          | Type                               | Behavior                                                                                         |
| ---------------- | ---------------------------------- | ------------------------------------------------------------------------------------------------ |
| `set_led_status` | `mecanumbot_msgs/srv/SetLedStatus` | Builds a command packet with mode/color per panel and writes it to serial (`/dev/arduino_nano`). |
| `get_led_status` | `mecanumbot_msgs/srv/GetLedStatus` | Sends a status request byte, parses serial feedback frame, and returns current LED state fields. |

### Behavior

- Uses a custom byte protocol with start byte, checksum, and fixed frame layout.
- Performs serial parsing and checksum validation for returned LED feedback.
- Communicates directly with Arduino Nano at `115200` baud.

## LED value reference

| Mode         | Value |
| ------------ | ----- |
| `WAVE_RIGHT` | `1`   |
| `WAVE_LEFT`  | `2`   |
| `PULSE`      | `3`   |
| `SOLID`      | `4`   |

| Color    | Value |
| -------- | ----- |
| `BLACK`  | `0`   |
| `WHITE`  | `1`   |
| `GREEN`  | `2`   |
| `RED`    | `3`   |
| `BLUE`   | `4`   |
| `CYAN`   | `5`   |
| `PINK`   | `6`   |
| `YELLOW` | `7`   |
