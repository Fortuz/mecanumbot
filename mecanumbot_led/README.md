# mecanumbot_led

ROS 2 LED control service node for the robot LED controller (an Arduino Nano microcontroller, not to be confused with the Jetson Orin Nano the node itself runs on).

The node runs on the robot's Jetson Orin Nano and talks to the Arduino over USB serial at `/dev/arduino_nano`. That symlink is created by `mecanumbot_description/udev/90_mecanumbot_nano.rules`, which must be installed before the node can open the port.

## Node: `mecanumbot_led_service_node`

Executable: `mecanumbot_led_service`. The node registers as `mecanumbot_led_service_node`; the base launch starts it as `/mecanumbot/mecanumbot_led_service`, so its services resolve to `/mecanumbot/set_led_status` and `/mecanumbot/get_led_status` there.

### Services handled

| Service          | Type                               | Behavior                                                                                         |
| ---------------- | ---------------------------------- | ------------------------------------------------------------------------------------------------ |
| `set_led_status` | `mecanumbot_msgs/srv/SetLedStatus` | Builds a command packet with mode/color per panel and writes it to serial (`/dev/arduino_nano`). |
| `get_led_status` | `mecanumbot_msgs/srv/GetLedStatus` | Sends a status request byte, parses serial feedback frame, and returns current LED state fields. |

### Behavior

- Uses a custom byte protocol with start byte, checksum, and fixed frame layout (start `0xAA`, eight mode/colour bytes, the progress fill and its colour, XOR checksum; status request `0xAC`, feedback start `0xAB`).
- Performs serial parsing and checksum validation for returned LED feedback.
- Communicates directly with Arduino Nano at `115200` baud.

### Progress fill

`SetLedStatus` carries `progress` and `progress_color` beside the four panels. `progress`
is how many of a panel's 8 LEDs are drawn in `progress_color` instead of the panel's own
colour (0 is no fill, 8 the whole panel); it is the same on all four panels, and the
panel's mode runs over both colours, so a wave stays one wave with a bar of another
colour inside it. The LED condition of the leading study fills the panels this way as
the robot nears its target.

The fill is drawn by the firmware (`mecanumbot_microcontrollers/Mecanumbot_Nano_LED`),
which grows it towards the front of the robot on every strip: `FILL_FROM_LAST_*` in
`LEDutils.h` says which end that is per panel. The two bytes it travels in used to hold
a duration the firmware never read, so the frame is the same 12 bytes: firmware that
predates the fill ignores it, and the new firmware treats an old sender's duration as
"no fill". `get_led_status` does not report the fill.

## LED value reference

| Mode         | Value |
| ------------ | ----- |
| `WAVE_RIGHT` | `1`   |
| `WAVE_LEFT`  | `2`   |
| `PULSE`      | `3`   |
| `SOLID`      | `4`   |
| `FAST_BLINK` | `5`   |
| `SLOW_BLINK` | `6`   |

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
