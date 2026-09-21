import rclpy
from mecanumbot_msgs.msg import OpenCRState
from mecanumbot_msgs.srv import SetLedStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from mecanumbot_core.alerts import LEDS_OFF, Debounce, choose_alarm, dxl_bus_silent


def get_device_model():
    try:
        with open("/proc/device-tree/model", "r") as f:
            return f.read().strip().lower()
    except FileNotFoundError:
        return ""


MODEL = get_device_model()

# Consecutive readings below the threshold before the alarm is raised.
ALERT_AFTER_SAMPLES = 5

if "raspberry pi" in MODEL:
    print("Running on Raspberry Pi")
elif "nvidia jetson" in MODEL:
    print("Running on Jetson")
else:
    print("Unknown device:", MODEL)


class MecanumbotBatteryAlert(Node):
    """
    Show the robot's hardware alarms on the LEDs.

    Two of them: a low battery (red fast blink) and a Dynamixel bus the OpenCR
    cannot read (yellow fast blink) -- the wheels then neither report nor, as a
    rule, drive, which from outside looks like the robot ignoring its commands.
    """

    def __init__(self):
        super().__init__("mecanumbot_battery_alert")
        self.get_logger().info("MecanumbotBatteryAlert node has started.")

        # Get battery threshold from parameter or use default
        self.declare_parameter("battery_threshold", 9.7)  # Default threshold in volts
        # Seconds the bus has to stay silent before the alarm goes up, and has to
        # stay readable before it comes down again.
        self.declare_parameter("dxl_bus_alert_after", 1.0)
        self.declare_parameter("dxl_bus_clear_after", 1.0)

        timer_period = 1  # seconds
        self.callback_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.timer_callback, callback_group=self.callback_group
        )
        self.srv_client = self.create_client(SetLedStatus, "set_led_status")

        # Read the parameter rather than shadowing it: the declared value was
        # being overwritten by a hardcoded copy, so the parameter did nothing.
        self.battery_threshold = self.get_parameter("battery_threshold").value
        self.alert = False
        self.alert_num = {"opencr": 0, "orin": 0}
        self.dxl_bus = Debounce(
            self.get_parameter("dxl_bus_alert_after").value,
            self.get_parameter("dxl_bus_clear_after").value,
        )
        # The alarm on the LEDs now, so clearing it is sent once, not every second.
        self.shown = None

        self.opencr_state_subscription = self.create_subscription(
            OpenCRState,
            "opencr_state",
            self.opencr_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.cr_battery_subscription = self.create_subscription(
            BatteryState,
            "cr_battery_state",
            lambda msg: self.batterystate_callback(msg, "opencr"),
            10,
            callback_group=self.callback_group,
        )
        if MODEL and "nvidia jetson" in MODEL:
            self.orin_battery_subscription = self.create_subscription(
                BatteryState,
                "orin_battery_state",
                lambda msg: self.batterystate_callback(msg, "orin"),
                10,
                callback_group=self.callback_group,
            )

    def set_leds(self, color, mode):
        """Drive all four LED groups to one colour and mode."""
        req = SetLedStatus.Request()
        req.fl_color, req.fr_color, req.bl_color, req.br_color = (color,) * 4
        req.fl_mode, req.fr_mode, req.bl_mode, req.br_mode = (mode,) * 4
        self.pending_future = self.srv_client.call_async(req)

    def timer_callback(self):
        """Re-send the alarm every second, or clear the LEDs once it is gone."""
        alarm = choose_alarm(self.alert, self.dxl_bus.active)
        if alarm is not None:
            self.set_leds(*alarm)
        elif self.shown is not None:
            # Clear to off rather than restoring whatever a trial had set, which
            # the node does not know; the alarm itself already clobbers that.
            self.set_leds(*LEDS_OFF)
        self.shown = alarm

    def opencr_state_callback(self, msg):
        """Watch the wheels' error fields for the board's no-answer sentinel."""
        was_up = self.dxl_bus.active
        now = self.get_clock().now().nanoseconds * 1e-9
        errors = (msg.err_bl, msg.err_br, msg.err_fl, msg.err_fr)
        if self.dxl_bus.update(dxl_bus_silent(errors), now) != was_up:
            if self.dxl_bus.active:
                self.get_logger().error(
                    "Dynamixel bus silent: the OpenCR cannot read the wheels. "
                    "LEDs blink yellow. Check DXL power and cabling."
                )
            else:
                self.get_logger().info("Dynamixel bus answering again.")

    def batterystate_callback(self, msg, battery):
        voltage = msg.voltage
        if voltage < self.battery_threshold:
            self.get_logger().warn(
                f"{battery} battery low: {voltage:.2f} V", throttle_duration_sec=5.0
            )
            self.alert_num[battery] += 1
            if self.alert_num[battery] > ALERT_AFTER_SAMPLES:
                self.alert = True
        else:
            # Count consecutive low samples, not low samples ever seen. Without
            # this the counter latched past the threshold on the first flat
            # battery of the session and every later dip alarmed instantly.
            # Cleared by the timer, which knows whether another alarm is up.
            self.alert_num[battery] = 0
            self.alert = False


def main(args=None):
    rclpy.init(args=args)
    mecanumbot_battery_alert = MecanumbotBatteryAlert()
    executor = MultiThreadedExecutor()
    executor.add_node(mecanumbot_battery_alert)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mecanumbot_battery_alert.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
