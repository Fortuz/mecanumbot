import rclpy
from mecanumbot_msgs.srv import SetLedStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


def get_device_model():
    try:
        with open("/proc/device-tree/model", "r") as f:
            return f.read().strip().lower()
    except FileNotFoundError:
        return ""


MODEL = get_device_model()

# SetLedStatus vocabulary, from mecanumbot_led.
BLACK, RED = 0, 3
SOLID, FAST_BLINK = 4, 5

# Consecutive readings below the threshold before the alarm is raised.
ALERT_AFTER_SAMPLES = 5

if "raspberry pi" in MODEL:
    print("Running on Raspberry Pi")
elif "nvidia jetson" in MODEL:
    print("Running on Jetson")
else:
    print("Unknown device:", MODEL)


class MecanumbotBatteryAlert(Node):
    def __init__(self):
        super().__init__("mecanumbot_battery_alert")
        self.get_logger().info("MecanumbotBatteryAlert node has started.")

        # Get battery threshold from parameter or use default
        self.declare_parameter("battery_threshold", 9.7)  # Default threshold in volts

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
        if self.alert:
            self.set_leds(RED, FAST_BLINK)

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
            self.alert_num[battery] = 0
            if self.alert:
                self.alert = False
                # Clear the alarm instead of re-sending it -- this branch used
                # to repeat the same red fast-blink, so the LEDs stayed in the
                # alarm state after the voltage recovered. It clears to off
                # rather than restoring whatever a trial had set, which the node
                # does not know; the alarm itself already clobbers that.
                self.set_leds(BLACK, SOLID)


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
