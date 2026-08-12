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

        self.battery_threshold = 9.7
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

    def timer_callback(self):
        if self.alert:
            req = SetLedStatus.Request()
            req.fl_color, req.fr_color, req.bl_color, req.br_color = 3, 3, 3, 3  # red
            req.fl_mode, req.fr_mode, req.bl_mode, req.br_mode = (
                5,
                5,
                5,
                5,
            )  # fast blink
            self.pending_future = self.srv_client.call_async(req)

    def batterystate_callback(self, msg, battery):
        voltage = msg.voltage
        if voltage < self.battery_threshold:
            self.get_logger().warn(f"{battery} battery low: {voltage:.2f} V")
            self.alert_num[battery] += 1
            if self.alert_num[battery] > 5:
                self.alert = True
        if self.alert:
            if voltage > self.battery_threshold:
                self.alert = False
                req = SetLedStatus.Request()
                req.fl_color, req.fr_color, req.bl_color, req.br_color = (
                    3,
                    3,
                    3,
                    3,
                )  # red
                req.fl_mode, req.fr_mode, req.bl_mode, req.br_mode = (
                    5,
                    5,
                    5,
                    5,
                )  # fast blink
                self.pending_future = self.srv_client.call_async(req)


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
