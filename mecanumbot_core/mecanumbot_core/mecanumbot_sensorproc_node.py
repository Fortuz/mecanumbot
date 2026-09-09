import math

import rclpy
from geometry_msgs.msg import TransformStamped
from mecanumbot_msgs.msg import OpenCRState
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# from tf_transformations import quaternion_from_euler, euler_from_quaternion # You may need to install 'ros-humble-tf-transformations'
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time as RosTime
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat, quat2euler

TICK_TO_RAD = 0.005061
# A backwards stamp step larger than this is a real clock discontinuity (NTP
# correcting the Jetson, which has no RTC, or the board restarting its stamp),
# not callback jitter, so the timeline is re-based on it instead of waited out.
CLOCK_STEP_TOLERANCE_NS = 500_000_000
# The INA219 read is two blocking I2C transactions. At 50 Hz it is both wasteful
# and the one thing in the tick that can overrun the timer period.
ORIN_BATTERY_PERIOD_TICKS = 50
GRIPPER_MIDPOINT_COMPENSATE_CONSTANT = 2.618  # 150 deg diff in rads
NECK_MIDPOINT_COMPENSATE_CONSTANT = 3.8172  # 218 deg diff in rads


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

if MODEL and "nvidia jetson" in MODEL:
    import adafruit_ina219
    import board
    import busio


################################################ MAIN CLASS ################################################
class Mecanumbot_Sensorproc_Node(Node):

    def __init__(self, namespace=""):

        super().__init__("mecanumbot_sensorproc_node", namespace=namespace)
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameters(
            namespace=namespace,
            parameters=[
                (
                    "robot_params.wheel.vel_tick",
                    0.229,
                ),  # meaning of one tick between velocity values [rot/min]
                ("robot_params.wheel.radius", 0.0325),  # radius [m]
                (
                    "robot_params.wheel.sep_x",
                    0.129,
                ),  # distance between front and back wheels [m]
                (
                    "robot_params.wheel.sep_y",
                    0.300,
                ),  # distance between left and right wheels [m]
                (
                    "robot_params.battery.min_voltage",
                    9.6,
                ),  # minimum voltage of battery [V]
                (
                    "robot_params.battery.max_voltage",
                    12.6,
                ),  # maximum voltage of battery [V]
                ("odom_params.frame_id", "odom"),
                ("odom_params.child_frame_id", "base_footprint"),
                ("odom_params.from_imu", False),
                ("imu_params.frame_id", "imu_link"),
                ("use_state_stamp_for_dt", False),
                ("require_state_stamp", False),
                ("has_object_threshold", 250.0),
            ],
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        resolved_namespace = self.get_namespace().strip("/")
        self.namespace = resolved_namespace

        # Odom parameters
        self.odom_from_imu = self.get_parameter("odom_params.from_imu").value
        self.odom_frame_id = self.get_parameter("odom_params.frame_id").value
        self.odom_child_frame_id = self.get_parameter(
            "odom_params.child_frame_id"
        ).value
        self.imu_frame_id = self.get_parameter("imu_params.frame_id").value
        self.use_state_stamp_for_dt = bool(
            self.get_parameter("use_state_stamp_for_dt").value
        )
        self.require_state_stamp = bool(self.get_parameter("require_state_stamp").value)

        if self.namespace != "" and self.namespace is not None:

            self.odom_frame_id = self.namespace + "/" + self.odom_frame_id
            self.odom_child_frame_id = self.namespace + "/" + self.odom_child_frame_id
            self.imu_frame_id = self.namespace + "/" + self.imu_frame_id
        self.get_logger().info(f"Namespace: {self.namespace}")
        self.get_logger().info(
            f"Odom Frame ID: {self.odom_frame_id}, Child Frame ID: {self.odom_child_frame_id}, IMU Frame ID: {self.imu_frame_id}"
        )
        # Robot parameters
        self.vel_tick = (
            self.get_parameter("robot_params.wheel.vel_tick").value / 60
        )  # rot/min to rot/s
        self.wheel_radius = self.get_parameter("robot_params.wheel.radius").value  # m
        self.wheel_sep_x = self.get_parameter("robot_params.wheel.sep_x").value  # m
        self.wheel_sep_y = self.get_parameter("robot_params.wheel.sep_y").value
        self.battery_min_voltage = self.get_parameter(
            "robot_params.battery.min_voltage"
        ).value  # V
        self.battery_max_voltage = self.get_parameter(
            "robot_params.battery.max_voltage"
        ).value  # V

        self.scale = (
            self.vel_tick * 2 * math.pi * self.wheel_radius
        )  # tick - unit diff of wheel velocoties in rpm, 2Rpi - distance/rotation, wheel_radius - m
        self.wheel_dist_scale = (self.wheel_sep_x + self.wheel_sep_y) / 2

        self.has_object_threshold = self.get_parameter("has_object_threshold").value

        # Initialize messages

        self.cr_state = OpenCRState()
        self.odom = Odometry()
        self.imu = Imu()
        self.transform = TransformStamped()
        self.joint_state = JointState()
        self.cr_battery_state = BatteryState()
        self.orin_battery_state = BatteryState()

        self.dms_buffer = [0] * 11
        self.has_object = False
        # Publishers

        self.odom_publisher = self.create_publisher(
            Odometry, "odom", 10, callback_group=self.callback_group
        )
        self.imu_publisher = self.create_publisher(
            Imu, "imu", 10, callback_group=self.callback_group
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10, callback_group=self.callback_group
        )
        self.cr_battery_state_publisher = self.create_publisher(
            BatteryState, "cr_battery_state", 10, callback_group=self.callback_group
        )
        self.object_state_publisher = self.create_publisher(
            Bool, "has_object", 10, callback_group=self.callback_group
        )

        if MODEL and "nvidia jetson" in MODEL:
            self.orin_battery_state_publisher = self.create_publisher(
                BatteryState,
                "orin_battery_state",
                10,
                callback_group=self.callback_group,
            )

        timer_period = 0.02  # seconds
        # The tick MUST NOT be reentrant. It advances a shared timeline and then
        # stamps every message from it, so two ticks overlapping (which the
        # MultiThreadedExecutor will happily do whenever one overruns the 20 ms
        # period) interleaves those two steps and publishes two samples carrying
        # the same stamp. Cartographer CHECK-fails on a non-increasing odometry
        # stamp and aborts the whole mapping run, so keep the timer serialized.
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.timer_callback, callback_group=self.timer_callback_group
        )

        self.board_subscription = self.create_subscription(
            OpenCRState,
            "opencr_state",
            self.crstate_callback,
            10,
            callback_group=self.callback_group,
        )
        self.board_subscription  # prevent unused variable warning

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.dt = 0.0  # [s]
        self.last_state_stamp_ns = None
        self.last_stamp_source = None
        self.tick_count = 0

        self.last_yaw_angle = 0.0
        self.wrote_error_once = False

        if MODEL and "nvidia jetson" in MODEL:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.ina_sensor = adafruit_ina219.INA219(self.i2c)
                self.get_logger().info("INA219 sensor initialized successfully.")
            except Exception as e:
                self.get_logger().info(f"INA219 sensor not found: {e}")

    def crstate_callback(self, data):
        self.cr_state = data

    def timer_callback(self):
        if not rclpy.ok():
            return

        stamp = self.cr_state.header.stamp
        if self.use_state_stamp_for_dt and (stamp.sec != 0 or stamp.nanosec != 0):
            raw_time = RosTime.from_msg(stamp)
            stamp_source = "state"
        else:
            if self.use_state_stamp_for_dt and self.require_state_stamp:
                return
            raw_time = self.get_clock().now()
            stamp_source = "clock"

        raw_stamp_ns = raw_time.nanoseconds
        if (
            self.last_stamp_source is not None
            and self.last_stamp_source != stamp_source
        ):
            self.last_state_stamp_ns = None

        if self.last_state_stamp_ns is None:
            current_stamp_ns = raw_stamp_ns
            self.dt = 0.0
        elif raw_stamp_ns > self.last_state_stamp_ns:
            current_stamp_ns = raw_stamp_ns
            self.dt = (current_stamp_ns - self.last_state_stamp_ns) * 1e-9
        elif self.last_state_stamp_ns - raw_stamp_ns > CLOCK_STEP_TOLERANCE_NS:
            # A genuine clock step. Follow it: every other node's stamps (scan
            # above all) jumped too, so pinning ours to the old timeline is what
            # would make this node inconsistent with the rest of the graph.
            self.get_logger().warn(
                "Clock stepped backwards by "
                f"{(self.last_state_stamp_ns - raw_stamp_ns) * 1e-9:.3f} s "
                f"on the {stamp_source} timeline; re-basing odometry."
            )
            current_stamp_ns = raw_stamp_ns
            self.dt = 0.0
        else:
            # Time did not advance: a repeated OpenCR stamp, or small clock
            # jitter. Skip the tick entirely rather than clamping it forward.
            # Clamping used to emit a *duplicate* stamp, which is what killed
            # cartographer_node ("Check failed: data.time > ...prev") and what
            # makes tf2 log TF_REPEATED_DATA. The next tick is 20 ms away and
            # dt keeps accumulating from last_state_stamp_ns, so nothing drifts.
            return

        self.last_time = self.current_time
        self.current_time = RosTime(nanoseconds=current_stamp_ns)
        self.last_state_stamp_ns = current_stamp_ns
        self.last_stamp_source = stamp_source

        self.set_odom()
        self.set_imu()
        self.set_joint_state()
        self.set_cr_battery_state()
        self.set_object_state()
        self.tick_count += 1
        orin_battery_due = self.tick_count % ORIN_BATTERY_PERIOD_TICKS == 0
        if MODEL and "nvidia jetson" in MODEL:
            if hasattr(self, "ina_sensor"):
                if orin_battery_due:
                    self.set_orin_battery_state()
            else:
                if not self.wrote_error_once:
                    self.get_logger().info(
                        "INA219 sensor not available, skipping orin_battery_state update."
                    )
                    self.wrote_error_once = True

        # Publish messages
        try:
            self.odom_publisher.publish(self.odom)
            self.imu_publisher.publish(self.imu)
            self.joint_state_publisher.publish(self.joint_state)
            self.cr_battery_state_publisher.publish(self.cr_battery_state)
            self.object_state_publisher.publish(Bool(data=self.has_object))
            if MODEL and "nvidia jetson" in MODEL:
                if hasattr(self, "ina_sensor") and orin_battery_due:
                    self.orin_battery_state_publisher.publish(self.orin_battery_state)
        except Exception:
            if rclpy.ok():
                raise

    def set_odom(self):

        self.odom.header.stamp = self.current_time.to_msg()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.odom_child_frame_id

        Vx_tick = (
            self.cr_state.vel_bl
            + self.cr_state.vel_br
            + self.cr_state.vel_fl
            + self.cr_state.vel_fr
        ) / 4
        Vy_tick = (
            self.cr_state.vel_bl
            - self.cr_state.vel_br
            - self.cr_state.vel_fl
            + self.cr_state.vel_fr
        ) / 4
        Wz_tick = (
            -self.cr_state.vel_bl
            + self.cr_state.vel_br
            - self.cr_state.vel_fl
            + self.cr_state.vel_fr
        ) / 4

        self.odom.twist.twist.linear.x = Vx_tick * self.scale  # m/s
        self.odom.twist.twist.linear.y = Vy_tick * self.scale  # m/s
        self.odom.twist.twist.angular.z = (
            Wz_tick * self.scale / self.wheel_dist_scale
        )  # rad/s
        # self.get_logger().info(f'Wheel ticks: BL: {self.cr_state.vel_bl}, BR: {self.cr_state.vel_br}, FL: {self.cr_state.vel_fl}, FR: {self.cr_state.vel_fr}')
        # self.get_logger().info(f'Wheel tick Velocities: Vx_tick: {Vx_tick}, Vy_tick: {Vy_tick}, Wz_tick: {Wz_tick}, scale: {self.scale}, wheel_dist_scale: {self.wheel_dist_scale}')
        # self.get_logger().info(f'Calculated Velocities: Vx: {msg.twist.twist.linear.x}, Vy: {msg.twist.twist.linear.y}, Wz: {msg.twist.twist.angular.z}')

        dx = self.odom.twist.twist.linear.x * self.dt
        dy = self.odom.twist.twist.linear.y * self.dt
        dtheta = self.odom.twist.twist.angular.z * self.dt

        self.odom.pose.pose.position.x = self.odom.pose.pose.position.x + (
            math.cos(self.last_yaw_angle) * dx - math.sin(self.last_yaw_angle) * dy
        )
        self.odom.pose.pose.position.y = self.odom.pose.pose.position.y + (
            math.sin(self.last_yaw_angle) * dx + math.cos(self.last_yaw_angle) * dy
        )
        self.odom.pose.pose.position.z = 0.0
        # self.get_logger().info(f'Publishing: dx: {dx}, dy: {dy}, dtheta: {dtheta}')
        # self.get_logger().info(f'Current Odom: x: {self.odom.pose.pose.position.x}, y: {self.odom.pose.pose.position.y}, theta: {self.odom.pose.pose.orientation.z}')
        if self.odom_from_imu:
            # Yaw from the OpenCR's fused IMU rather than from the wheels.
            #
            # Worth knowing which one is running: mecanum rollers slip most
            # under yaw, and the wheel branch below cannot see it -- it
            # integrates the commanded rotation and reports it as fact, so a
            # spin in place is where wheel odometry lies hardest and where
            # anything downstream that trusts it (slam_toolbox's scan-match
            # prior, above all) is misled. The IMU drifts instead of slipping,
            # which is the slower and more forgiving error of the two.
            #
            # The quaternion is absolute in the IMU's own zero, so `odom` is
            # not the pose at start-up but a frame rotated by whatever yaw the
            # board woke up believing. Nothing downstream cares -- odom is a
            # frame, not a place -- but it does mean the two branches are not
            # interchangeable mid-run.
            self.odom.pose.pose.orientation.x = self.cr_state.imu_orientation_x
            self.odom.pose.pose.orientation.y = self.cr_state.imu_orientation_y
            self.odom.pose.pose.orientation.z = self.cr_state.imu_orientation_z
            self.odom.pose.pose.orientation.w = self.cr_state.imu_orientation_w
            # Update last_yaw_angle from IMU quaternion
            e = quat2euler(
                (
                    self.cr_state.imu_orientation_w,
                    self.cr_state.imu_orientation_x,
                    self.cr_state.imu_orientation_y,
                    self.cr_state.imu_orientation_z,
                )
            )
            self.last_yaw_angle = e[2]  # Yaw angle
        else:
            new_yaw = (self.last_yaw_angle + dtheta) % (2 * math.pi)
            # Convert roll=0, pitch=0, yaw=new_yaw to a normalized quaternion
            quaternion = euler2quat(0, 0, new_yaw)
            self.last_yaw_angle = new_yaw
            self.odom.pose.pose.orientation.w = quaternion[0]
            self.odom.pose.pose.orientation.x = quaternion[1]
            self.odom.pose.pose.orientation.y = quaternion[2]
            self.odom.pose.pose.orientation.z = quaternion[3]

        # Outside the branch, deliberately. This used to sit inside the `else`,
        # so `odom_params.from_imu: true` published an /odom topic and NO
        # `odom -> base_footprint` transform at all -- which is not a degraded
        # robot, it is a TF tree with a hole in it and no SLAM, no costmaps and
        # no nav2. That made the IMU branch untestable, which is most of why it
        # was still marked TODO.
        self.transform.header.stamp = self.current_time.to_msg()
        self.transform.header.frame_id = self.odom_frame_id
        self.transform.child_frame_id = self.odom_child_frame_id
        self.transform.transform.translation.x = self.odom.pose.pose.position.x
        self.transform.transform.translation.y = self.odom.pose.pose.position.y
        self.transform.transform.translation.z = 0.0
        self.transform.transform.rotation = self.odom.pose.pose.orientation
        try:
            self.tf_broadcaster.sendTransform(self.transform)
        except Exception:
            if rclpy.ok():
                raise

    def set_imu(self):

        self.imu.header.stamp = self.current_time.to_msg()
        self.imu.header.frame_id = self.imu_frame_id

        # Fill IMU data from OpenCRState
        self.imu.orientation.x = self.cr_state.imu_orientation_x
        self.imu.orientation.y = self.cr_state.imu_orientation_y
        self.imu.orientation.z = self.cr_state.imu_orientation_z
        self.imu.orientation.w = self.cr_state.imu_orientation_w
        self.imu.angular_velocity.x = self.cr_state.imu_angular_vel_x
        self.imu.angular_velocity.y = self.cr_state.imu_angular_vel_y
        self.imu.angular_velocity.z = self.cr_state.imu_angular_vel_z
        self.imu.linear_acceleration.x = self.cr_state.imu_linear_acc_x
        self.imu.linear_acceleration.y = self.cr_state.imu_linear_acc_y
        self.imu.linear_acceleration.z = self.cr_state.imu_linear_acc_z

    def set_joint_state(self):

        self.joint_state.header.stamp = self.current_time.to_msg()
        self.joint_state.name = [
            f"{self.namespace}/wheel_backleft_joint",
            f"{self.namespace}/wheel_backright_joint",
            f"{self.namespace}/wheel_frontleft_joint",
            f"{self.namespace}/wheel_frontright_joint",
            f"{self.namespace}/head_joint",
            f"{self.namespace}/grabber_left_joint",
            f"{self.namespace}/grabber_right_joint",
        ]

        access_posis = [
            NECK_MIDPOINT_COMPENSATE_CONSTANT + self.cr_state.pos_n * TICK_TO_RAD,
            GRIPPER_MIDPOINT_COMPENSATE_CONSTANT - self.cr_state.pos_gl * TICK_TO_RAD,
            GRIPPER_MIDPOINT_COMPENSATE_CONSTANT - self.cr_state.pos_gr * TICK_TO_RAD,
        ]

        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, *access_posis]
        self.joint_state.velocity = [0.0] * 7
        self.joint_state.effort = [0.0] * 7

    def set_cr_battery_state(
        self,
    ):  # could be more accurate - Temperature. cell values, status. etc.

        self.cr_battery_state.header.stamp = self.current_time.to_msg()

        self.cr_battery_state.voltage = self.cr_state.battery_voltage
        self.cr_battery_state.percentage = (
            self.cr_state.battery_voltage - self.battery_min_voltage
        ) / (self.battery_max_voltage - self.battery_min_voltage)
        self.cr_battery_state.charge = (
            self.cr_battery_state.capacity * self.cr_battery_state.percentage
        )

    def set_orin_battery_state(
        self,
    ):  # placeholder for orin battery state, currently set to 100%

        # Timestamp
        self.orin_battery_state.header.stamp = self.get_clock().now().to_msg()
        self.orin_battery_state.header.frame_id = "base_footprint"

        # Sensor readings
        bus_voltage = self.ina_sensor.bus_voltage  # volts
        current_ma = self.ina_sensor.current  # mA

        # Fill ROS BatteryState fields
        self.orin_battery_state.voltage = float(bus_voltage)
        self.orin_battery_state.current = float(current_ma) / 1000.0  # convert mA -> A

        # Optional values
        self.orin_battery_state.temperature = float("nan")
        self.orin_battery_state.charge = float("nan")
        self.orin_battery_state.capacity = float("nan")
        self.orin_battery_state.design_capacity = float("nan")

        percentage = (bus_voltage - self.battery_min_voltage) / (
            self.battery_max_voltage - self.battery_min_voltage
        )
        percentage = max(0.0, min(1.0, percentage))
        self.orin_battery_state.percentage = percentage

        # Power supply status
        self.orin_battery_state.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        self.orin_battery_state.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD
        )
        self.orin_battery_state.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        )

        self.orin_battery_state.present = True

    def set_object_state(self):
        self.dms_buffer.append(self.cr_state.dms)
        self.dms_buffer.pop(0)
        self.has_object = (
            sum(
                dms > self.has_object_threshold or dms == -1.0
                for dms in self.dms_buffer
            )
            > len(self.dms_buffer) / 2
        )


def main(args=None):
    rclpy.init(args=args)

    sensorproc_node = Mecanumbot_Sensorproc_Node()
    executor = MultiThreadedExecutor()
    executor.add_node(sensorproc_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        sensorproc_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
