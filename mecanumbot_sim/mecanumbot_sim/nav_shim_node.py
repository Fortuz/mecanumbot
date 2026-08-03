import math
import uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from mecanumbot_msgs.srv import SetLedStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_SUCCEEDED = 4


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MecanumbotSimNavShimNode(Node):
    """
    Small sim-only adapter for the existing behaviour tree.

    The real behaviour tree talks to Nav2 via /goal_pose, /amcl_pose, and
    /navigate_to_pose/_action/status. This node emulates just those contracts and
    converts goals into /cmd_vel so we can test BT logic without launching Nav2.
    """

    def __init__(self):
        super().__init__("mecanumbot_sim_nav_shim_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("odom_topic", "/mecanumbot/odom"),
                ("goal_topic", "/goal_pose"),
                ("cmd_vel_topic", "/cmd_vel"),
                ("amcl_pose_topic", "/amcl_pose"),
                ("nav_status_topic", "/navigate_to_pose/_action/status"),
                ("control_rate_hz", 20.0),
                ("max_linear_speed", 0.16),
                ("max_lateral_speed", 0.10),
                ("max_angular_speed", 0.75),
                ("linear_gain", 0.7),
                ("angular_gain", 1.6),
                ("position_tolerance", 0.08),
                ("yaw_tolerance", 0.12),
                ("enable_dummy_led_service", True),
            ],
        )

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_lateral_speed = float(self.get_parameter("max_lateral_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)

        transient_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.latest_odom = None
        self.active_goal = None
        self.active_goal_uuid = None
        self.active_goal_stamp = None
        self.status_history = []
        self.goal_started = False

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.goal_callback,
            10,
        )
        self.cmd_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.amcl_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            transient_qos,
        )
        self.status_publisher = self.create_publisher(
            GoalStatusArray,
            str(self.get_parameter("nav_status_topic").value),
            10,
        )

        if bool(self.get_parameter("enable_dummy_led_service").value):
            self.create_service(
                SetLedStatus, "/set_led_status", self.set_led_status_callback
            )

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.timer_callback)
        self.get_logger().info("Simulation Nav2/AMCL shim started.")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.publish_amcl_pose(msg)

    def goal_callback(self, msg: PoseStamped) -> None:
        self.active_goal = msg
        self.active_goal_uuid = list(uuid.uuid4().bytes)
        self.active_goal_stamp = self.get_clock().now().to_msg()
        self.goal_started = False
        self.record_status(STATUS_ACCEPTED)
        self.get_logger().info(
            "Accepted sim goal: "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

    def set_led_status_callback(self, request, response):
        response.success = True
        response.message = "Sim LED shim accepted command."
        return response

    def timer_callback(self) -> None:
        if self.latest_odom is not None:
            self.publish_amcl_pose(self.latest_odom)

        if self.active_goal is None or self.latest_odom is None:
            self.publish_zero_cmd()
            self.publish_status_array()
            return

        if not self.goal_started:
            self.goal_started = True
            self.record_status(STATUS_EXECUTING)

        cmd, reached = self.compute_command()
        self.cmd_publisher.publish(cmd)

        if reached:
            self.record_status(STATUS_SUCCEEDED)
            self.publish_zero_cmd()
            self.get_logger().info("Sim goal reached.")
            self.active_goal = None
            self.active_goal_uuid = None
            self.active_goal_stamp = None
            self.goal_started = False

        self.publish_status_array()

    def publish_amcl_pose(self, odom: Odometry) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose = odom.pose.pose
        msg.pose.covariance = odom.pose.covariance
        self.amcl_publisher.publish(msg)

    def compute_command(self) -> tuple[Twist, bool]:
        robot_pose = self.latest_odom.pose.pose
        goal_pose = self.active_goal.pose

        dx = goal_pose.position.x - robot_pose.position.x
        dy = goal_pose.position.y - robot_pose.position.y
        distance = math.hypot(dx, dy)

        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        goal_yaw = yaw_from_quaternion(goal_pose.orientation)
        yaw_error = normalize_angle(goal_yaw - robot_yaw)

        cmd = Twist()
        if distance > self.position_tolerance:
            speed = min(self.max_linear_speed, self.linear_gain * distance)
            x_robot = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
            y_robot = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy
            cmd.linear.x = clamp(
                speed * x_robot / distance,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
            cmd.linear.y = clamp(
                speed * y_robot / distance,
                -self.max_lateral_speed,
                self.max_lateral_speed,
            )
            desired_heading = math.atan2(dy, dx)
            yaw_error = normalize_angle(desired_heading - robot_yaw)

        if abs(yaw_error) > self.yaw_tolerance:
            cmd.angular.z = clamp(
                self.angular_gain * yaw_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

        reached = (
            distance <= self.position_tolerance
            and abs(normalize_angle(goal_yaw - robot_yaw)) <= self.yaw_tolerance
        )
        return cmd, reached

    def publish_zero_cmd(self) -> None:
        if not rclpy.ok():
            return
        self.cmd_publisher.publish(Twist())

    def record_status(self, status_code: int) -> None:
        if self.active_goal_uuid is None:
            return

        status = GoalStatus()
        status.goal_info.goal_id.uuid = self.active_goal_uuid
        status.goal_info.stamp = (
            self.active_goal_stamp or self.get_clock().now().to_msg()
        )
        status.status = status_code
        self.status_history.append(status)
        self.status_history = self.status_history[-5:]

    def publish_status_array(self) -> None:
        msg = GoalStatusArray()
        msg.status_list = self.status_history
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimNavShimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.publish_zero_cmd()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
