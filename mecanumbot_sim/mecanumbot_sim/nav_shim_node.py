import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from mecanumbot_msgs.srv import SetLedStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future


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


class NavRun:
    """
    One accepted navigation goal, driven pose by pose from the node's timer.

    The action's execute callback waits on `done`; the timer is what drives the
    robot and completes it, so nothing blocks the single-threaded executor.
    """

    def __init__(self, goal_handle, poses):
        self.goal_handle = goal_handle
        self.poses = list(poses)
        self.index = 0
        self.done = Future()
        self.outcome = GoalStatus.STATUS_SUCCEEDED

    @property
    def pose(self):
        if self.index >= len(self.poses):
            return None
        return self.poses[self.index].pose

    @property
    def last(self):
        return self.index >= len(self.poses) - 1

    @property
    def remaining(self):
        return max(0, len(self.poses) - self.index)

    def finish(self, outcome):
        if self.done.done():
            return
        self.outcome = outcome
        self.done.set_result(outcome)


class MecanumbotSimNavShimNode(Node):
    """
    Small sim-only stand-in for Nav2, so behaviour trees run without it.

    The behaviour trees navigate through Nav2's actions -- `navigate_to_pose`
    for a single goal and `navigate_through_poses` for a run of route waypoints
    -- and localize off `/amcl_pose`. This node serves both actions for real
    (goal ids, results, cancellation and feedback all come from rclpy's action
    server, which is also what publishes the `_action/status` topics the trees
    watch) and turns the goals into `/cmd_vel`.

    `/goal_pose` is still accepted, the way Nav2's own navigator accepts it: the
    pose is forwarded into `navigate_to_pose` rather than driven separately, so
    there is one path through this node however a goal arrives.

    It drives straight at each pose and does not avoid obstacles.
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
                ("navigate_to_pose_action", "navigate_to_pose"),
                ("navigate_through_poses_action", "navigate_through_poses"),
                ("control_rate_hz", 20.0),
                ("max_linear_speed", 0.16),
                ("max_lateral_speed", 0.10),
                ("max_angular_speed", 0.75),
                ("linear_gain", 0.7),
                ("angular_gain", 1.6),
                ("position_tolerance", 0.08),
                ("yaw_tolerance", 0.12),
                # Waypoints on the way through a route are passed rather than
                # stopped at, so they are done with once the robot is near them
                # and their heading does not have to be taken up.
                ("waypoint_tolerance", 0.35),
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
        self.waypoint_tolerance = float(self.get_parameter("waypoint_tolerance").value)

        transient_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.latest_odom = None
        self.run = None
        self.run_started_at = None
        self.driving = False

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

        to_pose_action = str(self.get_parameter("navigate_to_pose_action").value)
        through_poses_action = str(
            self.get_parameter("navigate_through_poses_action").value
        )
        self.to_pose_server = ActionServer(
            self,
            NavigateToPose,
            to_pose_action,
            execute_callback=self.execute_to_pose,
            cancel_callback=self.cancel_callback,
        )
        self.through_poses_server = ActionServer(
            self,
            NavigateThroughPoses,
            through_poses_action,
            execute_callback=self.execute_through_poses,
            cancel_callback=self.cancel_callback,
        )
        # Goals published on /goal_pose go in through our own action, so every
        # goal is a proper action goal whichever way it arrived.
        self.to_pose_client = ActionClient(self, NavigateToPose, to_pose_action)

        if bool(self.get_parameter("enable_dummy_led_service").value):
            self.create_service(
                SetLedStatus, "/set_led_status", self.set_led_status_callback
            )

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.timer_callback)
        self.get_logger().info("Simulation Nav2/AMCL shim started.")

    # --- inputs --------------------------------------------------------------

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.publish_amcl_pose(msg)

    def goal_callback(self, msg: PoseStamped) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.to_pose_client.send_goal_async(goal)
        self.get_logger().info(
            "Forwarding a /goal_pose goal into navigate_to_pose: "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )

    def set_led_status_callback(self, request, response):
        response.success = True
        response.message = "Sim LED shim accepted command."
        return response

    # --- action servers ------------------------------------------------------

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_to_pose(self, goal_handle):
        run = self.start_run(goal_handle, [goal_handle.request.pose])
        await run.done
        return self.finish_run(run, goal_handle, NavigateToPose.Result())

    async def execute_through_poses(self, goal_handle):
        poses = list(goal_handle.request.poses)
        if not poses:
            goal_handle.abort()
            return NavigateThroughPoses.Result()
        run = self.start_run(goal_handle, poses)
        await run.done
        return self.finish_run(run, goal_handle, NavigateThroughPoses.Result())

    def start_run(self, goal_handle, poses):
        """Take over from whatever was being driven, the way nav2 preempts."""
        if self.run is not None and not self.run.done.done():
            self.get_logger().info("A new goal arrived; abandoning the previous one.")
            self.run.finish(GoalStatus.STATUS_ABORTED)
        self.run = NavRun(goal_handle, poses)
        self.run_started_at = self.get_clock().now()
        self.get_logger().info(f"Accepted a sim goal of {len(poses)} pose(s).")
        return self.run

    def finish_run(self, run, goal_handle, result):
        if run.outcome == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        elif run.outcome == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        if self.run is run:
            self.run = None
            self.stop_driving()
        return result

    # --- driving -------------------------------------------------------------

    def timer_callback(self) -> None:
        if self.latest_odom is not None:
            self.publish_amcl_pose(self.latest_odom)

        run = self.run
        if run is None or run.done.done() or self.latest_odom is None:
            self.stop_driving()
            return

        if run.goal_handle.is_cancel_requested:
            self.get_logger().info("Sim goal cancelled.")
            run.finish(GoalStatus.STATUS_CANCELED)
            self.stop_driving()
            return

        cmd, reached = self.compute_command(run)
        self.driving = True
        self.cmd_publisher.publish(cmd)
        self.publish_feedback(run)

        if not reached:
            return

        run.index += 1
        if run.index >= len(run.poses):
            self.get_logger().info("Sim goal reached.")
            run.finish(GoalStatus.STATUS_SUCCEEDED)
            self.stop_driving()
        else:
            self.get_logger().info(
                f"Sim waypoint passed, {run.remaining} pose(s) to go."
            )

    def compute_command(self, run) -> tuple:
        robot_pose = self.latest_odom.pose.pose
        goal_pose = run.pose

        dx = goal_pose.position.x - robot_pose.position.x
        dy = goal_pose.position.y - robot_pose.position.y
        distance = math.hypot(dx, dy)

        robot_yaw = yaw_from_quaternion(robot_pose.orientation)
        goal_yaw = yaw_from_quaternion(goal_pose.orientation)
        yaw_error = normalize_angle(goal_yaw - robot_yaw)

        # A waypoint on the way through is passed, not arrived at: near enough
        # is enough, and its heading is only a hint about where the route goes.
        tolerance = self.position_tolerance if run.last else self.waypoint_tolerance

        cmd = Twist()
        if distance > tolerance:
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

        if distance > tolerance:
            return cmd, False
        if not run.last:
            return cmd, True
        return cmd, abs(normalize_angle(goal_yaw - robot_yaw)) <= self.yaw_tolerance

    # --- outputs -------------------------------------------------------------

    def publish_amcl_pose(self, odom: Odometry) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose = odom.pose.pose
        msg.pose.covariance = odom.pose.covariance
        self.amcl_publisher.publish(msg)

    def publish_feedback(self, run) -> None:
        robot_pose = self.latest_odom.pose.pose
        goal_pose = run.poses[-1].pose
        elapsed = self.get_clock().now() - self.run_started_at

        if isinstance(run.goal_handle.request, NavigateThroughPoses.Goal):
            feedback = NavigateThroughPoses.Feedback()
            feedback.number_of_poses_remaining = run.remaining
        else:
            feedback = NavigateToPose.Feedback()

        feedback.current_pose.header.frame_id = "map"
        feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
        feedback.current_pose.pose = robot_pose
        feedback.navigation_time = elapsed.to_msg()
        feedback.distance_remaining = math.hypot(
            goal_pose.position.x - robot_pose.position.x,
            goal_pose.position.y - robot_pose.position.y,
        )
        run.goal_handle.publish_feedback(feedback)

    def stop_driving(self) -> None:
        """
        Stop the robot once, then leave `/cmd_vel` alone.

        Nav2 does not hold the topic down while it is idle, and neither may
        this: the behaviour trees turn in place by publishing `/cmd_vel`
        themselves, and a shim repeating zeroes at 20 Hz cancels every one of
        those turns out.
        """
        if not self.driving:
            return
        self.driving = False
        self.publish_zero_cmd()

    def publish_zero_cmd(self) -> None:
        if not rclpy.ok():
            return
        self.cmd_publisher.publish(Twist())


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
