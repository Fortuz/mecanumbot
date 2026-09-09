import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool


class ExplorationUncertaintyMonitor(Node):
    """
    Monitors slam_toolbox localization uncertainty via pose covariance trace.
    When the trace exceeds the threshold:
      1. Pauses explore_lite  (/explore/resume → False)
      2. Navigates back to the map origin to force a loop closure
      3. Resumes explore_lite (/explore/resume → True)
    """

    def __init__(self):
        super().__init__("exploration_uncertainty_monitor")

        self.declare_parameter("uncertainty_threshold", 0.05)
        self.declare_parameter("check_period", 5.0)
        self.declare_parameter("revisit_x", 0.0)
        self.declare_parameter("revisit_y", 0.0)

        self._threshold = self.get_parameter("uncertainty_threshold").value
        self._revisit_x = self.get_parameter("revisit_x").value
        self._revisit_y = self.get_parameter("revisit_y").value

        self._latest_cov_trace = 0.0
        self._revisiting = False

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._explore_pause_client = self.create_client(SetBool, "/explore/resume")

        self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self._pose_cb, 10
        )
        self.create_timer(self.get_parameter("check_period").value, self._check)

        self.get_logger().info(
            f"Uncertainty monitor active — threshold: {self._threshold:.4f}, "
            f"revisit point: ({self._revisit_x}, {self._revisit_y})"
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        # 2D trace: x, y, yaw variance (6x6 row-major covariance matrix)
        self._latest_cov_trace = c[0] + c[7] + c[35]

    def _check(self):
        trace = self._latest_cov_trace
        self.get_logger().debug(f"Covariance trace: {trace:.6f}")

        if trace > self._threshold and not self._revisiting:
            self.get_logger().warn(
                f"Uncertainty high ({trace:.4f} > {self._threshold}), "
                "pausing explore_lite and returning to origin for loop closure."
            )
            self._revisiting = True
            self._pause_explore(then=self._send_revisit_goal)

    # --- explore_lite pause/resume -----------------------------------------

    def _pause_explore(self, then):
        """Pauses explore_lite, then calls the provided callback."""
        if not self._explore_pause_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/explore/resume service unavailable, proceeding without pause.")
            then()
            return
        req = SetBool.Request()
        req.data = False  # False = stop
        future = self._explore_pause_client.call_async(req)
        future.add_done_callback(lambda f: then())

    def _resume_explore(self):
        """Resumes explore_lite after a revisit."""
        if not self._explore_pause_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/explore/resume service unavailable, exploration will not resume automatically.")
            return
        req = SetBool.Request()
        req.data = True  # True = resume
        self._explore_pause_client.call_async(req)
        self.get_logger().info("Explore_lite resumed, continuing exploration.")

    # --- NavigateToPose revisit --------------------------------------------

    def _send_revisit_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server not available.")
            self._finish_revisit()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self._revisit_x
        goal.pose.pose.position.y = self._revisit_y
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Revisit goal rejected.")
            self._finish_revisit()
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.get_logger().info("Revisit complete.")
        self._finish_revisit()

    def _finish_revisit(self):
        self._revisiting = False
        self._resume_explore()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationUncertaintyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
