import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class ExplorationUncertaintyMonitor(Node):
    """
    Figyeli a slam_toolbox lokalizációs bizonytalanságát.
    Ha a pose covariance trace egy küszöb fölé megy, a robotot visszaküldi
    az indulási pontra loop closure kényszerítéséhez, majd az m-explore-ros2
    automatikusan folytatja az explorationt.
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

        self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self._pose_cb, 10
        )
        period = self.get_parameter("check_period").value
        self.create_timer(period, self._check)

        self.get_logger().info(
            f"Uncertainty monitor aktív — küszöb: {self._threshold:.4f}, "
            f"revisit: ({self._revisit_x}, {self._revisit_y})"
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        c = msg.pose.covariance
        # 2D trace: x, y, yaw varianciák (6x6 mátrix, sor-major)
        self._latest_cov_trace = c[0] + c[7] + c[35]

    def _check(self):
        trace = self._latest_cov_trace
        self.get_logger().debug(f"Covariance trace: {trace:.6f}")

        if trace > self._threshold and not self._revisiting:
            self.get_logger().warn(
                f"Bizonytalanság magas ({trace:.4f} > {self._threshold}), "
                "visszatérés az origóhoz loop closure kényszerítéséhez."
            )
            self._revisiting = True
            self._send_revisit_goal()

    def _send_revisit_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server nem elérhető.")
            self._revisiting = False
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
            self.get_logger().warn("Revisit goal visszautasítva.")
            self._revisiting = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.get_logger().info("Revisit kész, exploration folytatódik.")
        self._revisiting = False


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationUncertaintyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
