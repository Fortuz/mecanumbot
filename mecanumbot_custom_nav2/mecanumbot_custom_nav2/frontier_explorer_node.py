"""
The autoslam explorer: drive the robot around until the place has been scanned.

This node is T1. It watches slam_toolbox's occupancy grid, runs the RRT frontier
detector over it, sends the best frontier to nav2 as a `NavigateToPose` goal, and
decides when the whole thing is finished. It drives nothing itself -- every
translation goes through nav2, exactly as the behaviour trees do, so obstacle
avoidance and the costmaps are untouched.

The server's half of the loop arrives on one topic. `MapCloudAgreement` says how
much of the 2D map the Deep3R cloud accounts for and which regions it is least
sure about, and this node uses it twice: as a term in the exit criteria, and as a
source of goals. A region the cloud is thin over is a place to go and *look*
again, and more frames from a better viewpoint is the only feedback a
feed-forward reconstruction can actually use. Those goals are interleaved with
frontier goals rather than replacing them, because a robot that only ever
services the server's complaints stops exploring.

    /map ─────────────────► RRT detection ──► cluster, revalidate, score ──┐
    /amcl_pose ───────────►                                                │
    deep3r/map_agreement ─► uncertain regions ─────────────────────────────┤
                                                                           ▼
                                                          nav2 NavigateToPose
                                                                           │
    exploration/state, /frontiers (markers), /finished ◄────────────────────┘

Publishing `exploration/finished` is how T1 hands over: whatever starts T2 waits
for that latch rather than for a wall-clock guess.
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseArray, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mecanumbot_msgs.msg import MapCloudAgreement

from mecanumbot_custom_nav2 import frontiers as frontier_tools
from mecanumbot_custom_nav2.exit_criteria import (
    CloudProgress,
    ExitCriteria,
    ExplorationProgress,
    travelled,
)
from mecanumbot_custom_nav2.occupancy import Grid
from mecanumbot_custom_nav2.rrt import FrontierSearch

# Both the map and AMCL publish transient-local, so a subscriber that joins late
# still gets the current value instead of waiting for the next update -- and the
# map updates every 5 s by default, which is a long time to sit still.
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class FrontierExplorer(Node):
    """RRT frontier exploration with the server's reconstruction in the loop."""

    def __init__(self):
        super().__init__("mecanumbot_frontier_explorer")
        self._declare_parameters()

        self.grid = None
        self.robot_xy = None
        self.previous_xy = None
        self.distance = 0.0
        self.started = self.get_clock().now()
        self.finished = False
        self.finish_reason = ""
        self.goal = None
        self.goal_handle = None
        self.goal_sent_at = None
        self.goal_source = ""
        self.revisit_points = []
        self.last_known_cells = None
        self.goals_sent = 0

        self.search = FrontierSearch(
            step_size=self._get("rrt_step_size"),
            snap_radius=self._get("rrt_snap_radius"),
            global_iterations=int(self._get("rrt_global_iterations")),
            local_iterations=int(self._get("rrt_local_iterations")),
            local_radius=self._get("rrt_local_radius"),
            seed=int(self._get("rrt_seed")) or None,
        )
        self.progress = ExplorationProgress(window=self._get("progress_window"))
        self.cloud = CloudProgress(window=self._get("cloud_window"))
        self.criteria = ExitCriteria(
            frontier_quiet_time=self._get("frontier_quiet_time"),
            min_cells_per_metre=self._get("min_cells_per_metre"),
            max_growth_fraction=self._get("max_map_growth"),
            loop_closure_settle=self._get("loop_closure_settle"),
            min_agreement=self._get("min_agreement"),
            cloud_reset_settle=self._get("cloud_reset_settle"),
            min_runtime=(self._get("min_runtime") or None),
            min_grid_coverage=self._get("min_grid_coverage"),
            max_uncertain_regions=int(self._get("max_uncertain_regions")),
            uncertain_score_threshold=self._get("uncertain_score_threshold"),
            max_cloud_growth=self._get("max_cloud_growth"),
            cloud_verdict_timeout=self._get("cloud_verdict_timeout"),
            require_cloud=self._get("require_cloud"),
            max_duration=self._positive("max_duration"),
            max_distance=self._positive("max_distance"),
            min_battery_voltage=self._positive("min_battery_voltage"),
        )

        self.create_subscription(
            OccupancyGrid, self._get("map_topic"), self._on_map, LATCHED
        )
        # T1 runs under slam_toolbox, which publishes map -> odom but no
        # /amcl_pose; T2 runs under AMCL against the saved map and publishes
        # both. So the pose source is a parameter rather than an assumption, and
        # `tf` is the default because that is the phase this node exists for.
        self.tf_buffer = None
        self.tf_listener = None
        if self._get("pose_source") == "tf":
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._get("pose_topic"),
                self._on_pose,
                LATCHED,
            )
        self.create_subscription(
            MapCloudAgreement, self._get("agreement_topic"), self._on_agreement, 10
        )
        # Which regions are worth revisiting is `mecanumbot_map_agreement`'s
        # decision, not this node's: the same policy has to hold in T2, when
        # this explorer is not running at all. Here they are only goals to
        # interleave with the frontiers.
        self.create_subscription(
            PoseArray, self._get("revisit_topic"), self._on_revisits, 10
        )

        self.finished_publisher = self.create_publisher(
            Bool, self._get("finished_topic"), LATCHED
        )
        self.state_publisher = self.create_publisher(
            String, self._get("state_topic"), 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self._get("marker_topic"), 10
        )
        self.navigator = ActionClient(self, NavigateToPose, self._get("nav2_action"))

        self.finished_publisher.publish(Bool(data=False))
        self.timer = self.create_timer(1.0 / self._get("rate"), self._cycle)
        self.get_logger().info(
            "frontier explorer up; waiting for a map and an AMCL pose"
        )

    # --- parameters -----------------------------------------------------------

    def _declare_parameters(self):
        defaults = {
            # --- topics and rates ---------------------------------------------
            "map_topic": "/map",
            # tf | amcl. Under slam_toolbox there is no /amcl_pose to subscribe
            # to, so the robot's position has to be read out of the transform
            # tree instead.
            "pose_source": "tf",
            "pose_topic": "/amcl_pose",
            "map_frame": "map",
            "base_frame": "mecanumbot/base_link",
            "agreement_topic": "/mecanumbot/deep3r/map_agreement",
            "revisit_topic": "/mecanumbot/deep3r/revisit_regions",
            "finished_topic": "exploration/finished",
            "state_topic": "exploration/state",
            "marker_topic": "exploration/frontiers",
            "nav2_action": "/navigate_to_pose",
            "rate": 1.0,
            # --- how the map is read ------------------------------------------
            "free_threshold": 25,
            "occupied_threshold": 65,
            # --- the RRT ------------------------------------------------------
            "rrt_step_size": 1.0,
            "rrt_snap_radius": 0.5,
            "rrt_global_iterations": 60,
            "rrt_local_iterations": 30,
            "rrt_local_radius": 5.0,
            "rrt_seed": 0,
            # --- turning candidates into a goal --------------------------------
            "cluster_radius": 0.6,
            "revalidate_radius": 0.4,
            "gain_radius": 1.5,
            "gain_weight": 1.0,
            "cost_weight": 0.4,
            "min_gain": 8,
            "hysteresis": 0.25,
            "hysteresis_radius": 1.0,
            "goal_timeout": 60.0,
            "goal_reached_distance": 0.6,
            # --- servicing the server's uncertain regions ----------------------
            # Which regions those are, and when one counts as looked at, belong
            # to `mecanumbot_map_agreement`. All that is left here is whether to
            # take its goals at all, and how often.
            "revisit_uncertain": True,
            "uncertain_every": 3,
            # --- exit criteria -------------------------------------------------
            "progress_window": 45.0,
            "cloud_window": 60.0,
            "frontier_quiet_time": 20.0,
            "min_cells_per_metre": 25.0,
            "max_map_growth": 0.01,
            "loop_closure_settle": 15.0,
            "min_agreement": 0.15,
            "cloud_reset_settle": 30.0,
            "min_runtime": 60.0,
            "min_grid_coverage": 0.75,
            "max_uncertain_regions": 3,
            "uncertain_score_threshold": 0.5,
            "max_cloud_growth": 0.02,
            "cloud_verdict_timeout": 30.0,
            "require_cloud": True,
            "max_duration": 0.0,
            "max_distance": 0.0,
            "min_battery_voltage": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _get(self, name):
        return self.get_parameter(name).value

    def _positive(self, name):
        """Read a budget parameter, where 0 means "no limit"."""
        value = self._get(name)
        return None if not value else float(value)

    # --- subscriptions --------------------------------------------------------

    def _on_map(self, msg):
        self.grid = Grid.from_message(
            msg,
            free_threshold=int(self._get("free_threshold")),
            occupied_threshold=int(self._get("occupied_threshold")),
        )
        known = self.grid.known_cells()
        # slam_toolbox re-rasterises the whole grid after a loop closure, which
        # shows up here as the known area *shrinking* -- the only thing that
        # makes it go down. Anything grown against the old free space has to be
        # thrown away, detector trees included.
        if self.last_known_cells is not None and known < self.last_known_cells * 0.98:
            self.progress.note_loop_closure(self._now())
            self.search.restart(self.robot_xy)
            self.get_logger().info("map re-rasterised (loop closure); trees restarted")
        self.last_known_cells = known

    def _on_pose(self, msg):
        position = msg.pose.pose.position
        self._set_robot_xy((position.x, position.y))

    def _update_pose_from_tf(self):
        """Read the robot's position out of the transform tree."""
        if self.tf_buffer is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self._get("map_frame"),
                self._get("base_frame"),
                rclpy.time.Time(),
            )
        except Exception as error:  # the chain is not up yet, or has gone stale
            self.get_logger().warn(
                f"no {self._get('map_frame')} -> {self._get('base_frame')} "
                f"transform ({error})",
                throttle_duration_sec=5.0,
            )
            return
        self._set_robot_xy(
            (transform.transform.translation.x, transform.transform.translation.y)
        )

    def _set_robot_xy(self, xy):
        self.robot_xy = xy
        self.distance += travelled(self.previous_xy, self.robot_xy)
        self.previous_xy = self.robot_xy

    def _on_revisits(self, msg):
        # Already filtered and ordered by the handler, which also drops a region
        # once the robot has been near it -- so nothing is tracked as serviced
        # here. A list that empties is the loop having closed.
        self.revisit_points = [(p.position.x, p.position.y) for p in msg.poses]

    def _on_agreement(self, msg):
        self.cloud.add(
            self._now(),
            msg.grid_coverage,
            msg.cloud_points,
            msg.uncertain_scores,
            uncertain_points=[(p.x, p.y) for p in msg.uncertain_points],
            cloud_map_id=msg.cloud_map_id,
            # The `placed` half of the CLOUD criterion. The explorer logged this
            # number without ever testing it, so a run over a misplaced cloud
            # finished exactly like a good one.
            agreement=msg.agreement,
        )
        self.get_logger().info(
            f"server: cloud covers {msg.grid_coverage * 100:.0f}% of the map, "
            f"agreement {msg.agreement * 100:.0f}%, "
            f"{len(msg.uncertain_points)} uncertain region(s)"
        )

    # --- the cycle ------------------------------------------------------------

    def _cycle(self):
        if self.finished:
            return
        self._update_pose_from_tf()
        if self.grid is None or self.robot_xy is None:
            return

        now = self._now()
        self.progress.add(now, self.grid.known_cells(), self.distance)

        points = self.search.step(self.grid, self.robot_xy)
        best, scored = frontier_tools.best(
            self.grid,
            points,
            self.robot_xy,
            cluster_radius=self._get("cluster_radius"),
            revalidate_radius=self._get("revalidate_radius"),
            current_goal=self.goal,
            gain_radius=self._get("gain_radius"),
            gain_weight=self._get("gain_weight"),
            cost_weight=self._get("cost_weight"),
            min_gain=int(self._get("min_gain")),
            hysteresis=self._get("hysteresis"),
            hysteresis_radius=self._get("hysteresis_radius"),
        )
        self.criteria.note_frontiers(now, len(scored))
        self._publish_markers(scored)

        verdict = self.criteria.evaluate(
            now,
            self.progress,
            cloud=self.cloud,
            elapsed=now - self._seconds(self.started),
            distance=self.distance,
        )
        self._publish_state(verdict, scored)

        if verdict.finished:
            self._finish(verdict)
            return

        self._drive(best, scored)

    def _drive(self, best, scored):
        """Keep a goal in flight: hold the current one, or pick the next."""
        if self._goal_in_flight():
            return

        target, source = self._next_goal(best, scored)
        if target is None:
            return
        self._send_goal(target, source)

    def _next_goal(self, best, scored):
        """
        Choose between the best frontier and a region the server is unsure about.

        Every `uncertain_every` goals, a pending uncertain region wins if there
        is one. Interleaving rather than prioritising is deliberate: the
        server's regions are about reconstruction *quality* in space the robot
        has already mapped, so servicing them first would stall exploration
        behind a reconstruction that keeps finding new things to complain about.
        """
        uncertain = self._pending_uncertain()
        every = max(1, int(self._get("uncertain_every")))
        if uncertain and self.goals_sent % every == 0:
            return uncertain[0], "uncertain"
        if best is not None:
            return best.point, "frontier"
        if uncertain:
            return uncertain[0], "uncertain"
        return None, ""

    def _pending_uncertain(self):
        """Regions the handler says are still worth a look, best first."""
        if not self._get("revisit_uncertain"):
            return []
        return self.revisit_points

    # --- nav2 -----------------------------------------------------------------

    def _goal_in_flight(self):
        if self.goal_handle is None and self.goal_sent_at is None:
            return False
        elapsed = self._now() - self.goal_sent_at if self.goal_sent_at else 0.0
        if elapsed > float(self._get("goal_timeout")):
            self.get_logger().info(
                f"goal timed out after {elapsed:.0f} s, cancelling and re-deciding"
            )
            self._cancel_goal()
            return False
        if self.goal is not None and self.robot_xy is not None:
            reached = math.hypot(
                self.goal[0] - self.robot_xy[0], self.goal[1] - self.robot_xy[1]
            )
            if reached <= float(self._get("goal_reached_distance")):
                self._arrived()
                return False
        return True

    def _send_goal(self, point, source):
        if not self.navigator.server_is_ready():
            self.get_logger().warn("nav2 action server not ready", once=True)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point[0])
        goal.pose.pose.position.y = float(point[1])
        # Face the way the robot is already heading. A frontier has no natural
        # orientation, and asking nav2 for a specific yaw at one costs a turn at
        # the end of every leg for nothing.
        goal.pose.pose.orientation.w = 1.0

        self.goal = (float(point[0]), float(point[1]))
        self.goal_source = source
        self.goal_sent_at = self._now()
        self.goals_sent += 1
        self.navigator.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f"goal #{self.goals_sent} ({source}) at "
            f"x={point[0]:.2f} y={point[1]:.2f}"
        )

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:  # the server went away mid-request
            self.get_logger().warn(f"goal never landed ({error})")
            self._clear_goal()
            return
        if not handle.accepted:
            self.get_logger().warn("nav2 rejected the goal")
            self._clear_goal()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.get_logger().warn(f"no result for the goal ({error})")
            self._clear_goal()
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._arrived()
            return
        # A frontier nav2 cannot reach is not a frontier worth keeping: it is
        # behind something, and the detector will happily propose it again for
        # ever. Recording it as serviced is the cheapest way to stop that.
        self.get_logger().info(f"nav2 gave up on the goal (status {status})")
        self._retire_goal()

    def _arrived(self):
        self.get_logger().info(f"arrived at the {self.goal_source} goal")
        self._retire_goal()

    def _retire_goal(self):
        # Nothing to record for an uncertain goal: the handler marks a region
        # serviced from TF the moment the robot is near it, however it got there.
        self._clear_goal()

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._retire_goal()

    def _clear_goal(self):
        self.goal = None
        self.goal_handle = None
        self.goal_sent_at = None
        self.goal_source = ""

    # --- reporting ------------------------------------------------------------

    def _finish(self, verdict):
        self.finished = True
        self.finish_reason = verdict.trigger
        self._cancel_goal()
        self.finished_publisher.publish(Bool(data=True))
        self.get_logger().info(
            f"T1 finished ({'complete' if verdict.complete else 'cut short'}), "
            f"trigger={verdict.trigger}: {verdict.summary()}"
        )
        self.get_logger().info(
            f"drove {self.distance:.1f} m in "
            f"{self._now() - self._seconds(self.started):.0f} s"
        )

    def _publish_state(self, verdict, scored):
        self.state_publisher.publish(
            String(
                data=(
                    f"frontiers={len(scored)} distance={self.distance:.1f}m "
                    f"goal={self.goal_source or 'none'} | {verdict.summary()}"
                )
            )
        )

    def _publish_markers(self, scored):
        markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.g = 1.0
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0
        for frontier in scored:
            marker.points.append(Point(x=frontier.x, y=frontier.y, z=0.1))
        markers.markers.append(marker)

        if self.goal is not None:
            chosen = Marker()
            chosen.header = marker.header
            chosen.ns = "frontier_goal"
            chosen.id = 1
            chosen.type = Marker.SPHERE
            chosen.action = Marker.ADD
            chosen.scale.x = chosen.scale.y = chosen.scale.z = 0.35
            chosen.color.r = 1.0
            chosen.color.a = 0.9
            chosen.pose.position.x = self.goal[0]
            chosen.pose.position.y = self.goal[1]
            chosen.pose.position.z = 0.1
            chosen.pose.orientation.w = 1.0
            markers.markers.append(chosen)
        self.marker_publisher.publish(markers)

    # --- clock ----------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _seconds(stamp):
        return stamp.nanoseconds / 1e9


def main(args=None):
    """Run the frontier explorer node."""
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
