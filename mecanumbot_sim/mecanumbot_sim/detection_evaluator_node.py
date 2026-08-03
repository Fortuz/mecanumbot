import math

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from mecanumbot_msgs.msg import SimActor, SimActorArray, SimDetectionEvaluation
from rclpy.node import Node


class MecanumbotSimDetectionEvaluatorNode(Node):
    def __init__(self):
        super().__init__("mecanumbot_sim_detection_evaluator_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("actors_topic", "/sim/actors"),
                ("subject_pose_topic", "subject_pose"),
                ("detections_topic", "dr_spaam/dets"),
                ("evaluation_topic", "/sim/detection_evaluation"),
                ("evaluation_rate_hz", 4.0),
                ("subject_match_radius", 0.55),
                ("wall_false_positive_radius", 0.55),
                ("require_raw_detection", True),
            ],
        )

        self.subject_match_radius = float(
            self.get_parameter("subject_match_radius").value
        )
        self.wall_false_positive_radius = float(
            self.get_parameter("wall_false_positive_radius").value
        )
        self.require_raw_detection = bool(
            self.get_parameter("require_raw_detection").value
        )
        evaluation_rate_hz = float(self.get_parameter("evaluation_rate_hz").value)

        self.latest_actors = None
        self.latest_subject_pose = None
        self.latest_detection_count = 0

        self.create_subscription(
            SimActorArray,
            str(self.get_parameter("actors_topic").value),
            self.actors_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("subject_pose_topic").value),
            self.subject_pose_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("detections_topic").value),
            self.detections_callback,
            10,
        )
        self.evaluation_publisher = self.create_publisher(
            SimDetectionEvaluation,
            str(self.get_parameter("evaluation_topic").value),
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(evaluation_rate_hz, 0.1), self.timer_callback
        )
        self.get_logger().info("Simulation detection evaluator started.")

    def actors_callback(self, msg: SimActorArray) -> None:
        self.latest_actors = msg

    def subject_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_subject_pose = msg

    def detections_callback(self, msg: PoseArray) -> None:
        self.latest_detection_count = len(msg.poses)

    def timer_callback(self) -> None:
        if not rclpy.ok() or self.latest_actors is None:
            return

        evaluation = self.build_evaluation()
        try:
            self.evaluation_publisher.publish(evaluation)
        except Exception:
            if rclpy.ok():
                raise

    def build_evaluation(self) -> SimDetectionEvaluation:
        msg = SimDetectionEvaluation()
        msg.header = self.latest_actors.header
        msg.scenario_name = self.latest_actors.scenario_name
        msg.nearest_actor_id = -1
        msg.raw_detection_count = int(self.latest_detection_count)
        msg.raw_detection_available = self.latest_detection_count > 0
        msg.nearest_actor_distance = float("inf")
        msg.subject_error = float("inf")
        msg.nearest_wall_distance = float("inf")

        if self.latest_subject_pose is None:
            msg.status = "no_detection"
            return msg

        msg.detection_available = True
        actors = [
            actor for actor in self.latest_actors.actors if actor.visible_to_lidar
        ]
        if not actors:
            msg.status = "no_visible_actors"
            return msg

        subject_actor = self.find_subject_actor(actors)
        nearest_actor, nearest_distance = self.find_nearest_actor(
            actors, self.latest_subject_pose
        )
        nearest_wall, nearest_wall_distance = self.find_nearest_wall(
            actors, self.latest_subject_pose
        )

        if nearest_actor is not None:
            msg.nearest_actor_id = nearest_actor.id
            msg.nearest_actor_name = nearest_actor.name
            msg.nearest_actor_kind = nearest_actor.kind
            msg.nearest_actor_distance = float(nearest_distance)

        if nearest_wall is not None:
            msg.nearest_wall_distance = float(nearest_wall_distance)

        if subject_actor is None:
            msg.status = "subject_missing"
            return msg

        msg.subject_error = float(
            self.distance_to_actor(self.latest_subject_pose, subject_actor)
        )
        msg.false_wall_lock = (
            nearest_actor is not None
            and nearest_actor.kind == "wall"
            and nearest_distance <= self.wall_false_positive_radius
        )
        msg.wrong_human_lock = (
            nearest_actor is not None
            and nearest_actor.kind == "human"
            and not nearest_actor.is_subject
            and nearest_distance <= self.subject_match_radius
        )
        raw_detection_ok = msg.raw_detection_available or not self.require_raw_detection
        msg.subject_tracking_ok = (
            raw_detection_ok
            and msg.subject_error <= self.subject_match_radius
            and not msg.false_wall_lock
            and not msg.wrong_human_lock
        )

        if msg.subject_tracking_ok and self.require_raw_detection:
            msg.status = "tracking_subject"
        elif msg.subject_tracking_ok:
            msg.status = "oracle_tracking_subject"
        elif not msg.raw_detection_available:
            msg.status = "pose_memory_only"
        elif msg.false_wall_lock:
            msg.status = "false_wall_lock"
        elif msg.wrong_human_lock:
            msg.status = "wrong_human_lock"
        else:
            msg.status = "subject_error_high"

        return msg

    @staticmethod
    def find_subject_actor(actors: list[SimActor]) -> SimActor | None:
        for actor in actors:
            if actor.is_subject:
                return actor
        return None

    @classmethod
    def find_nearest_actor(
        cls, actors: list[SimActor], pose: PoseStamped
    ) -> tuple[SimActor | None, float]:
        if not actors:
            return None, float("inf")
        nearest_actor = min(
            actors, key=lambda actor: cls.distance_to_actor(pose, actor)
        )
        return nearest_actor, cls.distance_to_actor(pose, nearest_actor)

    @classmethod
    def find_nearest_wall(
        cls, actors: list[SimActor], pose: PoseStamped
    ) -> tuple[SimActor | None, float]:
        wall_actors = [actor for actor in actors if actor.kind == "wall"]
        if not wall_actors:
            return None, float("inf")
        nearest_wall = min(
            wall_actors, key=lambda actor: cls.distance_to_actor(pose, actor)
        )
        return nearest_wall, cls.distance_to_actor(pose, nearest_wall)

    @staticmethod
    def distance_to_actor(pose: PoseStamped, actor: SimActor) -> float:
        dx = pose.pose.position.x - actor.pose.position.x
        dy = pose.pose.position.y - actor.pose.position.y
        return math.hypot(dx, dy)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimDetectionEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
