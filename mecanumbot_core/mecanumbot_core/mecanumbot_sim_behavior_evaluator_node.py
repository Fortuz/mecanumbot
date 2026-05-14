import math

import rclpy
from geometry_msgs.msg import Twist
from mecanumbot_msgs.msg import SimActor, SimActorArray, SimBehaviorEvaluation, SimDetectionEvaluation
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MecanumbotSimBehaviorEvaluatorNode(Node):
    def __init__(self):
        super().__init__('mecanumbot_sim_behavior_evaluator_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('actors_topic', '/sim/actors'),
                ('detection_evaluation_topic', '/sim/detection_evaluation'),
                ('odom_topic', 'odom'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('behavior_evaluation_topic', '/sim/behavior_evaluation'),
                ('evaluation_rate_hz', 4.0),
                ('moving_speed_threshold', 0.03),
                ('human_stop_distance', 0.45),
                ('wall_stop_distance', 0.35),
                ('follow_min_distance', 0.55),
                ('follow_max_distance', 2.00),
            ],
        )

        self.moving_speed_threshold = float(self.get_parameter('moving_speed_threshold').value)
        self.human_stop_distance = float(self.get_parameter('human_stop_distance').value)
        self.wall_stop_distance = float(self.get_parameter('wall_stop_distance').value)
        self.follow_min_distance = float(self.get_parameter('follow_min_distance').value)
        self.follow_max_distance = float(self.get_parameter('follow_max_distance').value)

        self.latest_actors = None
        self.latest_detection_evaluation = None
        self.latest_odom = None
        self.latest_cmd_vel = None

        self.create_subscription(
            SimActorArray,
            str(self.get_parameter('actors_topic').value),
            self.actors_callback,
            10,
        )
        self.create_subscription(
            SimDetectionEvaluation,
            str(self.get_parameter('detection_evaluation_topic').value),
            self.detection_evaluation_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self.cmd_vel_callback,
            10,
        )
        self.behavior_publisher = self.create_publisher(
            SimBehaviorEvaluation,
            str(self.get_parameter('behavior_evaluation_topic').value),
            10,
        )

        evaluation_rate_hz = float(self.get_parameter('evaluation_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(evaluation_rate_hz, 0.1), self.timer_callback)
        self.get_logger().info('Simulation behavior evaluator started.')

    def actors_callback(self, msg: SimActorArray) -> None:
        self.latest_actors = msg

    def detection_evaluation_callback(self, msg: SimDetectionEvaluation) -> None:
        self.latest_detection_evaluation = msg

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.latest_cmd_vel = msg

    def timer_callback(self) -> None:
        if not rclpy.ok():
            return

        evaluation = self.build_evaluation()
        try:
            self.behavior_publisher.publish(evaluation)
        except Exception:
            if rclpy.ok():
                raise

    def build_evaluation(self) -> SimBehaviorEvaluation:
        msg = SimBehaviorEvaluation()
        msg.nearest_actor_distance = float('inf')
        msg.nearest_wall_distance = float('inf')
        msg.nearest_human_distance = float('inf')
        msg.subject_distance = float('inf')

        if self.latest_actors is not None:
            msg.header = self.latest_actors.header
            msg.scenario_name = self.latest_actors.scenario_name

        if self.latest_actors is None:
            msg.status = 'waiting_for_actors'
            msg.reason = 'No /sim/actors message received yet.'
            return msg
        if self.latest_odom is None:
            msg.status = 'waiting_for_odom'
            msg.reason = 'No odometry message received yet.'
            return msg

        cmd = self.latest_cmd_vel or Twist()
        msg.command_available = self.latest_cmd_vel is not None
        msg.commanded_speed = float(self.commanded_speed(cmd))
        msg.robot_moving = msg.commanded_speed > self.moving_speed_threshold

        actors = [actor for actor in self.latest_actors.actors if actor.visible_to_lidar]
        nearest_actor, nearest_actor_distance = self.find_nearest_actor(actors)
        nearest_wall, nearest_wall_distance = self.find_nearest_kind(actors, 'wall')
        nearest_human, nearest_human_distance = self.find_nearest_kind(actors, 'human')
        subject = self.find_subject_actor(actors)

        if nearest_actor is not None:
            msg.nearest_actor_name = nearest_actor.name
            msg.nearest_actor_kind = nearest_actor.kind
            msg.nearest_actor_distance = float(nearest_actor_distance)
        if nearest_wall is not None:
            msg.nearest_wall_distance = float(nearest_wall_distance)
        if nearest_human is not None:
            msg.nearest_human_distance = float(nearest_human_distance)
        if subject is not None:
            msg.subject_distance = float(self.distance_to_actor(subject))

        detection = self.latest_detection_evaluation
        if detection is None:
            msg.target_valid = False
            msg.status = 'waiting_for_detection_evaluation'
            msg.reason = 'No /sim/detection_evaluation message received yet.'
            return msg

        msg.target_valid = bool(detection.subject_tracking_ok)
        msg.unsafe_wall_proximity = msg.nearest_wall_distance < self.wall_stop_distance
        msg.unsafe_proximity = msg.nearest_human_distance < self.human_stop_distance
        msg.unsafe_wrong_target_motion = msg.robot_moving and (
            bool(detection.false_wall_lock)
            or bool(detection.wrong_human_lock)
            or detection.status in ('false_wall_lock', 'wrong_human_lock', 'subject_error_high')
        )
        msg.unsafe_stale_detection_motion = msg.robot_moving and (
            not msg.target_valid
            and (
                not bool(detection.raw_detection_available)
                or detection.status in ('no_detection', 'pose_memory_only')
            )
        )
        too_close_to_subject = subject is not None and msg.subject_distance < self.follow_min_distance
        too_far_from_subject = subject is not None and msg.subject_distance > self.follow_max_distance

        msg.unsafe_motion = (
            msg.robot_moving
            and (
                msg.unsafe_proximity
                or msg.unsafe_wall_proximity
                or msg.unsafe_wrong_target_motion
                or msg.unsafe_stale_detection_motion
                or too_close_to_subject
            )
        )

        if msg.unsafe_motion and msg.unsafe_stale_detection_motion:
            msg.status = 'unsafe_motion_with_stale_detection'
            msg.reason = 'Robot is moving while raw detector output is missing or stale.'
        elif msg.unsafe_motion and msg.unsafe_wrong_target_motion:
            msg.status = 'unsafe_motion_with_invalid_target'
            msg.reason = f'Detector association is {detection.status}.'
        elif msg.unsafe_motion and msg.unsafe_proximity:
            msg.status = 'unsafe_close_to_human'
            msg.reason = 'Robot is moving inside the human stop distance.'
        elif msg.unsafe_motion and msg.unsafe_wall_proximity:
            msg.status = 'unsafe_close_to_wall'
            msg.reason = 'Robot is moving inside the wall stop distance.'
        elif msg.robot_moving and too_close_to_subject:
            msg.status = 'unsafe_too_close_to_subject'
            msg.reason = 'Robot is moving closer than the follow minimum distance.'
        elif not msg.robot_moving and not msg.target_valid:
            msg.status = 'safe_idle_invalid_target'
            msg.reason = 'Target is invalid, but robot is stopped.'
        elif not msg.robot_moving:
            msg.status = 'safe_idle'
            msg.reason = 'Robot is stopped.'
        elif msg.target_valid and too_far_from_subject:
            msg.status = 'safe_approaching_subject'
            msg.reason = 'Robot is moving with a valid target outside the follow band.'
        elif msg.target_valid:
            msg.status = 'safe_following_subject'
            msg.reason = 'Robot is moving with a valid target and safe distances.'
        else:
            msg.status = 'caution_moving_without_valid_target'
            msg.reason = f'Detector association is {detection.status}.'

        return msg

    @staticmethod
    def commanded_speed(cmd: Twist) -> float:
        linear = math.hypot(cmd.linear.x, cmd.linear.y)
        return math.hypot(linear, cmd.angular.z)

    @staticmethod
    def find_subject_actor(actors: list[SimActor]) -> SimActor | None:
        for actor in actors:
            if actor.is_subject:
                return actor
        return None

    def find_nearest_actor(self, actors: list[SimActor]) -> tuple[SimActor | None, float]:
        if not actors:
            return None, float('inf')
        actor = min(actors, key=self.distance_to_actor)
        return actor, self.distance_to_actor(actor)

    def find_nearest_kind(self, actors: list[SimActor], kind: str) -> tuple[SimActor | None, float]:
        candidates = [actor for actor in actors if actor.kind == kind]
        if not candidates:
            return None, float('inf')
        actor = min(candidates, key=self.distance_to_actor)
        return actor, self.distance_to_actor(actor)

    def distance_to_actor(self, actor: SimActor) -> float:
        dx = self.latest_odom.pose.pose.position.x - actor.pose.position.x
        dy = self.latest_odom.pose.pose.position.y - actor.pose.position.y
        return math.hypot(dx, dy)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimBehaviorEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
