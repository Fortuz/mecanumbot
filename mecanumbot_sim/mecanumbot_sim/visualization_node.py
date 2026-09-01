import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from mecanumbot_msgs.msg import (
    SimActor,
    SimActorArray,
    SimBehaviorEvaluation,
    SimDetectionEvaluation,
)
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from mecanumbot_sim.sim_scenarios import (
    ARM_LENGTH,
    HIP_Z,
    LEG_LATERAL_OFFSET,
    LEG_LENGTH,
    SHOULDER_LATERAL,
    SHOULDER_LOCAL_Z,
    SPINE_BASE_Z,
    TORSO_LOWER_LENGTH,
    TORSO_UPPER_LENGTH,
    YOGA_MAT_LENGTH,
    YOGA_MAT_WIDTH,
    load_sim_scenario,
)

#: Fallback prop extents, used when no scenario path is given. Props carry no size
#: on the wire, so the marker sizes come from the scenario file the backend loaded.
DEFAULT_CUBE_EDGE = 0.07
MAT_THICKNESS = 0.006


class MecanumbotSimVisualizationNode(Node):
    def __init__(self):
        super().__init__("mecanumbot_sim_visualization_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("actors_topic", "/sim/actors"),
                ("props_topic", "/sim/props"),
                ("detections_topic", "dr_spaam/dets"),
                ("subject_pose_topic", "subject_pose"),
                ("detection_evaluation_topic", "/sim/detection_evaluation"),
                ("behavior_evaluation_topic", "/sim/behavior_evaluation"),
                ("actor_markers_topic", "/sim/actor_markers"),
                ("prop_markers_topic", "/sim/prop_markers"),
                ("scenario_path", ""),
                ("detection_markers_topic", "/sim/detection_markers"),
                ("evaluation_markers_topic", "/sim/evaluation_markers"),
                ("publish_rate_hz", 4.0),
                ("show_detection_markers", True),
            ],
        )

        self.show_detection_markers = bool(
            self.get_parameter("show_detection_markers").value
        )
        self.prop_geometry = self.load_prop_geometry(
            str(self.get_parameter("scenario_path").value)
        )
        self.latest_actors = None
        self.latest_props = None
        self.latest_detections = None
        self.latest_subject_pose = None
        self.latest_detection_evaluation = None
        self.latest_behavior_evaluation = None

        self.create_subscription(
            SimActorArray,
            str(self.get_parameter("actors_topic").value),
            self.actors_callback,
            10,
        )
        self.create_subscription(
            SimActorArray,
            str(self.get_parameter("props_topic").value),
            self.props_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("detections_topic").value),
            self.detections_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("subject_pose_topic").value),
            self.subject_pose_callback,
            10,
        )
        self.create_subscription(
            SimDetectionEvaluation,
            str(self.get_parameter("detection_evaluation_topic").value),
            self.detection_evaluation_callback,
            10,
        )
        self.create_subscription(
            SimBehaviorEvaluation,
            str(self.get_parameter("behavior_evaluation_topic").value),
            self.behavior_evaluation_callback,
            10,
        )

        self.actor_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("actor_markers_topic").value),
            10,
        )
        self.prop_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("prop_markers_topic").value),
            10,
        )
        self.detection_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("detection_markers_topic").value),
            10,
        )
        self.evaluation_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("evaluation_markers_topic").value),
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(
            1.0 / max(publish_rate_hz, 0.1), self.timer_callback
        )
        self.get_logger().info("Simulation visualization labels started.")

    def load_prop_geometry(self, scenario_path: str) -> dict:
        """
        Map prop name -> (body_name, size) from the scenario the backend loaded.

        `/sim/props` carries poses only, so without the scenario the markers would
        have to guess how big a mat or a brick is. A missing or unreadable path is
        not fatal: the defaults above still draw something recognisable.
        """
        if not scenario_path:
            return {}
        try:
            scenario = load_sim_scenario(scenario_path)
        except (OSError, ValueError) as error:
            self.get_logger().warning(f"Prop marker sizes unavailable: {error}")
            return {}
        return {prop.name: prop for prop in scenario.props}

    def actors_callback(self, msg: SimActorArray) -> None:
        self.latest_actors = msg

    def props_callback(self, msg: SimActorArray) -> None:
        self.latest_props = msg

    def detections_callback(self, msg: PoseArray) -> None:
        self.latest_detections = msg

    def subject_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_subject_pose = msg

    def detection_evaluation_callback(self, msg: SimDetectionEvaluation) -> None:
        self.latest_detection_evaluation = msg

    def behavior_evaluation_callback(self, msg: SimBehaviorEvaluation) -> None:
        self.latest_behavior_evaluation = msg

    def timer_callback(self) -> None:
        if not rclpy.ok():
            return

        try:
            if self.latest_actors is not None:
                self.actor_marker_publisher.publish(
                    self.build_actor_markers(self.latest_actors)
                )
            if self.latest_props is not None:
                self.prop_marker_publisher.publish(
                    self.build_prop_markers(self.latest_props)
                )
            if self.show_detection_markers and self.latest_detections is not None:
                self.detection_marker_publisher.publish(
                    self.build_detection_markers(self.latest_detections)
                )
            self.evaluation_marker_publisher.publish(self.build_evaluation_markers())
        except Exception:
            if rclpy.ok():
                raise

    def build_actor_markers(self, actors_msg: SimActorArray) -> MarkerArray:
        markers = [
            self.delete_all_marker(actors_msg.header, "sim_actors"),
            self.delete_all_marker(actors_msg.header, "sim_world"),
        ]
        marker_id = 1
        world_markers = self.world_body_markers(actors_msg.header, marker_id)
        markers.extend(world_markers)
        marker_id += len(world_markers)
        for actor in actors_msg.actors:
            body_markers = self.actor_body_markers(actors_msg.header, actor, marker_id)
            markers.extend(body_markers)
            marker_id += len(body_markers)
            markers.append(self.actor_label_marker(actors_msg.header, actor, marker_id))
            marker_id += 1
            if self.is_actor_moving(actor):
                markers.append(
                    self.actor_velocity_marker(actors_msg.header, actor, marker_id)
                )
                marker_id += 1
        return MarkerArray(markers=markers)

    def build_prop_markers(self, props_msg: SimActorArray) -> MarkerArray:
        markers = [
            self.delete_all_marker(props_msg.header, "sim_props"),
            self.delete_all_marker(props_msg.header, "sim_prop_labels"),
        ]
        marker_id = 1
        for prop in props_msg.actors:
            markers.append(self.prop_body_marker(props_msg.header, prop, marker_id))
            marker_id += 1
            markers.append(self.prop_label_marker(props_msg.header, prop, marker_id))
            marker_id += 1
        return MarkerArray(markers=markers)

    def prop_body_marker(self, header, prop: SimActor, marker_id: int) -> Marker:
        config = self.prop_geometry.get(prop.name)
        marker = self.base_marker(header, "sim_props", marker_id)
        marker.type = Marker.CUBE
        marker.pose = deepcopy(prop.pose)

        if prop.kind == "mat":
            length, width = (
                config.mat_extents()
                if config is not None
                else (YOGA_MAT_LENGTH, YOGA_MAT_WIDTH)
            )
            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = MAT_THICKNESS
            marker.pose.position.z = 0.5 * MAT_THICKNESS
            # The second mat in the pool is the drop-off pad, not the exercise mat.
            if config is not None and config.body_name == "sim_mat_1":
                self.set_color(marker, 0.95, 0.80, 0.15, 0.55)
            else:
                self.set_color(marker, 0.16, 0.45, 0.55, 0.55)
            return marker

        edge = float(config.size[0]) if config and config.size else DEFAULT_CUBE_EDGE
        marker.scale.x = edge
        marker.scale.y = edge
        marker.scale.z = edge
        self.set_color(marker, 0.72, 0.30, 0.20, 0.90)
        return marker

    def prop_label_marker(self, header, prop: SimActor, marker_id: int) -> Marker:
        marker = self.base_marker(header, "sim_prop_labels", marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = deepcopy(prop.pose)
        marker.pose.position.z = 0.22 if prop.kind == "cube" else 0.10
        marker.scale.z = 0.10
        self.set_color(marker, 0.90, 0.90, 0.90, 0.95)
        marker.text = prop.name
        return marker

    def build_detection_markers(self, detections_msg: PoseArray) -> MarkerArray:
        markers = [
            self.delete_all_marker(detections_msg.header, "sim_detection_labels")
        ]
        marker_id = 1
        for index, pose in enumerate(detections_msg.poses):
            markers.append(
                self.detection_label_marker(
                    detections_msg.header, pose, index, marker_id
                )
            )
            marker_id += 1

        if self.latest_subject_pose is not None:
            markers.append(
                self.tracked_subject_marker(self.latest_subject_pose, marker_id)
            )
            marker_id += 1
            markers.append(
                self.tracked_subject_label_marker(self.latest_subject_pose, marker_id)
            )
        return MarkerArray(markers=markers)

    def build_evaluation_markers(self) -> MarkerArray:
        header = None
        if self.latest_actors is not None:
            header = self.latest_actors.header
        elif self.latest_detection_evaluation is not None:
            header = self.latest_detection_evaluation.header
        elif self.latest_behavior_evaluation is not None:
            header = self.latest_behavior_evaluation.header

        if header is None:
            return MarkerArray()

        markers = [self.delete_all_marker(header, "sim_evaluation_status")]
        markers.append(self.status_text_marker(header, 1))
        return MarkerArray(markers=markers)

    def actor_body_markers(
        self, header, actor: SimActor, marker_id: int
    ) -> list[Marker]:
        if actor.kind == "wall":
            return [self.wall_body_marker(header, actor, marker_id)]
        return self.human_body_markers(header, actor, marker_id)

    def wall_body_marker(self, header, actor: SimActor, marker_id: int) -> Marker:
        marker = self.base_marker(header, "sim_actors", marker_id)
        marker.color.a = 0.65
        self.apply_actor_color(marker, actor)
        marker.type = Marker.CUBE
        marker.pose = deepcopy(actor.pose)
        marker.pose.position.z = 0.60
        marker.scale.x = 1.10
        marker.scale.y = 0.10
        marker.scale.z = 1.20
        return marker

    def world_body_markers(self, header, marker_id: int) -> list[Marker]:
        # Fixed MuJoCo world geometry that contributes to LiDAR but is not a scenario actor.
        specs = [
            ("wall_north", 0.0, 2.0, 0.25, 4.40, 0.10, 0.50, 0.60, 0.60, 0.60),
            ("wall_south", 0.0, -2.0, 0.25, 4.40, 0.10, 0.50, 0.60, 0.60, 0.60),
            ("wall_east", 2.0, 0.0, 0.25, 0.10, 4.40, 0.50, 0.60, 0.60, 0.60),
            ("wall_west", -2.0, 0.0, 0.25, 0.10, 4.40, 0.50, 0.60, 0.60, 0.60),
            ("box_obstacle_a", 0.9, 0.4, 0.15, 0.36, 0.36, 0.30, 0.0, 0.80, 0.0),
            ("box_obstacle_b", -0.7, -0.8, 0.20, 0.24, 0.50, 0.40, 1.0, 0.50, 0.05),
        ]
        markers = []
        for offset, (_, x, y, z, sx, sy, sz, red, green, blue) in enumerate(specs):
            marker = self.base_marker(header, "sim_world", marker_id + offset)
            marker.type = Marker.CUBE
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = sx
            marker.scale.y = sy
            marker.scale.z = sz
            self.set_color(marker, red, green, blue, 0.25)
            markers.append(marker)
        return markers

    def human_body_markers(
        self, header, actor: SimActor, marker_id: int
    ) -> list[Marker]:
        """
        Draw the figure in its neutral stance, at the dimensions the MJCF uses.

        `/sim/actors` carries the actor's ground pose and nothing else, so RViz
        cannot show the articulation an exercise gait produces - see the README.
        What it can do is agree with the simulator at rest, which is what the
        segment lengths below are for.
        """
        # (part, local y, centre z, diameter, height, colour)
        parts = (
            (
                "torso_lower",
                0.0,
                SPINE_BASE_Z + TORSO_LOWER_LENGTH / 2.0,
                0.32,
                TORSO_LOWER_LENGTH,
                (1.00, 0.42, 0.04),
            ),
            (
                "torso_upper",
                0.0,
                SPINE_BASE_Z + TORSO_LOWER_LENGTH + TORSO_UPPER_LENGTH / 2.0,
                0.29,
                TORSO_UPPER_LENGTH,
                (1.00, 0.42, 0.04),
            ),
            (
                "arm_left",
                SHOULDER_LATERAL,
                SPINE_BASE_Z + TORSO_LOWER_LENGTH + SHOULDER_LOCAL_Z - ARM_LENGTH / 2.0,
                0.10,
                ARM_LENGTH,
                (0.95, 0.55, 0.15),
            ),
            (
                "arm_right",
                -SHOULDER_LATERAL,
                SPINE_BASE_Z + TORSO_LOWER_LENGTH + SHOULDER_LOCAL_Z - ARM_LENGTH / 2.0,
                0.10,
                ARM_LENGTH,
                (0.95, 0.55, 0.15),
            ),
            (
                "leg_left",
                LEG_LATERAL_OFFSET,
                HIP_Z - LEG_LENGTH / 2.0,
                0.12,
                LEG_LENGTH,
                (0.00, 0.80, 0.00),
            ),
            (
                "leg_right",
                -LEG_LATERAL_OFFSET,
                HIP_Z - LEG_LENGTH / 2.0,
                0.12,
                LEG_LENGTH,
                (0.00, 0.80, 0.00),
            ),
        )

        markers = []
        for offset, (_, local_y, z, diameter, height, colour) in enumerate(parts):
            marker = self.human_part_marker(
                header, actor, marker_id + offset, 0.0, local_y, z, diameter, height
            )
            self.set_color(marker, *colour, 0.70)
            markers.append(marker)

        head = self.human_part_marker(
            header,
            actor,
            marker_id + len(parts),
            0.0,
            0.0,
            SPINE_BASE_Z + TORSO_LOWER_LENGTH + 0.38,
            0.17,
            0.17,
        )
        head.type = Marker.SPHERE
        self.set_color(head, 0.90, 0.72, 0.58, 0.85)
        markers.append(head)
        return markers

    def human_part_marker(
        self,
        header,
        actor: SimActor,
        marker_id: int,
        local_x: float,
        local_y: float,
        z: float,
        diameter: float,
        height: float,
    ) -> Marker:
        marker = self.base_marker(header, "sim_actors", marker_id)
        marker.type = Marker.CYLINDER
        marker.pose = deepcopy(actor.pose)
        world_x, world_y = self.local_actor_offset(actor, local_x, local_y)
        marker.pose.position.x = world_x
        marker.pose.position.y = world_y
        marker.pose.position.z = z
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = height
        return marker

    def actor_label_marker(self, header, actor: SimActor, marker_id: int) -> Marker:
        marker = self.base_marker(header, "sim_actor_labels", marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = deepcopy(actor.pose)
        marker.scale.z = 0.12
        marker.color.a = 1.0
        self.apply_actor_color(marker, actor)

        if actor.kind == "wall":
            marker.pose.position.z = 0.16
            marker.scale.z = 0.10
            marker.text = "WALL"
        elif actor.is_subject:
            marker.pose.position.z = 1.05
            marker.pose.position.y -= 0.10
            marker.text = "SUBJECT"
        else:
            marker.pose.position.z = 0.95
            marker.pose.position.y += 0.10
            marker.text = "DISTRACTOR"
        return marker

    def actor_velocity_marker(self, header, actor: SimActor, marker_id: int) -> Marker:
        marker = self.base_marker(header, "sim_actor_velocity", marker_id)
        marker.type = Marker.ARROW
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.scale.y = 0.055
        marker.scale.z = 0.075
        marker.color.a = 0.9
        self.apply_actor_color(marker, actor)

        speed = math.hypot(actor.twist.linear.x, actor.twist.linear.y)
        if speed <= 0.001:
            return marker

        start = Point()
        start.x = actor.pose.position.x
        start.y = actor.pose.position.y
        start.z = 0.08
        end = Point()
        end.x = start.x + 0.35 * actor.twist.linear.x / speed
        end.y = start.y + 0.35 * actor.twist.linear.y / speed
        end.z = start.z
        marker.points.append(start)
        marker.points.append(end)
        return marker

    def detection_label_marker(
        self, header, pose, index: int, marker_id: int
    ) -> Marker:
        marker = self.base_marker(header, "sim_detection_labels", marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = deepcopy(pose)
        marker.pose.position.z = 0.30
        marker.scale.z = 0.11
        marker.color.r = 1.0
        marker.color.g = 0.95
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.text = f"D{index + 1}"
        return marker

    def tracked_subject_marker(self, pose_msg: PoseStamped, marker_id: int) -> Marker:
        marker = self.base_marker(pose_msg.header, "sim_tracked_subject", marker_id)
        marker.type = Marker.SPHERE
        marker.pose = deepcopy(pose_msg.pose)
        marker.pose.position.z = 0.05
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.15
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 0.9
        return marker

    def tracked_subject_label_marker(
        self, pose_msg: PoseStamped, marker_id: int
    ) -> Marker:
        marker = self.base_marker(pose_msg.header, "sim_tracked_subject", marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose = deepcopy(pose_msg.pose)
        marker.pose.position.z = 0.22
        marker.scale.z = 0.10
        marker.color.r = 0.15
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "TRACK"
        return marker

    def status_text_marker(self, header, marker_id: int) -> Marker:
        marker = self.base_marker(header, "sim_evaluation_status", marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = -0.70
        marker.pose.position.y = -1.35
        marker.pose.position.z = 0.75
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.12
        marker.color.a = 1.0

        detection_status = "waiting"
        detection_actor = ""
        if self.latest_detection_evaluation is not None:
            detection_status = self.latest_detection_evaluation.status
            detection_actor = self.latest_detection_evaluation.nearest_actor_name

        behavior_status = "waiting"
        if self.latest_behavior_evaluation is not None:
            behavior_status = self.latest_behavior_evaluation.status

        unsafe = (
            self.latest_behavior_evaluation is not None
            and self.latest_behavior_evaluation.unsafe_motion
        )
        if unsafe:
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.10
        elif (
            self.latest_detection_evaluation is not None
            and self.latest_detection_evaluation.subject_tracking_ok
        ):
            marker.color.r = 0.20
            marker.color.g = 1.0
            marker.color.b = 0.25
        else:
            marker.color.r = 1.0
            marker.color.g = 0.85
            marker.color.b = 0.10

        actor_line = f"near: {detection_actor}" if detection_actor else "near: none"
        marker.text = f"P: {detection_status}\n{actor_line}\nB: {behavior_status}"
        return marker

    @staticmethod
    def base_marker(header, namespace: str, marker_id: int) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    @staticmethod
    def delete_all_marker(header, namespace: str) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def apply_actor_color(marker: Marker, actor: SimActor) -> None:
        if actor.kind == "wall":
            MecanumbotSimVisualizationNode.set_color(
                marker, 0.60, 0.60, 0.60, marker.color.a
            )
        elif actor.is_subject:
            MecanumbotSimVisualizationNode.set_color(
                marker, 0.10, 1.00, 0.25, marker.color.a
            )
        else:
            MecanumbotSimVisualizationNode.set_color(
                marker, 1.00, 0.50, 0.05, marker.color.a
            )

    @staticmethod
    def set_color(
        marker: Marker, red: float, green: float, blue: float, alpha: float
    ) -> None:
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    @staticmethod
    def local_actor_offset(
        actor: SimActor, local_x: float, local_y: float
    ) -> tuple[float, float]:
        q = actor.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        x = actor.pose.position.x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        y = actor.pose.position.y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
        return x, y

    @staticmethod
    def is_actor_moving(actor: SimActor) -> bool:
        speed = math.hypot(actor.twist.linear.x, actor.twist.linear.y)
        return speed > 0.01 or abs(actor.twist.angular.z) > 0.01


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimVisualizationNode()
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
