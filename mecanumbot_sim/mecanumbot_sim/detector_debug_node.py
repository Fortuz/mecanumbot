import math

import rclpy
from geometry_msgs.msg import Point, PoseArray
from mecanumbot_msgs.msg import SimActor, SimActorArray
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class MecanumbotSimDetectorDebugNode(Node):
    def __init__(self):
        super().__init__("mecanumbot_sim_detector_debug_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("actors_topic", "/sim/actors"),
                ("detections_topic", "dr_spaam/dets"),
                ("debug_markers_topic", "/sim/detector_debug_markers"),
                ("map_frame", "map"),
                ("publish_rate_hz", 4.0),
                ("max_detections", 8),
            ],
        )

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.max_detections = int(self.get_parameter("max_detections").value)
        self.latest_actors = None
        self.latest_detections = None
        self.last_tf_error = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            SimActorArray,
            str(self.get_parameter("actors_topic").value),
            self.actors_callback,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("detections_topic").value),
            self.detections_callback,
            10,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("debug_markers_topic").value),
            10,
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(
            1.0 / max(publish_rate_hz, 0.1), self.timer_callback
        )
        self.get_logger().info("Simulation detector debug markers started.")

    def actors_callback(self, msg: SimActorArray) -> None:
        self.latest_actors = msg

    def detections_callback(self, msg: PoseArray) -> None:
        self.latest_detections = msg

    def timer_callback(self) -> None:
        if not rclpy.ok():
            return

        header = None
        if self.latest_actors is not None:
            header = self.latest_actors.header
        elif self.latest_detections is not None:
            header = self.latest_detections.header
            header.frame_id = self.map_frame

        if header is None:
            return

        try:
            self.marker_publisher.publish(self.build_markers(header))
        except Exception:
            if rclpy.ok():
                raise

    def build_markers(self, header) -> MarkerArray:
        markers = [self.delete_all_marker(header)]
        marker_id = 1

        if self.latest_detections is None:
            markers.append(
                self.status_marker(
                    header, marker_id, "detector: waiting for detections"
                )
            )
            return MarkerArray(markers=markers)

        actors = []
        if self.latest_actors is not None:
            actors = [
                actor for actor in self.latest_actors.actors if actor.visible_to_lidar
            ]

        detection_points = self.detections_in_map()
        markers.append(
            self.status_marker(
                header,
                marker_id,
                f"detector debug: {len(detection_points)} dets, {len(actors)} truth actors",
            )
        )
        marker_id += 1

        for index, point in enumerate(detection_points[: self.max_detections]):
            markers.append(self.detection_sphere(header, point, marker_id))
            marker_id += 1

            nearest_actor, distance = self.nearest_actor(point, actors)
            if nearest_actor is not None:
                actor_point = self.actor_point(nearest_actor)
                markers.append(
                    self.association_line(
                        header, point, actor_point, marker_id, nearest_actor
                    )
                )
                marker_id += 1
                label = f"D{index + 1}->{nearest_actor.name}\n{distance:.2f}m"
            else:
                label = f"D{index + 1}->none"

            markers.append(self.detection_label(header, point, marker_id, label))
            marker_id += 1

        if self.last_tf_error:
            markers.append(
                self.status_marker(
                    header, marker_id, f"TF: {self.last_tf_error}", y=-1.65
                )
            )

        return MarkerArray(markers=markers)

    def detections_in_map(self) -> list[Point]:
        if self.latest_detections is None:
            return []

        source_frame = self.latest_detections.header.frame_id
        if not source_frame:
            self.last_tf_error = "empty detection frame"
            return []

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.last_tf_error = str(exc)
            return []

        self.last_tf_error = ""
        return [
            self.transform_point(pose.position, transform.transform)
            for pose in self.latest_detections.poses
        ]

    @staticmethod
    def transform_point(point, transform) -> Point:
        q = transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        out = Point()
        out.x = (
            transform.translation.x + point.x * math.cos(yaw) - point.y * math.sin(yaw)
        )
        out.y = (
            transform.translation.y + point.x * math.sin(yaw) + point.y * math.cos(yaw)
        )
        out.z = 0.10
        return out

    @staticmethod
    def nearest_actor(
        point: Point, actors: list[SimActor]
    ) -> tuple[SimActor | None, float]:
        if not actors:
            return None, float("inf")
        nearest = min(
            actors,
            key=lambda actor: MecanumbotSimDetectorDebugNode.distance(point, actor),
        )
        return nearest, MecanumbotSimDetectorDebugNode.distance(point, nearest)

    @staticmethod
    def distance(point: Point, actor: SimActor) -> float:
        return math.hypot(
            point.x - actor.pose.position.x, point.y - actor.pose.position.y
        )

    @staticmethod
    def actor_point(actor: SimActor) -> Point:
        point = Point()
        point.x = actor.pose.position.x
        point.y = actor.pose.position.y
        point.z = 0.10
        return point

    def detection_sphere(self, header, point: Point, marker_id: int) -> Marker:
        marker = self.base_marker(header, marker_id)
        marker.type = Marker.SPHERE
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        self.set_color(marker, 1.0, 0.10, 0.10, 0.95)
        return marker

    def association_line(
        self,
        header,
        detection_point: Point,
        actor_point: Point,
        marker_id: int,
        actor: SimActor,
    ) -> Marker:
        marker = self.base_marker(header, marker_id)
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.025
        marker.points.append(detection_point)
        marker.points.append(actor_point)
        if actor.kind == "wall":
            self.set_color(marker, 0.85, 0.85, 0.85, 0.85)
        elif actor.is_subject:
            self.set_color(marker, 0.10, 1.0, 0.25, 0.85)
        else:
            self.set_color(marker, 1.0, 0.50, 0.05, 0.85)
        return marker

    def detection_label(
        self, header, point: Point, marker_id: int, text: str
    ) -> Marker:
        marker = self.base_marker(header, marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.10
        marker.text = text
        self.set_color(marker, 1.0, 0.95, 0.05, 1.0)
        return marker

    def status_marker(
        self, header, marker_id: int, text: str, y: float = -1.35
    ) -> Marker:
        marker = self.base_marker(header, marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = 0.95
        marker.pose.position.y = y
        marker.pose.position.z = 0.75
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.10
        marker.text = text
        self.set_color(marker, 1.0, 0.95, 0.05, 1.0)
        return marker

    def base_marker(self, header, marker_id: int) -> Marker:
        marker = Marker()
        marker.header = header
        marker.header.frame_id = self.map_frame
        marker.ns = "sim_detector_debug"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime.sec = 1
        return marker

    def delete_all_marker(self, header) -> Marker:
        marker = Marker()
        marker.header = header
        marker.header.frame_id = self.map_frame
        marker.ns = "sim_detector_debug"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def set_color(
        marker: Marker, red: float, green: float, blue: float, alpha: float
    ) -> None:
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimDetectorDebugNode()
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
