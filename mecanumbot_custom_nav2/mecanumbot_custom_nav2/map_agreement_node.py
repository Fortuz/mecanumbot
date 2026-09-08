"""
The robot's end of the 2D/3D comparison: turning a verdict into navigation.

The server compares its point cloud against the robot's occupancy grid and says
where they disagree. This node takes that verdict and produces the three things
the rest of the robot can actually use:

* a **keepout mask** aligned to `/map`, marking the places the cloud says there
  is structure the lidar could not see. Published with its `CostmapFilterInfo`
  so `nav2_costmap_2d::KeepoutFilter` consumes it directly -- no new costmap
  plugin, no patch to nav2, just a mask on a topic;
* a **revisit list** of the regions still worth looking at again, which the
  frontier explorer interleaves with its own goals;
* **markers**, colour-coded by kind, because an invisible keepout that stops
  the robot in an apparently empty corridor is otherwise unexplainable.

Why this is a node of its own rather than more code in the explorer: the
explorer exists only during T1, and the keepout matters just as much in T2, when
the robot is driving to an object across a room it mapped an hour ago. The
policy -- `agreement.py` -- is the same in both phases, so it runs in one place
and both phases subscribe.

    deep3r/map_agreement ──►  AgreementModel  ──┬──► deep3r/keepout_mask
    /map  (for the grid shape) ─────────────────┤    deep3r/costmap_filter_info
                                                ├──► deep3r/revisit_regions
                                                └──► deep3r/agreement_markers

**It only ever adds obstacles.** A `map_only` region -- the lidar seeing
something the camera could not reconstruct, which is what glass and thin dark
panels look like -- subtracts nothing. The reconstruction is not allowed to talk
the robot into driving somewhere the lidar says it cannot.
"""

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mecanumbot_msgs.msg import MapCloudAgreement

from mecanumbot_custom_nav2.agreement import (
    CLOUD_ONLY,
    DISAGREEMENT,
    KEEPOUT,
    MAP_ONLY,
    REVISIT,
    UNOBSERVED,
    AgreementModel,
    stamp_mask,
)

# Costmap filter masks and their info are latched: nav2's filter subscribes once
# at configure time and must get the current mask, not wait for the next one.
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# Marker colours per kind: red for what the robot must avoid, amber for what it
# should go and look at, grey for what it is deliberately ignoring.
KIND_COLOURS = {
    CLOUD_ONLY: (1.0, 0.2, 0.1),
    MAP_ONLY: (0.3, 0.5, 1.0),
    UNOBSERVED: (1.0, 0.7, 0.1),
    DISAGREEMENT: (0.7, 0.7, 0.7),
}


class MapAgreementHandler(Node):
    """Consume the server's comparison verdict and act on it."""

    def __init__(self):
        super().__init__("mecanumbot_map_agreement")
        self._declare_parameters()

        self.grid_info = None
        self.verdicts = 0

        self.model = AgreementModel(
            clearance=self._get("robot_clearance"),
            lidar_height=self._get("lidar_height"),
            robot_height=self._get("robot_height"),
            min_keepout_score=self._get("min_keepout_score"),
            min_revisit_score=self._get("min_revisit_score"),
            merge_radius=self._get("merge_radius"),
            forget_after=self._get("forget_after"),
            serviced_radius=self._get("serviced_radius"),
        )

        self.create_subscription(
            MapCloudAgreement, self._get("agreement_topic"), self._on_agreement, 10
        )
        self.create_subscription(
            OccupancyGrid, self._get("map_topic"), self._on_map, LATCHED
        )

        self.mask_publisher = self.create_publisher(
            OccupancyGrid, self._get("mask_topic"), LATCHED
        )
        self.info_publisher = self.create_publisher(
            CostmapFilterInfo, self._get("filter_info_topic"), LATCHED
        )
        self.revisit_publisher = self.create_publisher(
            PoseArray, self._get("revisit_topic"), 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self._get("marker_topic"), 10
        )

        # Regions are marked serviced by the robot simply having been near
        # them, read off TF rather than off any one goal. It works the same in
        # T1 under slam_toolbox and in T2 under AMCL, and it is right however
        # the robot got there -- an explorer goal, the seek tree, or a person
        # driving it with the joystick.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self._get("service_check_rate"), self._check_serviced)

        self._publish_filter_info()
        self.get_logger().info(
            "map agreement handler up; waiting for a map and a verdict from the server"
        )

    # --- parameters -----------------------------------------------------------

    def _declare_parameters(self):
        defaults = {
            # --- topics --------------------------------------------------------
            "agreement_topic": "/mecanumbot/deep3r/map_agreement",
            "map_topic": "/map",
            "mask_topic": "/mecanumbot/deep3r/keepout_mask",
            "filter_info_topic": "/mecanumbot/deep3r/costmap_filter_info",
            "revisit_topic": "/mecanumbot/deep3r/revisit_regions",
            "marker_topic": "/mecanumbot/deep3r/agreement_markers",
            # --- the robot's vertical envelope [m] -----------------------------
            # What it rolls over, where the lidar plane is, and what it drives
            # under. The middle band is the one the 2D map is blind to.
            "robot_clearance": 0.03,
            "lidar_height": 0.20,
            "robot_height": 0.45,
            # --- what is worth acting on ---------------------------------------
            "min_keepout_score": 0.4,
            "min_revisit_score": 0.5,
            # --- housekeeping ---------------------------------------------------
            "merge_radius": 0.5,
            "forget_after": 120.0,
            "serviced_radius": 1.0,
            # How often the robot's position is checked against the regions, and
            # the frames to read it between.
            "service_check_rate": 2.0,
            "map_frame": "map",
            "base_frame": "mecanumbot/base_link",
            # --- the mask -------------------------------------------------------
            "publish_mask": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _get(self, name):
        return self.get_parameter(name).value

    # --- subscriptions --------------------------------------------------------

    def _on_map(self, msg):
        # Only the geometry is kept. The mask has to be the same shape, origin
        # and resolution as the map nav2 is planning on, or the filter places
        # every keepout somewhere else entirely.
        self.grid_info = msg.info

    def _on_agreement(self, msg):
        self.verdicts += 1
        self.model.update(
            self._now(),
            [(p.x, p.y) for p in msg.uncertain_points],
            list(msg.uncertain_scores),
            kinds=list(msg.uncertain_kinds),
            heights=list(msg.uncertain_heights),
            radii=list(msg.uncertain_radii),
            map_id=msg.map_id,
        )
        self._report()
        self._publish_mask()
        self._publish_revisits()
        self._publish_markers()

    def _check_serviced(self):
        """Mark any region the robot is currently standing near as looked at."""
        if not self.model.regions:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self._get("map_frame"), self._get("base_frame"), rclpy.time.Time()
            )
        except Exception as error:  # the chain is not up yet, or has gone stale
            self.get_logger().warn(
                f"no {self._get('map_frame')} -> {self._get('base_frame')} "
                f"transform ({error})",
                throttle_duration_sec=10.0,
            )
            return
        before = sum(1 for r in self.model.regions if r.serviced)
        self.model.mark_serviced(
            transform.transform.translation.x, transform.transform.translation.y
        )
        after = sum(1 for r in self.model.regions if r.serviced)
        if after > before:
            self.get_logger().info(
                f"{after - before} region(s) looked at; "
                f"{len(self.model.revisits())} still pending"
            )
            self._publish_revisits()

    # --- products -------------------------------------------------------------

    def _publish_filter_info(self):
        """Announce the mask so `KeepoutFilter` knows where to find it."""
        info = CostmapFilterInfo()
        info.header.frame_id = self._get("map_frame")
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = 0  # keepout / lanes filter
        info.filter_mask_topic = self._get("mask_topic")
        # base + data * multiplier, so the mask's 100 reads as 100 -- lethal.
        info.base = 0.0
        info.multiplier = 1.0
        self.info_publisher.publish(info)

    def _publish_mask(self):
        if not self._get("publish_mask"):
            return
        if self.grid_info is None:
            self.get_logger().warn(
                "a verdict arrived before any map; nothing to align a mask to",
                once=True,
            )
            return

        keepouts = self.model.keepouts()
        mask = OccupancyGrid()
        mask.header.frame_id = self._get("map_frame")
        mask.header.stamp = self.get_clock().now().to_msg()
        mask.info = self.grid_info
        mask.data = stamp_mask(
            self.grid_info.width,
            self.grid_info.height,
            self.grid_info.resolution,
            self.grid_info.origin.position.x,
            self.grid_info.origin.position.y,
            keepouts,
        )
        self.mask_publisher.publish(mask)
        # Re-announced with every mask: a filter that came up before this node
        # did would otherwise never learn the topic.
        self._publish_filter_info()

    def _publish_revisits(self):
        """Publish the regions still worth a look, best first."""
        poses = PoseArray()
        poses.header.frame_id = self._get("map_frame")
        poses.header.stamp = self.get_clock().now().to_msg()
        for region in self.model.revisits():
            pose = Pose()
            pose.position.x = region.x
            pose.position.y = region.y
            # The height rides in z so a consumer can tell an overhang worth
            # photographing from a patch of floor nobody has looked at.
            pose.position.z = region.height or 0.0
            pose.orientation.w = 1.0
            poses.poses.append(pose)
        self.revisit_publisher.publish(poses)

    def _report(self):
        actions, kinds = self.model.summary()
        blind = sum(
            1
            for r in self.model.regions
            if r.kind == CLOUD_ONLY and self.model.invisible_to_lidar(r.height)
        )
        self.get_logger().info(
            f"verdict {self.verdicts}: {len(self.model.regions)} region(s) -- "
            f"{actions[KEEPOUT]} keepout, {actions[REVISIT]} to revisit; "
            f"{blind} below the lidar plane (the map cannot see these); "
            f"kinds {kinds[CLOUD_ONLY]}/{kinds[MAP_ONLY]}/"
            f"{kinds[UNOBSERVED]}/{kinds[DISAGREEMENT]} "
            "cloud_only/map_only/unobserved/disagreement"
        )

    def _publish_markers(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = self._get("map_frame")

        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = stamp
        clear.ns = "agreement"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, region in enumerate(self.model.regions):
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = stamp
            marker.ns = "agreement"
            marker.id = index + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = max(0.1, 2.0 * region.radius)
            # Drawn at its real height, so a table top and a step on the floor
            # look different in RViz -- which is the whole distinction the
            # keepout decision turns on.
            marker.scale.z = max(0.05, region.height or 0.05)
            marker.pose.position.x = region.x
            marker.pose.position.y = region.y
            marker.pose.position.z = marker.scale.z / 2.0
            marker.pose.orientation.w = 1.0
            red, green, blue = KIND_COLOURS.get(region.kind, KIND_COLOURS[DISAGREEMENT])
            marker.color.r, marker.color.g, marker.color.b = red, green, blue
            marker.color.a = 0.9 if self.model.action_for(region) == KEEPOUT else 0.4
            markers.markers.append(marker)

        self.marker_publisher.publish(markers)

    # --- clock ----------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    """Run the map agreement handler."""
    rclpy.init(args=args)
    node = MapAgreementHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
