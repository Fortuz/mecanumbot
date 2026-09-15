"""
Republish the lidar scan on a fixed angular grid, for slam_toolbox.

See `scan_grid.py` for why: the LD08 changes its scan geometry every
revolution, and slam_toolbox drops every scan that does not match the first
one it saw. Everything else keeps reading the driver's own topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .scan_grid import grid_geometry, regrid


class ScanGridNode(Node):
    """`scan` in, the same revolution on `bins` fixed sectors out."""

    def __init__(self):
        super().__init__("mecanumbot_scan_grid_node")
        self.declare_parameter("input_topic", "/mecanumbot/scan")
        self.declare_parameter("output_topic", "/mecanumbot/scan_grid")
        # 1.8 degree sectors: wider than the LD08's ~1.76 degree spacing, so a
        # full revolution fills every one, and the same count on every scan.
        self.declare_parameter("bins", 200)
        self.bins = int(self.get_parameter("bins").value)
        self.angle_min, self.angle_max, self.increment = grid_geometry(self.bins)

        output = str(self.get_parameter("output_topic").value)
        source = str(self.get_parameter("input_topic").value)
        self.publisher = self.create_publisher(LaserScan, output, qos_profile_sensor_data)
        self.create_subscription(LaserScan, source, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f"{source} -> {output} on {self.bins} fixed sectors of "
            f"{self.increment * 57.29578:.2f} deg, for slam_toolbox")

    def _on_scan(self, msg):
        ranges, intensities = regrid(
            msg.angle_min, msg.angle_increment, msg.ranges,
            msg.range_min, msg.range_max, self.bins,
            intensities=msg.intensities if len(msg.intensities) else None)
        out = LaserScan()
        # Same stamp and frame: the revolution is the same, only resampled.
        out.header = msg.header
        out.angle_min = self.angle_min
        out.angle_max = self.angle_max
        out.angle_increment = self.increment
        out.scan_time = msg.scan_time
        out.time_increment = msg.scan_time / self.bins if msg.scan_time > 0.0 else 0.0
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = intensities
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
