#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray
import csv
import time

class MetricLogger(Node):
    def __init__(self):
        super().__init__('metric_logger')
        
        # Open CSV files
        self.gt_file = open('ground_truth.csv', 'w', newline='')
        self.gt_writer = csv.writer(self.gt_file)
        self.gt_writer.writerow(['bag_time_sec', 'x', 'y'])

        self.pred_file = open('predictions.csv', 'w', newline='')
        self.pred_writer = csv.writer(self.pred_file)
        self.pred_writer.writerow(['bag_time_sec', 'x', 'y'])

        # Subscribe to RViz "Publish Point"
        self.create_subscription(PointStamped, '/clicked_point', self.click_cb, 10)
        
        # Subscribe to your DR-SPAAM outputs (Ensure topic name matches your param)
        self.create_subscription(PoseArray, '/dets', self.pred_cb, 10)

        self.get_logger().info("Metric Logger Started. Ensure use_sim_time is TRUE.")
        self.get_logger().info("Use RViz 'Publish Point' to click on people.")

    def click_cb(self, msg):
        time_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        self.gt_writer.writerow([time_sec, msg.point.x, msg.point.y])
        self.get_logger().info(f"Saved Ground Truth: x={msg.point.x:.2f}, y={msg.point.y:.2f}")

    def pred_cb(self, msg):
        time_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        for pose in msg.poses:
            self.pred_writer.writerow([time_sec, pose.position.x, pose.position.y])

    def destroy_node(self):
        self.gt_file.close()
        self.pred_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MetricLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()