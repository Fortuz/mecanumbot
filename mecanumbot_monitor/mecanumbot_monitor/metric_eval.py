#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import csv

class MetricLogger(Node):
    def __init__(self):
        super().__init__('metric_eval')
        
        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Determine your local sensor frame (change this if yours is different)
        self.sensor_frame = 'mecanumbot/base_scan' 

        self.current_frame_index = -1
        self.latest_dets = None
        self.latest_dets_time = -1.0
        self.last_saved_dets_time = -100.0 
        
        self.gt_file = open('ground_truth.csv', 'w', newline='')
        self.gt_writer = csv.writer(self.gt_file)
        self.gt_writer.writerow(['frame_index', 'x', 'y'])

        self.pred_file = open('predictions.csv', 'w', newline='')
        self.pred_writer = csv.writer(self.pred_file)
        self.pred_writer.writerow(['frame_index', 'x', 'y'])

        self.create_subscription(PoseStamped, '/goal_pose', self.click_cb, 10)
        self.create_subscription(PoseArray, '/mecanumbot/dets', self.pred_cb, 10)

        self.get_logger().info("Metric Logger Started.")
        self.get_logger().info(f"Automatically transforming clicks to '{self.sensor_frame}'.")

    def pred_cb(self, msg):
        self.latest_dets = msg
        self.latest_dets_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

    def click_cb(self, msg: PoseStamped):
        if self.latest_dets is None:
            self.get_logger().warn("No predictions received yet! Ignoring.")
            return

        # 1. Transform the Click from 'map' to 'base_scan'
        try:
            # Look up the transform from the click's frame (usually 'map') to 'base_scan'
            transform = self.tf_buffer.lookup_transform(
                self.sensor_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            
            # Apply the mathematical transform to the clicked pose
            local_click = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            
            # Extract the newly aligned coordinates
            aligned_x = local_click.position.x
            aligned_y = local_click.position.y
            
        except Exception as e:
            self.get_logger().error(f"Could not transform click: {e}")
            return

        # 2. Frame Logic (Same as before)
        if self.latest_dets_time > self.last_saved_dets_time + 2.0:
            self.current_frame_index += 1
            self.last_saved_dets_time = self.latest_dets_time

            for pose in self.latest_dets.poses:
                self.pred_writer.writerow([self.current_frame_index, pose.position.x, pose.position.y])
            self.get_logger().info(f"=== New Frame [{self.current_frame_index}] ===")

        # 3. Save the ALIGNED Ground Truth coordinates
        self.gt_writer.writerow([self.current_frame_index, aligned_x, aligned_y])
        self.get_logger().info(f"Saved GT Click (Aligned): x={aligned_x:.2f}, y={aligned_y:.2f}")

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