#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import threading
from rosbag2_interfaces.srv import Pause, Resume

class PlaybackStepper(Node):
    def __init__(self, duration=5.0):
        super().__init__('playback_stepper')
        
        # Set how many seconds you want each chunk to play
        self.duration = duration 
        
        self.pause_client = self.create_client(Pause, '/rosbag2_player/pause')
        self.resume_client = self.create_client(Resume, '/rosbag2_player/resume')
        
        self.get_logger().info('Waiting for rosbag to start...')
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            pass
            
        self.get_logger().info('Connected to rosbag player!')
        
        # Run terminal prompt in a background thread so it doesn't block ROS
        self.input_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.input_thread.start()

    def control_loop(self):
        while rclpy.ok():
            try:
                input(f"\n---> Press [ENTER] to play the next {self.duration} seconds...")
                
                # Command rosbag to Resume
                self.resume_client.call_async(Resume.Request())
                self.get_logger().info(f"Playing for {self.duration}s...")
                
                # Wait for the specified duration
                time.sleep(self.duration)
                
                # Command rosbag to Pause
                self.pause_client.call_async(Pause.Request())
                self.get_logger().info("Paused. Ready for RViz clicks.")
                
            except (KeyboardInterrupt, EOFError):
                self.get_logger().info("Exiting stepper...")
                break

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the stepper (Change to 10.0 if you prefer longer gaps)
    node = PlaybackStepper(duration=5.0) 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()