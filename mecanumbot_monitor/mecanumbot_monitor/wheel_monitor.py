import rclpy
from rclpy.node import Node
import threading

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist

class WheelMonitor(Node):

    def __init__(self,namespace=''):
        super().__init__('wheel_monitor', namespace=namespace)
        self.callback_group = ReentrantCallbackGroup()
        self.cmd_lock = threading.RLock()
        self.loop_count = 0
        self.dt = 0.5
         # Log parameters
        self.timer = self.create_timer(self.dt, self.timer_callback, callback_group=self.callback_group)
        self.publishing_state_count = 27
        self.idle_state_count = 0
        self.vel_cmd = Twist()
        self.vel_cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def timer_callback(self):
        if self.loop_count < 6:
            if self.publishing_state_count == 27 :
                self.get_logger().info("Robot in idle state")
                if self.idle_state_count < 5:
                    self.idle_state_count += 1
                else:
                    self.get_logger().info(f"Preparing to start loop {self.loop_count + 1}")
                    self.loop_count += 1
                    self.idle_state_count = 0
                    self.publishing_state_count = 0
                    self.vel_cmd.linear.x = 0.0
            else:
                self.publishing_state_count += 1
                self.vel_cmd_publisher.publish(self.vel_cmd)
                self.get_logger().info(f"Publishing velocity command: {self.vel_cmd.linear.x:.2f} m/s")
                self.vel_cmd.linear.x += 0.01
                
            

def main(args=None):
    rclpy.init(args=args)
    monitor_node = WheelMonitor()
    executor = MultiThreadedExecutor()
    executor.add_node(monitor_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        monitor_node.destroy_node()

        if rclpy.ok():   # <- prevents double shutdown
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()
