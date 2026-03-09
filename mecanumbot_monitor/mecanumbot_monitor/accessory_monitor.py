import rclpy
from rclpy.node import Node
import threading

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from mecanumbot_msgs.msg import AccessMotorCmd

MECANUMBOT_MIN_GRIPPER_POS = 1.6
MECANUMBOT_FRONT_GRIPPER_POS = 5.12
MECANUMBOT_MAX_GRIPPER_POS = 8.54

MECANUMBOT_MIN_CAM_POS = 2.0
MECANUMBOT_MAX_CAM_POS = 8.6

class AccessoryMonitor(Node):

    def __init__(self,namespace=''):
        super().__init__('accessory_monitor', namespace=namespace)
        self.callback_group = ReentrantCallbackGroup()
        self.cmd_lock = threading.RLock()
        self.loop_count = 0
        self.dt = 0.5
         # Log parameters
        self.timer = self.create_timer(self.dt, self.timer_callback, callback_group=self.callback_group)
        self.publishing_state_count = 4
        self.idle_state_count = 0
        self.accessory_cmd = AccessMotorCmd()
        self.cmd_publisher = self.create_publisher(AccessMotorCmd, 'cmd_accessory_pos', 10)

    def timer_callback(self):
        if self.loop_count < 7:
            if self.publishing_state_count == 4:
                self.get_logger().info("Robot in idle state")
                if self.idle_state_count < 5:
                    self.idle_state_count += 1
                else:
                    self.publishing_state_count = 0
                    self.idle_state_count = 0
                    self.get_logger().info(f"Preparing to start loop {self.loop_count + 1}")
                    self.loop_count += 1
            else:
                self.assert_accessory_cmd()
                self.publishing_state_count += 1
                self.cmd_publisher.publish(self.accessory_cmd)

    def assert_accessory_cmd(self):
            if self.publishing_state_count == 0: # Cam up
                self.accessory_cmd.n_pos = MECANUMBOT_MAX_CAM_POS
                self.get_logger().info(f"Publishing accessory command: n_pos={self.accessory_cmd.n_pos:.2f}")
            if self.publishing_state_count == 1: # Gripper open
                self.accessory_cmd.gl_pos = (MECANUMBOT_MAX_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS)/2
                self.accessory_cmd.gr_pos = (MECANUMBOT_MIN_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS)/2
                self.get_logger().info(f"Publishing accessory command: gl_pos={self.accessory_cmd.gl_pos:.2f}, gr_pos={self.accessory_cmd.gr_pos:.2f}")
            if self.publishing_state_count == 2: # Cam down
                self.accessory_cmd.n_pos = (MECANUMBOT_MIN_CAM_POS + MECANUMBOT_MAX_CAM_POS)/2
                self.get_logger().info(f"Publishing accessory command: n_pos={self.accessory_cmd.n_pos:.2f}")
            if self.publishing_state_count == 3: # Gripper close
                self.accessory_cmd.gr_pos = (MECANUMBOT_MAX_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS)/2
                self.accessory_cmd.gl_pos = (MECANUMBOT_MIN_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS)/2
                self.get_logger().info(f"Publishing accessory command: gl_pos={self.accessory_cmd.gl_pos:.2f}, gr_pos={self.accessory_cmd.gr_pos:.2f}")

def main(args=None):
    rclpy.init(args=args)
    monitor_node = AccessoryMonitor()
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
    main()