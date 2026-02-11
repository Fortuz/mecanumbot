import os 
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mecanumbot_msgs.msg import OpenCRState,AccessMotorCmd
from geometry_msgs.msg import Twist
import serial
import struct
import time
import math
import threading

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
############################################### HELPER FUNCTIONS ################################################
def crc8_ccitt(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ 0x07
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF
################################################ MAIN CLASS ################################################
class Mecanumbot_IO_Node(Node):

    def __init__(self,namespace=''):
        super().__init__('mecanumbot_io_node', namespace=namespace)
        self.callback_group = ReentrantCallbackGroup()
        self.cmd_lock = threading.RLock()

        default_device = 'COM3' if os.name == 'nt' else '/dev/ttyACM0'
        self.declare_parameters(
        namespace=namespace,
        parameters=[
        ('dev_params.device_name', default_device),
        ('dev_params.baudrate', 1000000),

        #Robot parameters
        ('robot_params.wheel.radius', 0.0325),
        ('robot_params.wheel.separation_x', 0.129),
        ('robot_params.wheel.separation_y', 0.300),
        ('robot_params.wheel.vel_tick',0.229),
        ('robot_params.accessory.neck_default', 850),
        ('robot_params.accessory.grabber_default', 512),
        # Packet parameters
        ('packet_params.payload_fmt', '<27h14f'),
        ('packet_params.seq_size', 1),
        ('packet_params.crc_size', 1),
        ('packet_params.magic', '55AA'),  # list because YAML can't store bytes

        # Plausibility parameters
        ('plausibility_params.max_wheel_speed', 10000),
        ('plausibility_params.min_pos', -1000),
        ('plausibility_params.max_pos', 10000),
        ('plausibility_params.max_float_abs', 1e7),
         ])
        
        # Retrieve parameters
        self.device_name = self.get_parameter('dev_params.device_name').value
        self.baudrate = self.get_parameter('dev_params.baudrate').value
        self.wheel_radius = self.get_parameter('robot_params.wheel.radius').value
        self.wheel_separation_x = self.get_parameter('robot_params.wheel.separation_x').value
        self.wheel_separation_y = self.get_parameter('robot_params.wheel.separation_y').value

        self.payload_fmt = self.get_parameter('packet_params.payload_fmt').value
        self.seq_size = self.get_parameter('packet_params.seq_size').value
        self.crc_size = self.get_parameter('packet_params.crc_size').value
        self.magic = bytes.fromhex(self.get_parameter('packet_params.magic').value)

        self.payload_size = struct.calcsize(self.payload_fmt)
        self.full_packet_size = len(self.magic) + self.seq_size + self.payload_size + self.crc_size

        self.vel_tick = self.get_parameter('robot_params.wheel.vel_tick').value/60 # rot/min to rot/s, in 1 int diff
        self.max_wheel_speed = self.get_parameter('plausibility_params.max_wheel_speed').value
        self.min_pos = self.get_parameter('plausibility_params.min_pos').value
        self.max_pos = self.get_parameter('plausibility_params.max_pos').value
        self.max_float_abs = self.get_parameter('plausibility_params.max_float_abs').value
        self.neck_default = self.get_parameter('robot_params.accessory.neck_default').value
        self.grabber_default = self.get_parameter('robot_params.accessory.grabber_default').value


        self.scale =  self.vel_tick * 2 * math.pi * self.wheel_radius # tick - unit diff of wheel velocoties in rpm, 2Rpi - distance/rotation, wheel_radius - m
        self.wheel_dist_scale = (self.wheel_separation_x + self.wheel_separation_y) / 2 # 

         # Log parameters

        self.get_logger().info(f"Device: {self.device_name} @ {self.baudrate} baud")
        self.get_logger().info(f"Packet: payload_size={self.payload_size}, full_packet_size={self.full_packet_size}")
        self.get_logger().info(f"Plausibility: max_speed={self.max_wheel_speed}, pos_range=[{self.min_pos},{self.max_pos}], max_float={self.max_float_abs}")

        self.rx_lock = threading.Lock() 
        self.init_serial()
        self.init_reader_thread()

        self.vals = None
        self.opencr_publisher_ = self.create_publisher(OpenCRState, 'opencr_state', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, 
                                       self.timer_callback,
                                       callback_group=self.callback_group)
        self.opencr_state = OpenCRState()

        self.i = 0
        self.rx_buffer = bytearray()
        

        self.cmd_outputs = {'BL_vel':0,'BR_vel':0,'FL_vel':0,'FR_vel':0,
                            'N_pos':self.neck_default,'GL_pos':self.grabber_default,'GR_pos':self.grabber_default}
        self.vel_subscription = self.create_subscription(Twist,
                                                         'cmd_vel', 
                                                         self.vel_cmd_callback, 
                                                         10,
                                                         callback_group = self.callback_group)
        self.pos_subscription = self.create_subscription(AccessMotorCmd, 
                                                         'cmd_accessory_pos', 
                                                         self.access_motor_cmd_callback,
                                                         10,
                                                         callback_group=self.callback_group)
        self.vel_subscription  # prevent unused variable warning
        self.pos_subscription  # prevent unused variable warning

    def init_serial(self):
        try:
            self.ser = serial.Serial(self.device_name, self.baudrate, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None

    def init_reader_thread(self):
        self.reader = threading.Thread(target=self.read_thread_fn, daemon=True)
        self.reader.start()
    
    def close_serial(self):
        if self.ser is not None:
            self.ser.close()
        self.get_logger().info("Serial port closed.")
        if self.reader is not None:
            self.reader.join(timeout=1.0)
    # Plausibility check for received payload
    # Returns True if the payload is plausible, False otherwise
    # A payload is plausible if it fits within certain ranges for wheel speeds, positions, and float values
    def plausible_payload(self, vals):
        shorts = vals[:27]
        floats = vals[27:]
        # wheel velocities check
        for v in shorts[:4]:
            if abs(v) > self.max_wheel_speed:
                self.get_logger().warn(f"Wheel speed too high: {v}")
                return False
        # positions check
        for p in shorts[20:23]:
            if p < self.min_pos or p > self.max_pos:
                self.get_logger().warn("Minimum or maximum position wrong.")
                return False
        # floats check
        for f in floats:
            if math.isnan(f) or math.isinf(f) or abs(f) > self.max_float_abs:
                self.get_logger().warn("Payload has float error")
                return False
        return True

    def update_opencr_state_in(self):
        with self.rx_lock:
            if self.vals is None:
                self.get_logger().warn("No valid data received yet.")
                return
            vals = self.vals
        shorts = vals[:27]
        floats = vals[27:]
        self.opencr_state.header.stamp = self.get_clock().now().to_msg()
        self.opencr_state.cmd_vel_bl = shorts[0]
        self.opencr_state.cmd_vel_br = shorts[1]
        self.opencr_state.cmd_vel_fl = shorts[2]
        self.opencr_state.cmd_vel_fr = shorts[3]
        self.opencr_state.vel_bl = shorts[4]
        self.opencr_state.vel_br = shorts[5]
        self.opencr_state.vel_fl = shorts[6]
        self.opencr_state.vel_fr = shorts[7]
        self.opencr_state.pos_bl = shorts[8]
        self.opencr_state.pos_br = shorts[9]
        self.opencr_state.pos_fl = shorts[10]
        self.opencr_state.pos_fr = shorts[11]
        self.opencr_state.curr_bl = shorts[12]
        self.opencr_state.curr_br = shorts[13]
        self.opencr_state.curr_fl = shorts[14]
        self.opencr_state.curr_fr = shorts[15]
        self.opencr_state.acc_bl = shorts[16]
        self.opencr_state.acc_br = shorts[17]
        self.opencr_state.acc_fl = shorts[18]
        self.opencr_state.acc_fr = shorts[19]
        self.opencr_state.pos_n = shorts[20]
        self.opencr_state.pos_gl = shorts[21]
        self.opencr_state.pos_gr = shorts[22]
        self.opencr_state.err_bl = shorts[23]
        self.opencr_state.err_br = shorts[24]
        self.opencr_state.err_fl = shorts[25]
        self.opencr_state.err_fr = shorts[26]
        self.opencr_state.battery_voltage = floats[0]
        self.opencr_state.imu_angular_vel_x = floats[1]
        self.opencr_state.imu_angular_vel_y = floats[2]
        self.opencr_state.imu_angular_vel_z = floats[3]
        self.opencr_state.imu_linear_acc_x = floats[4]
        self.opencr_state.imu_linear_acc_y = floats[5]
        self.opencr_state.imu_linear_acc_z = floats[6]
        self.opencr_state.imu_magnetic_x = floats[7]
        self.opencr_state.imu_magnetic_y = floats[8]
        self.opencr_state.imu_magnetic_z = floats[9]
        self.opencr_state.imu_orientation_w = floats[10]
        self.opencr_state.imu_orientation_x = floats[11]
        self.opencr_state.imu_orientation_y = floats[12]
        self.opencr_state.imu_orientation_z = floats[13]

    def vel_cmd_callback(self, msg):
        # Standard mecanum kinematics
        Vx = msg.linear.x
        Vy = msg.linear.y
        Wz = msg.angular.z

        # Calculate raw velocities
        # Note: Pre-calculating reduces time inside the lock
        bl_raw = (Vx + Vy - (Wz * self.wheel_dist_scale)) / self.scale
        br_raw = (Vx - Vy + (Wz * self.wheel_dist_scale)) / self.scale
        fl_raw = (Vx - Vy - (Wz * self.wheel_dist_scale)) / self.scale
        fr_raw = (Vx + Vy + (Wz * self.wheel_dist_scale)) / self.scale

        with self.cmd_lock:
            # Correct clamping logic: max(min(val, upper), lower)
            self.cmd_outputs['BL_vel'] = max(min(bl_raw, 300), -300)
            self.cmd_outputs['FL_vel'] = max(min(fl_raw, 300), -300)
            self.cmd_outputs['BR_vel'] = max(min(br_raw, 300), -300)
            self.cmd_outputs['FR_vel'] = max(min(fr_raw, 300), -300)

            
            # Send immediately (Event-Driven)
            self.update_motor_cmds_out()

    def access_motor_cmd_callback(self, msg):
        with self.cmd_lock:
            # Now safely updates the shared dictionary
            self.cmd_outputs['N_pos'] = msg.n_pos * 100
            self.cmd_outputs['GL_pos'] = msg.gl_pos * 100
            self.cmd_outputs['GR_pos'] = msg.gr_pos * 100
            
            # Send immediately (Event-Driven)
            self.update_motor_cmds_out()

    def update_motor_cmds_out(self):
        fmt = '<7h'
        # RLock allows us to re-acquire the lock here safely
        with self.cmd_lock:
            try:
                message_bytes = struct.pack(fmt,
                                            int(self.cmd_outputs['BL_vel']), 
                                            int(self.cmd_outputs['BR_vel']), 
                                            int(self.cmd_outputs['FL_vel']), 
                                            int(self.cmd_outputs['FR_vel']),
                                            int(self.cmd_outputs['N_pos']), 
                                            int(self.cmd_outputs['GL_pos']), 
                                            int(self.cmd_outputs['GR_pos']))
                
                if self.ser is not None and self.ser.is_open:
                    self.ser.write(message_bytes)
                    self.ser.flush()
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")

    def timer_callback(self):
        self.get_logger().info("Timer callback triggered")
        self.update_opencr_state_in()
        self.opencr_publisher_.publish(self.opencr_state)
        self.get_logger().info("OpencR State Published")
        self.get_logger().info('Publishing: "%s"' % self.opencr_state)
        #self.update_motor_cmds_out()
        self.i += 1

    def read_thread_fn(self):
        """Continuously read serial data, extract packets, and update self.vals safely."""
        if self.ser is None:
            self.get_logger().error("Serial not initialized; reader thread exiting.")
            return

        while rclpy.ok():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if chunk:
                    self.rx_buffer.extend(chunk)

                # Keep buffer bounded
                if len(self.rx_buffer) > 4096:
                    self.rx_buffer = self.rx_buffer[-2048:]

                # Try to find magic header
                idx = self.rx_buffer.find(self.magic)
                if idx == -1:
                    time.sleep(0.001)
                    continue

                # Not enough bytes for full packet -> wait
                if len(self.rx_buffer) - idx < self.full_packet_size:
                    time.sleep(0.001)
                    continue

                # Extract packet
                start = idx
                end = start + self.full_packet_size
                packet = bytes(self.rx_buffer[start:end])

                seq = packet[len(self.magic)]
                payload_bytes = packet[len(self.magic) + self.seq_size:
                                    len(self.magic) + self.seq_size + self.payload_size]
                recv_crc = packet[-1]

                # Compute CRC
                computed_crc = crc8_ccitt(packet[:-1])

                # Bad CRC → discard only magic byte and keep scanning
                if computed_crc != recv_crc:
                    del self.rx_buffer[start:start + 1]
                    self.get_logger.warning(f'Magic byte wrong, computed: {computed_crc}, recieved: {recv_crc}')
                    continue

                # Unpack packet
                vals = struct.unpack(self.payload_fmt, payload_bytes)

                # Check plausibility
                if not self.plausible_payload(vals):
                    del self.rx_buffer[start:start + 1]
                    continue

                # Success — commit parsed values
                with self.rx_lock:
                    self.vals = vals

                # Remove parsed packet from buffer
                del self.rx_buffer[:end]

            except serial.SerialException as e:
                self.get_logger().error(f"Serial error in read thread: {e}")
                time.sleep(0.5)

            except Exception as e:
                self.get_logger().error(f"Unexpected error in read thread: {e}")
                time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)

    io_node = Mecanumbot_IO_Node()

    # Use the MultiThreadedExecutor
    # num_threads=None will default to the number of CPU cores
    executor = MultiThreadedExecutor()
    executor.add_node(io_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        io_node.close_serial()
        io_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()