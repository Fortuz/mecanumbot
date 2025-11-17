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
        default_device = 'COM3' if os.name == 'nt' else '/dev/ttyACM0'
        self.declare_parameters(
        namespace=namespace,
        parameters=[
        ('dev_params.device_name', default_device),
        ('dev_params.baudrate', 57600),

        #Robot parameters
        ('robot_params.wheel.radius', 0.0325),
        ('robot_params.wheel.separation_x', 0.129),
        ('robot_params.wheel.separation_y', 0.300),
        ('robot_params.wheel.vel_tick',0.299),
        ('robot_params.accessory.neck_default', 330),
        ('robot_params.accessory.grabber_default', 512),
        # Packet parameters
        ('packet_params.payload_fmt', '<23h14f'),
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

        self.vel_tick = self.get_parameter('robot_params.wheel.vel_tick').value/60 # rot/min to rot/s
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
        
        self.init_serial()

        self.opencr_publisher_ = self.create_publisher(OpenCRState, 'mecanumbot/opencr_state', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.opencr_state = OpenCRState()
        self.i = 0
        self.vals = None
        self.cmd_outputs = {'BL_vel':0,'BR_vel':0,'FL_vel':0,'FR_vel':0,
                            'N_pos':self.neck_default,'GL_pos':self.grabber_default,'GR_pos':self.grabber_default}
        self.vel_subscription = self.create_subscription(Twist, 'mecanumbot/cmd_vel', self.vel_cmd_callback, 10)
        self.pos_subscription = self.create_subscription(AccessMotorCmd, 'mecanumbot/cmd_accessory_pos', self.access_motor_cmd_callback, 10)
        self.vel_subscription  # prevent unused variable warning
        self.pos_subscription  # prevent unused variable warning

    def init_serial(self):
        try:
            self.ser = serial.Serial(self.device_name, self.baudrate, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.ser = None
    def close_serial(self):
        if self.ser is not None:
            self.ser.close()
        self.get_logger().info("Serial port closed.")
    # Plausibility check for received payload
    # Returns True if the payload is plausible, False otherwise
    # A payload is plausible if it fits within certain ranges for wheel speeds, positions, and float values
    def plausible_payload(self):
        # vals: tuple of 23 shorts then 14 floats
        shorts = self.vals[:23]
        floats = self.vals[23:]
        # wheel velocities check
        for v in shorts[:4]:
            if abs(v) > self.max_wheel_speed:
                return False
        # positions check
        for p in shorts[20:23]:
            if p < self.min_pos or p > self.max_pos:
                return False
        # floats check
        for f in floats:
            if math.isnan(f) or math.isinf(f) or abs(f) > self.max_float_abs:
                return False
        return True

    def update_opencr_state_in(self):
        shorts = self.vals[:23]
        floats = self.vals[23:]
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

    def vel_cmd_callback(self,msg):
        # Standard mecanum kinematics
        Vx = msg.linear.x  # m/s
        Vy = msg.linear.y  # m/s
        Wz = msg.angular.z  # rad/s

        self.cmd_outputs['BL_vel']= (4/self.scale) * (Vx + Vy - (Wz * self.wheel_dist_scale))
        self.cmd_outputs['BR_vel']= (4/self.scale) * (Vx - Vy + (Wz * self.wheel_dist_scale))
        self.cmd_outputs['FL_vel']= (4/self.scale) * (Vx - Vy - (Wz * self.wheel_dist_scale))
        self.cmd_outputs['FR_vel']= (4/self.scale) * (Vx + Vy + (Wz * self.wheel_dist_scale))

    def access_motor_cmd_callback(self,msg):
        self.cmd_outputs['N_pos']=msg.n_pos*100
        self.cmd_outputs['GL_pos']=msg.gl_pos*100
        self.cmd_outputs['GR_pos']=msg.gr_pos*100

    def update_motor_cmds_out(self):
        fmt = '<7h'
        message_bytes = struct.pack(fmt,
                                    int(self.cmd_outputs['BL_vel']), int(self.cmd_outputs['BR_vel']), int(self.cmd_outputs['FL_vel']), int(self.cmd_outputs['FR_vel']),
                                    int(self.cmd_outputs['N_pos']), int(self.cmd_outputs['GL_pos']), int(self.cmd_outputs['GR_pos']))
        self.ser.write(message_bytes)
        time.sleep(0.02)

    def timer_callback(self):
        #self.get_logger().info("Timer callback triggered")
        self.read_thread_fn()
        self.opencr_publisher_.publish(self.opencr_state)
        #self.get_logger().info("OpencR State Published")
        #self.get_logger().info('Publishing: "%s"' % self.opencr_state)
        self.update_motor_cmds_out()
        self.i += 1

    def read_thread_fn(self):
        buf = bytearray()
        try:
            # read some bytes (blocking read with timeout)
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)

            # avoid buffer growing too large
            if len(buf) > 4096:
                # keep last 2048 bytes as a fallback
                buf = buf[-2048:]

            # search for magic header
            idx = buf.find(self.magic)
            if idx == -1:
                # not found yet, continue reading
                time.sleep(0.001)

            # If found but not enough bytes for full packet yet, continue reading
            if len(buf) - idx < self.full_packet_size:
                time.sleep(0.001)

            # we have at least one full candidate packet
            start = idx
            end = start + self.full_packet_size
            packet = bytes(buf[start:end])

            # extract components
            seq = packet[len(self.magic)]
            payload_bytes = packet[len(self.magic) + self.seq_size : len(self.magic) + self.seq_size + self.payload_size]
            recv_crc = packet[-1]

            # compute CRC over all but last byte (including magic and seq and payload)
            computed_crc = crc8_ccitt(packet[:-1])

            if computed_crc != recv_crc:
                # CRC mismatch -> drop this magic occurrence and continue searching
                # drop just the first byte of the current search window to resync
                del buf[start]

            # CRC ok -> unpack payload
            try:
                self.vals = struct.unpack(self.payload_fmt, payload_bytes)
            except struct.error:
                # something wrong with size/format; drop header and resync
                del buf[start]
            if not self.plausible_payload():
                # payload fails heuristic plausibility -> drop header and resync
                del buf[start]
            # Valid packet: consume bytes up to end
            del buf[:end]
            self.update_opencr_state_in()

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error in read thread: {e}")
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Unexpected error in read thread: {e}")
            time.sleep(0.1)
        finally:
            if not self.ser.is_open:
                self.get_logger().info("Serial closed, exiting read thread.")

def main(args=None):
    rclpy.init(args=args)

    io_node = Mecanumbot_IO_Node()

    rclpy.spin(io_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    io_node.close_serial()
    io_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()