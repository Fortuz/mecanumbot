import os 
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, BatteryState
from geometry_msgs.msg import Twist
from mecanumbot_msgs.msg import OpenCRState
import math
from builtin_interfaces.msg import Time
from transforms3d.euler import quat2euler, euler2quat
#from tf_transformations import quaternion_from_euler, euler_from_quaternion # You may need to install 'ros-humble-tf-transformations'
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

try:
    import board
    import busio
    import adafruit_ina219
except:
    pass
TICK_TO_RAD = 0.005061
MIDPOINT_COMPENSATE_CONSTANT = 2.618 #150 deg diff in rads

################################################ MAIN CLASS ################################################
class Mecanumbot_Sensorproc_Node(Node):

    def __init__(self,namespace=''):

        super().__init__('mecanumbot_sensorproc_node',namespace=namespace)
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameters(
        namespace=namespace,
        parameters=[
        ('robot_params.wheel.vel_tick', 0.229), # meaning of one tick between velocity values [rot/min]
        ('robot_params.wheel.radius', 0.0325), # radius [m]
        ('robot_params.wheel.sep_x',0.129), # distance between front and back wheels [m]
        ('robot_params.wheel.sep_y',0.300), # distance between left and right wheels [m]
        ('robot_params.battery.min_voltage',9.6), #minimum voltage of battery [V]
        ('robot_params.battery.max_voltage',12.6), #maximum voltage of battery [V]
        ('odom_params.frame_id', 'odom'),
        ('odom_params.child_frame_id', 'base_footprint'),
        ('odom_params.from_imu', False),
        ('imu_params.frame_id', 'imu_link')
         ])
        
        self.tf_broadcaster = TransformBroadcaster(self)
        resolved_namespace = self.get_namespace().strip('/')
        self.namespace = resolved_namespace

         # Odom parameters
        self.odom_from_imu = self.get_parameter('odom_params.from_imu').value
        self.odom_frame_id = self.get_parameter('odom_params.frame_id').value
        self.odom_child_frame_id = self.get_parameter('odom_params.child_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_params.frame_id').value

        if self.namespace != '' and self.namespace is not None:

            self.odom_frame_id = self.namespace + '/' + self.odom_frame_id
            self.odom_child_frame_id = self.namespace + '/' + self.odom_child_frame_id
            self.imu_frame_id = self.namespace + '/' + self.imu_frame_id
        self.get_logger().info(f'Namespace: {self.namespace}')
        self.get_logger().info(f'Odom Frame ID: {self.odom_frame_id}, Child Frame ID: {self.odom_child_frame_id}, IMU Frame ID: {self.imu_frame_id}')
         # Robot parameters
        self.vel_tick = self.get_parameter('robot_params.wheel.vel_tick').value/60 # rot/min to rot/s
        self.wheel_radius = self.get_parameter('robot_params.wheel.radius').value # m
        self.wheel_sep_x = self.get_parameter('robot_params.wheel.sep_x').value # m
        self.wheel_sep_y = self.get_parameter('robot_params.wheel.sep_y').value
        self.battery_min_voltage = self.get_parameter('robot_params.battery.min_voltage').value # V
        self.battery_max_voltage = self.get_parameter('robot_params.battery.max_voltage').value # V

        self.scale =  self.vel_tick * 2 * math.pi * self.wheel_radius # tick - unit diff of wheel velocoties in rpm, 2Rpi - distance/rotation, wheel_radius - m
        self.wheel_dist_scale = (self.wheel_sep_x + self.wheel_sep_y) / 2 # 
         # Initialize messages

        self.cr_state = OpenCRState()
        self.odom = Odometry()
        self.imu = Imu()
        self.joint_state = JointState()
        self.cr_battery_state = BatteryState()
        self.orin_battery_state = BatteryState()
         # Publishers

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10,callback_group=self.callback_group)
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10,callback_group=self.callback_group)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10,callback_group=self.callback_group)
        self.cr_battery_state_publisher = self.create_publisher(BatteryState, 'cr_battery_state', 10,callback_group=self.callback_group)
        self.orin_battery_state_publisher = self.create_publisher(BatteryState, 'orin_battery_state', 10,callback_group=self.callback_group)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.callback_group)
        
        self.board_subscription = self.create_subscription(OpenCRState, 'opencr_state', self.crstate_callback, 10,callback_group=self.callback_group)
        self.board_subscription  # prevent unused variable warning

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.dt = (self.current_time.nanoseconds - self.last_time.nanoseconds) * 1e-9 #[s]

        self.last_yaw_angle = 0.0
        
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.ina_sensor = adafruit_ina219.INA219(self.i2c)
            self.get_logger().info("INA219 sensor initialized successfully.")
        except Exception as e:
            self.get_logger().info(f"INA219 sensor not found: {e}")

    def crstate_callback(self,data):
        self.cr_state = data

    def timer_callback(self):
        self.last_time = self.current_time
        self.current_time = self.get_clock().now()
        self.dt = (self.current_time.nanoseconds - self.last_time.nanoseconds) * 1e-9 #[s]
        self.set_odom()
        self.set_imu()
        self.set_joint_state()
        self.set_cr_battery_state()
        self.set_orin_battery_state()

         # Publish messages
        self.odom_publisher.publish(self.odom)
        self.imu_publisher.publish(self.imu)
        self.joint_state_publisher.publish(self.joint_state)
        self.cr_battery_state_publisher.publish(self.cr_battery_state)
        self.orin_battery_state_publisher.publish(self.orin_battery_state)

    def set_odom(self):
            
            msg = Odometry()
            msg.header.stamp = self.current_time.to_msg()
            msg.header.frame_id = self.odom_frame_id
            msg.child_frame_id = self.odom_child_frame_id

            Vx_tick = (self.cr_state.vel_bl + self.cr_state.vel_br + self.cr_state.vel_fl + self.cr_state.vel_fr)/4
            Vy_tick = (self.cr_state.vel_bl - self.cr_state.vel_br - self.cr_state.vel_fl + self.cr_state.vel_fr)/4
            Wz_tick = (-self.cr_state.vel_bl + self.cr_state.vel_br - self.cr_state.vel_fl + self.cr_state.vel_fr)/4

            msg.twist.twist.linear.x = Vx_tick * self.scale  # m/s
            msg.twist.twist.linear.y = Vy_tick * self.scale # m/s
            msg.twist.twist.angular.z = Wz_tick * self.scale / self.wheel_dist_scale  # rad/s
            #self.get_logger().info(f'Wheel ticks: BL: {self.cr_state.vel_bl}, BR: {self.cr_state.vel_br}, FL: {self.cr_state.vel_fl}, FR: {self.cr_state.vel_fr}')
            #self.get_logger().info(f'Wheel tick Velocities: Vx_tick: {Vx_tick}, Vy_tick: {Vy_tick}, Wz_tick: {Wz_tick}, scale: {self.scale}, wheel_dist_scale: {self.wheel_dist_scale}')
            #self.get_logger().info(f'Calculated Velocities: Vx: {msg.twist.twist.linear.x}, Vy: {msg.twist.twist.linear.y}, Wz: {msg.twist.twist.angular.z}')
        
            dx =  msg.twist.twist.linear.x * self.dt
            dy =  msg.twist.twist.linear.y * self.dt
            dtheta = msg.twist.twist.angular.z * self.dt

            msg.pose.pose.position.x = self.odom.pose.pose.position.x + (math.cos(self.last_yaw_angle) * dx - math.sin(self.last_yaw_angle) * dy)
            msg.pose.pose.position.y = self.odom.pose.pose.position.y + (math.sin(self.last_yaw_angle) * dx + math.cos(self.last_yaw_angle) * dy)
            msg.pose.pose.position.z = 0.0
            #self.get_logger().info(f'Publishing: dx: {dx}, dy: {dy}, dtheta: {dtheta}')
            #self.get_logger().info(f'Current Odom: x: {self.odom.pose.pose.position.x}, y: {self.odom.pose.pose.position.y}, theta: {self.odom.pose.pose.orientation.z}')
            if self.odom_from_imu: #TODO
                # Orientation from IMU
                msg.pose.pose.orientation.x = self.cr_state.imu_orientation_x
                msg.pose.pose.orientation.y = self.cr_state.imu_orientation_y
                msg.pose.pose.orientation.z = self.cr_state.imu_orientation_z
                msg.pose.pose.orientation.w = self.cr_state.imu_orientation_w
                # Update last_yaw_angle from IMU quaternion
                e = quat2euler((self.cr_state.imu_orientation_x, 
                                           self.cr_state.imu_orientation_y, 
                                           self.cr_state.imu_orientation_z, 
                                           self.cr_state.imu_orientation_w))
                self.last_yaw_angle = e[2]  # Yaw angle
            else:
                new_yaw = (self.last_yaw_angle + dtheta) % (2 * math.pi)
                # Convert roll=0, pitch=0, yaw=new_yaw to a normalized quaternion
                quaternion = euler2quat(0, 0, new_yaw)
                self.last_yaw_angle = new_yaw
                msg.pose.pose.orientation.x = quaternion[0]
                msg.pose.pose.orientation.y = quaternion[1]
                msg.pose.pose.orientation.z = quaternion[2]
                msg.pose.pose.orientation.w = quaternion[3]
                # Orientation from odometry integration (not implemented)

            self.odom = msg

            t = TransformStamped()
            t.header.stamp = self.current_time.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.odom_child_frame_id
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
            
    def set_imu(self):
            
            msg = Imu()
            msg.header.stamp = self.current_time.to_msg()
            msg.header.frame_id = self.imu_frame_id

            # Fill IMU data from OpenCRState
            msg.orientation.x = self.cr_state.imu_orientation_x
            msg.orientation.y = self.cr_state.imu_orientation_y
            msg.orientation.z = self.cr_state.imu_orientation_z
            msg.orientation.w = self.cr_state.imu_orientation_w
            msg.angular_velocity.x = self.cr_state.imu_angular_vel_x
            msg.angular_velocity.y = self.cr_state.imu_angular_vel_y
            msg.angular_velocity.z = self.cr_state.imu_angular_vel_z
            msg.linear_acceleration.x = self.cr_state.imu_linear_acc_x
            msg.linear_acceleration.y = self.cr_state.imu_linear_acc_y
            msg.linear_acceleration.z = self.cr_state.imu_linear_acc_z

            self.imu = msg

    def set_joint_state(self):

        msg = JointState()
        msg.header.stamp = self.current_time.to_msg()
        msg.name = [
            f'{self.namespace}/wheel_backleft_joint', 
            f'{self.namespace}/wheel_backright_joint', 
            f'{self.namespace}/wheel_frontleft_joint', 
            f'{self.namespace}/wheel_frontright_joint',
            f'{self.namespace}/head_joint',
            f'{self.namespace}/grabber_left_joint',
            f'{self.namespace}/grabber_right_joint']

        access_posis = [
            MIDPOINT_COMPENSATE_CONSTANT - self.cr_state.pos_n * TICK_TO_RAD,
            MIDPOINT_COMPENSATE_CONSTANT - self.cr_state.pos_gl * TICK_TO_RAD,
            MIDPOINT_COMPENSATE_CONSTANT - self.cr_state.pos_gr * TICK_TO_RAD
        ]

        msg.position = [0.0,0.0,0.0,0.0, *access_posis]
        msg.velocity = [0.0] * 7
        msg.effort   = [0.0] * 7

        self.joint_state = msg
    
    def set_cr_battery_state(self): #could be more accurate - Temperature. cell values, status. etc.

        msg = BatteryState()
        msg.header.stamp = self.current_time.to_msg()
        
        msg.voltage = self.cr_state.battery_voltage
        msg.percentage = (self.cr_state.battery_voltage - self.battery_min_voltage) / (self.battery_max_voltage - self.battery_min_voltage)
        msg.charge = msg.capacity * msg.percentage

        self.cr_battery_state = msg
        
    def set_orin_battery_state(self): #placeholder for orin battery state, currently set to 100%

        msg = BatteryState()
        msg = BatteryState()

        # Timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "battery"

        # Sensor readings
        bus_voltage = self.ina_sensor.bus_voltage       # volts
        current_ma = self.ina_sensor.current            # mA

        # Fill ROS BatteryState fields
        msg.voltage = float(bus_voltage)
        msg.current = float(current_ma) / 1000.0    # convert mA -> A

        # Optional values
        msg.temperature = float("nan")
        msg.charge = float("nan")
        msg.capacity = float("nan")
        msg.design_capacity = float("nan")

        percentage = (bus_voltage - self.battery_min_voltage) / (self.battery_max_voltage - self.battery_min_voltage)
        percentage = max(0.0, min(1.0, percentage))
        msg.percentage = percentage

        # Power supply status
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        msg.present = True

        self.orin_battery_state = msg


def main(args=None):
    rclpy.init(args=args)

    sensorproc_node = Mecanumbot_Sensorproc_Node()
    executor = MultiThreadedExecutor()
    executor.add_node(sensorproc_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
        
if __name__ == '__main__':
    main()