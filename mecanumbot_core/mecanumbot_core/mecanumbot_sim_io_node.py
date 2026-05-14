import math
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
from mecanumbot_core.sim_actor_runtime import SimActorRuntime
from mecanumbot_core.sim_scenarios import SUPPORTED_ACTOR_BODIES, load_sim_scenario
from mecanumbot_msgs.msg import AccessMotorCmd, OpenCRState, SimActor, SimActorArray
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


TICK_TO_RAD = 0.005061
MIDPOINT_COMPENSATE_CONSTANT = 2.618


class MecanumbotSimIONode(Node):
    def __init__(self, namespace=''):
        super().__init__('mecanumbot_sim_io_node', namespace=namespace)

        default_model_path = str(
            Path(get_package_share_directory('mecanumbot_description')) / 'mujoco' / 'mecanumbot.xml'
        )
        self.declare_parameters(
            namespace=namespace,
            parameters=[
                ('model_path', default_model_path),
                ('publish_clock', True),
                ('show_viewer', True),
                ('scenario_path', ''),
                ('sim_rate_hz', 100.0),
                ('battery_voltage', 12.1),
                ('scan_rate_hz', 8.0),
                ('scan_count', 240),
                ('scan_range_min', 0.02),
                ('scan_range_max', 8.0),
                ('scan_angle_min', -3.141592653589793),
                ('scan_angle_max', 3.141592653589793),
                ('robot_params.wheel.radius', 0.0325),
                ('robot_params.wheel.separation_x', 0.129),
                ('robot_params.wheel.separation_y', 0.300),
                ('robot_params.wheel.vel_tick', 0.229),
                ('robot_params.accessory.neck_default', 850),
                ('robot_params.accessory.grabber_default', 512),
            ],
        )

        model_path = Path(self.get_parameter('model_path').value)
        self.publish_clock = bool(self.get_parameter('publish_clock').value)
        self.show_viewer = bool(self.get_parameter('show_viewer').value)
        self.scenario_path = str(self.get_parameter('scenario_path').value)
        self.sim_rate_hz = float(self.get_parameter('sim_rate_hz').value)
        self.battery_voltage = float(self.get_parameter('battery_voltage').value)
        self.scan_rate_hz = float(self.get_parameter('scan_rate_hz').value)
        self.scan_count = int(self.get_parameter('scan_count').value)
        self.scan_range_min = float(self.get_parameter('scan_range_min').value)
        self.scan_range_max = float(self.get_parameter('scan_range_max').value)
        self.scan_angle_min = float(self.get_parameter('scan_angle_min').value)
        self.scan_angle_max = float(self.get_parameter('scan_angle_max').value)
        self.wheel_radius = float(self.get_parameter('robot_params.wheel.radius').value)
        self.wheel_separation_x = float(self.get_parameter('robot_params.wheel.separation_x').value)
        self.wheel_separation_y = float(self.get_parameter('robot_params.wheel.separation_y').value)
        self.vel_tick = float(self.get_parameter('robot_params.wheel.vel_tick').value)
        self.neck_default = int(self.get_parameter('robot_params.accessory.neck_default').value)
        self.grabber_default = int(self.get_parameter('robot_params.accessory.grabber_default').value)

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.scale = (self.vel_tick / 60.0) * 2.0 * math.pi * self.wheel_radius
        self.wheel_dist_scale = (self.wheel_separation_x + self.wheel_separation_y) / 2.0
        self.sim_period = 1.0 / self.sim_rate_hz
        self.substeps = max(1, round(self.sim_period / self.model.opt.timestep))

        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.scan_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_scan')
        self.wheel_joint_ids = {
            'bl': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'wheel_backleft_joint'),
            'br': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'wheel_backright_joint'),
            'fl': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'wheel_frontleft_joint'),
            'fr': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'wheel_frontright_joint'),
        }
        self.wheel_actuator_ids = {
            'bl': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'bl'),
            'br': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'br'),
            'fl': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'fl'),
            'fr': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'fr'),
        }
        # Match the real robot/OpenCR sign convention to the mirrored wheel joints in MuJoCo.
        self.wheel_sign = {
            'bl': -1.0,
            'br': 1.0,
            'fl': -1.0,
            'fr': 1.0,
        }
        self.accessory_joint_ids = {
            'n': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'head_joint'),
            'gl': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'grabber_left_joint'),
            'gr': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'grabber_right_joint'),
        }
        self.accessory_actuator_ids = {
            'n': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'head'),
            'gl': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'grabber_left'),
            'gr': mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'grabber_right'),
        }

        self.cmd_ticks = {'bl': 0, 'br': 0, 'fl': 0, 'fr': 0}
        self.accessory_ticks = {'n': self.neck_default, 'gl': self.grabber_default, 'gr': self.grabber_default}
        self.last_linear_velocity = np.zeros(3)
        self.last_scan_time = -1.0
        self.geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.actor_instances = []
        self.scenario = None

        self.set_accessory_pose_from_ticks()
        self.load_scenario()
        mujoco.mj_forward(self.model, self.data)
        self.viewer = None
        if self.show_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.opencr_publisher = self.create_publisher(OpenCRState, 'opencr_state', 10)
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10) if self.publish_clock else None
        self.actor_publisher = self.create_publisher(SimActorArray, '/sim/actors', 10)
        self.subject_gt_publisher = self.create_publisher(PoseStamped, '/sim/subject_pose_ground_truth', 10)

        self.create_subscription(Twist, 'cmd_vel', self.vel_cmd_callback, 10)
        self.create_subscription(AccessMotorCmd, 'cmd_accessory_pos', self.access_motor_cmd_callback, 10)
        self.timer = self.create_timer(self.sim_period, self.timer_callback)

        self.get_logger().info(f'Loaded MuJoCo model: {model_path}')
        if self.scenario is not None:
            self.get_logger().info(f'Loaded sim scenario: {self.scenario.name} ({self.scenario_path})')

    def vel_cmd_callback(self, msg: Twist) -> None:
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        bl_raw = (vx + vy - (wz * self.wheel_dist_scale)) / self.scale
        br_raw = (vx - vy + (wz * self.wheel_dist_scale)) / self.scale
        fl_raw = (vx - vy - (wz * self.wheel_dist_scale)) / self.scale
        fr_raw = (vx + vy + (wz * self.wheel_dist_scale)) / self.scale

        self.cmd_ticks['bl'] = int(max(min(bl_raw, 300), -300))
        self.cmd_ticks['br'] = int(max(min(br_raw, 300), -300))
        self.cmd_ticks['fl'] = int(max(min(fl_raw, 300), -300))
        self.cmd_ticks['fr'] = int(max(min(fr_raw, 300), -300))

    def access_motor_cmd_callback(self, msg: AccessMotorCmd) -> None:
        self.accessory_ticks['n'] = int(msg.n_pos * 100.0)
        self.accessory_ticks['gl'] = int(msg.gl_pos * 100.0)
        self.accessory_ticks['gr'] = int(msg.gr_pos * 100.0)

    def ticks_to_wheel_rad_s(self, tick_value: int) -> float:
        return tick_value * (self.vel_tick / 60.0) * 2.0 * math.pi

    def wheel_rad_s_to_ticks(self, angular_velocity: float) -> int:
        denom = (self.vel_tick / 60.0) * 2.0 * math.pi
        if abs(denom) < 1e-9:
            return 0
        return int(round(angular_velocity / denom))

    def accessory_ticks_to_angle(self, ticks: int) -> float:
        return MIDPOINT_COMPENSATE_CONSTANT - (ticks * TICK_TO_RAD)

    def joint_angle_to_ticks(self, angle: float) -> int:
        return int(round((MIDPOINT_COMPENSATE_CONSTANT - angle) / TICK_TO_RAD))

    def set_accessory_pose_from_ticks(self) -> None:
        for key, joint_id in self.accessory_joint_ids.items():
            qpos_adr = self.model.jnt_qposadr[joint_id]
            angle = self.accessory_ticks_to_angle(self.accessory_ticks[key])
            self.data.qpos[qpos_adr] = angle

    def load_scenario(self) -> None:
        self.hide_supported_actor_bodies()
        if not self.scenario_path:
            return

        self.scenario = load_sim_scenario(self.scenario_path)
        for actor in self.scenario.actors:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, actor.body_name)
            if body_id < 0:
                raise RuntimeError(f'Scenario actor body {actor.body_name} was not found in the MuJoCo model')
            mocap_id = self.model.body_mocapid[body_id]
            if mocap_id < 0:
                raise RuntimeError(f'Scenario actor body {actor.body_name} is not mocap-enabled in the MuJoCo model')

            self.set_actor_pose(mocap_id, actor.x, actor.y, actor.z, actor.yaw)
            self.actor_instances.append(
                {
                    'config': actor,
                    'runtime': SimActorRuntime(actor),
                    'body_id': body_id,
                    'mocap_id': mocap_id,
                }
            )

    def hide_supported_actor_bodies(self) -> None:
        for index, body_name in enumerate(SUPPORTED_ACTOR_BODIES):
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id < 0:
                continue
            mocap_id = self.model.body_mocapid[body_id]
            if mocap_id < 0:
                continue
            self.set_actor_pose(mocap_id, 25.0 + float(index), 25.0, 0.0, 0.0)

    def set_actor_pose(self, mocap_id: int, x: float, y: float, z: float, yaw: float) -> None:
        self.data.mocap_pos[mocap_id] = np.array([x, y, z], dtype=np.float64)
        self.data.mocap_quat[mocap_id] = np.array(
            [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
            dtype=np.float64,
        )

    def publish_actor_state(self) -> None:
        actors_msg = SimActorArray()
        actors_msg.header.stamp = self.current_sim_time_msg()
        actors_msg.header.frame_id = 'map'
        actors_msg.scenario_name = self.scenario.name if self.scenario is not None else 'empty'

        subject_actor = None
        for actor_instance in self.actor_instances:
            actor = actor_instance['config']
            runtime = actor_instance['runtime']
            body_id = actor_instance['body_id']
            pose = self.data.xpos[body_id]
            quat = self.data.xquat[body_id]

            actor_msg = SimActor()
            actor_msg.id = actor.actor_id
            actor_msg.name = actor.name
            actor_msg.kind = actor.kind
            actor_msg.is_subject = actor.is_subject
            actor_msg.visible_to_lidar = actor.visible_to_lidar
            actor_msg.pose.position.x = float(pose[0])
            actor_msg.pose.position.y = float(pose[1])
            actor_msg.pose.position.z = float(pose[2])
            actor_msg.pose.orientation.w = float(quat[0])
            actor_msg.pose.orientation.x = float(quat[1])
            actor_msg.pose.orientation.y = float(quat[2])
            actor_msg.pose.orientation.z = float(quat[3])
            actor_msg.twist.linear.x = float(runtime.state.vx)
            actor_msg.twist.linear.y = float(runtime.state.vy)
            actor_msg.twist.linear.z = float(runtime.state.vz)
            actor_msg.twist.angular.z = float(runtime.state.wz)
            actors_msg.actors.append(actor_msg)

            if actor.is_subject:
                subject_actor = actor_msg

        self.actor_publisher.publish(actors_msg)

        if subject_actor is not None:
            subject_msg = PoseStamped()
            subject_msg.header = actors_msg.header
            subject_msg.pose = subject_actor.pose
            self.subject_gt_publisher.publish(subject_msg)

    def update_actor_states(self) -> None:
        for actor_instance in self.actor_instances:
            runtime = actor_instance['runtime']
            mocap_id = actor_instance['mocap_id']
            runtime.update(self.sim_period)
            self.set_actor_pose(
                mocap_id,
                runtime.state.x,
                runtime.state.y,
                runtime.state.z,
                runtime.state.yaw,
            )

    def apply_controls(self) -> None:
        for key, actuator_id in self.wheel_actuator_ids.items():
            self.data.ctrl[actuator_id] = self.wheel_sign[key] * self.ticks_to_wheel_rad_s(self.cmd_ticks[key])

        for key, actuator_id in self.accessory_actuator_ids.items():
            self.data.ctrl[actuator_id] = self.accessory_ticks_to_angle(self.accessory_ticks[key])

    def publish_clock_msg(self) -> None:
        if self.clock_publisher is None:
            return

        clock_msg = Clock()
        clock_msg.clock = self.current_sim_time_msg()
        self.clock_publisher.publish(clock_msg)

    def current_sim_time_msg(self) -> Time:
        msg = Time()
        secs = int(self.data.time)
        nanosecs = int((self.data.time - secs) * 1e9)
        msg.sec = secs
        msg.nanosec = nanosecs
        return msg

    def scan_frame_id(self) -> str:
        namespace = self.get_namespace().strip('/')
        if namespace:
            return f'{namespace}/base_scan'
        return 'base_scan'

    def build_opencr_state(self) -> OpenCRState:
        msg = OpenCRState()
        msg.header.stamp = self.current_sim_time_msg()

        wheel_states = {}
        for key, joint_id in self.wheel_joint_ids.items():
            dof_adr = self.model.jnt_dofadr[joint_id]
            qpos_adr = self.model.jnt_qposadr[joint_id]
            wheel_states[key] = {
                'vel_ticks': self.wheel_rad_s_to_ticks(self.wheel_sign[key] * float(self.data.qvel[dof_adr])),
                'pos_ticks': self.wheel_rad_s_to_ticks(self.wheel_sign[key] * float(self.data.qpos[qpos_adr])),
            }

        msg.cmd_vel_bl = self.cmd_ticks['bl']
        msg.cmd_vel_br = self.cmd_ticks['br']
        msg.cmd_vel_fl = self.cmd_ticks['fl']
        msg.cmd_vel_fr = self.cmd_ticks['fr']
        msg.vel_bl = wheel_states['bl']['vel_ticks']
        msg.vel_br = wheel_states['br']['vel_ticks']
        msg.vel_fl = wheel_states['fl']['vel_ticks']
        msg.vel_fr = wheel_states['fr']['vel_ticks']
        msg.pos_bl = wheel_states['bl']['pos_ticks']
        msg.pos_br = wheel_states['br']['pos_ticks']
        msg.pos_fl = wheel_states['fl']['pos_ticks']
        msg.pos_fr = wheel_states['fr']['pos_ticks']

        msg.curr_bl = 0
        msg.curr_br = 0
        msg.curr_fl = 0
        msg.curr_fr = 0
        msg.acc_bl = 0
        msg.acc_br = 0
        msg.acc_fl = 0
        msg.acc_fr = 0

        msg.pos_n = self.joint_angle_to_ticks(self.data.qpos[self.model.jnt_qposadr[self.accessory_joint_ids['n']]])
        msg.pos_gl = self.joint_angle_to_ticks(self.data.qpos[self.model.jnt_qposadr[self.accessory_joint_ids['gl']]])
        msg.pos_gr = self.joint_angle_to_ticks(self.data.qpos[self.model.jnt_qposadr[self.accessory_joint_ids['gr']]])

        msg.battery_voltage = self.battery_voltage

        base_quat = self.data.xquat[self.base_body_id]
        base_lin_vel = self.data.qvel[0:3].copy()
        base_ang_vel = self.data.qvel[3:6].copy()
        linear_acc = (base_lin_vel - self.last_linear_velocity) / self.sim_period
        self.last_linear_velocity = base_lin_vel

        msg.imu_angular_vel_x = float(base_ang_vel[0])
        msg.imu_angular_vel_y = float(base_ang_vel[1])
        msg.imu_angular_vel_z = float(base_ang_vel[2])
        msg.imu_linear_acc_x = float(linear_acc[0])
        msg.imu_linear_acc_y = float(linear_acc[1])
        msg.imu_linear_acc_z = float(linear_acc[2])
        msg.imu_magnetic_x = 0.0
        msg.imu_magnetic_y = 0.0
        msg.imu_magnetic_z = 0.0
        msg.imu_orientation_w = float(base_quat[0])
        msg.imu_orientation_x = float(base_quat[1])
        msg.imu_orientation_y = float(base_quat[2])
        msg.imu_orientation_z = float(base_quat[3])
        return msg

    def publish_scan(self) -> None:
        if self.scan_body_id < 0 or self.scan_rate_hz <= 0.0:
            return
        if self.last_scan_time >= 0.0 and (self.data.time - self.last_scan_time) < (1.0 / self.scan_rate_hz):
            return

        scan = LaserScan()
        scan.header.stamp = self.current_sim_time_msg()
        scan.header.frame_id = self.scan_frame_id()
        scan.angle_min = self.scan_angle_min
        scan.angle_max = self.scan_angle_max
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        scan.angle_increment = (self.scan_angle_max - self.scan_angle_min) / float(max(self.scan_count - 1, 1))
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate_hz

        origin = self.data.xpos[self.scan_body_id].copy()
        xmat = self.data.xmat[self.scan_body_id].reshape(3, 3)
        bodyexclude = self.base_body_id
        ranges = []
        geomid = np.array([-1], dtype=np.int32)

        for i in range(self.scan_count):
            angle = self.scan_angle_min + i * scan.angle_increment
            local_dir = np.array([math.cos(angle), math.sin(angle), 0.0], dtype=np.float64)
            world_dir = xmat @ local_dir
            dist = mujoco.mj_ray(
                self.model,
                self.data,
                origin,
                world_dir,
                self.geomgroup,
                1,
                bodyexclude,
                geomid,
                None,
            )
            if dist < 0.0 or dist < self.scan_range_min or dist > self.scan_range_max:
                ranges.append(float('inf'))
            else:
                ranges.append(float(dist))

        scan.ranges = ranges
        scan.intensities = [0.0] * self.scan_count
        self.scan_publisher.publish(scan)
        self.last_scan_time = self.data.time

    def timer_callback(self) -> None:
        self.update_actor_states()
        self.apply_controls()
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer = None

        self.publish_clock_msg()
        self.publish_scan()
        self.publish_actor_state()
        self.opencr_publisher.publish(self.build_opencr_state())

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotSimIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
