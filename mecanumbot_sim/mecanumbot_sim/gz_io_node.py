"""
Gazebo Sim simulation backend.

Counterpart to `mujoco_io_node`. Both present the interface the real
`mecanumbot_io_node` presents (`opencr_state`, `scan`, `cmd_vel`,
`cmd_accessory_pos`) plus the simulation-only ground-truth topics, so
`mecanumbot_sensorproc_node`, the detectors, the evaluators and the behaviour
trees cannot tell which simulator is underneath.

This node is the ROS side of the bridge; Gazebo itself is reached through
`ros_gz_bridge`, so every topic named `/model/...` or `/world/...` below is a
bridged topic and not something a normal ROS node should be publishing to.

Fidelity notes, all deliberate and all shared with the MuJoCo backend:

* Base motion goes through Gazebo's VelocityControl rather than through wheel
  friction. Neither backend simulates mecanum roller contact - the MuJoCo model
  also uses plain cylinder wheels - so wheel telemetry is reported from the
  commanded ticks with position integrated over time. Against a kinematically
  driven base that is exactly right, and it is what keeps `opencr_state`
  populated without inventing a slip model neither simulator can support.
* `scan` and the IMU are genuine: they come from Gazebo's `gpu_lidar` and `imu`
  sensors raycast against real world geometry. That is the part the perception
  stack is actually exercised on.

One thing to check on a first run: `velocity_frame`. Gazebo's VelocityControl is
documented ambiguously about whether the commanded twist is body- or world-framed,
and it could not be verified while this was written. Default is `body` (the ROS
`cmd_vel` convention, passed straight through). If the robot drives along fixed
world axes instead of following its own heading, set it to `world` and this node
will rotate the twist by the current yaw before forwarding it.
"""

import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from mecanumbot_msgs.msg import AccessMotorCmd, OpenCRState, SimActor, SimActorArray
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float64

from mecanumbot_sim.kinematics import (
    WHEEL_KEYS,
    MecanumKinematics,
    accessory_cmd_to_ticks,
    accessory_ticks_to_angle,
    joint_angle_to_ticks,
)
from mecanumbot_sim.sim_actor_runtime import SimActorRuntime
from mecanumbot_sim.sim_scenarios import load_sim_scenario

ACCESSORY_JOINTS = {
    "n": "head_joint",
    "gl": "grabber_left_joint",
    "gr": "grabber_right_joint",
}


class MecanumbotGzIONode(Node):
    def __init__(self, namespace=""):
        super().__init__("mecanumbot_sim_io_node", namespace=namespace)

        self.declare_parameters(
            namespace=namespace,
            parameters=[
                ("model_name", "mecanumbot"),
                ("scenario_path", ""),
                ("velocity_frame", "body"),
                ("sim_rate_hz", 100.0),
                ("battery_voltage", 12.1),
                ("gz_scan_topic", "/scan_raw"),
                ("gz_imu_topic", "/imu_raw"),
                ("gz_joint_state_topic", "/model/mecanumbot/joint_state"),
                ("robot_params.wheel.radius", 0.0325),
                ("robot_params.wheel.separation_x", 0.129),
                ("robot_params.wheel.separation_y", 0.300),
                ("robot_params.wheel.vel_tick", 0.229),
                ("robot_params.accessory.neck_default", 850),
                ("robot_params.accessory.grabber_default", 512),
            ],
        )

        self.model_name = str(self.get_parameter("model_name").value)
        self.scenario_path = str(self.get_parameter("scenario_path").value)
        self.velocity_frame = str(self.get_parameter("velocity_frame").value).lower()
        self.sim_rate_hz = float(self.get_parameter("sim_rate_hz").value)
        self.battery_voltage = float(self.get_parameter("battery_voltage").value)
        self.neck_default = int(
            self.get_parameter("robot_params.accessory.neck_default").value
        )
        self.grabber_default = int(
            self.get_parameter("robot_params.accessory.grabber_default").value
        )

        if self.velocity_frame not in ("body", "world"):
            self.get_logger().warn(
                f'velocity_frame "{self.velocity_frame}" is not body/world, falling back to body'
            )
            self.velocity_frame = "body"

        self.kinematics = MecanumKinematics(
            wheel_radius=float(self.get_parameter("robot_params.wheel.radius").value),
            wheel_separation_x=float(
                self.get_parameter("robot_params.wheel.separation_x").value
            ),
            wheel_separation_y=float(
                self.get_parameter("robot_params.wheel.separation_y").value
            ),
            vel_tick=float(self.get_parameter("robot_params.wheel.vel_tick").value),
        )

        self.sim_period = 1.0 / self.sim_rate_hz
        self.cmd_ticks = {key: 0 for key in WHEEL_KEYS}
        self.wheel_position_rad = {key: 0.0 for key in WHEEL_KEYS}
        self.accessory_ticks = {
            "n": self.neck_default,
            "gl": self.grabber_default,
            "gr": self.grabber_default,
        }
        self.measured_accessory_rad = {}
        self.last_twist = Twist()
        self.latest_imu = None
        self.yaw = 0.0

        # --- Gazebo-facing publishers (bridged) -------------------------------
        self.gz_cmd_vel_publisher = self.create_publisher(
            Twist, f"/model/{self.model_name}/cmd_vel", 10
        )
        self.gz_accessory_publishers = {
            key: self.create_publisher(
                Float64, f"/model/{self.model_name}/{joint}/cmd_pos", 10
            )
            for key, joint in ACCESSORY_JOINTS.items()
        }

        # --- Robot-interface publishers ---------------------------------------
        self.opencr_publisher = self.create_publisher(OpenCRState, "opencr_state", 10)
        self.scan_publisher = self.create_publisher(LaserScan, "scan", 10)
        self.actor_publisher = self.create_publisher(SimActorArray, "/sim/actors", 10)
        self.prop_publisher = self.create_publisher(SimActorArray, "/sim/props", 10)
        self.subject_gt_publisher = self.create_publisher(
            PoseStamped, "/sim/subject_pose_ground_truth", 10
        )

        # --- Subscriptions -----------------------------------------------------
        self.create_subscription(Twist, "cmd_vel", self.vel_cmd_callback, 10)
        self.create_subscription(
            AccessMotorCmd, "cmd_accessory_pos", self.access_motor_cmd_callback, 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("gz_scan_topic").value),
            self.scan_callback,
            10,
        )
        self.create_subscription(
            Imu, str(self.get_parameter("gz_imu_topic").value), self.imu_callback, 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("gz_joint_state_topic").value),
            self.joint_state_callback,
            10,
        )

        self.scenario = None
        self.actor_instances = []
        self.prop_instances = []
        self.load_scenario()
        self.publish_accessory_commands()

        self.timer = self.create_timer(self.sim_period, self.timer_callback)

        self.get_logger().info(
            f'Gazebo Sim backend attached to model "{self.model_name}" '
            f"(velocity_frame={self.velocity_frame})"
        )
        if self.scenario is not None:
            self.get_logger().info(
                f"Loaded sim scenario: {self.scenario.name} ({self.scenario_path})"
            )

    # ------------------------------------------------------------------ scenario

    def load_scenario(self) -> None:
        if not self.scenario_path:
            return

        self.scenario = load_sim_scenario(self.scenario_path)
        for actor in self.scenario.actors:
            self.actor_instances.append(
                {
                    "config": actor,
                    "runtime": SimActorRuntime(actor),
                    "publisher": self.create_publisher(
                        Twist, f"/model/{actor.body_name}/cmd_vel", 10
                    ),
                }
            )

        for prop in self.scenario.props:
            prop_instance = {"config": prop, "pose": None}
            if prop.body_name.startswith("sim_cube"):
                # Physics moves the cubes, so unlike the actors their pose is read
                # back out of Gazebo rather than dead-reckoned. `gz_backend.launch.py`
                # bridges this topic for every cube the scenario names.
                prop_instance["subscription"] = self.create_subscription(
                    Pose,
                    f"/model/{prop.body_name}/pose",
                    self.make_prop_pose_callback(prop_instance),
                    10,
                )
            self.prop_instances.append(prop_instance)

    @staticmethod
    def make_prop_pose_callback(prop_instance):
        def callback(msg: Pose) -> None:
            prop_instance["pose"] = msg

        return callback

    # ------------------------------------------------------------------ commands

    def vel_cmd_callback(self, msg: Twist) -> None:
        self.last_twist = msg
        self.cmd_ticks = self.kinematics.body_twist_to_wheel_ticks(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )
        self.gz_cmd_vel_publisher.publish(self.to_gz_twist(msg))

    def to_gz_twist(self, msg: Twist) -> Twist:
        """Forward the commanded twist in whichever frame VelocityControl wants."""
        if self.velocity_frame == "body":
            return msg

        rotated = Twist()
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        rotated.linear.x = msg.linear.x * cos_yaw - msg.linear.y * sin_yaw
        rotated.linear.y = msg.linear.x * sin_yaw + msg.linear.y * cos_yaw
        rotated.linear.z = msg.linear.z
        rotated.angular = msg.angular
        return rotated

    def access_motor_cmd_callback(self, msg: AccessMotorCmd) -> None:
        self.accessory_ticks["n"] = accessory_cmd_to_ticks(msg.n_pos)
        self.accessory_ticks["gl"] = accessory_cmd_to_ticks(msg.gl_pos)
        self.accessory_ticks["gr"] = accessory_cmd_to_ticks(msg.gr_pos)
        self.publish_accessory_commands()

    def publish_accessory_commands(self) -> None:
        for key, publisher in self.gz_accessory_publishers.items():
            command = Float64()
            command.data = accessory_ticks_to_angle(self.accessory_ticks[key])
            publisher.publish(command)

    # ------------------------------------------------------------------ feedback

    def scan_callback(self, msg: LaserScan) -> None:
        """Republish the Gazebo scan under the frame the rest of the stack expects."""
        msg.header.frame_id = self.scan_frame_id()
        self.scan_publisher.publish(msg)

    def imu_callback(self, msg: Imu) -> None:
        self.latest_imu = msg
        self.yaw = self.yaw_from_quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )

    def joint_state_callback(self, msg: JointState) -> None:
        for key, joint in ACCESSORY_JOINTS.items():
            if joint in msg.name:
                self.measured_accessory_rad[key] = msg.position[msg.name.index(joint)]

    @staticmethod
    def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def scan_frame_id(self) -> str:
        namespace = self.get_namespace().strip("/")
        if namespace:
            return f"{namespace}/base_scan"
        return "base_scan"

    # ------------------------------------------------------------------ outputs

    def build_opencr_state(self) -> OpenCRState:
        msg = OpenCRState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for key in WHEEL_KEYS:
            self.wheel_position_rad[key] += (
                self.kinematics.ticks_to_wheel_rad_s(self.cmd_ticks[key])
                * self.sim_period
            )

        msg.cmd_vel_bl = self.cmd_ticks["bl"]
        msg.cmd_vel_br = self.cmd_ticks["br"]
        msg.cmd_vel_fl = self.cmd_ticks["fl"]
        msg.cmd_vel_fr = self.cmd_ticks["fr"]
        msg.vel_bl = self.cmd_ticks["bl"]
        msg.vel_br = self.cmd_ticks["br"]
        msg.vel_fl = self.cmd_ticks["fl"]
        msg.vel_fr = self.cmd_ticks["fr"]
        msg.pos_bl = self.kinematics.wheel_rad_s_to_ticks(self.wheel_position_rad["bl"])
        msg.pos_br = self.kinematics.wheel_rad_s_to_ticks(self.wheel_position_rad["br"])
        msg.pos_fl = self.kinematics.wheel_rad_s_to_ticks(self.wheel_position_rad["fl"])
        msg.pos_fr = self.kinematics.wheel_rad_s_to_ticks(self.wheel_position_rad["fr"])

        msg.curr_bl = 0
        msg.curr_br = 0
        msg.curr_fl = 0
        msg.curr_fr = 0
        msg.acc_bl = 0
        msg.acc_br = 0
        msg.acc_fl = 0
        msg.acc_fr = 0

        # Prefer what Gazebo actually achieved on the accessory joints, but stay
        # populated before the first joint_state arrives.
        msg.pos_n = self.accessory_position_ticks("n")
        msg.pos_gl = self.accessory_position_ticks("gl")
        msg.pos_gr = self.accessory_position_ticks("gr")

        msg.battery_voltage = self.battery_voltage

        if self.latest_imu is not None:
            imu = self.latest_imu
            msg.imu_angular_vel_x = imu.angular_velocity.x
            msg.imu_angular_vel_y = imu.angular_velocity.y
            msg.imu_angular_vel_z = imu.angular_velocity.z
            msg.imu_linear_acc_x = imu.linear_acceleration.x
            msg.imu_linear_acc_y = imu.linear_acceleration.y
            msg.imu_linear_acc_z = imu.linear_acceleration.z
            msg.imu_orientation_w = imu.orientation.w
            msg.imu_orientation_x = imu.orientation.x
            msg.imu_orientation_y = imu.orientation.y
            msg.imu_orientation_z = imu.orientation.z
        else:
            msg.imu_orientation_w = 1.0

        msg.imu_magnetic_x = 0.0
        msg.imu_magnetic_y = 0.0
        msg.imu_magnetic_z = 0.0
        return msg

    def accessory_position_ticks(self, key: str) -> int:
        if key in self.measured_accessory_rad:
            return joint_angle_to_ticks(self.measured_accessory_rad[key])
        return self.accessory_ticks[key]

    def update_actor_states(self) -> None:
        for actor_instance in self.actor_instances:
            runtime = actor_instance["runtime"]
            runtime.update(self.sim_period)

            command = Twist()
            command.linear.x = runtime.state.vx
            command.linear.y = runtime.state.vy
            command.linear.z = runtime.state.vz
            command.angular.z = runtime.state.wz
            actor_instance["publisher"].publish(command)

    def publish_actor_state(self) -> None:
        actors_msg = SimActorArray()
        actors_msg.header.stamp = self.get_clock().now().to_msg()
        actors_msg.header.frame_id = "map"
        actors_msg.scenario_name = (
            self.scenario.name if self.scenario is not None else "empty"
        )

        subject_actor = None
        for actor_instance in self.actor_instances:
            actor = actor_instance["config"]
            state = actor_instance["runtime"].state

            actor_msg = SimActor()
            actor_msg.id = actor.actor_id
            actor_msg.name = actor.name
            actor_msg.kind = actor.kind
            actor_msg.is_subject = actor.is_subject
            actor_msg.visible_to_lidar = actor.visible_to_lidar
            actor_msg.pose.position.x = float(state.x)
            actor_msg.pose.position.y = float(state.y)
            actor_msg.pose.position.z = float(state.z)
            actor_msg.pose.orientation.w = float(math.cos(state.yaw / 2.0))
            actor_msg.pose.orientation.x = 0.0
            actor_msg.pose.orientation.y = 0.0
            actor_msg.pose.orientation.z = float(math.sin(state.yaw / 2.0))
            actor_msg.twist.linear.x = float(state.vx)
            actor_msg.twist.linear.y = float(state.vy)
            actor_msg.twist.linear.z = float(state.vz)
            actor_msg.twist.angular.z = float(state.wz)
            actors_msg.actors.append(actor_msg)

            if actor.is_subject:
                subject_actor = actor_msg

        self.actor_publisher.publish(actors_msg)

        if subject_actor is not None:
            subject_msg = PoseStamped()
            subject_msg.header = actors_msg.header
            subject_msg.pose = subject_actor.pose
            self.subject_gt_publisher.publish(subject_msg)

    def publish_prop_state(self) -> None:
        """
        Republish the scenario props as ground truth.

        Mats never move, so their configured pose is the truth. Cubes report the
        pose bridged out of Gazebo; until the first message arrives, the spawn pose
        stands in for it.
        """
        if not self.prop_instances:
            return

        props_msg = SimActorArray()
        props_msg.header.stamp = self.get_clock().now().to_msg()
        props_msg.header.frame_id = "map"
        props_msg.scenario_name = (
            self.scenario.name if self.scenario is not None else "empty"
        )

        for prop_instance in self.prop_instances:
            prop = prop_instance["config"]
            prop_msg = SimActor()
            prop_msg.id = prop.prop_id
            prop_msg.name = prop.name
            prop_msg.kind = prop.kind
            prop_msg.is_subject = False
            # Both prop kinds sit below the lidar plane, so neither shows up in `scan`.
            prop_msg.visible_to_lidar = False

            if prop_instance["pose"] is not None:
                prop_msg.pose = prop_instance["pose"]
            else:
                prop_msg.pose.position.x = float(prop.x)
                prop_msg.pose.position.y = float(prop.y)
                prop_msg.pose.position.z = float(prop.z)
                prop_msg.pose.orientation.w = float(math.cos(prop.yaw / 2.0))
                prop_msg.pose.orientation.z = float(math.sin(prop.yaw / 2.0))
            props_msg.actors.append(prop_msg)

        self.prop_publisher.publish(props_msg)

    def timer_callback(self) -> None:
        self.update_actor_states()
        self.publish_actor_state()
        self.publish_prop_state()
        self.opencr_publisher.publish(self.build_opencr_state())


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotGzIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
