"""
Gazebo Sim backend: simulator, model spawning and the ROS<->gz bridge.

Not meant to be launched directly during normal use - `mecanumbot_bringup`'s
`sim.launch.py` includes this with `backend:=gazebo`. It is a backend fragment,
the MuJoCo equivalent of which is a single Node action and so needs no file.

Actor and prop models are spawned from the scenario YAML, which is parsed here
with the same `load_sim_scenario` the runtime nodes use, so the launch and the
node can never disagree about where something starts.

Two scenario features are MuJoCo-only and are ignored here: an actor's `gait`
(the Gazebo human is a single rigid link, so its legs cannot be displaced) and a
prop's `size`/`mass` (`ros_gz_sim create` spawns the model file as written, so
the props come out at the defaults baked into `models/sim_mat`/`models/sim_cube`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mecanumbot_sim.sim_scenarios import load_sim_scenario

WORLD_NAME = "mecanumbot_arena"
ROBOT_MODEL_NAME = "mecanumbot"

#: Half the cube edge in `models/sim_cube`, so a cube with no z in the scenario is
#: spawned resting on the floor rather than half-buried in it.
DEFAULT_CUBE_SPAWN_Z = 0.035


def resolve_gz_sim_launch() -> str:
    """
    Find the simulator launch file across ros_gz naming changes.

    Humble shipped `ign_gazebo.launch.py` early on and renamed it to
    `gz_sim.launch.py` later; both exist in the wild on Fortress.
    """
    try:
        share = get_package_share_directory("ros_gz_sim")
    except Exception as error:
        raise RuntimeError(
            "The Gazebo backend needs ros_gz_sim, which is not installed. "
            "Install it with: sudo apt install ros-humble-ros-gz  "
            "(or launch with backend:=mujoco, which needs no Gazebo)."
        ) from error

    for candidate in ("gz_sim.launch.py", "ign_gazebo.launch.py"):
        path = os.path.join(share, "launch", candidate)
        if os.path.exists(path):
            return path
    raise RuntimeError(
        "Neither gz_sim.launch.py nor ign_gazebo.launch.py was found in ros_gz_sim. "
        "Install the Gazebo bridge with: sudo apt install ros-humble-ros-gz"
    )


def bridge_arguments(actor_body_names, cube_body_names=()) -> list:
    """
    ros_gz_bridge topic specs.

    Direction markers: `[` is Gazebo->ROS, `]` is ROS->Gazebo, `@` is both.
    """
    args = [
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/scan_raw@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/imu_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        f"/model/{ROBOT_MODEL_NAME}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
        f"/model/{ROBOT_MODEL_NAME}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
    ]
    for joint in ("head_joint", "grabber_left_joint", "grabber_right_joint"):
        args.append(
            f"/model/{ROBOT_MODEL_NAME}/{joint}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double"
        )
    for body_name in actor_body_names:
        args.append(
            f"/model/{body_name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"
        )
    # Cubes are the one scenario body physics moves on its own, so their pose has
    # to come back out of Gazebo rather than being dead-reckoned like the actors.
    for body_name in cube_body_names:
        args.append(
            f"/model/{body_name}/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose"
        )
    return args


def spawn_actions(context, *args, **kwargs):
    """Build the spawn and bridge actions once launch arguments are resolved."""
    namespace = LaunchConfiguration("namespace").perform(context)
    scenario_path = LaunchConfiguration("scenario_path").perform(context)

    sim_share = get_package_share_directory("mecanumbot_sim")
    models_root = os.path.join(sim_share, "models")

    actions = [
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_mecanumbot",
            output="screen",
            arguments=[
                "-world",
                WORLD_NAME,
                "-file",
                os.path.join(models_root, ROBOT_MODEL_NAME, "model.sdf"),
                "-name",
                ROBOT_MODEL_NAME,
                "-x",
                "0.0",
                "-y",
                "0.0",
                "-z",
                "0.0096",
            ],
        ),
    ]

    actor_body_names = []
    cube_body_names = []
    if scenario_path:
        scenario = load_sim_scenario(scenario_path)
        for actor in scenario.actors:
            # Body name decides which model file is used: sim_human_0 -> sim_human.
            model_dir = actor.body_name.rsplit("_", 1)[0]
            actor_body_names.append(actor.body_name)
            actions.append(
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name=f"spawn_{actor.body_name}",
                    output="screen",
                    arguments=[
                        "-world",
                        WORLD_NAME,
                        "-file",
                        os.path.join(models_root, model_dir, "model.sdf"),
                        "-name",
                        actor.body_name,
                        "-x",
                        str(actor.x),
                        "-y",
                        str(actor.y),
                        "-z",
                        str(actor.z),
                        "-Y",
                        str(actor.yaw),
                    ],
                )
            )

        for prop in scenario.props:
            # Body name decides which model file is used: sim_cube_0 -> sim_cube.
            model_dir = prop.body_name.rsplit("_", 1)[0]
            if model_dir == "sim_cube":
                cube_body_names.append(prop.body_name)
            actions.append(
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name=f"spawn_{prop.body_name}",
                    output="screen",
                    arguments=[
                        "-world",
                        WORLD_NAME,
                        "-file",
                        os.path.join(models_root, model_dir, "model.sdf"),
                        "-name",
                        prop.body_name,
                        "-x",
                        str(prop.x),
                        "-y",
                        str(prop.y),
                        "-z",
                        str(prop.z if prop.z > 0.0 else DEFAULT_CUBE_SPAWN_Z),
                        "-Y",
                        str(prop.yaw),
                    ],
                )
            )

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            arguments=bridge_arguments(actor_body_names, cube_body_names),
            parameters=[{"use_sim_time": True}],
        )
    )

    actions.append(
        Node(
            package="mecanumbot_sim",
            executable="mecanumbot_sim_gz_io_node",
            name="mecanumbot_sim_io_node",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "model_name": ROBOT_MODEL_NAME,
                    "scenario_path": scenario_path,
                    "velocity_frame": LaunchConfiguration("velocity_frame").perform(
                        context
                    ),
                }
            ],
            remappings=[
                (f"/{namespace}/cmd_vel", "/cmd_vel"),
                (f"/{namespace}/cmd_accessory_pos", "/cmd_accessory_pos"),
            ],
        )
    )

    return actions


def generate_launch_description():
    sim_share = get_package_share_directory("mecanumbot_sim")
    world_path = os.path.join(sim_share, "worlds", f"{WORLD_NAME}.sdf")
    models_root = os.path.join(sim_share, "models")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="mecanumbot",
                description="Robot namespace",
            ),
            DeclareLaunchArgument(
                "scenario_path",
                default_value="",
                description=(
                    "Scenario YAML; actors and props are spawned from it. "
                    "Empty means an empty arena."
                ),
            ),
            DeclareLaunchArgument(
                "velocity_frame",
                default_value="body",
                description="Frame Gazebo VelocityControl interprets cmd_vel in: body or world",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value=f"-r {world_path}",
                description="Arguments passed to the Gazebo Sim server (-r runs on start, -s is headless)",
            ),
            # Both spellings so the model path works across ros_gz/Gazebo versions.
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", models_root),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_root),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(resolve_gz_sim_launch()),
                launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
            ),
            OpaqueFunction(function=spawn_actions),
        ]
    )
