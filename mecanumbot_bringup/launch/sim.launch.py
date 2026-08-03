"""Single entry point for every simulated run of the mecanumbot.

Replaces the five launch files this package used to carry
(`launch_mecanumbot_sim`, `_sim_mapping`, `_sim_perception`, `_sim_behaviour`
and `_truth_twin`), which were layered includes of each other. The layers are now
the `mode` argument, and everything they started lives in `mecanumbot_sim`.

Two orthogonal choices:

  backend : mujoco | gazebo   which simulator provides the robot
  mode    : base | mapping | perception | truth_twin | behaviour   what runs on top

The backend only decides where `opencr_state`, `scan` and `/clock` come from, so
every mode works with either simulator.

  base        drivers only: sim backend + sensorproc + robot_state_publisher
  mapping     base + slam_toolbox, for building a map of the sim arena
  perception  base + the real DR-SPAAM detector + detection/behaviour evaluators
  truth_twin  base + oracle subject tracking instead of the detector, so behaviour
              can be evaluated without detector error in the loop
  behaviour   truth_twin + the Nav2 shim + one behaviour tree (see `condition`)

Examples:
  ros2 launch mecanumbot_bringup sim.launch.py
  ros2 launch mecanumbot_bringup sim.launch.py mode:=perception
  ros2 launch mecanumbot_bringup sim.launch.py backend:=gazebo mode:=behaviour condition:=Doglike
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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

BT_EXECUTABLES = {
    "Doglike": "doglike_leading_bt_node",
    "Control": "control_leading_bt_node",
    "LED": "LED_leading_bt_node",
}

DETECTOR_MODES = ("perception",)
ORACLE_MODES = ("truth_twin", "behaviour")
EVALUATED_MODES = ("perception", "truth_twin", "behaviour")
RVIZ_MODES = ("mapping", "perception", "truth_twin", "behaviour")


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    backend = LaunchConfiguration("backend").perform(context).lower()
    mode = LaunchConfiguration("mode").perform(context).lower()
    scenario = LaunchConfiguration("scenario").perform(context)
    condition = LaunchConfiguration("condition").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in ("true", "1")
    show_viewer = LaunchConfiguration("show_viewer").perform(context).lower() in (
        "true",
        "1",
    )
    velocity_frame = LaunchConfiguration("velocity_frame").perform(context)
    behaviour_yaml = LaunchConfiguration("behaviour_yaml").perform(context)

    if backend not in ("mujoco", "gazebo"):
        raise RuntimeError(f'backend must be mujoco or gazebo, got "{backend}"')
    if mode not in ("base", "mapping", "perception", "truth_twin", "behaviour"):
        raise RuntimeError(
            "mode must be base, mapping, perception, truth_twin or behaviour, "
            f'got "{mode}"'
        )
    if mode == "behaviour" and condition not in BT_EXECUTABLES:
        raise RuntimeError(
            f'condition must be one of {sorted(BT_EXECUTABLES)}, got "{condition}"'
        )

    bringup_share = get_package_share_directory("mecanumbot_bringup")
    sim_share = get_package_share_directory("mecanumbot_sim")
    core_share = get_package_share_directory("mecanumbot_core")
    description_share = get_package_share_directory("mecanumbot_description")

    scenario_path = scenario
    if scenario_path and not os.path.isabs(scenario_path):
        scenario_path = os.path.join(
            sim_share, "config", "sim_scenarios", scenario_path
        )
        if not scenario_path.endswith(".yaml"):
            scenario_path += ".yaml"

    core_yaml = os.path.join(core_share, "param", "mecanumbot_core_parameters.yaml")
    urdf_path = os.path.join(description_share, "urdf", "mecanumbot.urdf")
    robot_description = Command(["xacro ", urdf_path, " namespace:=", namespace])

    actions = []

    # ---------------------------------------------------------------- backend
    if backend == "mujoco":
        actions.append(
            Node(
                package="mecanumbot_sim",
                executable="mecanumbot_sim_mujoco_io_node",
                name="mecanumbot_sim_io_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    core_yaml,
                    {
                        # The MuJoCo backend is the clock source, so it must not itself
                        # wait on /clock.
                        "use_sim_time": False,
                        "scenario_path": scenario_path,
                        "publish_clock": True,
                        "show_viewer": show_viewer,
                    },
                ],
                remappings=[
                    (f"/{namespace}/cmd_vel", "/cmd_vel"),
                    (f"/{namespace}/cmd_accessory_pos", "/cmd_accessory_pos"),
                ],
            )
        )
    else:
        gz_args = "-r " + os.path.join(sim_share, "worlds", "mecanumbot_arena.sdf")
        if not show_viewer:
            gz_args = "-s " + gz_args
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_share, "launch", "gz_backend.launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "scenario_path": scenario_path,
                    "velocity_frame": velocity_frame,
                    "gz_args": gz_args,
                }.items(),
            )
        )

    # ------------------------------------------------------- common robot stack
    actions.append(
        Node(
            package="mecanumbot_core",
            executable="mecanumbot_sensorproc_node",
            name="mecanumbot_sensorproc_node",
            namespace=namespace,
            output="screen",
            parameters=[
                core_yaml,
                {
                    "use_sim_time": True,
                    "namespace": namespace,
                    "use_state_stamp_for_dt": True,
                    "require_state_stamp": True,
                },
            ],
        )
    )
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": True},
            ],
            remappings=[("/joint_states", f"/{namespace}/joint_states")],
        )
    )

    if mode != "base":
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_identity",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", "map", f"{namespace}/odom"],
                parameters=[{"use_sim_time": True}],
            )
        )

    # ------------------------------------------------------------- subject pose
    if mode in ORACLE_MODES:
        actions.append(
            Node(
                package="mecanumbot_sim",
                executable="mecanumbot_sim_oracle_subject_node",
                name="mecanumbot_sim_oracle_subject_node",
                namespace=namespace,
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        )

    if mode in DETECTOR_MODES:
        detector_yaml = os.path.join(
            get_package_share_directory("mecanumbot_sensorprocess_smart"),
            "param",
            "lidar_peopledetect_config.yaml",
        )
        actions.append(
            Node(
                package="mecanumbot_sensorprocess_smart",
                executable="mecanumbot_lidar_detect_people",
                name="mecanumbot_lidar_detect_people",
                namespace=namespace,
                output="screen",
                parameters=[
                    detector_yaml,
                    {
                        "use_sim_time": True,
                        "leading_mode": True,
                        "robot_profile": "sim",
                        "tracker_mode": "simple",
                        "enable_map_filter": False,
                        "use_detection_header_frame": True,
                        "reuse_last_subject_pose": True,
                    },
                ],
            )
        )

    # --------------------------------------------------------------- evaluation
    if mode in EVALUATED_MODES:
        detection_params = {
            "use_sim_time": True,
            "subject_pose_topic": "subject_pose",
            "actors_topic": "/sim/actors",
            "evaluation_topic": "/sim/detection_evaluation",
        }
        if mode in DETECTOR_MODES:
            detection_params["detections_topic"] = "dr_spaam/dets"
        else:
            # No detector in the loop: score the oracle track instead of raw returns.
            detection_params["require_raw_detection"] = False

        actions.append(
            Node(
                package="mecanumbot_sim",
                executable="mecanumbot_sim_detection_evaluator_node",
                name="mecanumbot_sim_detection_evaluator_node",
                namespace=namespace,
                output="screen",
                parameters=[detection_params],
            )
        )
        actions.append(
            Node(
                package="mecanumbot_sim",
                executable="mecanumbot_sim_behavior_evaluator_node",
                name="mecanumbot_sim_behavior_evaluator_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "actors_topic": "/sim/actors",
                        "detection_evaluation_topic": "/sim/detection_evaluation",
                        "odom_topic": "odom",
                        "cmd_vel_topic": "/cmd_vel",
                        "behavior_evaluation_topic": "/sim/behavior_evaluation",
                    }
                ],
            )
        )

        visualization_params = {
            "use_sim_time": True,
            "actors_topic": "/sim/actors",
            "subject_pose_topic": "subject_pose",
            "detection_evaluation_topic": "/sim/detection_evaluation",
            "behavior_evaluation_topic": "/sim/behavior_evaluation",
            "actor_markers_topic": "/sim/actor_markers",
            "evaluation_markers_topic": "/sim/evaluation_markers",
        }
        if mode in DETECTOR_MODES:
            visualization_params["detections_topic"] = "dr_spaam/dets"
            visualization_params["detection_markers_topic"] = "/sim/detection_markers"
        else:
            visualization_params["show_detection_markers"] = False

        actions.append(
            Node(
                package="mecanumbot_sim",
                executable="mecanumbot_sim_visualization_node",
                name="mecanumbot_sim_visualization_node",
                namespace=namespace,
                output="screen",
                parameters=[visualization_params],
            )
        )

    if mode in DETECTOR_MODES:
        actions.append(
            Node(
                package="mecanumbot_sim",
                executable="mecanumbot_sim_detector_debug_node",
                name="mecanumbot_sim_detector_debug_node",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "actors_topic": "/sim/actors",
                        "detections_topic": "dr_spaam/dets",
                        "debug_markers_topic": "/sim/detector_debug_markers",
                        "map_frame": "map",
                    }
                ],
            )
        )

    # ------------------------------------------------------------------ mapping
    if mode == "mapping":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "mapping.launch.py")
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            )
        )

    # ---------------------------------------------------------------- behaviour
    if mode == "behaviour":
        actions.append(SetEnvironmentVariable(name="YAML_PATH", value=behaviour_yaml))
        actions.append(
            SetEnvironmentVariable(name="BEHAVIOUR_YAML_PATH", value=behaviour_yaml)
        )
        actions.append(
            Node(
                package="mecanumbot_sim",
                executable="mecanumbot_sim_nav_shim_node",
                name="mecanumbot_sim_nav_shim_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": f"/{namespace}/odom",
                        "goal_topic": "/goal_pose",
                        "cmd_vel_topic": "/cmd_vel",
                        "amcl_pose_topic": "/amcl_pose",
                        "nav_status_topic": "/navigate_to_pose/_action/status",
                    }
                ],
            )
        )
        actions.append(
            Node(
                package="mecanumbot_leading_behaviour",
                executable=BT_EXECUTABLES[condition],
                name=BT_EXECUTABLES[condition],
                namespace=namespace,
                output="screen",
                parameters=[behaviour_yaml, {"use_sim_time": True}],
                remappings=[
                    (f"/{namespace}/cmd_vel", "/cmd_vel"),
                    (f"/{namespace}/cmd_accessory_pos", "/cmd_accessory_pos"),
                ],
            )
        )

    # --------------------------------------------------------------------- rviz
    if use_rviz and mode in RVIZ_MODES:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "rviz2.launch.py")
                ),
            )
        )

    return actions


def generate_launch_description():
    behaviour_share = get_package_share_directory("mecanumbot_leading_behaviour")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="mujoco",
                description="Simulator providing the robot: mujoco or gazebo",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="base",
                description="base | mapping | perception | truth_twin | behaviour",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="mecanumbot",
                description="Robot namespace",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value="single_human_follow",
                description=(
                    "Scenario name from mecanumbot_sim/config/sim_scenarios, or an "
                    "absolute path. Empty for an empty arena."
                ),
            ),
            DeclareLaunchArgument(
                "condition",
                default_value="Doglike",
                description="Behaviour condition when mode:=behaviour: Doglike | Control | LED",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz (ignored for mode:=base)",
            ),
            DeclareLaunchArgument(
                "show_viewer",
                default_value="true",
                description="Show the simulator GUI (MuJoCo viewer / Gazebo GUI)",
            ),
            DeclareLaunchArgument(
                "velocity_frame",
                default_value="body",
                description=(
                    "Gazebo backend only: frame VelocityControl reads cmd_vel in. "
                    "Switch to world if the robot ignores its own heading."
                ),
            ),
            DeclareLaunchArgument(
                "behaviour_yaml",
                default_value=os.path.join(
                    behaviour_share, "config", "sim_behaviour_setting_constants.yaml"
                ),
                description="Behaviour constants YAML used when mode:=behaviour",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
