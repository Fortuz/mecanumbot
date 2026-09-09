import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

mecanumbot_description_pkg_share = get_package_share_directory("mecanumbot_description")


def with_exploration_bt(nav2_params_file):
    """Point bt_navigator at the exploration BT that adds a 360° spin after
    each frontier approach, ensuring full camera coverage before the next
    frontier is requested."""
    bt_dir = os.path.join(mecanumbot_description_pkg_share, "behavior_trees")
    return RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": os.path.join(
                bt_dir, "exploration_nav_to_pose.xml"
            ),
        },
        convert_types=True,
    )


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="mecanumbot", description="Robot namespace"
    )
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    declare_use_joy = DeclareLaunchArgument(
        "use_joy", default_value="true", description="Launch joystick teleop"
    )
    declare_use_web = DeclareLaunchArgument(
        "use_web", default_value="false", description="Launch web GUI"
    )
    declare_uncertainty_threshold = DeclareLaunchArgument(
        "uncertainty_threshold",
        default_value="0.05",
        description="Pose covariance trace threshold that triggers a revisit (m^2 + rad^2)",
    )

    # --- File paths ---
    core_yaml = os.path.join(
        get_package_share_directory("mecanumbot_core"), "param", "mecanumbot_core_parameters.yaml"
    )
    slam_mapping_yaml = os.path.join(
        mecanumbot_description_pkg_share, "param", "mecanumbot_slam_mapping.yaml"
    )
    nav2_params_file = with_exploration_bt(os.path.join(
        mecanumbot_description_pkg_share, "param", "mecanumbot_custom_nav2_no_keepout.yaml"
    ))
    state_publisher_path = os.path.join(
        get_package_share_directory("mecanumbot_bringup"), "launch", "mecanumbot_state_publisher.launch.py"
    )
    audio_path = os.path.join(
        get_package_share_directory("mecanumbot_audio"), "launch", "input_handler.launch.py"
    )
    joy_path = os.path.join(
        get_package_share_directory("mecanumbot_joy"), "launch", "joy_teleop.launch.py"
    )
    web_path = os.path.join(
        get_package_share_directory("mecanumbot_web"), "launch", "web.launch.py"
    )

    actions = [
        declare_namespace,
        declare_sim_time,
        declare_use_joy,
        declare_use_web,
        declare_uncertainty_threshold,
        LogInfo(msg="[exploration] slam_toolbox mapping mode + m-explore-ros2"),

        # --- Robot base ---
        Node(
            package="mecanumbot_core", executable="mecanumbot_io_node",
            name="mecanumbot_io_node", namespace=namespace,
            parameters=[core_yaml, {"use_sim_time": use_sim_time}],
            remappings=[("/mecanumbot/cmd_vel", "/cmd_vel"), ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos")],
            output="screen",
        ),
        Node(
            package="mecanumbot_core", executable="mecanumbot_battery_alert",
            name="mecanumbot_battery_alert", namespace=namespace,
            parameters=[core_yaml, {"use_sim_time": use_sim_time}],
            remappings=[("/mecanumbot/cmd_vel", "/cmd_vel"), ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos")],
            output="screen",
        ),
        Node(
            package="mecanumbot_core", executable="mecanumbot_sensorproc_node",
            name="mecanumbot_sensorproc_node", namespace=namespace,
            parameters=[core_yaml, {"use_sim_time": use_sim_time, "namespace": namespace}],
            output="screen",
        ),
        Node(
            package="ld08_driver", executable="ld08_driver",
            name="ld08_driver_node", namespace=namespace,
            parameters=[
                {"port": "/dev/ld08_lidar"},
                {"frame_id": [namespace, "/base_scan"]},
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
        Node(
            package="mecanumbot_led", executable="mecanumbot_led_service",
            name="mecanumbot_led_service", namespace=namespace,
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(audio_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_path),
            condition=IfCondition(LaunchConfiguration("use_joy")),
            launch_arguments={"namespace": namespace, "joystick_profile": "auto"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_path),
            condition=IfCondition(LaunchConfiguration("use_web")),
            launch_arguments={"namespace": namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_publisher_path),
            launch_arguments={"use_sim_time": use_sim_time, "namespace": namespace}.items(),
        ),

        # --- SLAM: slam_toolbox mapping mode (builds the map live) ---
        Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[slam_mapping_yaml, {"use_sim_time": use_sim_time}],
        ),

        # --- Nav2 navigation stack (no AMCL, no map_server — slam_toolbox provides /map) ---
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "params_file": nav2_params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                }.items(),
            ),
        ]),

        # --- Frontier exploration ---
        Node(
            package="explore_lite", executable="explore",
            name="explore",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_base_frame": "mecanumbot/base_link",
                "costmap_topic": "/global_costmap/costmap",
                "costmap_updates_topic": "/global_costmap/costmap_updates",
                "visualize": True,
                "planner_frequency": 0.5,   # replan frontier every 2 s
                "progress_timeout": 30.0,   # pick new frontier if no progress within 30 s
                "potential_scale": 3.0,
                "gain_scale": 1.0,
                "transform_tolerance": 0.5,
                "min_frontier_size": 0.3,   # ignore frontiers narrower than 30 cm
            }],
            output="screen",
        ),

        # --- Uncertainty monitor: pauses explore_lite and navigates back to origin when
        #     localization covariance grows too large, forcing a loop closure. ---
        Node(
            package="mecanumbot_monitor",
            executable="exploration_uncertainty_monitor",
            name="exploration_uncertainty_monitor",
            output="screen",
            parameters=[{
                "uncertainty_threshold": ParameterValue(
                    LaunchConfiguration("uncertainty_threshold"), value_type=float
                ),
                "check_period": 5.0,        # seconds between covariance checks
                "revisit_x": 0.0,           # return to map origin (start pose)
                "revisit_y": 0.0,
            }],
        ),
    ]

    return LaunchDescription(actions)
