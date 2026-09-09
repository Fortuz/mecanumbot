"""
Operator-PC tools: the RViz view, and optionally an off-board person detector.

**Perception is not started here by default.** Every behaviour launch file now
includes `mecanumbot_sensorprocess_smart/launch/perception.launch.py` with the
detector that behaviour needs, and the base launch starts none of it -- so a
DR-SPAAM node started unconditionally here is either a second detector
publishing onto the same `/mecanumbot/dr_spaam/dets` and `subject_pose` as the
robot's own (same node name, same namespace, same ROS_DOMAIN_ID), or a detector
nothing subscribes to.

`use_people_detection:=true` starts it anyway, for the one case that still wants
it: running DR-SPAAM off the robot, on a machine that is not the Orin Nano,
against the robot's `scan`. Start it on ONE machine only -- if it is running
here, the behaviour launch on the robot needs `use_perception:=false`, or the
robot's own perception needs `use_lidar_people:=false`.

Nav2 is deliberately not started from this file: the robot's base launch brings
up its own, and a second one on the operator PC would fight it for `/cmd_vel`
and the costmaps. The SSID-driven map/keepout selection that block used is kept
below, commented, because that is where it was written down.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RVIZ_LAUNCH = os.path.join(
    get_package_share_directory("mecanumbot_bringup"),
    "launch",
    "rviz2.launch.py",
)


def generate_launch_description():
    """Build the launch description for the operator-PC tools."""
    yaml_file = os.path.join(
        get_package_share_directory("mecanumbot_sensorprocess_smart"),
        "config",
        "lidar_peopledetect_config.yaml",
    )
    actions = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Open RViz with the workspace's model.rviz config",
        ),
        DeclareLaunchArgument(
            "use_people_detection",
            default_value="false",
            description=(
                "Run DR-SPAAM here instead of on the robot. Off by default: "
                "the behaviour that needs people detection starts it itself, "
                "and two of these on one ROS domain publish over each other"
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(RVIZ_LAUNCH),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        Node(
            namespace="mecanumbot",
            package="mecanumbot_sensorprocess_smart",
            executable="mecanumbot_lidar_detect_people",
            name="mecanumbot_lidar_detect_people",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_people_detection")),
            parameters=[yaml_file],
            remappings=[
                ("map", "/map"),
                ("keepout_filter_mask", "/keepout_filter_mask"),
            ],
        ),
    ]
    return LaunchDescription(actions)


"""mecanumbot_description_pkg_share = get_package_share_directory('mecanumbot_description')

def get_wifi_ssid():
    # Prefer nmcli if available
    try:
        out = subprocess.check_output([
            "nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"
        ], stderr=subprocess.DEVNULL, text=True, timeout=2)
        for line in out.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1].strip()
    except Exception:
        pass

    # Fallback to iwgetid
    try:
        out = subprocess.check_output(["iwgetid", "-r"], stderr=subprocess.DEVNULL, text=True, timeout=2)
        return out.strip() if out.strip() else None
    except Exception:
        pass

    return None


def choose_launch_profile(ssid):
    ssid_to_map = {
        "MecanumNet": "AI_dept",
        "MecanumetoNet": "LED_exp",
    }
    map_name = ssid_to_map.get(ssid, "AI_dept")
    map_dir = os.path.join(mecanumbot_description_pkg_share, 'maps', map_name)
    map_path = os.path.join(map_dir, f"{map_name}.yaml")
    keepout_matches = sorted(glob.glob(os.path.join(map_dir, "*_keepout.yaml")))
    keepout_path = keepout_matches[0] if keepout_matches else ""
    use_keepout = map_name == "LED_exp" and bool(keepout_path)
    nav2_params_file = os.path.join(
        mecanumbot_description_pkg_share,
        'param',
        'mecanumbot_custom_nav2.yaml' if use_keepout else 'mecanumbot_custom_nav2_no_keepout.yaml'
    )
    return map_name, map_path, keepout_path, nav2_params_file, use_keepout


def generate_launch_description():
    
    rviz_config_dir = os.path.join(get_package_share_directory('mecanumbot_description'),'rviz','model.rviz')
    detected_ssid = get_wifi_ssid()
    map_name, map_file, keepout_file, nav2_params_file, _ = choose_launch_profile(detected_ssid)
    yaml_file = os.path.join(get_package_share_directory('mecanumbot_sensorprocess_smart'),'config','lidar_peopledetect_config.yaml')
    launch_actions = [
        LogInfo(msg=f"[launch_external] Detected WiFi SSID: {detected_ssid if detected_ssid else 'None'}"),
        LogInfo(msg=f"[launch_external] Chosen map setting: {map_name} ({map_file})"),
        LogInfo(msg=f"[launch_external] Keepout mask: {keepout_file if keepout_file else 'none'}"),
        LogInfo(msg=f"[launch_external] Nav2 params: {nav2_params_file}"),
    ]

    if detected_ssid == "MecanumetoNet":
        launch_actions.extend([
            GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch',
                            'bringup_launch.py'
                        )
                    ),
                    launch_arguments={
                        "map": map_file,
                        "use_keepout_zones": "true",
                        "params_file": nav2_params_file,
                        "use_sim_time": "false",
                    }.items()
                ),
            ]),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'yaml_filename': keepout_file},
                    {'topic_name': "/keepout_filter_mask"}
                ],
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='keepout_costmap_filter_info_server',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {
                        'type': 0,
                        'filter_info_topic': '/keepout_costmap_filter_info',
                        'mask_topic': '/keepout_filter_mask',
                        'base': 0.0,
                        'multiplier': 1.0,
                    },
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout_zone',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']},
                ],
            ),
            Node(
                namespace="mecanumbot",
                package="mecanumbot_sensorprocess_smart",
                executable="mecanumbot_lidar_detect_people",
                name="mecanumbot_lidar_detect_people",
                output="screen",
                parameters=[yaml_file],
                remappings=[
                    ('map', '/map')
                ]
            ),
            Node(
                namespace="mecanumbot",
                package="mecanumbot_sensorprocess_smart",
                executable="mecanumbot_detect_tennis",
                name="mecanumbot_detect_tennis",
                output="screen"
            )
        ])
    else:
        launch_actions.extend([
            GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch',
                            'bringup_launch.py'
                        )
                    ),
                    launch_arguments={
                        "map": map_file,
                        "params_file": nav2_params_file,
                        "use_sim_time": "false",
                    }.items()
                ),
            ]),
        ])
    return LaunchDescription(launch_actions)"""
