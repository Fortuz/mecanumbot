import os
import glob
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import subprocess

mecanumbot_description_pkg_share = get_package_share_directory('mecanumbot_description')

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
    yaml_file = os.path.join(get_package_share_directory('mecanumbot_sensorprocess_smart'),'param','lidar_peopledetect_config.yaml')
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
    return LaunchDescription(launch_actions)