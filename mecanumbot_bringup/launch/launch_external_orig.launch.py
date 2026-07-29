import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import subprocess

mecanumbot_description_pkg_share = get_package_share_directory('mecanumbot_description')


def with_mecanumbot_behavior_trees(nav2_params_file):
    """Swap bt_navigator onto the mecanumbot behaviour trees.

    Mirrors the helper in launch_mecanumbot_base.launch.py. RewrittenYaml only
    replaces keys that already exist, so both BT params are declared in the YAML
    with the nav2 stock trees as fallback values.
    """
    bt_dir = os.path.join(mecanumbot_description_pkg_share, 'behavior_trees')
    return RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml':
                os.path.join(bt_dir, 'mecanumbot_nav_to_pose.xml'),
            'default_nav_through_poses_bt_xml':
                os.path.join(bt_dir, 'mecanumbot_nav_through_poses.xml'),
        },
        convert_types=True,
    )

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


def choose_default_map(ssid):
    ssid_to_map = {
        "MecanumNet": "AI_dept",
        "MecanumetoNet": "ethodept_old",
    }
    map_name = ssid_to_map.get(ssid, "AI_dept")
    map_path = os.path.join(mecanumbot_description_pkg_share, 'maps', map_name, f"{map_name}.yaml")
    return map_name, map_path


def generate_launch_description():
    
    rviz_config_dir = os.path.join(get_package_share_directory('mecanumbot_description'),'rviz','model.rviz')
    param_file = with_mecanumbot_behavior_trees(
        os.path.join(mecanumbot_description_pkg_share, 'param', 'mecanumbot_custom_nav2_no_keepout.yaml'))
    detected_ssid = get_wifi_ssid()
    map_name, map_file = choose_default_map(detected_ssid)
    yaml_file = os.path.join(get_package_share_directory('mecanumbot_sensorprocess_smart'),'param','lidar_peopledetect_config.yaml')
    return LaunchDescription([
        LogInfo(msg=f"[launch_external] Detected WiFi SSID: {detected_ssid if detected_ssid else 'None'}"),
        LogInfo(msg=f"[launch_external] Chosen map setting: {map_name} ({map_file})"),
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
                    "params_file": param_file,
                    "use_sim_time": "false",
                }.items()
            ),
        ]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

    ])
'''
            Node(
            namespace="mecanumbot",
            package="mecanumbot_sensorprocess_smart",
            executable="mecanumbot_detect_tennis",
            name="mecanumbot_detect_tennis", 
            output="screen"
        ),
        Node(
            namespace="mecanumbot",
            package="mecanumbot_sensorprocess_smart",
            executable="mecanumbot_lidar_detect_people",
            name="mecanumbot_lidar_detect_people",  # must match YAML top-level key
            output="screen",
            parameters=[yaml_file],
            remappings=[
                ('map', '/map')]
            
)
)'''
