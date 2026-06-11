import os
import glob
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

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

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='mecanumbot',
        description='Robot namespace'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    # --- File Paths ---
    core_yaml = os.path.join(
        get_package_share_directory('mecanumbot_core'),
        'param',
        'mecanumbot_core_parameters.yaml'
    )
    
    lidar_detect_yaml = os.path.join(
        get_package_share_directory('mecanumbot_sensorprocess_smart'),
        'param',
        'lidar_peopledetect_config.yaml'
    )

    state_publisher_path = os.path.join(
        get_package_share_directory('mecanumbot_bringup'),
        'launch',
        'mecanumbot_state_publisher.launch.py'
    )

    camera_path = os.path.join(
        get_package_share_directory('mecanumbot_bringup'),
        'launch',
        'camera.launch.py'
    )

    audio_path = os.path.join(
        get_package_share_directory('mecanumbot_audio'),
        'launch',
        'input_handler.launch.py'
    )

    rviz_config_dir = os.path.join(get_package_share_directory('mecanumbot_description'),'rviz','model.rviz')

    # --- Network & Profile Logic ---
    detected_ssid = get_wifi_ssid()
    map_name, map_file, keepout_file, nav2_params_file, _ = choose_launch_profile(detected_ssid)

    # --- Build Launch Actions ---
    launch_actions = [
        declare_namespace,
        declare_sim_time,
        
        LogInfo(msg=f"[onboard_bringup] Detected WiFi SSID: {detected_ssid if detected_ssid else 'None'}"),
        LogInfo(msg=f"[onboard_bringup] Chosen map setting: {map_name} ({map_file})"),
        LogInfo(msg=f"[onboard_bringup] Keepout mask: {keepout_file if keepout_file else 'none'}"),
        LogInfo(msg=f"[onboard_bringup] Nav2 params: {nav2_params_file}"),

        # mecanumbot_core IO node
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_io_node',
            name='mecanumbot_io_node',
            namespace=namespace,
            parameters=[core_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos')],
            output='screen'
        ),
        
        # mecanumbot_core Battery Alert
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_battery_alert',
            name='mecanumbot_battery_alert',
            namespace=namespace,
            parameters=[core_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos')],
            output='screen'
        ),
        
        # mecanumbot_core Sensor Processing node
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_sensorproc_node',
            name='mecanumbot_sensorproc_node',
            namespace=namespace,
            parameters=[core_yaml, {'use_sim_time': use_sim_time, 'namespace': namespace}],
            output='screen'
        ),

        # LD08 Lidar driver node
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver_node',
            namespace=namespace,
            parameters=[
                {'port': '/dev/ld08_lidar'},
                {'frame_id': [namespace, '/base_scan']},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # LED control service node
        Node(
            package='mecanumbot_led',
            executable='mecanumbot_led_service',
            name='mecanumbot_led_service',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Audio input handler
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(audio_path),
        ),

        # State publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_publisher_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace
            }.items()
        ),
    ]

    # --- Conditional Navigation & Vision Nodes ---
    if detected_ssid == "MecanumetoNet":
        launch_actions.extend([
            GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                "map": map_file,
                "params_file": nav2_params_file,
                "use_sim_time": use_sim_time,
                "namespace": namespace,          # <-- ADD THIS: Passes the mecanumbot namespace
                "use_namespace": "true",         # <-- ADD THIS: Forces Nav2 to use it
                "autostart": "true",             # <-- ADD THIS: Ensures the lifecycle manager starts
            }.items()
                ),
            ]),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
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
                    {'use_sim_time': use_sim_time},
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
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']},
                ],
            ),
            Node(
                namespace=namespace,
                package="mecanumbot_sensorprocess_smart",
                executable="mecanumbot_lidar_detect_people",
                name="mecanumbot_lidar_detect_people",
                output="screen",
                parameters=[lidar_detect_yaml],
                remappings=[
                    ('map', '/map')
                ]
            ),
            Node(
                namespace=namespace,
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
                        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        "map": map_file,
                        "params_file": nav2_params_file,
                        "use_sim_time": use_sim_time,
                        "namespace": "",          # <-- ADD THIS: Passes the mecanumbot namespace
                        "use_namespace": "true",
                        "autostart": "true",
                    }.items()
                ),
            ]),
            LogInfo(msg="LaunchActions Extended"),
        ])

    return LaunchDescription(launch_actions)

    '''
    # Optional / Commented Out Camera Nodes
    Node(
        package='mecanumbot_camera_stream',
        executable='compressed_camera_publisher_node',
        name='compressed_camera_publisher_node',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time,'camera_backend': 'usb', 'device': '/dev/video0'}],
        output='screen'
    ),
    
    Node(
        namespace="mecanumbot",
        package="mecanumbot_sensorprocess_smart",
        executable="mecanumbot_cam_detect_people",
        name="mecanumbot_cam_detect_people", 
        output="screen",
        parameters=[core_yaml],
        remappings=[
            ('map', '/map')
        ]
    ),

    # Optimized compressed camera publisher
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_path)
    ),
    '''