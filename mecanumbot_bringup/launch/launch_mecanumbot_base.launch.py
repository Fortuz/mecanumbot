import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    use_nav2 = LaunchConfiguration("use_nav2")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="mecanumbot", description="Robot namespace"
    )

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock",
    )

    declare_use_joy = DeclareLaunchArgument(
        "use_joy",
        default_value="true",
        description="Launch joy_node and the onboard joystick node",
    )

    declare_use_nav2 = DeclareLaunchArgument(
        "use_nav2",
        default_value="true",
        description=(
            "Bring up nav2 against the STUDY parameters, with AMCL and a saved "
            "map. Set false for T1: mecanumbot_autoslam's launch_autoslam "
            "brings its own nav2 without AMCL, because slam_toolbox owns "
            "map -> odom during exploration and two things estimating the same "
            "transform is the most confusing way for a run to fail. This "
            "argument was documented for months before it existed -- passing it "
            "did nothing and nav2 came up anyway."
        ),
    )

    declare_use_web = DeclareLaunchArgument(
        "use_web",
        default_value="true",
        description="Launch the robot-hosted web GUI on port 8080",
    )

    declare_joystick_profile = DeclareLaunchArgument(
        "joystick_profile",
        default_value="auto",
        description="Joystick profile stem, or 'auto' to detect from the pad",
    )

    # --- File Paths ---
    core_yaml = os.path.join(
        get_package_share_directory("mecanumbot_core"),
        "param",
        "mecanumbot_core_parameters.yaml",
    )

    state_publisher_path = os.path.join(
        get_package_share_directory("mecanumbot_bringup"),
        "launch",
        "mecanumbot_state_publisher.launch.py",
    )

    camera_path = os.path.join(
        get_package_share_directory("mecanumbot_bringup"), "launch", "camera.launch.py"
    )

    audio_path = os.path.join(
        get_package_share_directory("mecanumbot_audio"),
        "launch",
        "input_handler.launch.py",
    )

    joy_path = os.path.join(
        get_package_share_directory("mecanumbot_joy"),
        "launch",
        "joy_teleop.launch.py",
    )

    web_path = os.path.join(
        get_package_share_directory("mecanumbot_web"),
        "launch",
        "web.launch.py",
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory("mecanumbot_description"), "rviz", "model.rviz"
    )

    nav2_path = os.path.join(
        get_package_share_directory("mecanumbot_bringup"), "launch", "nav2.launch.py"
    )

    # --- Build Launch Actions ---
    launch_actions = [
        declare_namespace,
        declare_sim_time,
        declare_use_joy,
        declare_use_nav2,
        declare_use_web,
        declare_joystick_profile,
        # mecanumbot_core IO node
        Node(
            package="mecanumbot_core",
            executable="mecanumbot_io_node",
            name="mecanumbot_io_node",
            namespace=namespace,
            parameters=[core_yaml, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/mecanumbot/cmd_vel", "/cmd_vel"),
                ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
            ],
            output="screen",
        ),
        # mecanumbot_core Battery Alert
        Node(
            package="mecanumbot_core",
            executable="mecanumbot_battery_alert",
            name="mecanumbot_battery_alert",
            namespace=namespace,
            parameters=[core_yaml, {"use_sim_time": use_sim_time}],
            remappings=[
                ("/mecanumbot/cmd_vel", "/cmd_vel"),
                ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
            ],
            output="screen",
        ),
        # mecanumbot_core Sensor Processing node
        Node(
            package="mecanumbot_core",
            executable="mecanumbot_sensorproc_node",
            name="mecanumbot_sensorproc_node",
            namespace=namespace,
            parameters=[
                core_yaml,
                {"use_sim_time": use_sim_time, "namespace": namespace},
            ],
            output="screen",
        ),
        # LD08 Lidar driver node
        Node(
            package="ld08_driver",
            executable="ld08_driver",
            name="ld08_driver_node",
            namespace=namespace,
            parameters=[
                {"port": "/dev/ld08_lidar"},
                {"frame_id": [namespace, "/base_scan"]},
                {"use_sim_time": use_sim_time},
            ],
            output="screen",
        ),
        # LED control service node
        Node(
            package="mecanumbot_led",
            executable="mecanumbot_led_service",
            name="mecanumbot_led_service",
            namespace=namespace,
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        # Audio input handler
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(audio_path),
        ),
        # Onboard joystick: joy_node plus the profile-driven teleop node.
        # This is the only /joy consumer on the robot.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_path),
            condition=IfCondition(LaunchConfiguration("use_joy")),
            launch_arguments={
                "namespace": namespace,
                "joystick_profile": LaunchConfiguration("joystick_profile"),
            }.items(),
        ),
        # Robot-hosted web GUI: joystick editing, diagnostics, behaviour params
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_path),
            condition=IfCondition(LaunchConfiguration("use_web")),
            launch_arguments={"namespace": namespace}.items(),
        ),
        # State publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_publisher_path),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "namespace": namespace,
            }.items(),
        ),
        # Perception is NOT started here. It used to be, so every run of the
        # robot -- a teleop session, a mapping run, T1 exploration -- carried a
        # DR-SPAAM detector, a DeepStream network and the fusion node whether or
        # not anything subscribed to them. On an Orin Nano that is not free: the
        # network is most of the GPU, and it holds the camera open so nothing
        # else can have it, which is why there was no
        # /camera/image_raw/compressed to record a trial from.
        #
        # It is started by the behaviour that needs it instead, with the
        # detector that behaviour needs:
        #
        #   ros2 launch mecanumbot_sensorprocess_smart perception.launch.py \
        #       detector:=pose|fetch|none use_camera:=true|false
        #
        # which is what the leading, ostensive, seek and fetch launch files
        # include. `mecanumbot_peopledetect.launch.py` is the same thing under
        # its older name, for a run with no tree.
    ]

    # --- Navigation ---
    # The study nav2 stack (AMCL, the SSID's map, its keepout mask) lives in
    # nav2.launch.py so that the web GUI can restart it on its own: a restart
    # stops whatever of it is on the graph -- including the copy started here --
    # and launches that file again, without touching the drivers.
    launch_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_path),
            condition=IfCondition(use_nav2),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )
    )

    return LaunchDescription(launch_actions)

    """
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
    """
