import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Path to your custom YAML file
    # Replace 'your_package_name' with the actual name of your ROS 2 package
    config_file_path = os.path.join(
        get_package_share_directory("mecanumbot_description"),
        "param",
        "mecanumbot_slam_mapping.yaml",
    )

    # 2. Declare arguments (like use_sim_time)
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation/Gazebo clock if true",
    )

    # 3. Define the SLAM Toolbox Node
    start_async_slam_toolbox_node = Node(
        parameters=[config_file_path, {"use_sim_time": use_sim_time}],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )

    # 4. Create Launch Description and add actions
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
