import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    description_prefix = get_package_share_directory("mecanumbot_description")

    cartographer_config_dir = LaunchConfiguration(
        "cartographer_config_dir", default=os.path.join(description_prefix, "param")
    )
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="mecanumbot_lds.lua"
    )

    rviz_config = os.path.join(
        get_package_share_directory("mecanumbot_description"), "rviz", "gmapping.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "cartographer_config_dir", default_value=cartographer_config_dir
            ),
            DeclareLaunchArgument(
                "configuration_basename", default_value=configuration_basename
            ),
            # CARTOGRAPHER NODE
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                namespace="mecanumbot",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    cartographer_config_dir,
                    "-configuration_basename",
                    configuration_basename,
                ],
                remappings=[
                    ("scan", "/mecanumbot/scan"),
                    ("imu", "/mecanumbot/imu"),
                    ("odom", "/mecanumbot/odom"),
                    # NOTE: Global /tf and /tf_static are intentionally NOT remapped here
                ],
            ),
            # ──────────────────────────────────────────────
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                namespace="mecanumbot",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "resolution": 0.05,
                        "publish_period_sec": 1.0,
                    }
                ],
                remappings=[
                    ("submap_list", "/mecanumbot/submap_list"),
                    ("map", "/map"),  # Forces map topic to global /map for RViz
                ],
            ),
        ]
    )
