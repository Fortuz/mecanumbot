"""
T1: scan the place. SLAM, nav2 without localization, and the frontier explorer.

This is the whole first pass in one launch file. It brings up slam_toolbox in
mapping mode, nav2 against the exploration parameter file in
`mecanumbot_description`, and the RRT explorer that drives the two of them
around until the exit criteria are met.

Nav2 comes from `nav2_bringup/navigation_launch.py` rather than the usual
`bringup_launch.py`, and that choice is the whole point: `navigation_launch`
starts the controller, planner, behaviours and BT navigator and *nothing else*
-- no `map_server`, no AMCL. During T1 there is no map to localize against and
slam_toolbox owns `map -> odom`; bringing AMCL up as well gives two things
estimating the same transform, which is the single most confusing way for an
exploration run to fail.

It does **not** bring up the robot's drivers, its camera, or the Deep3R client.
Those come from their own launch files, and starting them here would give two
launch files that both own the OpenCR link:

    ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py use_nav2:=false
    ros2 launch mecanumbot_deep3r deep3r.launch.py
    ros2 launch mecanumbot_custom_nav2 autoslam.launch.py

`use_nav2:=false` matters for the same reason: the base launch starts nav2
against the *study* parameters, with AMCL and a saved map.

When the explorer is satisfied it latches `/mecanumbot/exploration/finished`.
Saving the map is deliberately not automatic -- the map T2 localizes against is
worth looking at before it is written, and `map_saver_cli` is one command:

    ros2 run nav2_map_server map_saver_cli -f <maps>/AI_dept/AI_dept
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DESCRIPTION_SHARE = get_package_share_directory("mecanumbot_description")
EXPLORER_SHARE = get_package_share_directory("mecanumbot_custom_nav2")
NAV2_SHARE = get_package_share_directory("nav2_bringup")


def generate_launch_description():
    """Build the launch description for the autoslam exploration pass."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params")
    nav2_params = LaunchConfiguration("nav2_params")
    explorer_params = LaunchConfiguration("explorer_params")
    agreement_params = LaunchConfiguration("agreement_params")
    namespace = LaunchConfiguration("namespace")
    require_cloud = LaunchConfiguration("require_cloud")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock (sim.launch.py sets this)",
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(
                DESCRIPTION_SHARE, "param", "mecanumbot_slam_mapping.yaml"
            ),
            description="slam_toolbox parameters",
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(
                DESCRIPTION_SHARE, "param", "mecanumbot_exploration_nav2.yaml"
            ),
            description=(
                "Nav2 parameters. The exploration file has no AMCL block and no "
                "static layer, because slam_toolbox owns map -> odom during T1."
            ),
        ),
        DeclareLaunchArgument(
            "explorer_params",
            default_value=os.path.join(
                EXPLORER_SHARE, "config", "frontier_explorer.yaml"
            ),
            description="Frontier explorer constants",
        ),
        DeclareLaunchArgument(
            "agreement_params",
            default_value=os.path.join(
                EXPLORER_SHARE, "config", "map_agreement.yaml"
            ),
            description="Constants for the 2D/3D comparison handler",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="mecanumbot",
            description="Namespace for the explorer node",
        ),
        DeclareLaunchArgument(
            "require_cloud",
            default_value="true",
            description=(
                "Whether T1 may only end once the server says the reconstruction "
                "is good enough. Set false for a mapping-only dry run with no "
                "Deep3R link -- otherwise the exit criteria are never satisfied."
            ),
        ),
    ]

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    # Navigation only: controller, planner, behaviours, BT navigator, smoother.
    # No map_server and no AMCL -- see the module docstring.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV2_SHARE, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
            # Nav2 stays outside the robot namespace, as it does everywhere else
            # in this workspace: the trees and this explorer address it with
            # absolute names.
            "namespace": "",
        }.items(),
    )

    explorer = Node(
        package="mecanumbot_custom_nav2",
        executable="mecanumbot_frontier_explorer_node",
        name="mecanumbot_frontier_explorer",
        namespace=namespace,
        output="screen",
        parameters=[
            explorer_params,
            {"use_sim_time": use_sim_time, "require_cloud": require_cloud},
        ],
    )

    # Runs in both phases -- see its README. It is started here because T1 is
    # where the verdicts begin arriving, and started again by the seek launch
    # for T2; running two is harmless (they publish the same mask) but pointless,
    # so a session that spans both phases should keep this one alive.
    agreement = Node(
        package="mecanumbot_custom_nav2",
        executable="mecanumbot_map_agreement_node",
        name="mecanumbot_map_agreement",
        output="screen",
        parameters=[agreement_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(
                msg=(
                    "autoslam (T1): slam_toolbox + nav2 (no AMCL) + RRT explorer "
                    "+ the 2D/3D comparison handler"
                )
            ),
            LogInfo(
                msg=[
                    "exit criteria from ",
                    explorer_params,
                    " -- watch /mecanumbot/exploration/state",
                ]
            ),
            slam,
            nav2,
            agreement,
            explorer,
        ]
    )
