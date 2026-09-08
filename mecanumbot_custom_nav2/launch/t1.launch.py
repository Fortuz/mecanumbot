"""
T1 in one terminal: drivers, the Deep3R client, SLAM, nav2 and the explorer.

`autoslam.launch.py` deliberately does not start the robot's drivers or the
Deep3R client -- it owns exploration, and a launch file that also owned the
OpenCR link would be two files fighting over one serial port. That separation is
right, and it costs three terminals and an ordering you have to remember. This
file is the convenience wrapper over it: one Ctrl-C stops everything, and the
ordering below is applied rather than remembered.

    ros2 launch mecanumbot_custom_nav2 t1.launch.py

What it starts, in order:

1. `launch_mecanumbot_base.launch.py` with **use_nav2:=false**. The base launch
   otherwise brings up nav2 against the *study* parameters with AMCL and a saved
   map, and that stack and this one both publish `map -> odom` and both serve
   `navigate_to_pose`. Hard-coded false here rather than exposed, because there
   is no T1 in which you want the study stack.
2. `deep3r.launch.py`, after `deep3r_delay` seconds -- the camera topic it
   subscribes to has to exist, and the node's first act is to load the standalone
   client and dial the tunnel.
3. `autoslam.launch.py`, after `explorer_delay` seconds -- slam_toolbox wants
   odometry and a scan before its first update, and the explorer wants a map.

The delays are generous rather than tuned. Nothing here is a race that a
correctly ordered start would lose; they exist so the logs read in the order the
subsystems actually came up, which is most of what makes a failed start
diagnosable in a single scrolling terminal.

**The output is three stacks in one terminal.** That is the trade. When
something is wrong and the noise is in the way, fall back to the three-terminal
form -- it is the same three launch files, and `mecanumbot_custom_nav2`'s README
documents them under "Starting T1".

The server on the cluster is *not* started by this and cannot be: it is a Slurm
job on the other side of an SSH tunnel. Bring it up first, and check the tunnel,
before running this. See `RoboCamStreamProcessing/README.md`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

BRINGUP_SHARE = get_package_share_directory("mecanumbot_bringup")
DEEP3R_SHARE = get_package_share_directory("mecanumbot_deep3r")
EXPLORER_SHARE = get_package_share_directory("mecanumbot_custom_nav2")


def generate_launch_description():
    """Build the launch description for a whole T1 pass in one terminal."""
    use_deep3r = LaunchConfiguration("use_deep3r")
    require_cloud = LaunchConfiguration("require_cloud")
    deep3r_delay = LaunchConfiguration("deep3r_delay")
    explorer_delay = LaunchConfiguration("explorer_delay")
    server = LaunchConfiguration("server")
    client_path = LaunchConfiguration("client_path")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(BRINGUP_SHARE, "launch", "launch_mecanumbot_base.launch.py")
        ),
        # Not a parameter of this file: there is no T1 in which the study nav2
        # stack is what you want, and making it settable would only make it
        # possible to get wrong.
        launch_arguments={"use_nav2": "false"}.items(),
    )

    deep3r = TimerAction(
        period=deep3r_delay,
        actions=[
            LogInfo(msg="[t1] starting the Deep3R client -- the tunnel must be up"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(DEEP3R_SHARE, "launch", "deep3r.launch.py")
                ),
                launch_arguments={
                    "server": server,
                    "client_path": client_path,
                }.items(),
            ),
        ],
        condition=IfCondition(use_deep3r),
    )

    explorer = TimerAction(
        period=explorer_delay,
        actions=[
            LogInfo(msg="[t1] starting SLAM, nav2 and the frontier explorer"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(EXPLORER_SHARE, "launch", "autoslam.launch.py")
                ),
                launch_arguments={"require_cloud": require_cloud}.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_deep3r",
            default_value="true",
            description=(
                "Start the Deep3R client. false is mapping only, and then "
                "require_cloud must be false too or the exit criteria can "
                "never be satisfied and T1 will not finish."
            ),
        ),
        DeclareLaunchArgument(
            "require_cloud",
            default_value="true",
            description=(
                "Let the server's verdict decide when T1 is over. false drops "
                "the `placed` test -- right for a mapping dry run, wrong "
                "during a trial, because a failed T1 then looks exactly like a "
                "successful one."
            ),
        ),
        DeclareLaunchArgument(
            "deep3r_delay", default_value="8.0",
            description="Seconds to wait for the camera topic before the client.",
        ),
        DeclareLaunchArgument(
            "explorer_delay", default_value="15.0",
            description="Seconds to wait for odometry and a scan before SLAM.",
        ),
        DeclareLaunchArgument(
            "server", default_value="tcp://127.0.0.1:5555",
            description="Local end of the forward tunnel, not a cluster address.",
        ),
        DeclareLaunchArgument(
            "client_path", default_value="~/robocam_client.py",
            description="Deployed robocam_client.py, or the directory holding it.",
        ),

        LogInfo(msg=[
            "[t1] one-terminal start. Ctrl-C stops everything. "
            "The cluster server is NOT started by this -- bring it up first.",
        ]),
        LogInfo(msg="[t1] starting drivers and camera (nav2 off: autoslam brings its own)"),
        base,
        deep3r,
        explorer,
    ])
