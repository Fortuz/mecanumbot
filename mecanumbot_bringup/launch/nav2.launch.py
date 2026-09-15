"""
The study navigation stack: nav2 with AMCL, the saved map, and the keepout mask.

This used to be the tail of `launch_mecanumbot_base.launch.py`, and the base
launch still includes it under `use_nav2`, so a normal bringup is unchanged. It
is its own file so that nav2 can be started **without** the drivers: the web
GUI's diagnostics page restarts it by stopping whatever study nav2 is on the
graph and launching this file again. That is also the only way back to a study
session after a T1 pass, whose preflight signals the nav2 container away rather
than merely deactivating it -- a lifecycle reset has nothing left to reset.

Everything is chosen when the file is launched, not when the robot booted: a
restart re-reads the SSID, so moving the robot between rooms and restarting
picks up the other room's map. `NAV2_PARAMS_FILE` overrides the parameter file
here exactly as it did in the base launch.

    ros2 launch mecanumbot_bringup nav2.launch.py
"""

import glob
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

mecanumbot_description_pkg_share = get_package_share_directory("mecanumbot_description")


def with_mecanumbot_behavior_trees(nav2_params_file):
    """Point bt_navigator at our behaviour trees instead of the nav2 stock ones.

    The stock recovery round-robin spins 1.57 rad, waits 5 s and reverses 0.30 m
    at 0.05 m/s, which is roughly half a minute of very odd-looking behaviour in
    the middle of a leading trial. RewrittenYaml only replaces keys that already
    exist, so both params are present in the YAML with the nav2 defaults as
    fallback values.
    """
    bt_dir = os.path.join(mecanumbot_description_pkg_share, "behavior_trees")
    return RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": os.path.join(
                bt_dir, "mecanumbot_nav_to_pose.xml"
            ),
            "default_nav_through_poses_bt_xml": os.path.join(
                bt_dir, "mecanumbot_nav_through_poses.xml"
            ),
        },
        convert_types=True,
    )


def get_wifi_ssid():
    # Prefer nmcli if available
    try:
        out = subprocess.check_output(
            ["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2,
        )
        for line in out.splitlines():
            if line.startswith("yes:"):
                return line.split(":", 1)[1].strip()
    except Exception:
        pass

    # Fallback to iwgetid
    try:
        out = subprocess.check_output(
            ["iwgetid", "-r"], stderr=subprocess.DEVNULL, text=True, timeout=2
        )
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
    map_dir = os.path.join(mecanumbot_description_pkg_share, "maps", map_name)
    map_path = os.path.join(map_dir, f"{map_name}.yaml")
    keepout_matches = sorted(glob.glob(os.path.join(map_dir, "*_keepout.yaml")))
    keepout_path = keepout_matches[0] if keepout_matches else ""
    use_keepout = map_name == "LED_exp" and bool(keepout_path)
    nav2_params_file = os.path.join(
        mecanumbot_description_pkg_share,
        "param",
        (
            "mecanumbot_custom_nav2.yaml"
            if use_keepout
            else "mecanumbot_custom_nav2_no_keepout.yaml"
        ),
    )
    return map_name, map_path, keepout_path, nav2_params_file, use_keepout


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    detected_ssid = get_wifi_ssid()
    map_name, map_file, keepout_file, nav2_params_file, _ = choose_launch_profile(
        detected_ssid
    )

    # An override, because the SSID picks the file for a *study* run and not
    # every run is one. T2 of the Deep3R seeking system wants
    # `mecanumbot_seek_nav2.yaml`, which is the same configuration plus the
    # keepout filter that carries the point cloud's view of what the lidar plane
    # cannot see. Left unset, nothing changes for anybody.
    nav2_params_file = os.environ.get("NAV2_PARAMS_FILE") or nav2_params_file
    nav2_params = with_mecanumbot_behavior_trees(nav2_params_file)

    launch_actions = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock",
        ),
        LogInfo(
            msg=f"[nav2] Detected WiFi SSID: {detected_ssid if detected_ssid else 'None'}"
        ),
        LogInfo(msg=f"[nav2] Chosen map setting: {map_name} ({map_file})"),
        LogInfo(msg=f"[nav2] Keepout mask: {keepout_file if keepout_file else 'none'}"),
        LogInfo(msg=f"[nav2] Nav2 params: {nav2_params_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                )
            ),
            launch_arguments={
                "map": map_file,
                "params_file": nav2_params,
                "use_sim_time": use_sim_time,
                "namespace": "",
                "use_namespace": "true",
                "autostart": "true",
            }.items(),
        ),
    ]

    if detected_ssid == "MecanumetoNet":
        launch_actions.extend(
            [
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="keepout_filter_mask_server",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"yaml_filename": keepout_file},
                        {"topic_name": "/keepout_filter_mask"},
                    ],
                ),
                Node(
                    package="nav2_map_server",
                    executable="costmap_filter_info_server",
                    name="keepout_costmap_filter_info_server",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {
                            "type": 0,
                            "filter_info_topic": "/keepout_costmap_filter_info",
                            "mask_topic": "/keepout_filter_mask",
                            "base": 0.0,
                            "multiplier": 1.0,
                        },
                    ],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_keepout_zone",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"autostart": True},
                        {
                            "node_names": [
                                "keepout_filter_mask_server",
                                "keepout_costmap_filter_info_server",
                            ]
                        },
                    ],
                ),
            ]
        )

    return LaunchDescription(launch_actions)
