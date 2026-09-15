# mecanumbot_description

Robot description and runtime assets package.

This package does not contain ROS nodes. It provides files consumed by launch files and nodes in other packages.

## File and folder functions

| Path                                 | Function                                                                            |
| ------------------------------------ | ----------------------------------------------------------------------------------- |
| `behavior_trees/`                    | `mecanumbot_nav_to_pose.xml` and `mecanumbot_nav_through_poses.xml`: the `bt_navigator` trees `mecanumbot_bringup/nav2.launch.py` substitutes for the Nav2 stock ones. |
| `config/joystick/`                   | Joystick mapping profiles (`generic.yaml`, `ps4.yaml`, `xbox360.yaml`) read by `mecanumbot_joy`.                |
| `maps/`                              | Occupancy map assets (`.pgm` + `.yaml`) used by Nav2 and RViz: `AI_dept`, `LED_exp` (plus `LED_exp_keepout`) and `ethodept_old`, each with a `<map>_waypoints.yaml` used by `mecanumbot_demo_behaviours`. |
| `meshes/`                            | 3D mesh resources for robot visualization/simulation.                               |
| `param/mecanumbot_custom_nav2.yaml`  | Nav2 parameter set for localization/navigation behavior (with keepout mask).         |
| `param/mecanumbot_custom_nav2_no_keepout.yaml` | Nav2 parameter set used when no keepout mask applies to the selected map. |
| `param/mecanumbot_slam_mapping.yaml` | SLAM Toolbox parameter set for map creation (`mapping.launch.py`, and `mecanumbot_autoslam` for T1). Reads `/mecanumbot/scan_grid` from `mecanumbot_scan_grid_node`, not the driver's `/mecanumbot/scan`. |
| `param/mecanumbot_exploration_nav2.yaml` | Nav2 parameter set for the T1 exploration pass (no AMCL or map server), loaded by `mecanumbot_autoslam`'s `launch_autoslam.launch.py`. |
| `param/mecanumbot_seek_nav2.yaml`    | Nav2 parameter set for T2 of the seeking system: the no-keepout study set plus the point-cloud keepout filter. Selected with `NAV2_PARAMS_FILE`. |
| `param/mecanumbot_lds.lua`           | Cartographer configuration for the LD08 lidar.                                      |
| `param/ros-humble-slam-toolbox-2.6.10/` (+ `.orig.tar.gz`, `.debian.tar.xz`, `.dsc`) | Unpacked Debian source package of slam_toolbox 2.6.10. Not referenced by any launch file. |
| `rviz/model.rviz`                    | RViz display layout for quick bringup visualization.                                |
| `rviz/gmapping.rviz`                 | RViz layout referenced by `slam_gmapping.launch.py` (Cartographer).                 |
| `services/autostart.sh`              | Startup script run on the Jetson Orin Nano: sources ROS and the workspace, exports `ROS_DOMAIN_ID=19`, `ROS_LOCALHOST_ONLY=0` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, then launches `launch_mecanumbot_base.launch.py`. |
| `services/ros2.service`              | Systemd unit that runs `autostart.sh` on boot.                                      |
| `udev/90_mecanumbot_nano.rules`      | Udev rule for persistent device naming/permissions for Arduino Nano (LED handling). |
| `udev/91_opencr_board.rules`         | Udev rule creating the `/dev/opencr` symlink used by `mecanumbot_io_node`.          |
| `udev/99-turtlebot3-cdc.rules`       | Udev rule for board serial device handling; also creates the `/dev/ld08_lidar` symlink. |
| `udev/99-ps4-controller.rules`, `udev/99-xbox360-wireless.rules` | Permissions for the PS4 DualShock 4 and the Xbox 360 wireless adapter. |
| `urdf/mecanumbot.urdf`               | Main robot model consumed by `robot_state_publisher`.                               |
| `urdf/common_properties.urdf`        | Shared URDF properties/macros referenced by the main model.                         |

## Additional notes

- Udev rules must be installed to your system udev rules directory to take effect:
  `sudo cp udev/*.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`
- Parameter files in this package are loaded by launch files in `mecanumbot_bringup`, except `mecanumbot_exploration_nav2.yaml` (`mecanumbot_autoslam`) and `mecanumbot_seek_nav2.yaml` (passed in through `NAV2_PARAMS_FILE`).
- The files in `services/` are installed on the robot's Jetson Orin Nano. Both assume the `ubuntu` user and `/home/ubuntu/mecanumbot_ws`; edit `User=` in `ros2.service` and the source paths in `autostart.sh` if the Jetson uses a different account.
