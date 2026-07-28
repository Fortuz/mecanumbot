# mecanumbot_description

Robot description and runtime assets package.

This package does not contain ROS nodes. It provides files consumed by launch files and nodes in other packages.

## File and folder functions

| Path                                 | Function                                                                            |
| ------------------------------------ | ----------------------------------------------------------------------------------- |
| `maps/`                              | Occupancy map assets (`.pgm` + `.yaml`) used by Nav2 and RViz.                      |
| `meshes/`                            | 3D mesh resources for robot visualization/simulation.                               |
| `param/mecanumbot_custom_nav2.yaml`  | Nav2 parameter set for localization/navigation behavior (with keepout mask).         |
| `param/mecanumbot_custom_nav2_no_keepout.yaml` | Nav2 parameter set used when no keepout mask applies to the selected map. |
| `param/mecanumbot_slam_mapping.yaml` | SLAM Toolbox parameter set for map creation.                                        |
| `param/mecanumbot_lds.lua`           | Cartographer configuration for the LD08 lidar.                                      |
| `rviz/model.rviz`                    | RViz display layout for quick bringup visualization.                                |
| `services/autostart.sh`              | Startup script run on the Jetson Orin Nano: sources ROS and the workspace, then launches the base bringup. |
| `services/ros2.service`              | Systemd unit that runs `autostart.sh` on boot.                                      |
| `udev/90_mecanumbot_nano.rules`      | Udev rule for persistent device naming/permissions for Arduino Nano (LED handling). |
| `udev/91_opencr_board.rules`         | Udev rule creating the `/dev/opencr` symlink used by `mecanumbot_io_node`.          |
| `udev/99-turtlebot3-cdc.rules`       | Udev rule for board serial device handling.                                         |
| `urdf/mecanumbot.urdf`               | Main robot model consumed by `robot_state_publisher`.                               |
| `urdf/common_properties.urdf`        | Shared URDF properties/macros referenced by the main model.                         |

## Additional notes

- Udev rules must be installed to your system udev rules directory to take effect:
  `sudo cp udev/*.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`
- Parameter files in this package are loaded by launch files in `mecanumbot_bringup`.
- The files in `services/` are installed on the robot's Jetson Orin Nano. Both assume the `ubuntu` user and `/home/ubuntu/mecanumbot_ws`; edit `User=` in `ros2.service` and the source paths in `autostart.sh` if the Jetson uses a different account.
