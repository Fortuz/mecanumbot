# mecanumbot_description

Robot description and runtime assets package.

This package does not contain ROS nodes. It provides files consumed by launch files and nodes in other packages.

|--

## File and folder functions

| Path                                 | Function                                                                            |
| ------------------------------------ | ----------------------------------------------------------------------------------- |
| `maps/`                              | Occupancy map assets (`.pgm` + `.yaml`) used by Nav2 and RViz.                      |
| `meshes/`                            | 3D mesh resources for robot visualization/simulation.                               |
| `param/mecanumbot_custom_nav2.yaml`  | Nav2 parameter set for localization/navigation behavior.                            |
| `param/mecanumbot_slam_mapping.yaml` | SLAM Toolbox parameter set for map creation.                                        |
| `rviz/model.rviz`                    | RViz display layout for quick bringup visualization.                                |
| `udev/90_mecanumbot_nano.rules`      | Udev rule for persistent device naming/permissions for Arduino Nano (LED handling). |
| `udev/99-turtlebot3-cdc.rules`       | Udev rule for board serial device handling.                                         |
| `urdf/mecanumbot.urdf`               | Main robot model consumed by `robot_state_publisher`.                               |
| `urdf/common_properties.urdf`        | Shared URDF properties/macros referenced by the main model.                         |

## Additional notes

- Udev rules must be installed to your system udev rules directory to take effect.
- Parameter files in this package are loaded by launch files in `mecanumbot_bringup`.
