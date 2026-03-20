# mecanumbot_bringup

Launch orchestration package for booting the robot stack.

This package does not contain ROS nodes implemented in source code. It provides launch files that start/configure nodes from other packages.

## Launch file functions

| File | Function |
|---|---|
| `launch/launch_mecanumbot_base.launch.py` | Starts base robot runtime: `mecanumbot_io_node`, `mecanumbot_sensorproc_node`, `ld08_driver`, and robot state publisher include. |
| `launch/launch_external.launch.py` | Starts external tools (`nav2_bringup`, `rviz2`) and optional people detection node, with Wi-Fi SSID based map selection. |
| `launch/mapping.launch.py` | Starts `slam_toolbox` (`async_slam_toolbox_node`) using mapping parameter file. |
| `launch/mecanumbot_state_publisher.launch.py` | Starts `robot_state_publisher` with robot description from URDF/xacro. |
| `launch/rviz2.launch.py` | Starts RViz with the package-provided visualization config. |
| `launch/camera.launch.py` | Starts `v4l2_camera_node` with camera parameter file. |

## Additional notes

- This package is a coordinator: node logic lives in packages such as `mecanumbot_core`, `mecanumbot_led`, and external dependencies.
- Launch argument defaults (namespace, sim time, map/params) are defined in the launch files listed above.
