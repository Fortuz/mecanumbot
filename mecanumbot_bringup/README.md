# mecanumbot_bringup

Launch orchestration package for booting the robot stack.

This package does not contain ROS nodes implemented in source code. It provides launch files that start/configure nodes from other packages.

The onboard launch files run on the robot's NVIDIA Jetson Orin Nano. `launch_mecanumbot_base.launch.py` starts the full onboard stack — drivers, perception and Nav2 — on the Jetson itself; the remote PC is only needed for visualization and teleoperation.

## Launch file functions

| File                                          | Function                                                                                                                         |
| --------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `launch/launch_mecanumbot_base.launch.py`     | Full onboard runtime. Starts `mecanumbot_io_node`, `mecanumbot_battery_alert`, `mecanumbot_sensorproc_node`, `ld08_driver`, `mecanumbot_led_service`, the audio input handler, the robot state publisher include, the `mecanumbot_sensorprocess_smart` perception nodes, and a `nav2_bringup` include. Map, Nav2 parameter set and keepout mask are selected from the connected Wi-Fi SSID. |
| `launch/launch_external.launch.py`            | Starts external tools (`nav2_bringup`, `rviz2`) and people detection nodes on a remote PC, with the same Wi-Fi SSID based map selection. |
| `launch/mapping.launch.py`                    | Starts `slam_toolbox` (`async_slam_toolbox_node`) using mapping parameter file.                                                  |
| `launch/slam_gmapping.launch.py`              | Starts `cartographer_node` and `cartographer_occupancy_grid_node` using `mecanumbot_description/param/mecanumbot_lds.lua`.       |
| `launch/mecanumbot_state_publisher.launch.py` | Starts `robot_state_publisher` with robot description from URDF/xacro.                                                           |
| `launch/rviz2.launch.py`                      | Starts RViz with the package-provided visualization config.                                                                      |
| `launch/camera.launch.py`                     | Includes `mecanumbot_camera_stream/camera_compressed.launch.py` (JPEG compressed stream). Replaces the earlier raw `v4l2_camera_node` setup. |
| `launch/sim.launch.py`                        | The single entry point for every simulated run, intended for a development machine and not the Jetson. Two orthogonal arguments: `backend` (`mujoco` \| `gazebo`) picks the simulator, `mode` (`base` \| `mapping` \| `perception` \| `truth_twin` \| `behaviour`) picks what runs on top. Replaces the five `launch_mecanumbot_sim*` / `launch_mecanumbot_truth_twin` files, which were layered includes of one another. Everything it starts lives in `mecanumbot_sim`. |

## Base launch arguments

| Argument           | Default        | Function                                                                                                     |
| ------------------ | -------------- | ------------------------------------------------------------------------------------------------------------ |
| `namespace`        | `mecanumbot`   | Robot namespace.                                                                                             |
| `use_sim_time`     | `false`        | Use the simulation clock.                                                                                    |
| `use_joy`          | `true`         | Start `joy_node` and the onboard joystick node.                                                              |
| `use_web`          | `true`         | Start the robot-hosted web GUI on port 8080.                                                                 |
| `joystick_profile` | `auto`         | Joystick profile stem, or `auto` to detect from the pad.                                                     |
| `yolo_imgsz`       | `1280`         | Input size the DeepStream pose model expects. Selects `mecanumbot_sensorprocess_smart` `models/imgsz_<n>/`, so it has to be a size the model was exported at — 640 and 1280 are shipped. |
| `yolo_model`       | `yolo26m-pose` | Pose model stem inside that folder.                                                                          |

`yolo_imgsz` is the whole of the model-size choice: the detector rewrites the model,
engine and `infer-dims` lines of the nvinfer config to match it, so nothing has to be
edited by hand to switch sizes. See `mecanumbot_sensorprocess_smart/README.md`.

## Additional notes

- This package is a coordinator: node logic lives in packages such as `mecanumbot_core`, `mecanumbot_led`, and external dependencies.
- Launch argument defaults (namespace, sim time, map/params) are defined in the launch files listed above.
- SSID to map mapping used by the base and external launches: `MecanumNet` -> `AI_dept`, `MecanumetoNet` -> `LED_exp` (with keepout mask). Anything else falls back to `AI_dept`.
