# mecanumbot_bringup

Launch orchestration package for booting the robot stack.

This package does not contain ROS nodes implemented in source code. It provides launch files that start/configure nodes from other packages.

The onboard launch files run on the robot's NVIDIA Jetson Orin Nano. `launch_mecanumbot_base.launch.py` starts the onboard stack — drivers, Nav2, the joystick and the web GUI — on the Jetson itself; the remote PC is only needed for visualization and teleoperation.

**It no longer starts perception.** It used to, so every run of the robot — a teleop session, a mapping run, T1 exploration — carried a DR-SPAAM detector, a DeepStream network and the fusion node whether or not anything subscribed to them. On an Orin Nano that is not free: the network is most of the GPU, and it holds the camera open so nothing else can have it, which is why there was no `/camera/image_raw/compressed` to record a trial from. Perception is now started by the behaviour that needs it, from `mecanumbot_sensorprocess_smart/launch/perception.launch.py`, with the detector that behaviour needs. See that package's README.

## Launch file functions

| File                                          | Function                                                                                                                         |
| --------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `launch/launch_mecanumbot_base.launch.py`     | Full onboard runtime. Starts `mecanumbot_io_node`, `mecanumbot_battery_alert`, `mecanumbot_sensorproc_node`, `ld08_driver`, `mecanumbot_led_service`, the audio input handler, the robot state publisher include, and a `nav2_bringup` include. **No perception** — that is started by the behaviour that needs it. Map, Nav2 parameter set and keepout mask are selected from the connected Wi-Fi SSID. |
| `launch/launch_external.launch.py`            | Operator-PC tools: RViz (`use_rviz`, default true). It starts **no perception** unless asked (`use_people_detection:=true`, for running DR-SPAAM off the robot) — the behaviour that needs people detection starts it itself, and a second `mecanumbot_lidar_detect_people` on the domain publishes over the robot's. Nav2 and the SSID map selection are commented out at the bottom of the file. |
| `launch/mapping.launch.py`                    | Starts `slam_toolbox` (`async_slam_toolbox_node`) using mapping parameter file.                                                  |
| `launch/slam_gmapping.launch.py`              | Starts `cartographer_node` and `cartographer_occupancy_grid_node` using `mecanumbot_description/param/mecanumbot_lds.lua`.       |
| `launch/mecanumbot_state_publisher.launch.py` | Starts `robot_state_publisher` with robot description from URDF/xacro.                                                           |
| `launch/rviz2.launch.py`                      | Starts RViz with the package-provided visualization config.                                                                      |
| `launch/camera.launch.py`                     | Includes `mecanumbot_camera_stream/camera_compressed.launch.py` (JPEG compressed stream). Replaces the earlier raw `v4l2_camera_node` setup. Not included by the base launch: `perception.launch.py` starts the same publisher, with `use_camera:=true`, because whoever owns the camera has to be the same process that the detector is configured against. |
| `launch/sim.launch.py`                        | The single entry point for every simulated run, intended for a development machine and not the Jetson. Two orthogonal arguments: `backend` (`mujoco` \| `gazebo`) picks the simulator, `mode` (`base` \| `mapping` \| `perception` \| `truth_twin` \| `behaviour`) picks what runs on top. Replaces the five `launch_mecanumbot_sim*` / `launch_mecanumbot_truth_twin` files, which were layered includes of one another. Everything it starts lives in `mecanumbot_sim`. |

## Base launch arguments

| Argument           | Default        | Function                                                                                                     |
| ------------------ | -------------- | ------------------------------------------------------------------------------------------------------------ |
| `namespace`        | `mecanumbot`   | Robot namespace.                                                                                             |
| `use_sim_time`     | `false`        | Use the simulation clock.                                                                                    |
| `use_joy`          | `true`         | Start `joy_node` and the onboard joystick node.                                                              |
| `use_web`          | `true`         | Start the robot-hosted web GUI on port 8080.                                                                 |
| `joystick_profile` | `auto`         | Joystick profile stem, or `auto` to detect from the pad.                                                     |

The detector arguments (`yolo_imgsz`, `yolo_model`, `use_pose_detector`,
`use_fetch_detector`, `fetch_model`, `fetch_imgsz`) are **gone from here** and live on
`mecanumbot_sensorprocess_smart`'s `perception.launch.py`, which the behaviour launch
files include. The behaviour launchers forward them, so:

```bash
# perception on its own, no tree
ros2 launch mecanumbot_sensorprocess_smart perception.launch.py detector:=fetch

# or through the behaviour that wants it
ros2 launch mecanumbot_leading_behaviour launch_wifi_condition_sequence.launch.py \
    condition:=Doglike yolo_imgsz:=640
```

## Additional notes

- This package is a coordinator: node logic lives in packages such as `mecanumbot_core`, `mecanumbot_led`, and external dependencies.
- The perception pipeline deliberately does **not** live here, even though launch orchestration otherwise does: this package depends on `mecanumbot_web`, which depends on `mecanumbot_leading_behaviour`, so a behaviour package that depended on `mecanumbot_bringup` would close a dependency cycle. It lives in `mecanumbot_sensorprocess_smart`, which owns the nodes anyway.
- Launch argument defaults (namespace, sim time, map/params) are defined in the launch files listed above.
- SSID to map mapping used by the base and external launches: `MecanumNet` -> `AI_dept`, `MecanumetoNet` -> `LED_exp` (with keepout mask). Anything else falls back to `AI_dept`.
