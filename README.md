# Mecanumbot Project - Onboard Jetson codes

The Mecanumbot projects goal is to build a Turtlebot3 friend with mecanum wheel drive and fit into the Turtlebot3 family.`<br>`
This repository contains the Mecanumbot related packages. The repository made by using the original Turtlebot3 packeges.

## Project Repositories

[mecanumbot_microcontrollers](https://github.com/Fortuz/mecanumbot_microcontrollers) - Contains the microcontroller codes for the project `<br>`
[mecanumbot_remote](https://github.com/Fortuz/mecanumbot_remote) - Contains ROS2 packages for used on an external computer connected to the robot `<br>`
[mecanumbot](https://github.com/Fortuz/mecanumbot) - [This repository] Contains ROS2 packages run on the NVIDIA Jetson Orin Nano on the robot `<br>`
[mecanumbot_server](https://github.com/Fortuz/mecanumbot_server) - Contains ROS2 packages that talk to the off-board compute server (`mecanumbot_deep3r`) `<br>`
[mecanumbot_python](https://github.com/fegyobeno/mecanumbot_python.git) - Contains the native python scripts for manipulating the motors. Can be found on the robot locally in the ~/Sandbox folder `<br>`

Original sources: `<br>`
[Turtlebot3 - humble version](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble) `<br>`
[Turtlebot3_msgs - humble version](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/humble) `<br>`

## Packages in this repository

Ten packages, all of them part of the onboard Jetson stack (`mecanumbot_monitor`
is equally usable on a development machine). Simulation is no longer one of them:
`mecanumbot_sim` moved to [`mecanumbot_remote`](https://github.com/Fortuz/mecanumbot_remote),
because nothing about it runs on the robot.

| Package | Build type | Executables | Purpose |
| --- | --- | --- | --- |
| `mecanumbot` | `ament_cmake` (meta) | — | Dependency aggregation only. No nodes, no launch files: installing it pulls in the runtime packages. |
| `mecanumbot_core` | Python | `mecanumbot_io_node`, `mecanumbot_sensorproc_node`, `mecanumbot_battery_alert` | The only code that talks to the OpenCR board. `mecanumbot_io_node` owns the serial link (CRC8-CCITT framing) and the mecanum kinematics, turning `cmd_vel` into four wheel commands; `mecanumbot_sensorproc_node` turns board telemetry into `odom`, `imu`, `joint_states`, battery state, `has_object` and the `odom → base_footprint` transform, plus `orin_battery_state` from the INA219 when running on a Jetson; `mecanumbot_battery_alert` raises a visible low-battery alarm. |
| `mecanumbot_bringup` | `ament_cmake` | — (launch only) | Launch orchestration. No node logic of its own — it starts nodes from the other packages and picks map / Nav2 params from the current Wi-Fi SSID. Holds `launch_mecanumbot_base.launch.py` (full onboard stack), `launch_external.launch.py` (operator PC), the camera, state-publisher, mapping, SLAM and RViz launches, and `sim.launch.py`, the single entry point for every simulated run — the only launch file here that starts nodes from outside this repository, since `mecanumbot_sim` lives in `mecanumbot_remote`. |
| `mecanumbot_description` | `ament_cmake` | — (assets only) | Robot assets, no nodes: URDF/xacro and meshes, maps, Nav2/SLAM/Cartographer parameter sets, RViz layouts, the joystick profile YAMLs, the udev rules for `/dev/opencr`, `/dev/ld08_lidar` and `/dev/arduino_nano`, and the systemd autostart unit. |
| `mecanumbot_joy` | Python | `mecanumbot_joy_node` | The robot's joystick node and the only `/joy` consumer. Applies a YAML mapping profile (`mecanumbot_description/config/joystick/`) to publish `/cmd_vel`, `/cmd_accessory_pos` and LED service calls. Layout decoding, profile validation and the action vocabulary live in rclpy-free modules, so they are unit-tested without a ROS graph. Replaces the old operator-PC teleop and the retired `mecanumbot_gui` controller stack. |
| `mecanumbot_web` | Python | `mecanumbot_web_node` | The robot-hosted web GUI (Flask inside an rclpy node) at `http://<robot>:8080`, started by the base launch and disabled with `use_web:=false`. Three pages: `/joystick` edits and reloads the joystick profile with a live `/joy` readout, `/diagnostics` shows LED state, commanded-vs-measured movement and measured publish rates against nominal, `/behaviour` edits the leading-behaviour constants YAML. No login, no database. |
| `mecanumbot_led` | Python | `mecanumbot_led_service` | `set_led_status` / `get_led_status` services, forwarded over serial to the Arduino Nano LED controller at `/dev/arduino_nano`. |
| `mecanumbot_camera_stream` | Python | `camera_image_publisher_node`, `compressed_camera_publisher_node`, `h264_camera_publisher_node` | Camera capture on the Jetson: raw, JPEG-compressed and H.264 publishers. Picks its GStreamer pipeline from the detected board (`nvarguscamerasrc` + NVENC on the Orin Nano). |
| `mecanumbot_cam_optim` | `ament_cmake` (C++) | `camera_stream_node` | A C++ capture path alongside the Python one, built on `rclcpp`, `cv_bridge` and `image_transport`. No README; package manifest is still a TODO stub. |
| `mecanumbot_audio` | Python | `input_handler` | Publishes microphone blocks as `mecanumbot_msgs/AudioData`, with device auto-detection and reopen-on-failure. No README. |
| `mecanumbot_monitor` | Python | `wheel_monitor`, `accessory_monitor`, `rosbag_stepper`, `metric_eval` | Test and evaluation utilities: `wheel_monitor` and `accessory_monitor` generate repeatable command patterns for drivetrain and servo testing; `rosbag_stepper` chunks bag playback; `metric_eval` logs ground truth against detections to CSV. |

Every package except `mecanumbot`, `mecanumbot_audio` and `mecanumbot_cam_optim`
has its own README with the full publisher / subscriber / parameter tables — read
the relevant one before changing a node interface.

### What is *not* in this repository

The onboard stack is only part of the robot. The rest lives in the sibling repos
listed above: `mecanumbot_msgs` (every custom `.msg`/`.srv`, and a build
dependency of most packages here), `mecanumbot_sensorprocess_smart` (LiDAR and
camera people detection and their fusion), `mecanumbot_behaviours` (the
`py_trees_ros` behaviour trees), `mecanumbot_remote` (operator-PC keyboard teleop
and LED GUI, plus `mecanumbot_sim`) and `mecanumbot_server` (`mecanumbot_deep3r`,
the robot's end of the off-board CUT3R link).

**Simulation is in `mecanumbot_remote`.** `sim.launch.py` stays here — it is
launch orchestration like every other launch file in `mecanumbot_bringup` — but
every node, model, world, scenario and evaluator it starts is in
`mecanumbot_remote/mecanumbot_sim`, and `mecanumbot_bringup/package.xml` carries
the cross-repo `exec_depend` on it. A workspace with only this repository cloned
builds, but `sim.launch.py` will not run.

Building and using a robot with additional motors with different protocols, using a mecanum wheel drive sysetem instead of a differential drive system requires some changes in the original architecture so this repository is intend to provide a full functionality similar to the original Turtlebot3 repositories.

As a main source of information, documentation, codes and more the original [Turtlebot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) can be found here. Lot of the setup and codes came from the original Turtlebot3 project but a huge chunk of the codebase have been modified to accomodate the new motors, setup, and work with the mecanum wheel drive system. Altough the setups steps are basicaly the same some the important steps can be read below.

The project is made with Ubuntu 22.04 and ROS2 Humble. `<br>`

## Onboard computer

The whole onboard stack runs on an **NVIDIA Jetson Orin Nano** (JetPack 6, Ubuntu 22.04, ROS2 Humble). Earlier revisions of the robot used a Raspberry Pi; the packages in this repository are now developed and tested on the Orin Nano.

What the platform change means in practice:

| Area                | On the Jetson Orin Nano                                                                                                                                      |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| OS / ROS            | JetPack 6 (Ubuntu 22.04) + ROS2 Humble, `aarch64`.                                                                                                           |
| CSI camera          | `mecanumbot_camera_stream` uses the `nvarguscamerasrc` GStreamer pipeline instead of the Pi `libcamera` path. See `mecanumbot_camera_stream/README.md`.       |
| H.264 encoding      | Hardware NVENC (`nvv4l2h264enc`) is auto-detected, instead of the Pi `omxh264enc`. See `mecanumbot_camera_stream/OPTIMIZATION_GUIDE.md`.                     |
| Onboard power rail  | `mecanumbot_sensorproc_node` reads an INA219 over I2C and publishes `orin_battery_state`. This publisher only exists when the device tree reports a Jetson. |
| GPIO                | The `wiringpi` debug toggle from the Pi build is disabled; `wiringpi` is not installed and is not a dependency.                                               |
| Serial devices      | OpenCR, LD08 lidar and the LED Arduino Nano are still USB serial and are addressed through the udev rules in `mecanumbot_description/udev/`.                  |

Nodes detect the board at runtime by reading `/proc/device-tree/model`, so the same source tree still starts up on a non-Jetson machine — the Jetson-only publishers are simply skipped.

### Jetson specific setup

```
$ sudo usermod -aG dialout,i2c $USER   # serial + INA219 access, re-login afterwards
$ sudo nvpmodel -q                     # list available power modes
$ sudo nvpmodel -m 0                   # highest power mode (mode numbers differ per Orin Nano variant)
$ sudo jetson_clocks                   # lock clocks to max for consistent latency
$ pip install adafruit-blinka adafruit-circuitpython-ina219
```

`adafruit-blinka` provides the `board` and `busio` modules that `mecanumbot_sensorproc_node` imports on the Jetson.

Check that the board is what the nodes expect:

```
$ cat /proc/device-tree/model     # -> NVIDIA Jetson Orin Nano ...
$ sudo i2cdetect -y -r 7          # INA219 should show up (bus number can differ)
```

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/mecanumbot.jpg" width="600" alt="Mecanumbot">
</p>

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/turtlebot_architecture.drawio.png" width="600" alt="Architecture">
</p>

## Install

Run this on the Jetson Orin Nano (the `ubuntu` user and the `192.168.1.240` address below are the ones flashed on the current robot — adjust if yours differ).

```
$ ssh ubuntu@192.168.1.240
$ mkdir -p ~/mecanumbot_ws/src && cd ~/mecanumbot_ws/src
$ git clone https://github.com/Fortuz/mecanumbot.git
$ cd ~/mecanumbot_ws/
$ echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 2
$ echo 'source ~/mecanumbot_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ echo 'export OPENCR_PORT=/dev/ttyACM0' >> ~/.bashrc
$ echo 'export OPENCR_MODEL=mecanumbot' >> ~/.bashrc
$ echo 'export ROS_DOMAIN_ID=19' >> ~/.bashrc
$ echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
$ echo 'export TURTLEBOT3_MODEL=mecanumbot' >> ~/.bashrc
$ echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
$ echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp " >> ~/.bashrc
$ echo "alias start_robot='ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py'" >> ~/.bashrc
$ echo "alias start_robot_with_led='ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py & ros2 run mecanumbot_led mecanumbot_led_service & wait'" >> ~/.bashrc
$ source ~/.bashrc

```

The Orin Nano has enough RAM to build with more than one worker, but a full workspace build can still exhaust memory. If `colcon` gets OOM-killed, drop back to `--parallel-workers 1` or add swap.

Also install the udev rules so `/dev/opencr`, `/dev/ld08_lidar` and `/dev/arduino_nano` exist:

```
$ sudo cp ~/mecanumbot_ws/src/mecanumbot/mecanumbot_description/udev/*.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Autostart on boot

The startup script and the systemd unit are kept in the repository under `mecanumbot_description/services/`, so they do not have to be written by hand:

```bash
$ chmod +x ~/mecanumbot_ws/src/mecanumbot/mecanumbot_description/services/autostart.sh
$ sudo cp ~/mecanumbot_ws/src/mecanumbot/mecanumbot_description/services/ros2.service /etc/systemd/system/ros2.service
```

Both files reference the `ubuntu` user and `/home/ubuntu/mecanumbot_ws`. If the Jetson was flashed with a different username, edit the `User=` line in `ros2.service` and the source paths in `autostart.sh` before installing.

Reload systemd and enable the service:

```bash
$ sudo systemctl daemon-reload
$ sudo systemctl enable ros2.service
```

Check the service logs:

```bash
$ journalctl -u ros2.service -f
```

## Bringup

```
$ ssh ubuntu@192.168.1.240
$ start_robot_with_led
```

## Connect new PC to robot

[Remote PC]

Install Ubuntu 22.04 OP system [Ubuntu 22.04](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

Install ROS2 Humble [ROS2 Humble install tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

Set static IP for computer on MecanumNet (192.168.1.XXX, XXX between 2 and 255)

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/MecanumNet_settings.png" width="600" alt="Architecture">
</p>

```
$ sudo apt update
$ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-tf-transformations ros-humble-py-trees ros-humble-py-trees-ros ros-humble--rmw-cyclonedds-cpp
$ pip install torch
$ mkdir mecanumbot_directories #makes stuff prettier to have all in one space
$ cd mecanumbot_directories
$ git clone https://github.com/VisualComputingInstitute/DR-SPAAM-Detector.git  # leg detector nn
$ cd DR-SPAAM-DEetector/dr_spaam
$ python setup.py install
$ cd ../../
$ mkdir mecanumbot_ws &c cd mecanumbot_ws
$ mkdir src && cd src
$ git clone https://github.com/Fortuz/mecanumbot
$ git clone https://github.com/Fortuz/mecanumbot_remote
$ git clone https://github.com/Fortuz/mecanumbot_msgs
$ git clone https://github.com/hubaycsenge/mecanumbot_behaviours
$ cd ..
$ colcon build --symlink-install
$ echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
$ echo '~/mecanumbot_directories/mecanumbot_ws/install/setup.bash' >> ~/.bashrc
$ echo 'export ROS_DOMAIN_ID=19' >> ~/.bashrc
$ source ~/.bashrc
```

[ Mecanumbot ]

```
$ sudo nano /etc/chrony/chrony.conf
$ server 192.168.1.XXX minpoll 0 maxpoll 2 iburst #XXX is your PC's unique IP
$ sudo systemctl restart chrony
```

## TESTS

### LED control

Send a message to get back the actual status of the leds

```
$ ros2 service call /get_led_status mecanumbot_msgs/srv/GetLedStatus "{}"
```

Send a message to set the status of the leds

```
$ ros2 service call /set_led_status mecanumbot_msgs/srv/SetLedStatus "{
  fl_mode: 1,
  fl_color: 3,
  fr_mode: 1,
  fr_color: 3,
  br_mode: 1,
  br_color: 3,
  bl_mode: 1,
  bl_color: 4
}"
```

PC

```
$ echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp " >> ~/.bashrc
$ cd ~/dev_ws
$ source_ros
$ source_ws
$ ros2 run mecanumbot_teleop mecanumbot_keyboard
$ ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py
```

Parameters: `<br>`
wheel diameter: 65 mm `<br>`
wheel radius: 32.5 mm `<br>`
wheel thickness: 30 mm `<br>`
wheel separation x: 129 mm `<br>`
wheel separation y: 300 mm `<br>`