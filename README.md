# Mecanumbot Project - Onboard PI codes

The Mecanumbot projects goal is to build a Turtlebot3 friend with mecanum wheel drive and fit into the Turtlebot3 family.`<br>`
This repository contains the Mecanumbot related packages. The repository made by using the original Turtlebot3 packeges.

## Project Repositories

[mecanumbot_microcontrollers](https://github.com/Fortuz/mecanumbot_microcontrollers) - Contains the microcontroller codes for the project `<br>`
[mecanumbot_remote](https://github.com/Fortuz/mecanumbot_remote) - Contains ROS2 packages for used on an external computer connected to the robot `<br>`
[mecanumbot](https://github.com/Fortuz/mecanumbot) - [This repository] Contains ROS2 packages run on the Raspberry Pi on the robot `<br>`
[mecanumbot_python](https://github.com/fegyobeno/mecanumbot_python.git) - Contains the native python scripts for manipulating the motors. Can be found on the robot locally in the ~/Sandbox folder `<br>`

Original sources: `<br>`
[Turtlebot3 - humble version](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble) `<br>`
[Turtlebot3_msgs - humble version](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/humble) `<br>`

Building and using a robot with additional motors with different protocols, using a mecanum wheel drive sysetem instead of a differential drive system requires some changes in the original architecture so this repository is intend to provide a full functionality similar to the original Turtlebot3 repositories.

As a main source of information, documentation, codes and more the original [Turtlebot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) can be found here. Lot of the setup and codes came from the original Turtlebot3 project but a huge chunk of the codebase have been modified to accomodate the new motors, setup, and work with the mecanum wheel drive system. Altough the setups steps are basicaly the same some the important steps can be read below.

The project is made with Ubuntu 22.04 and ROS2 Humble. `<br>`

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/mecanumbot.jpg" width="600" alt="Mecanumbot">
</p>

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/turtlebot_architecture.drawio.png" width="600" alt="Architecture">
</p>

## Install

```
$ ssh ubuntu@192.168.1.240
$ mkdir -p ~/mecanumbot_ws/src && cd ~/mecanumbot_ws/src
$ git clone https://github.com/Fortuz/mecanumbot.git
$ cd ~/mecanumbot_ws/
$ echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
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

alternative git repo: `<br>`
[Mecanumbot repo](https://github.com/deborggraever/ros2-mecanum-bot/blob/main/mecanumbot_control/src/mecanumbot_control_node.cpp)
