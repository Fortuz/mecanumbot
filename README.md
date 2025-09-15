# Mecanumbot packages

The Mecanumbot projects goal is to build a Turtlebot3 friend with mecanum wheel drive and fit into the Turtlebot3 family.<br>
This repository contains the Mecanumbot related packages. The repository made by using the original Turtlebot3 packeges.

Original sources: <br>
[Turtlebot3 - humble version](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble) <br>
[Turtlebot3_msgs - humble version](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/humble) <br>

Building and using a robot with additional motors with different protocols, using a mecanum wheel drive sysetem instead of a differential drive system requires some changes in the original architecture so this repository is intend to provide a full functionality similar to the original Turtlebot3 repositories.

As a main source of information, documentation, codes and more the original [Turtlebot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) can be found here. Lot of the setup and codes came from the original Turtlebot3 project but a huge chunk of the codebase have been modified to accomodate the new motors, setup, and work with the mecanum wheel drive system. Altough the setups steps are basicaly the same some the important steps can be read below.

The project is made with Ubuntu 22.04 and ROS2 Humble.

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/mecanumbot.jpg" width="600" alt="Mecanumbot">
</p>

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/turtlebot_architecture.drawio.png" width="600" alt="Architecture">
</p>

## Project Repositories

[mecanumbot_microcontrollers](https://github.com/Fortuz/mecanumbot_microcontrollers) - Contains the microcontroller codes for the project <br>  
[mecanumbot_remote](https://github.com/Fortuz/mecanumbot_remote) - Contains ROS2 packages for used on an external computer connected to the robot <br>
[mecanumbot](https://github.com/Fortuz/mecanumbot) - [This repository] Contains ROS2 packages run on the Raspberry Pi on the robot <br>
[mecanumbot_python](https://github.com/fegyobeno/mecanumbot_python.git) - Contains the native python scripts for manipulating the motors. Can be found on the robot locally in the ~/Sandbox folder <br>

## SSH

```
$ ssh ubuntu@192.168.1.240
```

## TESTS

### LED control

[Terminal 1] - Start the ros node controlling the leds
```
$ ros2 run mecanumbot_led mecanumbot_led_service
```

[Terminal 2] - Send a message to get back the actual status of the leds
```
$ ros2 service call /get_led_status mecanumbot_msgs/srv/GetLedStatus "{}"
```
OR

[Terminal 2] - Send a message to set the status of the leds
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

SBC
```
ros2 launch mecanumbot_bringup robot.launch.py

```

PC
```
ros2 run mecanumbot_teleop mecanumbot_keyboard
ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py
```

Parameters: <br>
wheel diameter: 65 mm <br>
wheel radius: 32.5 mm <br> 
wheel thikness: 30 mm <br>
wheel separation x: 129 mm <br>
wheel separation y: 300 mm <br>


alternative git repo: <br>
[Mecanumbot repo](https://github.com/deborggraever/ros2-mecanum-bot/blob/main/mecanumbot_control/src/mecanumbot_control_node.cpp)
