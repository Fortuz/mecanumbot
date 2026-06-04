#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/ubuntu/mecanumbot_ws/install/setup.bash

export ROS_DOMAIN_ID=19
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch mecanumbot_bringup launch_mecanumbot_base.launch.py
