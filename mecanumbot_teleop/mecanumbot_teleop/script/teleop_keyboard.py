#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import sys

from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import TwistStamped, Vector3Stamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MECANUMBOT_MAX_LIN_VEL = 0.26
MECANUMBOT_MAX_ANG_VEL = 1.82
MECANUMBOT_MIN_CAM_POS = 2.0
MECANUMBOT_MAX_CAM_POS = 8.6
MECANUMBOT_MIN_GRIPPER_POS = 1.6
MECANUMBOT_FRONT_GRIPPER_POS = 5.12
MECANUMBOT_MAX_GRIPPER_POS = 8.54

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

msg = """
Control Your TurtleBot3 frined - Mecanumbot !
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

        
w/x : increase/decrease linear x velocity (Mecanumbot : ~ 0.26)
q/e : increase/decrease angular velocity (Mecanumbot : ~ 1.82)
a/d : increase/decrease linear y velocity (Mecanumbot : ~ 0.26)

Moving accessories:
        i    
   j    k    l
        
i/k : cam up/down
j/l : open/close gripper 

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity):
    print('currently:\tlinear x velocity {0}\t linear y velocity {1}\t angular velocity {2} '.format(
        target_linear_x_velocity,
        target_linear_y_velocity,
        target_angular_velocity))


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'mecanumbot':
        return constrain(velocity, -MECANUMBOT_MAX_LIN_VEL, MECANUMBOT_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'mecanumbot':
        return constrain(velocity, -MECANUMBOT_MAX_ANG_VEL, MECANUMBOT_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)

def create_twistcmd(linear_x, linear_y, angular_z, ROS_DISTRO):
    if ROS_DISTRO == 'humble':
        twist = Twist()
    else:
        twist = TwistStamped()
        twist.header.stamp = Clock().now().to_msg()
        twist.header.frame_id = ''
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_z

    return twist

def create_rotcmd(x, y, z, ROS_DISTRO):
    if ROS_DISTRO == 'humble':
        rot = Vector3()
    else:
        rot = Vector3Stamped()
        rot.header.stamp = Clock().now().to_msg()
        rot.header.frame_id = ''
    rot.x = x
    rot.y = y
    rot.z = z

    return rot

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    ROS_DISTRO = os.environ.get('ROS_DISTRO')
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('mecanumbot_keyboard')
    if ROS_DISTRO == 'humble':
        pub_vel = node.create_publisher(Twist, 'cmd_vel', qos)
        pub_cam = node.create_publisher(Vector3, 'cmd_cam_ori', qos)
        pub_gripper_left = node.create_publisher(Vector3, 'cmd_grabberleft_ori', qos)
        pub_gripper_right = node.create_publisher(Vector3, 'cmd_grabberright_ori', qos)
    else:
        pub_vel = node.create_publisher(TwistStamped, 'cmd_vel', qos)
        pub_cam = node.create_publisher(Vector3Stamped, 'cmd_cam_ori', qos)
        pub_gripper_left = node.create_publisher(Vector3Stamped, 'cmd_grabberleft_ori', qos)
        pub_gripper_right = node.create_publisher(Vector3Stamped, 'cmd_grabberright_ori', qos)

    status = 0
    target_linear_x_velocity  = 0.0
    target_linear_y_velocity  = 0.0
    target_angular_velocity   = 0.0
    control_linear_x_velocity = 0.0
    control_linear_y_velocity = 0.0
    control_angular_velocity  = 0.0

    control_cam_ori = [0.0,MECANUMBOT_MAX_CAM_POS,0.0] # x,y,z, moves around y axis of the robot 
    control_gripper_left_ori = [0.0,0.0,6.55] # x,y,z, moves around z axis of the robot
    control_gripper_right_ori = [0.0,0.0,3.6] # x,y,z, moves around y axis of the robot

    try:
        print(msg)
        while (1):
            key = get_key(settings)
            if key == 'w':
                target_linear_x_velocity =\
                    check_linear_limit_velocity(target_linear_x_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_x_velocity =\
                    check_linear_limit_velocity(target_linear_x_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'a':
                target_linear_y_velocity =\
                    check_linear_limit_velocity(target_linear_y_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'd':
                target_linear_y_velocity =\
                    check_linear_limit_velocity(target_linear_y_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'q':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'e':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_x_velocity = 0.0
                target_linear_y_velocity = 0.0
                control_linear_x_velocity = 0.0
                control_linear_y_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'i': #cam_up
                control_cam_ori[1] = MECANUMBOT_MAX_CAM_POS 
            elif key == 'k': #cam_down
                control_cam_ori[1] = (MECANUMBOT_MIN_CAM_POS + MECANUMBOT_MAX_CAM_POS)/2
            elif key == 'j': #open gripper,rot around robot axis z
                control_gripper_left_ori[2] = 3.6
                control_gripper_right_ori[2] = 6.55
            elif key == 'l': #close gripper, rot around robot axis z
                control_gripper_left_ori[2] = 6.55
                control_gripper_right_ori[2] = 3.6
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            control_linear_x_velocity = make_simple_profile(
                control_linear_x_velocity,
                target_linear_x_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))
            
            control_linear_y_velocity = make_simple_profile(
                control_linear_y_velocity,
                target_linear_y_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))
            twist = create_twistcmd(control_linear_x_velocity, control_linear_y_velocity, control_angular_velocity, ROS_DISTRO)
            pub_vel.publish(twist)
            gripper_left_ori =create_rotcmd(control_gripper_left_ori[0], control_gripper_left_ori[1], control_gripper_left_ori[2], ROS_DISTRO)
            pub_gripper_left.publish(gripper_left_ori)
            gripper_right_ori =create_rotcmd(control_gripper_right_ori[0], control_gripper_right_ori[1], control_gripper_right_ori[2], ROS_DISTRO)
            pub_gripper_right.publish(gripper_right_ori)
            cam_ori =create_rotcmd(control_cam_ori[0], control_cam_ori[1], control_cam_ori[2], ROS_DISTRO)
            pub_cam.publish(cam_ori)

    except Exception as e:
        print(e)

    finally:
        if ROS_DISTRO == 'humble':
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub_vel.publish(twist)
        else:
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = Clock().now().to_msg()
            twist_stamped.header.frame_id = ''
            twist_stamped.twist.linear.x = control_linear_x_velocity
            twist_stamped.twist.linear.y = control_linear_x_velocity
            twist_stamped.twist.linear.z = 0.0
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = control_angular_velocity
            pub_vel.publish(twist_stamped)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
