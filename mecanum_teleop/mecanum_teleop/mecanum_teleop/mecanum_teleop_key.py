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

# Node is a reworked version of original teleop_twist_keyboard node

import os
import select
import sys
import rclpy

from grabber_msg_interface.msg import GrabberPosition
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_XLIN_VEL = 0.26
MAX_YLIN_VEL = 0.26
MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

INSTANT_XLIN_VEL = 0.18
INSTANT_YLIN_VEL = 0.18
INSTANT_ANG_VEL = 1.82

DEFAULT_GRABBER_POSITION = 512.0 # straight forward
MIN_GRABBER_POSITION = 400.0
MAX_GRABBER_POSITION = 624.0

msg = """
Moving around:
    q   w   e      u   i   o   p
    a   s   d
        x
    
[Incremental mode]:
    w/x : increase/decrease X linear velocity (max 0.26)
    a/d : increase/decrease Y linear velocity (max 0.26)
    q/e : increase/decrease angular velocity (max 1.82)

[Instant mode]:
    w/x : set X linear velocity to +/-0.18
    a/d : set Y linear velocity to +/-0.18
    q/e : set angular velocity to +/-1.20

u/i : open/close left grabber
p/o : open/close right grabber
space key, s : force stop
m : switch between modes

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


def print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_pos, right_grabber_pos):
    print('[{0} mode]:   X linear velocity {1:.2f}   |   Y linear velocity {2:.2f}   |   Z angular velocity {3:.2f}   |   Left grabber: {4:.2f}   |   Right grabber: {5:.2f}'.format(
        "Incremental" if is_incremental else "Instant",
        target_x_linear_velocity,
        target_y_linear_velocity,
        target_angular_velocity,
        left_grabber_pos,
        right_grabber_pos))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_x_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_XLIN_VEL, MAX_XLIN_VEL)


def check_y_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_YLIN_VEL, MAX_YLIN_VEL)
    

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def check_grabber_position(position):
    return constrain(position, MIN_GRABBER_POSITION, MAX_GRABBER_POSITION)

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub_cmd_vel = node.create_publisher(Twist, 'cmd_vel', qos)
    pub_grabber_pos = node.create_publisher(GrabberPosition, 'grabber_goal_position', qos)

    is_incremental = True
    status = 0
    target_x_linear_velocity = 0.0
    target_y_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_x_linear_velocity = 0.0
    control_y_linear_velocity = 0.0
    control_angular_velocity = 0.0

    left_grabber_position = DEFAULT_GRABBER_POSITION
    right_grabber_position = DEFAULT_GRABBER_POSITION


    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w':  # Go forward
                if is_incremental:
                    target_x_linear_velocity = check_x_linear_limit_velocity(target_x_linear_velocity + LIN_VEL_STEP_SIZE)
                else:
                    target_x_linear_velocity = INSTANT_XLIN_VEL
                status = status + 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'x':  # Go back
                if is_incremental:
                    target_x_linear_velocity = check_x_linear_limit_velocity(target_x_linear_velocity - LIN_VEL_STEP_SIZE)
                else:
                    target_x_linear_velocity = -INSTANT_XLIN_VEL
                status = status + 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'a':  # Go (slide) left
                if is_incremental:
                    target_y_linear_velocity = check_y_linear_limit_velocity(target_y_linear_velocity + LIN_VEL_STEP_SIZE)
                else:
                    target_y_linear_velocity = INSTANT_YLIN_VEL
                status = status + 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'd':  # Go (slide) right
                if is_incremental:
                    target_y_linear_velocity = check_y_linear_limit_velocity(target_y_linear_velocity - LIN_VEL_STEP_SIZE)
                else:
                    target_y_linear_velocity = -INSTANT_YLIN_VEL
                status = status + 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'q':  # Turn left
                if is_incremental:
                    target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                else:
                    target_angular_velocity = INSTANT_ANG_VEL
                status = status + 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'e':  # Turn right
                if is_incremental:
                    target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                else:
                    target_angular_velocity = -INSTANT_ANG_VEL
                status = status + 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'u':  # Open left grabber
                left_grabber_position = check_grabber_position(MIN_GRABBER_POSITION)
                status += 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'i':  # Close left grabber
                left_grabber_position = check_grabber_position(MAX_GRABBER_POSITION)
                status += 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'o':  # Close right grabber
                right_grabber_position = check_grabber_position(MIN_GRABBER_POSITION)
                status += 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'p':  # Open right grabber
                right_grabber_position = check_grabber_position(MAX_GRABBER_POSITION)
                status += 1
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == ' ' or key == 's':  # Stop
                target_x_linear_velocity = 0.0
                control_x_linear_velocity = 0.0
                target_y_linear_velocity = 0.0
                control_y_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(is_incremental, target_x_linear_velocity, target_y_linear_velocity, target_angular_velocity, left_grabber_position, right_grabber_position)
            elif key == 'm':  # Switch velocity control mode between incremental and instant
                is_incremental = not is_incremental
                print("Velocity control mode changed to {} mode.".format("incremental" if is_incremental else "instant"))
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                # Tutorial message will only be displayed once, on start
                # print(msg)
                status = 0

            twist = Twist()

            control_x_linear_velocity = make_simple_profile(
                control_x_linear_velocity,
                target_x_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))
            
            control_y_linear_velocity = make_simple_profile(
                control_y_linear_velocity,
                target_y_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))


            twist.linear.x = control_x_linear_velocity
            twist.linear.y = control_y_linear_velocity
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub_cmd_vel.publish(twist)

            grabber_msg = GrabberPosition()
            grabber_msg.l = left_grabber_position
            grabber_msg.r = right_grabber_position
            pub_grabber_pos.publish(grabber_msg)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub_cmd_vel.publish(twist)

        grabber_msg = GrabberPosition()
        grabber_msg.l = DEFAULT_GRABBER_POSITION
        grabber_msg.r = DEFAULT_GRABBER_POSITION
        pub_grabber_pos.publish(grabber_msg)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
