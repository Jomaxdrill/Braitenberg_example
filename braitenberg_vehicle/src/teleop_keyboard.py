#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Author: Darby Lim
#Modified version for educational purposes

import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22 #m/s
BURGER_MAX_ANG_VEL = 2.84 #rad/s

#increments of velocities
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = "burger"

msg = """
Control Your TurtleBot3 braitenberg_vehicle!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22 m/s)
a/d : increase/decrease angular velocity (Burger : ~ 2.84 m/s)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    """
    This function reads a single character from the standard input.
    It uses different methods based on the operating system to achieve this.

    Parameters:
    settings (termios.struct_termios): The terminal settings to be restored after reading the character.
        This parameter is only used in non-Windows environments.

    Returns:
    str: The character read from the standard input.
    """
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


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear vel {0}\t angular vel {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output, entry, slop):
    """
    This function creates a simple profile for controlling the output value based on the entry value and a slop.
    The output value is adjusted based on the entry value and the slop to ensure smooth transitions.

    Parameters:
    output (float): The current output value.
    entry (float): The desired entry value.
    slop (float): The maximum allowed change in the output value per iteration.

    Returns:
    float: The updated output value based on the entry value and the slop.
    """
    if entry > output:
        output = min(entry, output + slop)
    elif entry < output:
        output = max(entry, output - slop)
    else:
        output = entry

    return output


def constrain(input_vel, low_bound, high_bound):
    """
    Constrains the input velocity to be within the specified lower and upper bounds.

    Parameters:
    input_vel (float): The input velocity to be constrained.
    low_bound (float): The lower bound for the input velocity.
    high_bound (float): The upper bound for the input velocity.

    Returns:
    float: The constrained input velocity. If the input velocity is less than the lower bound,
        it returns the lower bound. If the input velocity is greater than the upper bound,
        it returns the upper bound. Otherwise, it returns the input velocity as is.
    """
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)


def main():
    #set up keyboard input
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    #create the node to send commands
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    #begin initial conditions
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        #check if key was pressed and determine velocity and apply saturation if needed
        while(1):
            key = get_key(settings)
            print('key pressed is: %s' % key)
            if key == 'w':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status += 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                #press CTRL+C to end
                if (key == '\x03'):
                    break
            #show update to the user
            if status == 20:
                print(msg)
                status = 0

            twist = Twist()
            #set velocities according to control law
            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))
            #send velocities to publisher topic
            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        #when program ends reset velocities
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        #return terminal to orginal state
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
