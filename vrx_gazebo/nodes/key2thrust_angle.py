#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import math
import numpy
import select
import sys
import termios
import tty

instructions = """
Reading from the keyboard and Publishing Thrust Angles!
---------------------------
Change Thrust Angle clockwise: h
Change Thrust Angle counter-clockwise: ;

CTRL-C to quit
"""

moveBindings = {
        'h': -1,
        ';': 1,
    }


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def thrust_angle_status(angle):
    return "currently:\tthrust_angle: %s " % (angle)


if __name__ == "__main__":
    # Setup settings var for key reading
    settings = termios.tcgetattr(sys.stdin)

    # Setup ros publishers and node
    left_pub = rospy.Publisher('left_thrust_angle', Float32, queue_size=1)
    right_pub = rospy.Publisher('right_thrust_angle', Float32, queue_size=1)
    rospy.init_node('key2thrust_angle')

    # Initialize current angle and angle speed
    thrust_angle_speed = rospy.get_param("~thrust_angle_speed", 0.1)
    max_angle = rospy.get_param("~max_angle", math.pi / 2)
    curr_angle = 0

    try:
        print(instructions)
        print('Max Angle: {}'.format(max_angle))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                # Increment angle, but clip it between [-max_angle, max_angle]
                curr_angle += thrust_angle_speed * moveBindings[key]
                curr_angle = numpy.clip(curr_angle,
                                        -max_angle, max_angle).item()
            else:
                if (key == '\x03'):
                    break

            # Publish thrust angle
            angle_msg = Float32()
            angle_msg.data = curr_angle
            left_pub.publish(angle_msg)
            right_pub.publish(angle_msg)

    except Exception as e:
        print(e)

    finally:
        # Send 0 angle command at end
        angle_msg = Float32()
        angle_msg.data = 0
        left_pub.publish(angle_msg)
        right_pub.publish(angle_msg)

        # Set attributes
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
