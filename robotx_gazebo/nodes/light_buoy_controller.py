#!/usr/bin/env python
import rospy
from std_msgs.msg import ColorRGBA
from threading import Lock
from std_srvs.srv import Trigger
import random


class LightBuoyController(object):
    '''
    Node to control the simulated light buoy for the RobotX challenge.
    Sends ColorRGBA messages to the gazebo_ros_color plugin on each of the light buoy's 3 panels
    '''
    RED        = ColorRGBA(1, 0, 0, 1)
    GREEN      = ColorRGBA(0, 1, 0, 1)
    BLUE       = ColorRGBA(0, 0, 1, 1)
    YELLOW     = ColorRGBA(1, 1, 0, 1)
    OFF        = ColorRGBA(0, 0, 0, 1)
    COLORS     = [RED, GREEN, BLUE, YELLOW]
    COLORS_STR = ['Red', 'Green', 'Blue', 'Yellow']

    def __init__(self):
        # Create publisher for each panel
        self.lock = Lock()
        self.publishers = [rospy.Publisher('light_buoy/panel{}'.format(i + 1), ColorRGBA, queue_size=1)
                           for i in range(3)]
        # Initialize pattern to a random sequence
        self.new_pattern()
        # Go to next color in pattern every second (as per robotx challenge description)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.update)
        # Provide service to reset pattern
        self.new_code_service = rospy.Service('~new_pattern', Trigger, self.new_pattern)

    def new_pattern(self, *args):
        '''
        Set the 3 color pattern to a new random sequence, such that there
        are no consecutive repeats.
        '''
        with self.lock:
            # Flash off before starting new pattern
            self.state = 3
            # generate random sequence
            pattern_indicies = [random.randint(0, 2) for _ in range(3)]
            # Unsure there are no consecutive repeats (as per rules)
            while pattern_indicies[0] == pattern_indicies[1] or pattern_indicies[1] == pattern_indicies[2]:
                pattern_indicies[1] = random.randint(0, 2)
            # Print the sequence as a string
            pattern_str = 'New pattern: ' + ' '.join([self.COLORS_STR[i] for i in pattern_indicies])
            rospy.loginfo(pattern_str)
            self.pattern = [self.COLORS[i] for i in pattern_indicies] + [self.OFF]
            # Return new pattern (can be used to judge correctness)
            return {'success': True, 'message': pattern_str}

    def update(self, timer_event):
        with self.lock:
            # Reset incrementer if pattern needs to restart
            if self.state >= len(self.pattern):
                self.state = 0
            # Publish current pattern color to each panel
            msg = self.pattern[self.state]
            for pub in self.publishers:
                pub.publish(msg)
            # increment state for next iteration
            self.state += 1

if __name__ == '__main__':
    random.seed()
    rospy.init_node('light_buoy_controller')
    LightBuoyController()
    rospy.spin()
