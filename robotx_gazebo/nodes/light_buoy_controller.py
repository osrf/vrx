#!/usr/bin/env python
import rospy
from std_msgs.msg import ColorRGBA

class LightBuoyController(object):
    '''
    Node to control the simulated light buoy for the RobotX challenge.
    Sends ColorRGBA messages to the gazebo_ros_color plugin on each of the light buoy's 3 panels
    @todo Allow color pattern to be selected from ROS params or be randomly generated
    '''
    RED    = ColorRGBA(1, 0, 0, 1)
    GREEN  = ColorRGBA(0, 1, 0, 1)
    BLUE   = ColorRGBA(0, 0, 1, 1)
    YELLOW = ColorRGBA(1, 1, 0, 1)
    OFF    = ColorRGBA(0, 0, 0, 1)
    def __init__(self):
        # Create publisher for each panel
        self.publishers = [rospy.Publisher('light_buoy/panel{}'.format(i + 1), ColorRGBA, queue_size=1)
                           for i in range(3)]
        # Hard coded Y G B pattern
        self.pattern = [self.YELLOW, self.GREEN, self.BLUE, self.OFF]
        # Stores what index in pattern node is currently in
        self.state = 0
        # Go to next color in pattern every second (as per robotx challenge description)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.update)

    def update(self, timer_event):
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
    rospy.init_node('light_buoy_controller')
    LightBuoyController()
    rospy.spin()
