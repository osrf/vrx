#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist
from robotx_gazebo.msg import UsvDrive

class Node():
    def __init__(self,linear_scaling,angular_scaling):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.pub = None
        self.driveMsg =None

    def callback(self,data):
        rospy.logdebug("RX: Twist "+rospy.get_caller_id())
        rospy.logdebug("\tlinear:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.linear.x,
                                            data.linear.y,
                                            data.linear.z))
        rospy.logdebug("\tangular:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.angular.x,
                                            data.angular.y,
                                            data.angular.z))
        # scaling factors
        linfac = self.linear_scaling
        angfac = self.angular_scaling
        self.driveMsg.left = data.linear.x
        self.driveMsg.right = data.angular.z

        rospy.logdebug("TX: Drive ")
        rospy.logdebug("\tleft:%f, right:%f"%(self.driveMsg.left,
                                             self.driveMsg.right))
        #print("left: %f, right: %f"%(self.driveMsg.left,self.driveMsg.right))
        self.pub.publish(self.driveMsg)


if __name__ == '__main__':

    rospy.init_node('twist2drive', anonymous=True)

    # ROS Parameters
    in_topic = rospy.get_param('~input_topic','cmd_vel')
    out_topic = rospy.get_param('~output_topic','cmd_drive')
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',0.2)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',0.05)

    rospy.loginfo("Subscribing to <%s>, Publishing to <%s>"%(in_topic,out_topic))
    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))

    node=Node(linear_scaling,angular_scaling)

    # Publisher
    node.pub = rospy.Publisher(out_topic,UsvDrive,queue_size=10)
    node.driveMsg = UsvDrive()

    # Subscriber
    rospy.Subscriber(in_topic,Twist,node.callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
