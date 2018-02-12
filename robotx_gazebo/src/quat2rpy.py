#!/usr/bin/env python
'''
Node to convert from quaternions to rpy
'''

import rospy
import tf
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

class Node():
    def __init__(self,pose_index=None,mstates_index=None,
                 input_msg_type='Pose'):
        self.pubmsg = None
        self.pub = None
        self.pose_index = pose_index
        self.mstates_index = mstates_index
        self.input_msg_type = input_msg_type
        
        
    def callback(self,data):
        #rospy.loginfo("callback")
        if (not (pose_index==None)):
            data = data[pose_index]
        elif (not (mstates_index==None)):
            try:
                data = data.pose[mstates_index]
            except IndexError:
                rospy.logwarn("Index error with ModelStates index!")
                return
        elif ( (self.input_msg_type == 'Pose') or 
               (self.input_msg_type == 'Imu')):
            pass
        elif self.input_msg_type == 'Odometry':
            data = data.pose.pose
        else:
            rospy.logerr("Don't know what to do with message type %s"%
                         self.input_msg_type)
            sys.exit()
        q = (data.orientation.x,
             data.orientation.y,
             data.orientation.z,
             data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)
        self.pubmsg.x = euler[0]
        self.pubmsg.y = euler[1]
        self.pubmsg.z = euler[2]
        rospy.logdebug("publishing rpy: %.2f, %.2f, %.2f"
                      %(euler[0],euler[1],euler[2]))
        self.pub.publish(self.pubmsg)

if __name__ == '__main__':
    
    rospy.init_node('quat2rpy', anonymous=True)
    
    # ROS Parameters
    in_topic = 'in_topic'
    out_topic = 'out_topic'
    pose_index = rospy.get_param('~pose_index',None)
    mstates_index = rospy.get_param('~modelstates_index',None)
    inmsgtype = rospy.get_param('~input_msg_type','Pose')
    

    # Initiate node object
    node=Node(pose_index,mstates_index,input_msg_type=inmsgtype)
    node.pubmsg = Vector3()

    # Setup publisher
    node.pub = rospy.Publisher(out_topic,Vector3,queue_size=10)

    # Subscriber
    if (not(mstates_index == None)):
        inmsgtype = 'ModelStates[%d]'%mstates_index
        rospy.Subscriber(in_topic,ModelStates,node.callback)
    elif (not (pose_index == None)):
        inmsgtype = 'PoseArray[%d]'%pose_index
        # Setup subscriber
        rospy.Subscriber(in_topic,PoseArray,node.callback)
    else:
        if inmsgtype == 'Pose':
            # Setup subscriber
            rospy.Subscriber(in_topic,Pose,node.callback)
        elif inmsgtype == 'Imu':
            rospy.Subscriber(in_topic,Imu,node.callback)
        elif inmsgtype == 'Odometry':
            rospy.Subscriber(in_topic,Odometry,node.callback)
        else:
            rospy.logerror("I don't know how to deal with message type <%s>"%
                           inmsgtype)
            sys.exit()

    
    rospy.loginfo("Subscribing to %s, looking for %s messages."%
                  (in_topic,inmsgtype))

    rospy.loginfo("Publishing to %s, sending Vector3 messages"%
                  (out_topic))



    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
