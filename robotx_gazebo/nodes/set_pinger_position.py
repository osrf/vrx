#!/usr/bin/python
#Copyright 2018 Joanthan Wheare
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.


import rospy
import random

from usv_msgs.msg import RangeBearing
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

class pinger_position:
    def __init__(self):
        rospy.init_node("set_pinger_position")
        #Load the positions of the pingers.
        self.pinger_positions=list()
        i=1
        while rospy.has_param('~position_'+str(i)):
            self.pinger_positions.append(rospy.get_param('~position_'+str(i)))
            i=i+1
        #If there are no matching positions, initialise to the origin.
        if i==1:            
            self.pinger_positions.append([0,0,0])
        self.pinger_pub = rospy.Publisher("/pinger/set_pinger_position",Vector3,queue_size=10,latch=True)
        self.marker_pub = rospy.Publisher("/pinger/marker/ground_truth",Marker,queue_size=10,latch=True)
        
        while not rospy.is_shutdown():
            #Choose a position for the pinger.
            position = random.choice(self.pinger_positions)
            #Create a vector message.
            msg = Vector3()
            msg.x=position[0]
            msg.y=position[1]
            msg.z=position[2]
            self.pinger_pub.publish(msg)

            msg=Marker()
            msg.header.frame_id="/map"
            msg.header.stamp = rospy.get_rostime()
            msg.ns=""
            msg.id=0
            msg.type=Marker.SPHERE
            msg.action = Marker.ADD
            
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            msg.scale.x = 1.0
            msg.scale.y = 1.0
            msg.scale.z = 1.0

            msg.color.r = 1.0
            msg.color.g = 0.0
            msg.color.b = 0.0
            msg.color.a = 1.0
            
            self.marker_pub.publish(msg)

            rospy.sleep(10.)#change position every 10 seconds.
            
if __name__ == '__main__':
  pinger = pinger_position()
            
            

            
