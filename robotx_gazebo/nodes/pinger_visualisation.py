#!/usr/bin/python

#Copyright 2018 Joanthan Wheare (jonathan.wheare@flinders.edu.au)
#
#Redistribution and use in source and binary forms, with or without modification, 
#are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this 
#list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, 
#this list of conditions and the following disclaimer in the documentation 
#and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors 
#may be used to endorse or promote products derived from this software without 
#specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#BSD 3-term license.  Source: https://opensource.org/licenses/BSD-3-Clause
#Used under creative-commons attribution license.
#

#Generate a position for the pinger.  Also publishes a marker as a ground truth.
#

import rospy
import random

import math

from usv_msgs.msg import RangeBearing
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class pinger_visualisation:
    def __init__(self):
        rospy.init_node("pinger_visualisation")
        #Startup a publisher of the marker messages
        self.marker_pub = rospy.Publisher("/pinger/marker/signal",Marker,queue_size=10,latch=True)        
        #Start subscriber
        self.pinger_sub = rospy.Subscriber("/pinger/range_bearing",RangeBearing,self.pinger_callback)
        
        #Spin until closed
        rospy.spin()


    #Callback to handle an incoming range bearing message
    def pinger_callback(self,msg):
        vis_msg=Marker()
        #Create a Marker message


        #The frame ID and timestamp should be identical to the received pinger message
        vis_msg.header.frame_id = msg.header.frame_id
        vis_msg.header.stamp = msg.header.stamp
        vis_msg.ns=""
        vis_msg.id=0
        
        #Visualisation will be an arrow
        vis_msg.type=Marker.ARROW
        vis_msg.action = Marker.ADD
        
        #Poaition will be specified by start and end points.
        vis_msg.pose.position.x = 0
        vis_msg.pose.position.y = 0
        vis_msg.pose.position.z = 0
        vis_msg.pose.orientation.x = 0.0
        vis_msg.pose.orientation.y = 0.0
        vis_msg.pose.orientation.z = 0.0
        vis_msg.pose.orientation.w = 1.0

        vis_msg.scale.x = 0.1
        vis_msg.scale.y = 0.5
        vis_msg.scale.z = 0.5

        #Make it blue
        vis_msg.color.r = 0.0
        vis_msg.color.g = 0.0
        vis_msg.color.b = 1.0
        vis_msg.color.a = 1.0
        
        #When using start and end points, we need to include two more messages
        #of type geometry_msgs/Point.  These are appended to the list of points.
        #Origin is a 0,0.  Since the visualisation is in the sensor frame, arrow should
        #start at the sensor
        start_point = Point()
        start_point.x=0
        start_point.y=0
        start_point.z=0            
        vis_msg.points.append(start_point)
        #Finish at the estimated pinger position.
        end_point = Point()
        end_point.x = msg.range*math.cos(msg.elevation)*math.cos(msg.bearing)
        end_point.y = msg.range*math.cos(msg.elevation)*math.sin(msg.bearing)            
        end_point.z = msg.range*math.sin(msg.elevation)                        
        vis_msg.points.append(end_point)
        #Publish the message.            
        self.marker_pub.publish(vis_msg)
        
if __name__ == '__main__':
  pinger = pinger_visualisation()
            
            

            
