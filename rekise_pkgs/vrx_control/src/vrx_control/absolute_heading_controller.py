#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_msgs.msg import TFMessage
import math
from std_msgs.msg import Float64
from utils import *

# from vrx_ros.msg import PidDiagnose
# from vrx_ros.cfg import TwistDynamicConfig

#
import pypid

class AbsoluteHeadingController(Node):
    def __init__(self):
        super().__init__('abs_heading_controller')

        # Declare parameters 
        self.declare_parameter('cmd_topic', '/cmd_heading')
        self.declare_parameter('state_topic', '/odometry/filtered')
        self.declare_parameter('thrust_topic_l', '/wamv/thrusters/left/pos')
        self.declare_parameter('thrust_topic_r', '/wamv/thrusters/right/pos')
        
        self.declare_parameter('kp', -0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        self.rudder_max = 1.57 # 90 degrees

        self.pid = pypid.pypid.Pid(self.kp, self.ki, self.kd)

        # pid config for fwd velocity control
        self.pid.set_setpoint(0.0)
        self.pid.set_inputisangle(True)
        self.pid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*math.pi)  # cutoff freq. in rad/s
        self.pid.set_derivfilter(1,wc)

        self.publish_debug = True #None
        
        self.cmd_topic      = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.state_topic    = self.get_parameter('state_topic').get_parameter_value().string_value
        self.thrust_topic_l = self.get_parameter('thrust_topic_l').get_parameter_value().string_value
        self.thrust_topic_r = self.get_parameter('thrust_topic_r').get_parameter_value().string_value
        self.pid_diagnose_topic = 'yaw_pid_debug'

        
        # Declare all publishers and subscribers
        self.cmd_subscription   = self.create_subscription(Float64, self.cmd_topic, self.cmd_callback, 10)
        self.state_subscription = self.create_subscription(Odometry, self.state_topic, self.state_callback, 10)
        
        self.thruster_l_publisher = self.create_publisher(Float64, self.thrust_topic_l, 10)
        self.thruster_r_publisher = self.create_publisher(Float64, self.thrust_topic_r, 10)
        # self.debug_publisher      = self.create_publisher(PidDiagnose, self.pid_diagnose_topic, 10)

        # Other variables
        self.target_yaw    = Float64()

        self.current_yaw   = Float64()
        self.lasttime    = None
        
        self.get_logger().info('abs_heading_controller initialized')


    def cmd_callback(self, msg : Float64) : 
        self.target_yaw = msg.data
        self.get_logger().info('Recieved Command angle : "%s"' % self.target_yaw)

        self.pid.set_setpoint(self.target_yaw)


    def state_callback(self, msg : Odometry):
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        
        _, _, self.current_yaw = euler_from_quaternion(q)

        # self.get_logger().info('Current yaw : "%s"' % self.current_yaw)

        self.send_output()

    def send_output(self) :
        now = self.get_clock().now()
        # First time
        if self.lasttime is None : 
            self.lasttime = now
            return
        
        dt = now - self.lasttime
        dt = dt.nanoseconds // (10 ** 3)
        self.lasttime = self.get_clock().now()
        
        ruddermsg = self.get_output_msg(dt)

        self.thruster_l_publisher.publish(ruddermsg)
        self.thruster_r_publisher.publish(ruddermsg)
        self.get_logger().info('\tPublishing: "%s" to rudder' % ruddermsg.data)

        # if self.publish_debug : 
        #     debugmsg            = None
            
        #     debugmsg.pid        = vout[0]
        #     debugmsg.p          = vout[1]
        #     debugmsg.i          = vout[2]
        #     debugmsg.d          = vout[3]
        #     debugmsg.error      = vout[4]
        #     debugmsg.setpoint   = vout[5]
        #     debugmsg.derivative = vout[6]
        #     debugmsg.integral   = vout[7]
            # self.debug_publisher.publish(debugmsg)

    def get_output_msg(self, dt) : 
        ruddermsg      = Float64()
        yout           = self.pid.execute(dt, self.current_yaw)
        ruddermsg.data = self.saturate_rudder(float(yout[0]))

        return ruddermsg
    

    def saturate_rudder(self, rudder) : 
        if rudder > self.rudder_max:
            rudder = self.rudder_max

        elif rudder < -self.rudder_max:
            rudder = -self.rudder_max

        return rudder
    

def main(args=None):
    rclpy.init(args=args)
    abs_heading_controller = AbsoluteHeadingController()
    rclpy.spin(abs_heading_controller)
    abs_heading_controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
