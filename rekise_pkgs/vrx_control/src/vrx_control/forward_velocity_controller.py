#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_msgs.msg import TFMessage
import math
from std_msgs.msg import Float64

# from vrx_ros.msg import PidDiagnose
# from vrx_ros.cfg import TwistDynamicConfig
# from dynamic_reconfigure.server import Server

#
import pypid

class FwdVelocityController(Node):
    def __init__(self):
        super().__init__('fwd_vel_controller')

        # Declare parameters 
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', '/odometry/filtered')
        self.declare_parameter('thrust_topic_l', '/wamv/thrusters/left/thrust')
        self.declare_parameter('thrust_topic_r', '/wamv/thrusters/right/thrust')
        
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        
        self.pid = pypid.pypid.Pid(self.kp, self.ki, self.kd)
        self.pid.set_setpoint(0.0)
        self.pid.set_inputisangle(False)
        self.pid.set_derivfeedback(True)  # D term in feedback look
        fc = 50;  # cutoff freq in hz
        wc = fc*(2.0*math.pi)  # cutoff freq. in rad/s
        self.pid.set_derivfilter(1,wc)

        self.publish_debug = True #None
        
        self.cmd_topic      = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.state_topic    = self.get_parameter('state_topic').get_parameter_value().string_value
        self.thrust_topic_l = self.get_parameter('thrust_topic_l').get_parameter_value().string_value
        self.thrust_topic_r = self.get_parameter('thrust_topic_r').get_parameter_value().string_value
        self.pid_diagnose_topic = 'twist_pid_debug'

        
        # Declare all publishers and subscribers
        self.cmd_subscription   = self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.state_subscription = self.create_subscription(Odometry, self.state_topic, self.state_callback, 10)
        
        self.thruster_l_publisher = self.create_publisher(Float64, self.thrust_topic_l, 10)
        self.thruster_r_publisher = self.create_publisher(Float64, self.thrust_topic_r, 10)
        # self.debug_publisher      = self.create_publisher(PidDiagnose, self.pid_diagnose_topic, 10)

        # Other variables
        self.target_vel  = Float64()
        self.current_vel = Float64()
        self.lasttime    = None
        
        # srv = Server(YawDynamicConfig, self.dynamic_callback)
        self.get_logger().info('fwd_vel_controller initialized')


    def cmd_callback(self, msg : Twist) : 
        self.target_vel = msg.linear.x
        self.get_logger().info('Recieved Command Velocity : "%s"' % self.target_vel)
        self.pid.set_setpoint(self.target_vel)

    def state_callback(self, msg : Odometry):
        self.current_vel = msg.twist.twist.linear.x
        # self.get_logger().info('Current Velocity : "%s"' % self.current_vel)

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

        thrustmsg = self.get_output_msg(dt)
        
        self.thruster_l_publisher.publish(thrustmsg)
        self.thruster_r_publisher.publish(thrustmsg)
        self.get_logger().info('\tPublishing: "%s" to thruster' % thrustmsg.data)


    def get_output_msg(self, dt) : 
        thrustmsg = Float64()

        vout = self.pid.execute(dt, self.current_vel)
        thrustmsg.data = self.saturate_thrust(float(vout[0]))

        return thrustmsg
    
    def saturate_thrust(self, thrust) : 
        if thrust > self.thruster_max:
            thrust = self.thruster_max

        elif thrust < -self.thruster_max:
            thrust = -self.thruster_max

        return thrust

def main(args=None):
    rclpy.init(args=args)
    fwd_vel_controller = FwdVelocityController()
    rclpy.spin(fwd_vel_controller)
    fwd_vel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()