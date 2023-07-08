#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_msgs.msg import TFMessage
import math
from std_msgs.msg import Float64
from vrx_common_interfaces.msg import PidDiagnose

# from vrx_ros.msg import PidDiagnose
# from vrx_ros.cfg import TwistDynamicConfig
# from dynamic_reconfigure.server import Server

#
import pypid

class YawRateController(Node):
    def __init__(self):
        super().__init__('yaw_rate_controller')

        # Declare parameters 
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('state_topic', '/odometry/filtered')
        self.declare_parameter('thrust_topic_l', '/wamv/thrusters/left/pos')
        self.declare_parameter('thrust_topic_r', '/wamv/thrusters/right/pos')
        self.declare_parameter('pid_diagnose_topic', '/yaw_rate_pid_debug')
        
        self.declare_parameter('kp', 100.0)
        self.declare_parameter('ki', 0.00001)
        self.declare_parameter('kd', 0.0)

        # Fetch parameter values
        self.cmd_topic          = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.state_topic        = self.get_parameter('state_topic').get_parameter_value().string_value
        self.thrust_topic_l     = self.get_parameter('thrust_topic_l').get_parameter_value().string_value
        self.thrust_topic_r     = self.get_parameter('thrust_topic_r').get_parameter_value().string_value
        self.pid_diagnose_topic = self.get_parameter('pid_diagnose_topic').get_parameter_value().string_value

        self.kp                 = self.get_parameter('kp').get_parameter_value().double_value
        self.ki                 = self.get_parameter('ki').get_parameter_value().double_value
        self.kd                 = self.get_parameter('kd').get_parameter_value().double_value



        # -- Set up pid
        self.pid = pypid.pypid.Pid(self.kp, self.ki, self.kd)
        self.pid.set_setpoint(0.0)
        self.pid.set_inputisangle(False)
        self.pid.set_derivfeedback(True)  # D term in feedback look
        fc = 50;  # cutoff freq in hz
        wc = fc*(2.0*math.pi)  # cutoff freq. in rad/s
        self.pid.set_derivfilter(1,wc)

        
        # Subscribers
        self.cmd_subscription   = self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.state_subscription = self.create_subscription(Odometry, self.state_topic, self.state_callback, 10)
        
        # Publishers
        self.thruster_l_publisher = self.create_publisher(Float64, self.thrust_topic_l, 10)
        self.thruster_r_publisher = self.create_publisher(Float64, self.thrust_topic_r, 10)
        self.debug_publisher      = self.create_publisher(PidDiagnose, self.pid_diagnose_topic, 10)

        # Other variables
        self.target_yaw_rate    = Float64()
        self.current_yaw_rate   = Float64()
        self.lasttime           = None
        self.rudder_max         = 263.0
        self.publish_debug      = True #None

        
        self.get_logger().info('yaw_rate_controller initialized')


    def cmd_callback(self, msg : Twist) : 
        self.target_yaw_rate = msg.linear.x
        # self.get_logger().info('Recieved Command Velocity : "%s"' % self.target_yaw_rate)
        self.pid.set_setpoint(self.target_yaw_rate)

    def state_callback(self, msg : Odometry):
        self.current_yaw_rate = msg.twist.twist.angular.z
        # self.get_logger().info('Current Velocity : "%s"' % self.current_yaw_rate)

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

        ruddermsg, pidout = self.get_output_msg(dt)
        
        self.thruster_l_publisher.publish(ruddermsg)
        self.thruster_r_publisher.publish(ruddermsg)
        self.get_logger().info('\tPublishing: "%s" to rudder' % ruddermsg.data)

        if self.publish_debug : 
            debugmsg            = PidDiagnose()
            
            debugmsg.pid        = pidout[0]
            debugmsg.p          = pidout[1]
            debugmsg.i          = pidout[2]
            debugmsg.d          = pidout[3]
            debugmsg.error      = pidout[4]
            debugmsg.setpoint   = pidout[5]
            debugmsg.derivative = pidout[6]
            debugmsg.integral   = pidout[7]
            self.debug_publisher.publish(debugmsg)


    def get_output_msg(self, dt) : 
        ruddermsg = Float64()

        pidout = self.pid.execute(dt, self.current_yaw_rate)
        ruddermsg.data = self.saturate_rudder(float(pidout[0]))

        return ruddermsg, pidout
    

    def saturate_rudder(self, thrust) : 
        if thrust > self.rudder_max:
            thrust = self.rudder_max

        elif thrust < -self.rudder_max:
            thrust = -self.rudder_max

        return thrust
    


def main(args=None):
    rclpy.init(args=args)
    yaw_rate_controller = YawRateController()
    rclpy.spin(yaw_rate_controller)
    yaw_rate_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()