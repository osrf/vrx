#!/bin/bash
#rosrun rqt_plot rqt_plot /twist_pid_debug/d:derivative:error:p &
# ros2 run rqt_plot rqt_plot /fwd_vel_pid_debug/p:i:d:pid &

# ros2 run rqt_plot rqt_plot /fwd_vel_pid_debug/error:setpoint &
ros2 run rqt_plot rqt_plot /fwd_vel_pid_debug/error:setpoint &
# ros2 run rqt_plot rqt_plot /odometry/filtered/twist/twist/linear/x /fwd_vel_pid_debug/error:setpoint&
# ros2 run rqt_plot rqt_plot /odometry/filtered/twist/twist/linear/x &