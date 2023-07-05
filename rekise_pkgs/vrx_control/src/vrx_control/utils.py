#!/usr/bin/env python3
import math

     
def euler_from_quaternion(quaternion):
    # Convert a quaternion to euler angles
    x, y, z, w = quaternion

    roll  = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = math.asin(2 * (w * y - z * x))
    yaw   = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    # Convert euler angles to a quaternion
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]

def todeg(angle) : 
    return angle * 180 / math.pi