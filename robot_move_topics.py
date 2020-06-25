#!/usr/bin/env python


import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import numpy as np


def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def callback(data):

    roll,pitch,yaw = quaternion_to_euler(data.orientation.w,data.orientation.x,data.orientation.y,data.orientation.z)
    
    rospy.loginfo(int(pitch))
    rospy.loginfo(int(yaw))
    rospy.loginfo(int(roll))
    rospy.loginfo("----------")

def listener():
    
    rospy.init_node('listener', anonymous=True)    
    #rospy.Subscriber('/iris/motor_speed/0', Float32, callback)
    rospy.Subscriber('/iris/imu', Imu, callback)
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
