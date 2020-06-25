#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


def callback(data):

    #rospy.loginfo(data.axes[1])  # thoruttle 
    #rospy.loginfo(data.axes[2]) # roll
    #rospy.loginfo(data.axes[3]) # pitch 
    #rospy.loginfo(data.axes[0]) # yaw 
    rospy.loginfo(data.buttons[4])




def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/joy', Joy, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
