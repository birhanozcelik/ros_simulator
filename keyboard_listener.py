#!/usr/bin/env python3


import sys
sys.path.append("..")
import keyboard
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32



w=0
a=0
s=0
d=0
def publish_keyboard():    
    global w,a,s,d
    pub = rospy.Publisher('keyboard_values',Float32,queue_size=10)
    rospy.init_node('publish_keyboard', anonymous=True)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        if(keyboard.is_pressed('w')):
            time.sleep(0.2) 
            w = w+5
            
                            
        elif(keyboard.is_pressed('a')):
            time.sleep(0.2) 
            a = a+5
            
                    
        elif(keyboard.is_pressed('d')):
            time.sleep(0.2) 
            d = d+5
            
                    
        elif(keyboard.is_pressed('s')):
            time.sleep(0.2) 
            s = s+5
           
                    
        elif(keyboard.is_pressed('c')):
            break

        rospy.loginfo(w)
        rospy.loginfo(a)
        rospy.loginfo(s)
        rospy.loginfo(d)

        pub.publish(w)
        pub.publish(a)
        pub.publish(s)
        pub.publish(d)
        rate.sleep()

if __name__ == '__main__':
    try:
        
        publish_keyboard()
    except rospy.ROSInterruptException:
        pass
