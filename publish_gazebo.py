#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mav_msgs.msg import Actuators
from sensor_msgs.msg import Joy
import message_filters
from sensor_msgs.msg import Imu
import numpy as np
from std_srvs.srv import Empty
import math
from pid import PID
import time

from datetime import datetime
global tho,pitch,yaw,roll,front_left,front_right,back_left,back_right
global roll_imu,pitch_imu,yaw_imu
global pitch_gyro,roll_gyro,yaw_gyro
global rr_pid,pr_pid,yr_pid
pitch_gyro,roll_gyro,yaw_gyro = 0.0,0.0,0.0
tho =0.0
pitch = 0
back_right = 0
back_left = 0
front_right = 0
front_left = 0
roll=0

rr_pid = PID(p=0.7, i=0.9,imax=50)
pr_pid = PID(p=0.7,i=0.9,imax=50)
yr_pid = PID(p=2.5,i=0.9,imax=50) #0.01  0.0000001


def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll_imu_q = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch_imu_q = np.where(np.abs(sinp) >= 1,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw_imu_q = np.arctan2(siny_cosp, cosy_cosp)    
    return math.degrees(roll_imu_q), math.degrees(pitch_imu_q), math.degrees(yaw_imu_q)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin    
    valueScaled = float(value - leftMin) / float(leftSpan)    
    return rightMin + (valueScaled * rightSpan)

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

class Server:
    def __init__(self):
        self.joy = None
        self.imu = None

    def joy_callback(self, msg):
        # "Store" message received.
        # kumandadan gelen veriler        
        self.joy = msg
        global tho,yaw,pitch,roll
        tho = self.joy.axes[1]
        yaw = self.joy.axes[0]
        
        pitch = self.joy.axes[3]
        roll = self.joy.axes[2]
        #l1 = self.joy.buttons[4] #LB
        """if(l1 == 1):            
            reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset_simulation()"""
        tho=translate(tho,-1,1,0,1000)
        pitch = translate(pitch,-1,1,-45,45) #15
        roll = translate(roll,-1,1,-45,45) #15
        yaw = -translate(yaw,-1,1,-150,150) #45
        
        # Compute stuff.
        self.compute_stuff()

    def imu_callback(self, msg):        
        self.imu = msg
        global roll_imu,pitch_imu,yaw_imu,pitch_gyro,roll_gyro,yaw_gyro
        pitch_gyro = -math.degrees(self.imu.angular_velocity.y)
        roll_gyro = math.degrees(self.imu.angular_velocity.x) #z
        yaw_gyro = -math.degrees(self.imu.angular_velocity.z) #x
        roll_imu,pitch_imu,yaw_imu = quaternion_to_euler(self.imu.orientation.w,self.imu.orientation.x,self.imu.orientation.y,self.imu.orientation.z)        
        roll_imu *=-1
        pitch_imu *= -1
        yaw_imu *= -1
        self.compute_stuff()

    def compute_stuff(self):
        pub = rospy.Publisher('ardrone/command/motor_speed', Actuators, queue_size=10)
        rate = rospy.Rate(200)
        global tho,pitch_imu,pitch,roll,front_left,front_right,back_left,back_right,pitch_gyro,roll_gyro,yaw_gyro
        global rr_pid,pr_pid,yr_pid
        back_left,front_left,front_right,back_right=0.0,0.0,0.0,0.0
        if(tho > 200):
                

            

            """rol_out = max(min(rr_pid.get_pid(roll - roll_gyro, 1), 500), -500)            
            pit_out = max(min(pr_pid.get_pid(pitch - pitch_gyro, 1), 500), -500)
            yaw_out = max(min(yr_pid.get_pid(yaw - yaw_gyro, 1), 500), -500)"""
            
            rol_out = rr_pid.get_pid(roll - roll_gyro, 1)            
            pit_out = pr_pid.get_pid(pitch - pitch_gyro, 1)
            yaw_out = yr_pid.get_pid(yaw - yaw_gyro, 1)


            front_right = tho - (pit_out)+(rol_out) + yaw_out
            front_left = tho - (pit_out)-(rol_out) - yaw_out
            back_right = tho + (pit_out)+(rol_out) - yaw_out
            back_left =  tho + (pit_out)-(rol_out) + yaw_out
            """
            curr_time = datetime.now()
            formatted_time = curr_time.strftime('%M:%S.%f')
            f = open("/home/birhan/birhan/src/beginner_tutorials/src/scripts/selami2.txt","a")
            f.write(formatted_time +"\ntho: " + str(tho) +  " gyro_roll: " + str(roll_gyro) + " pitch_gyro: " + str(pitch_gyro)+ " yaw_gyro: " + str(yaw_gyro)+"\n" +"roll_out "+str(rol_out)+" pit out"+str(pit_out)+ " yaw out"+str(yaw_out)+"\n FR:"+str(front_right)+" FL:"+str(front_left)+" BR:"+str(back_right)+ " BL:"+str(back_left)+"\n")
            f.write("---------------------------\n")            
            f.close()"""
        else:
            front_right = tho
            front_left = tho
            back_right = tho
            back_left= tho
            rr_pid.reset_I()
            pr_pid.reset_I()
            yr_pid.reset_I()


        
        act = Actuators()
        a=[back_left,front_right,back_right,front_left]  #[sol arka,sağ ön, sağ arka, sol ön ]
        act.angular_velocities=a
        rospy.loginfo(act)        
        pub.publish(act)       
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber('/joy', Joy , server.joy_callback)
    rospy.Subscriber('/ardrone/imu', Imu, server.imu_callback)

    rospy.spin()




