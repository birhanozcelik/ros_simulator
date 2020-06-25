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
from math import isnan, pi, asin, atan, atan2
from squaternion import quat2euler
from datetime import datetime
global tho,pitch,yaw,roll,front_left,front_right,back_left,back_right
global roll_imu,pitch_imu,yaw_imu
global pitch_gyro,roll_gyro,yaw_gyro
global rr_pid,pr_pid,yr_pid
global rs_pid,ps_pid,ys_pid
global yaw_target
pitch_gyro,roll_gyro,yaw_gyro = 0.0,0.0,0.0
tho =0.0
pitch = 0.0
back_right = 0
back_left = 0
front_right = 0
front_left = 0
roll=0.0
yaw=0.0
yaw_target = 0.0
yaw_imu = 0.0
pitch_imu = 0.0
roll_imu=0.0
#roll stabilize 0.25
rs_pid = PID(p=0.2) #p=0.1, i=0.1,d=0.0000001, imax=5

#pitch stabilize 0.1
ps_pid = PID(p=0.2) #p=14.0, i=0.1,d= 0.0000001 ,imax=5

#yaw stabilize 0.1 
ys_pid = PID(p=0.2) #1.0,0.01,0.0000001

#roll rate
rr_pid = PID(p=0.7, i = 1 ,imax=50)

#pitch rate
pr_pid = PID(p=0.7,i = 1,imax=50)

#yaw rate
yr_pid = PID(p=2.5,i=1,imax=50) #0.01  0.0000001

def wrap_180(x):
  return x+360 if x < -180 else (x-360 if x > 180 else x)
def safe_asin(v):
  if isnan(v): return 0
  if v >= 1: return pi/2
  if v <= -1: return -pi/2
  return asin(v)
def sonumuteuler(w,x,y,z):
    
    (roll,pitch,yaw,)=quat2euler(w,x,y,z,degrees=True)
    return roll,pitch,yaw
    
    
def dmpGetEuler(w, x, y, z):
      yaw = atan2(2*x*y - 2*w*z, 2*w*w + 2*x*x - 1) * 180 / pi
      pitch = -safe_asin(2*x*z + 2*w*y) * 180 / pi
      roll = atan2(2*y*z - 2*w*x, 2*w*w + 2*z*z - 1) * 180 / pi
      return (yaw, pitch, roll)

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
        pitch = translate(pitch,-1,1,-30,30) #15
        roll = translate(roll,-1,1,-30,30) #15
        yaw = -translate(yaw,-1,1,-150,150) #45
        
        # Compute stuff.
        self.compute_stuff()

    def imu_callback(self, msg):        
        self.imu = msg
        global roll_imu,pitch_imu,yaw_imu,pitch_gyro,roll_gyro,yaw_gyro
        pitch_gyro = -math.degrees(self.imu.angular_velocity.y)
        roll_gyro = math.degrees(self.imu.angular_velocity.x) #z
        yaw_gyro = -math.degrees(self.imu.angular_velocity.z) #x
        #yaw_imu,pitch_imu,roll_imu = dmpGetEuler(self.imu.orientation.w,self.imu.orientation.x,self.imu.orientation.y,self.imu.orientation.z)        
        
        roll_imu,pitch_imu,yaw_imu=sonumuteuler(self.imu.orientation.w,self.imu.orientation.x,self.imu.orientation.y,self.imu.orientation.z)
        roll_imu =roll_imu * (-1)
        pitch_imu = pitch_imu * (-1)
        yaw_imu = yaw_imu * (-1)
       

        self.compute_stuff()

    def compute_stuff(self):
        pub = rospy.Publisher('ardrone/command/motor_speed', Actuators, queue_size=10)
        rate = rospy.Rate(200)
        global tho,pitch_imu,pitch,roll,yaw,front_left,front_right,back_left,back_right,pitch_gyro,roll_gyro,yaw_gyro
        global rr_pid,pr_pid,yr_pid,yaw_target,roll_imu,yaw_imu
        global rs_pid,ps_pid,ys_pid
        back_left,front_left,front_right,back_right=0.0,0.0,0.0,0.0
        
        if(tho > 200):
            rol_stab_out = constrain(rs_pid.get_pid(roll - roll_imu, 1), -250, 250)
            pitch_stab_out =  constrain(ps_pid.get_pid(pitch - pitch_imu, 1), -250, 250)
            yaw_stab_out = constrain(ys_pid.get_pid(wrap_180(yaw_target - yaw_imu), 1), -360, 360)
            if(abs(yaw) > 5):
                yaw_stab_out = yaw
                yaw_target = yaw_imu

            rol_out = constrain(rr_pid.get_pid(rol_stab_out - roll_gyro, 1), -500, 500)            
            pit_out = constrain(pr_pid.get_pid(pitch_stab_out - pitch_gyro, 1), -500, 500)
            yaw_out = constrain(yr_pid.get_pid(yaw_stab_out - yaw_gyro, 1), -500, 500)
            
            front_right = tho - (pit_out)+(rol_out) + yaw_out
            front_left = tho - (pit_out)-(rol_out) - yaw_out
            back_right = tho + (pit_out)+(rol_out) - yaw_out
            back_left =  tho + (pit_out)-(rol_out) + yaw_out  
            """
                


                hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
                hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
                hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
                hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
            """     
        else:
            front_right = tho
            front_left = tho
            back_right = tho
            back_left= tho
            yaw_target = yaw_imu
            rr_pid.reset_I()
            pr_pid.reset_I()
            yr_pid.reset_I()
            rs_pid.reset_I()
            ps_pid.reset_I()
            ys_pid.reset_I()

        
        act = Actuators()
        a=[back_left,front_right,back_right,front_left]
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




