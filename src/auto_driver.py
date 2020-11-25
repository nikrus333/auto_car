#!/usr/bin/env python3
# to do server info

import os
import rospy
import time

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import cv2




class SelfDriver(object):
   
    def __init__(self):
        #print('Hi, select mod control')
        rospy.init_node('auto_driver', anonymous=True)
        self.pub_vel = rospy.Publisher('/target_velocity',Vector3,queue_size=1)
        self.pub_pos = rospy.Publisher('/target_angles',Vector3,queue_size=1)
        #self.pub = rospy.Publisher('cmd',Point, guee_size = 1)
        #to do video
        #cap = cv2.VideoCapture(0)

    def pub(self, data = [0, 4]): # to do rename 
        vel = Vector3()
        vel.x = data[0]
        angle = Vector3()
        angle.y = data[1]
        if angle.y >= 3 and angle.y <= 6:
            self.pub_pos.publish(angle)
        if vel.x < 40:
            pass
        self.pub_vel.publish(vel)
       
   
                
        #self.pub.publish(Point(throtte,String,0.0))
      
class ContrlMode():
    def __init__(self):
        self.sub_j = rospy.Subscriber("joy",Joy,self.keys_input)
        self.mes = [0,0]
        
    def keys_input(self,data):
        vel = 200 * data.axes[1]
        pos = data.axes[3]  
        pos = (pos +2) * 2.25 
        self.mes[0] = vel
        self.mes[1] = pos
        #print(self.mes[1])
        return self.mes  

class Contrl():
    def __init__(self):
        self.sub_j = rospy.Subscriber("cmd_vel",Twist,self.car)
        self.mes = [0,0]
        
    def car(self,data):
        pose = data
        vel = pose.linear.x
        self.mes[0] = vel
        #print(self.mes[1])
        return self.mes  






if __name__ == "__main__":
    self_driver = SelfDriver()
    print("select contrl mod")
    print("1 - joy")
    print("2 - auto")
    input_1 = input()
    if input_1 == '1':
        mod = ContrlMode()
    elif input_1 == '2':
        mod = Contrl()
    
    stored_exception=None
    while not rospy.is_shutdown():
        try:
            self_driver.pub(mod.mes)
            if stored_exception:
                print('game over')
                break
        except KeyboardInterrupt:
            stored_exception=sys.exc_info()






        
