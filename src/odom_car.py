#!/usr/bin/env python3
# to do server info

import os
import rospy
import time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
import math
import numpy as np
import json
import os.path


class OdomCar():
    def __init__(self):
        rospy.init_node('odom_car', anonymous=True)
        self.sub_odom_car = rospy.Subscriber("/camera_motors_position",
                                                JointState, self.odom_cam)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.odom_broadcaster_laser = tf.TransformBroadcaster()
        self.odom = Odometry()
        # parametrs auto_car metr
        scale = 1
        self.pi = math.pi
        self.pi = math.pi
        self.len_base_car = 0.17 * scale  # lenght auto_car
        self.len_mass_c = 0.08 * scale
        self.radius_whell = 0.0325 * scale
        self.len_laser = 0.091 * scale #delite
        self.x = 0
        self.y = 0
        self.th = 0
        self.vx = 0
        self.vy = 0
        self.vth = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.r = rospy.Rate(50)
        self.pos = [0, 0]
        self.vel = [0, 0]
        self.angle_whell = []

        
        

    def odom_cam(self,data):  
        self.pos[0] = data.position[0]
        self.pos[1] = data.position[1]
     
        # arduino car
        n1 = 10
        n2 = 24
        if data.velocity[0] > 300 or data.velocity[0] < -300 :
           data.velocity = [0,0]
        w1 = 2 * math.pi *  data.velocity[0] * 0.229 / 60
        w2 = w1 * n1 / n2
        self.vel[0] = self.radius_whell * w2   # m/sec
        # print(data.velocity[0])
        self.vel[1] = data.velocity[1]

        
    def odome_car(self):
        flag_wheel_direction = True 
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        L = self.vel[0] * dt
        alfa = 0
        if self.pos[1] != 0:
            alfa = (self.pos[1] - 1 * self.pi) * 180 / self.pi
        if alfa < 0:
            alfa = 0
        #print('alfa', alfa)
        tetta = (self.angle_whell[int(alfa)][0] - self.angle_whell[int(alfa)][1]) / 2 
        #print('tetta', tetta)
        tetta = tetta * self.pi / 180 * -1
        #
        if tetta !=0: 
            R = math.sqrt(self.len_mass_c**2 + (1 / math.tan(tetta))**2 * self.len_base_car**2)
            delta_th =  L / R
            if tetta > 0:
                delta_th *= -1
        else:
            delta_th = 0
        delta_x = L * math.cos(self.th)
        delta_y = L * math.sin(self.th) 
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        odom_quat1 = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.odom_broadcaster.sendTransform((self.x, self.y, 0.), 
                                            odom_quat, self.current_time,
                                            "base_link","odom")
        self.odom_broadcaster_laser.sendTransform((0, 0, self.len_laser), 
                                                    odom_quat1, self.current_time, 
                                                    "laser","base_link")
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))
        self.odom.child_frame_id = "base_link"
        self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        self.odom_pub.publish(self.odom)
        self.last_time = self.current_time
        self.r.sleep()


    def angle_rul(self): #solution angle whell 
        try:  # loading data on angle values ​​occurs from a file
            path = '/home/nik/catkin_ws/src/auto_car/src/test.json'
            with open(path, 'r', encoding='utf-8') as f:
                data_j = json.load(f)
                change1 = []
                change2 = []
                for i in range(360):
                    if i < 90:
                        change2.append(data_j[i]['%d' % i])  
                    if i >= 270 and i < 360:
                        change1.append(data_j[i]['%d' % i])     
                    self.angle_whell = change1 + change2
                    #print(len(self.angle_whell))
                    #print(self.angle_whell)    
            
        except: 
            print('it is not possible to open the file')
      
def main():
    car = OdomCar()
    car.angle_rul()
    stored_exception=None
    while not rospy.is_shutdown():
        try:
            car.odome_car()
            if stored_exception:
                print('game over')
                break
        except KeyboardInterrupt:
            stored_exception=sys.exc_info()      

if __name__ == "__main__":
    main()