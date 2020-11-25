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


class OdomCar():

    def __init__(self):
        rospy.init_node('odom_car', anonymous=True)
        self.sub_odom_car = rospy.Subscriber("/camera_motors_position",JointState,self.odom_cam)
        self.odom_pub = rospy.Publisher("odom",Odometry,queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.odom_broadcaster_laser = tf.TransformBroadcaster()

        self.odom = Odometry()

        self.len_base_car = 10 # lenght auto_car
        self.len_mass_c = 5
        self.x = 0
        self.y = 0
        self.th = 0 
        self.vx = 0
        self.vy = 0
        self.vth = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.r = rospy.Rate(0.5)
        self.pos = [0,0]
        self.vel = [0,0]
        

    def odom_cam(self,data):  
        self.pos[0] = data.position[0]
        self.vel[0] = data.velocity[0]
        self.pos[1] = data.position[1]
        self.vel[1] = data.velocity[1]
        

    def odome_car(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        L = self.vel[0] * dt
        alfa = self.pos[1]
        if alfa !=0: 
            R = math.sqrt(self.len_mass_c**2 + (1 / math.tan(alfa))**2 * self.len_mass_c**2)
            delta_th = L / R
        else:
            delta_th = 0
        delta_x = L * math.cos(alfa) /10
        delta_y = L * math.sin(alfa) /10
        self.x += delta_x
        self.y += delta_y
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th




        '''
        delta_x = (self.vel[0]  *  math.sin(self.pos[1]) * dt) /10
        delta_y = (self.vel[0] * math.cos(self.pos[1])) * dt / 10
        self.x += delta_x
        self.y += delta_y
        self.th = self.pos[1]
        #self.th = 0.5
        '''
        #print('vel[0]',self.vel[0])
        #print('pos[1]',pos[1])
 
        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)
        odom_quat1 = tf.transformations.quaternion_from_euler(0,0,0)
        self.odom_broadcaster.sendTransform((self.x, self.y ,0.),odom_quat,self.current_time,"base_link","odom")
        self.odom_broadcaster_laser.sendTransform((0,0,1), odom_quat1,self.current_time,"laser","base_link")
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        self.odom.child_frame_id = "base_link"
        self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        self.odom_pub.publish(self.odom)
        self.last_time = self.current_time
        self.r.sleep()


    def rul_rul(self, data): #solution angle whell 
        def __init__(self):


        

if __name__ == "__main__":
    car = OdomCar()
    stored_exception=None
    while not rospy.is_shutdown():
        try:
            car.odome_car()
            if stored_exception:
                print('game over')
                break
        except KeyboardInterrupt:
            stored_exception=sys.exc_info()