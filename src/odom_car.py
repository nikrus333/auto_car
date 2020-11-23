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


class OdomCar(object):

    def __init__(self):
        rospy.init_node('odom_car', anonymous=True)
        self.sub_odom_car = rospy.Subscriber("/camera_motors_position",JointState,self.odom_car)
        self.odom_pub = rospy.Publisher("odom",Odometry,queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.odom = Odometry()
        print(type(self.odom))
        self.x = 0
        self.y = 0
        self.th = 0 
        self.vx = 0
        self.vy = 0
        self.vth = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.r = rospy.Rate(1)
        self.pos = [0,0]
        self.vel = [0,0]

    def odom_cam(self,data):  
        self.pos = data.position
        self.vel = data.velocity

    def odom_car(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        delta_x = (self.vel[0]  * dt)
        delta_y = (self.vel[0] * math.sin(self.pos[1])) * dt
        #delta_th = 
        
        self.x += delta_x
        self.y += delta_y
        #self.th = delta_th  
        self.th = 0.5
        print('vel[0]',self.vel[0])
        #print('pos[1]',pos[1])

        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)
        self.odom_broadcaster.sendTransform((self.x, self.y ,0.),odom_quat,self.current_time,"base_link","odom")
        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        self.odom.child_frame_id = "base_link"
        self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        print(self.odom.twist.twist.linear.x)
        self.odom_pub.publish(self.odom)

        self.last_time = self.current_time
        self.r.sleep()






if __name__ == "__main__":
    car = OdomCar()
    stored_exception=None
    while not rospy.is_shutdown():
        try:
            car.odom_car()
            if stored_exception:
                print('game over')
                break
        except KeyboardInterrupt:
            stored_exception=sys.exc_info()