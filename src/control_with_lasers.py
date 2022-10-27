#!/usr/bin/env python3
import serial
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy
import numpy as np

class SerialUSB():
    def __init__(self):
        self.ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200)
    
    def write_data(self, data):
        data_str = str(data[0]) + ',' + str(data[1])
        data_str_bytea = data_str.encode('utf-8')
        self.ser.write(data_str_bytea)
    
    def read_data(self):
        data_str = self.ser.read(10).decode('utf-8')
        return data_str

class MinimalSubscriber():

    def __init__(self):
        self.serial_1 = SerialUSB()
        rospy.init_node('car', anonymous=True)
        self.subscription = rospy.Subscriber(
           '/cmd_vel',
            Twist,
            self.listener_callback,
            10)
        self.sub_laser = rospy.Subscriber(
           '/scan',
            LaserScan,
            self.laser_callback,
            10)
        self.pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)	

    def listener_callback(self, msg, data):
        data = [msg.linear.x, msg.angular.z]
        print(data)
        self.data_angle = msg.angular.z
        self.serial_1.write_data(data)

    def laser_callback(self, data, d):
        mean = 0
        for count in range(100, 220):
            mean += data.ranges[count]
        mean /= 120
        msg = Twist()
        if mean < 1.5:
            k_control_angle = 20 * 0.5
            z =  k_control_angle / mean
            msg.linear.x = 1
            if z > 20:
                z = 20
            msg.angular.z = z
        else:
            msg.linear.x = 1
            msg.angular.z = 0
        self.pub.publish(msg)
    

def main():
    minimal_subscriber = MinimalSubscriber()
    stored_exception=None
    while not rospy.is_shutdown():
        try:
            if stored_exception:
                print('game over')
                break
        except KeyboardInterrupt:  
            stored_exception=sys.exc_info()      
    
if __name__ == "__main__":
    main()