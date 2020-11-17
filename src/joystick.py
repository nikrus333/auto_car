#! /usr/bin/env python
'''
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
sudo jstest /dev/input/js0
ls -l /dev/input/js0
sudo chmod a+rw /dev/input/js0
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node

