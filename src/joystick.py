#! /usr/bin/env python
'''
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
sudo jstest /dev/input/js0
ls -l /dev/input/js0
sudo chmod a+rw /dev/input/js0
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node
<node pkg="joy" type="joy_node" name = "joy_node"/>


360 lidar
sudo chmod 666 /dev/ttyUSB0
roslaunch hls_lfcd_lds_driver hlds_laser.launch
roslaunch hls_lfcd_lds_driver view_hlds_laser.launch



base_local_planner

rosrun gmapping slam_gmapping scan:=scan _base_frame:=base_link _odom_frame:=odom
