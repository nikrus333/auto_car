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


	<joint name="ang_base" type="fixed">
		<parent link="base_link"/>
		<child link="car"/>
		
		<origin rpy = "0 0 0" xyz="0 0 0" />
	</joint>
	<link name="car">
		<visual>
			<origin rpy="1.57 0 -1.57" xyz="0 0 0"/>
			<geometry>
				<mesh filename="package://auto_car/urdf/car.stl" scale="0.01 0.01 0.01"/>
			</geometry>
			
		</visual>
	</link>
	