#!/usr/bin/env python3
import rospy
import rosbag
import sys
from control.srv import * #this calls FKService.py
from motor.srv import *


	global desired_x
	global desired_y

#nodes to launch before rosrunning this node
# I am unsure

# Gains for motor 0:
kp = 2;
kd = 0.1;
ki = 0;

# Gains for motor 1: 
kp = 4;
kd = 0.1;
ki = 0;

# Gains for motor 2
kp = 1;
kd = 0; 
ki = 0;


# global variables at starting position - to be updated (x,y) 
desired_x = 2;
desired_y = 0;

bag = rosbag.Bag('desired_position.bag','w')

def ik_control():
	global desired_x
	global desired_y

	get_desired_thetas_resp = get_desired_thetas(desired_x, desired_y)

if __name__ == '__main__'

	global desired_x
	global desired_y

	#Create node for ik_control

	rospy.init_node('ik_control', anonymous = True)

	# prompt user for desired (x,y) position
	desired_x = int(input("Enter desired x position:\n")) 
	desired_y = int(input("Enter desired y position:\n")) 

	# set up IK_compute service proxy to call the IK compute
	rospy.wait_for_service('ik_compute')
	get_desired_thetas = rospy.ServiceProxy('ik_compute', IKCompute)


	

	# send the desired_x and desired_y to the IKService.py
	# IK service runs and returns theta1 and theta2

	# use theta1 and theta2 as inputs into the motor tuning script