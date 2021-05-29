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
	desired_x = float(input("Enter desired x position:\n")) 
	desired_y = float(input("Enter desired y position:\n")) 

	# set up IK_compute service proxy to call the IK compute
	rospy.wait_for_service('ik_compute')
	get_desired_thetas = rospy.ServiceProxy('ik_compute', IKCompute)


		# Check for singularity conditions
		# First check if y is negative.
		# Second, check if the x,y is outside the range of the arm fully extended.
		# Third, check if the x,y is inside the range of the arm fully collapsed. 
		if desired_y < 0 :
			print("Desired y is negative and unreachable")
			return False
			#set desired motor postion equal to the last desired motor position
		elif desired_x^2 + desired_y^2 > 4:
			print("Desired x, y is too big and outside of reachable workspace")
			return False
			#set desired motor postion equal to the last desired motor position
		elif desired_x^2 + desired_y^2 < 2:
			print("Desired x, y is too small and outside of reachable workspace")
			return False
			#set desired motor postion equal to the last desired motor position			sys.exit()


	# send the desired_x and desired_y to the IKService.py
	# IK service runs and returns theta1 and theta2

	# use theta1 and theta2 as inputs into the motor tuning script