#!/usr/bin/env python3
import rospy
import math
from control.srv import *

# TODO: update geometry params
L1 = 0.2015
L2 = 0.2

def handle_ik_compute(req):
	resp = IKComputeResponse()
	resp.success = True

	# Check for singularity conditions
		# First check if y is negative.
		# Second, check if the x,y is outside the range of the arm fully extended.
		# Third, check if the x,y is inside the range of the arm fully collapsed. 
	if req.y < 0 :
		print("Desired y is negative and unreachable")
		resp.success = False
		#set desired motor postion equal to the last desired motor position
	elif round(math.pow(req.x,2) + math.pow(req.y,2),3) > round(math.pow(L1+L2,2),3):
		print("Desired x, y is too big and outside of reachable workspace")
		resp.success = False
		#set desired motor postion equal to the last desired motor position
	elif math.pow(req.x,2) + math.pow(req.y,2) < math.pow(L1,2):
		print("Desired x, y is too small and outside of reachable workspace")
		resp.success = False

	if resp.success:
		# solve for theta2
		c2 = (math.pow(req.x,2) + math.pow(req.y,2) - math.pow(L1,2) - math.pow(L2,2)) / (2*L1*L2)
		c2 = round(c2,3)
		# sign decides elbow up/down
		s2 = math.sqrt(1 - math.pow(c2,2))

		resp.theta2 = math.atan2(s2,c2)

		# change of variable to solve for theta1
		k1 = L1 + L2*c2
		k2 = L2*s2

		resp.theta1 = math.atan2(req.y,req.x) - math.atan2(k2,k1)

	return resp


if __name__ =="__main__":
	rospy.init_node('ik_service')
	rospy.Service('ik_compute', IKCompute, handle_ik_compute)
	rospy.spin()