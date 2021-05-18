#!/usr/bin/env python3
import rospy
import math
from control.srv import *

# TODO: update geometry params
L1 = 1.0
L2 = 1.0

def handle_ik_compute(req):
	resp = IKComputeResponse()

	# solve for theta2
	c2 = (math.pow(req.x,2) + math.pow(req.y,2) - math.pow(L1,2) - math.pow(L2,2)) / (2*L1*L2)
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