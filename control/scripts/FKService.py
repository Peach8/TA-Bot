#!/usr/bin/env python3
import rospy
import math
import numpy as np
from control.srv import *

# TODO: update geometry params
L1 = 1.0
L2 = 1.0


def handle_fk_compute(req):
	resp = FKComputeResponse()

	# Solve for end of Link 1
	x1 = L1* math.cos(req.theta1)
	y1 = L1* math.sin(req.theta1)

	# Solve for end of Link 2
	resp.x = x1 + L2 * math.cos(req.theta1+req.theta2)
	resp.y = y1 + L2 * math.sin(req.theta1+req.theta2)

	return resp

	# Alternatively, if we want the full transformation matrix
	#T = np.array([[ math.cos(req.theta1+req.theta2), -math.sin(req.theta1+req.theta2), L2*math.cos(req.theta1+req.theta2)+L1*math.cos(req.theta1)],
	#	[math.sin(req.theta1+req.theta2), math.cos(req.theta1+req.theta2), L2*math.sin(req.theta1+req.theta2)+L1*math.sin(req.theta1)],
	#	[0, 0, 0, 1]])
	#resp.x = T[0][2]
	#resp.y = T[1][2]
	#return resp

if __name__ == "__ophelie_fk":
	rospy.init_node('fk_service')
	rospy.Service('fk_compute', FKCompute, handle_fk_compute)
	rospy.spin()