#!/usr/bin/env python3
import rospy
import math
import numpy as np
from control.srv import *

# TODO: update geometry params
L1 = 1.0
L2 = 1.0
q1 = 0.2
q2 = 0.3

def handle_fk_compute(req):
	resp = FKComputeResponse()

	# Solve for end of Link 1
	x1 = L1* math.cos(q1)
	y1 = L1* math.sin(q1)

	# Solve for end of Link 2
	resp.x2 = x1 + l2 * math.cos(q2)
	resp.y2 = y1 + L2 * math.sin(q2)

	return resp

	# Alternatively, if we want the full transformation matrix
	T = np.array([[ math.cos(q1+q2), -math.sin(q1+q2), L2*math.cos(q1+q2)+L1*cos(q1)],
		[math.sin(q1+q2), math.cos(q1+q2), L2*math.sin(q1+q2)+L1*math.sin(q1)],
		[0, 0, 0, 1]])
	#resp.xend = T[0][2]
	#resp.yend = T[1][2]
	#return resp

if __name__ == "__ophelie_fk":
	rospy.init_node('fk_service')
	rospy.Service('fk_compute', FKCompute, handle_fk_compute)
	rospy.spin()