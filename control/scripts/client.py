#!/usr/bin/env python3
import rospy
from control.srv import *

if __name__ == "__main__":
	rospy.wait_for_service("ik_compute")
	ik_compute = rospy.ServiceProxy("ik_compute", IKCompute)
	try:
		resp1 = ik_compute(1.0, 1.0)
		print(resp1.theta1)
		print(resp1.theta2)
	except rospy.ServiceException as e7:
		print(str(e7))