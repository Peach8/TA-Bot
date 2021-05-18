#!/usr/bin/env python3
import rospy
from common.srv import *

class PIDService:
	def __init__(self):
		self.kp = 0.0
		self.kd = 0.0
		self.ki = 0.0

		self.p_term = 0.0
		self.d_term = 0.0
		self.i_term = 0.0

		self.prev_error = 0.0
		self.updateHz = 10

		rospy.Service('pid_set_gains', PIDSetGains, self.handle_pid_set_gains)
		rospy.Service('pid_compute', PIDCompute, self.handle_pid_compute)

	def handle_pid_set_gains(self,req):
		self.kp = req.kp
		self.kd = req.kd
		self.ki = req.ki

		self.i_term = 0

		# check sum on client side to confirm update
		return req.kp + req.kd + req.ki

	def handle_pid_compute(self,req):
		self.p_term = self.kp*req.error
		self.d_term = self.kd*(req.error - self.prev_error)*self.updateHz
		self.i_term += self.ki*req.error/self.updateHz

		self.prev_error = req.error

		return self.p_term + self.d_term + self.i_term

if __name__ =="__main__":
	rospy.init_node('pid_service')
	PIDService()
	rospy.spin()


