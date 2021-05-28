#!/usr/bin/env python3
import rospy
import math
from control.srv import *

class PIDService:
	def __init__(self):
		self.kp = 0.0
		self.kd = 0.0
		self.ki = 0.0

		self.p_term = 0.0
		self.d_term = 0.0
		self.i_term = 0.0
		self.i_term_min = -1.0
		self.i_term_max = 1.0

		self.prev_error = 0.0
		self.update_hz = 10
		self.output_min = -math.inf
		self.output_max = math.inf

		rospy.Service('pid_set_gains', PIDSetGains, self.handle_pid_set_gains)
		rospy.Service('pid_compute', PIDCompute, self.handle_pid_compute)
		rospy.Service('pid_set_output_limits', PIDSetOutputLimits, self. handle_pid_set_output_limits)

	def handle_pid_set_gains(self,req):
		self.kp = req.kp
		self.kd = req.kd
		self.ki = req.ki

		# reset integrator term when new gains are defined
		self.i_term = 0

		# check sum on client side to confirm update
		return int(req.kp + req.kd + req.ki)

	def handle_pid_set_output_limits(self,req):
		self.output_min = req.output_min
		self.output_max = req.output_max

		# check sum on client side to confirm update
		return self.output_min + self.output_max

	def handle_pid_compute(self,req):
		self.p_term = self.kp*req.error
		self.d_term = self.kd*(req.error - self.prev_error)*self.update_hz
		self.i_term += self.ki*req.error/self.update_hz

		if self.i_term > self.i_term_max:
			self.i_term = self.i_term_max
		elif self.i_term < self.i_term_min:
			self.i_term = self.i_term_min

		output = self.p_term + self.d_term + self.i_term

		if output > self.output_max:
			output = self.output_max
		elif output < self.output_min:
			output = self.output_min

		self.prev_error = req.error

		return int(output)


if __name__ =="__main__":
	rospy.init_node('pid_service')
	PIDService()
	rospy.spin()


