#!/usr/bin/env python3
import rospy
import math
from control.srv import *

class PIDService1:
	def __init__(self):
		self.kp_up = 0.0
		self.kd_up = 0.0
		self.ki_up = 0.0

		self.kp_down = 0.0
		self.kd_down = 0.0
		self.ki_down = 0.0

		self.p_term = 0.0
		self.d_term = 0.0
		self.i_term = 0.0
		self.i_term_min = -40
		self.i_term_max = 40

		self.prev_error = 0.0
		self.update_hz = 60
		self.output_min = -math.inf
		self.output_max = math.inf
		self.pen_position_up = True

		rospy.Service('pid_set_gains1', PIDSetGains, self.handle_pid_set_gains)
		rospy.Service('pid_compute1', PIDCompute, self.handle_pid_compute)
		rospy.Service('pid_set_output_limits1', PIDSetOutputLimits, self. handle_pid_set_output_limits)

	def handle_pid_set_gains(self,req):
		self.kp_up = req.kp_up
		self.kd_up = req.kd_up
		self.ki_up = req.ki_up

		self.kp_down = req.kp_down
		self.kd_down = req.kd_down
		self.ki_down = req.ki_down

		# reset integrator term when new gains are defined
		self.i_term = 0

		# check sum on client side to confirm update
		return int(req.kp_up + req.kd_up + req.ki_up + req.kp_down + req.kd_down + req.ki_down)

	def handle_pid_set_output_limits(self,req):
		self.output_min = req.output_min
		self.output_max = req.output_max

		# check sum on client side to confirm update
		return self.output_min + self.output_max

	def handle_pid_compute(self,req):
		if self.pen_position_up != req.pen_pos_up:
			self.i_term = 0

		self.pen_position_up = req.pen_pos_up

		if req.pen_pos_up:
			self.p_term = self.kp_up*req.error
			self.d_term = self.kd_up*(req.error - self.prev_error)*self.update_hz
			self.i_term += self.ki_up*req.error/self.update_hz
			rospy.loginfo(f'compute_1: pen up gains...{[self.kp_up,self.kd_up,self.ki_up]}')
		else:
			self.p_term = self.kp_down*req.error
			self.d_term = self.kd_down*(req.error - self.prev_error)*self.update_hz
			self.i_term += self.ki_down*req.error/self.update_hz
			rospy.loginfo(f'compute_1: pen down gains...{[self.kp_down,self.kd_down,self.ki_down]}')

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

		rospy.loginfo(f'compute_1: computing pwm... error: {req.error} / output: {output}')
		
		return int(output)


if __name__ =="__main__":
	rospy.init_node('pid_service1',anonymous=True)
	PIDService1()
	rospy.spin()


