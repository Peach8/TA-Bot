#!/usr/bin/env python3
import rospy
import rosbag
import sys
from control.srv import *
from motor.srv import *

# nodes to launch before rosrunning this node
# - motor read_write node that includes all serial port stuff

# change the gains and re-run node
kp = 1.0
kd = 1.0
ki = 1.0

# change when switching motors to tune
joint_str = "joint1"

# TODO: reassign these if necessary
joint_id_dict = {"joint1": 0, "joint2": 1, "joint3": 2}

# global variables to be updated
actual_motor_position = 0
desired_motor_position = 0
desired_motor_pwm = SetMotorPWM()
desired_motor_pwm.id = joint_id_dict[joint_str]

bag = robag.Bag('actual_position', 'w')

def control_motor():
	global actual_motor_position
	global desired_motor_position
	global desired_motor_pwm

	get_motor_position_resp = get_motor_position(joint_id_dict[joint_str])
	actual_motor_position = get_motor_position_resp.position

	position_error = desired_motor_position-actual_motor_position

	pid_compute_resp = pid_compute(position_error)

	desired_motor_pwm.pwm = pid_compute_resp.output


if __name__ == '__main__':
	global actual_motor_position
	global desired_motor_position
	global desired_motor_pwm

	rospy.init_node("motor_tuning", anonymous=True)

	# set up motor encoder service proxy (service established in motor/read_write_node)
	rospy.wait_for_service("get_motor_position")
	get_motor_position = rospy.ServiceProxy("get_motor_position", GetMotorPosition)

	# set up pid service proxies
	rospy.wait_for_service(joint_str + "_pid_set_gains")
	pid_set_gains = rospy.ServiceProxy(joint_str + "_pid_set_gains", PIDSetGains)
	try:
		set_gains_resp = pid_set_gains(kp,kd,ki)
		# confirm response via sum
		if set_gains_resp.sum == kp+kd+ki:
			continue
		else:
			print("Invalid response from PIDSetGains service")
			sys.exit(1)
	except rospy.ServiceException as set_gains_exception:
		print(str(set_gains_exception))

	rospy.wait_for_service(joint_str + "_pid_compute")
	pid_compute = rospy.ServiceProxy(joint_str + "_pid_compute", PIDCompute)

	# set up publisher to write desired PWM to each motor (subscriber established in motor/read_write_node)
	pwm_pub = rospy.Publisher('control/motor_pwm', SetMotorPWM, queue_size=10)

	rospy.spin()
	rate = rospy.Rate(10) # Hz

	# print current position of the motor and prompt user for desired position
	get_motor_position_resp = get_motor_position(joint_id_dict[joint_str])
	print("current motor position: " + str(get_motor_position_resp.position))
	
	desired_motor_position = int(input("enter desired motor position:\n"))

	while not rospy.is_shutdown():
		control_motor()
		pwm_pub.publish(desired_motor_pwm)
		rospy.loginfo(actual_motor_position)
		bag.write(actual_motor_position)

		# sleep enough to maintain desired rate
		rate.sleep()

