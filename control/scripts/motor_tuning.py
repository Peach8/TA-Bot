#!/usr/bin/env python3
import rospy
import rosbag
import sys
import signal
from std_msgs.msg import Int32
from control.srv import *
from motor.srv import *
from motor.msg import *
import time

# 1 - position only ; 2 - velocity only ; 3 - pos-->velo cascade
CONTROL_APPROACH = 1

# change when switching motors to tune
joint_str = "joint1"

global actual_motor_position
global actual_motor_velocity

desired_motor_pwm = SetMotorPWM()
desired_motor_pwm.id = joint_id_dict[joint_str]

# change the gains and re-run node
kp_pos = 1.0
kd_pos = 0.0
ki_pos = 0.0

kp_vel = 1.0
kd_vel = 0.0
ki_vel = 0.0

global ff_pwm
ff_pwm = 0 # int

joint_id_dict = {"joint1": 0, "joint2": 1, "joint3": 2}

min_motor_position = 450
max_motor_position = 925

max_motor_velocity = 1023 # signed 10-bit int with units 0.229 rpm/bit


if CONTROL_APPROACH == 1 or CONTROL_APPROACH == 3:
	bag = rosbag.Bag('actual_position.bag', 'w')
elif CONTROL_APPROACH == 2:
	bag = rosbag.Bag('actual_velocity.bag', 'w')
global bag_data
bag_data Int32()


def control_motor_position(desired_motor_position):
	global ff_pwm
	global bag_data

	get_motor_position_resp = get_motor_position(joint_id_dict[joint_str])
	actual_motor_position = get_motor_position_resp.position

	bag_data.data = actual_motor_position
	bag.write('actual_value', bag_data)	

	position_error = desired_motor_position - actual_motor_position

	pid_compute_resp = pid_compute(position_error)
	return pid_compute_resp.outpout


def control_motor_velocity(desired_motor_velocity):
	global ff_pwm
	global bag_data

	get_motor_velocity_resp = get_motor_velocity(joint_id_dict[joint_str])
	actual_motor_velocity = get_motor_velocity_resp.velocity

	bag_data.data = actual_motor_velocity
	bag.write('actual_value', bag_data)	

	velocity_error = desired_motor_velocity - actual_motor_velocity

	pid_compute_resp = pid_compute(velocity_error)
	return pid_compute_resp.output


def cascade_control(desired_motor_position);
	desired_motor_velocity = control_motor_position(desired_motor_position)
	return control_motor_velocity(desired_motor_velocity)


def check_motor_position_bounds(desired_motor_position):
	if desired_motor_position < min_motor_position or desired_motor_position > max_motor_position:
		print("desired position out of bounds")
		sys.exit(0)


def check_motor_velocity_bounds(desired_motor_velocity):
	if desired_motor_velocity > max_motor_velocity:
		print("desired velocity out of bounds")
		sys.exit(0)	


def process_kill_handler(sig, frame):
	print("closing bag...")
	bag.close() # close rosbag on CTRL+C
	sys.exit(0)

if __name__ == '__main__':
	signal.signal(signal.SIGINT, process_kill_handler)

	rospy.init_node("motor_tuning", anonymous=True)

	# set up motor position and velocity service proxies (established in motor/read_write_node)
	rospy.wait_for_service('get_motor_position')
	get_motor_position = rospy.ServiceProxy('get_motor_position', GetMotorPosition)

	rospy.wait_for_service('get_motor_velocity')
	get_motor_velocity = rospy.ServiceProxy('get_motor_velocity, GetMotorVelocity')

	# set up position pid service proxies
	if CONTROL_APPROACH == 1 or CONTROL_APPROACH == 3:
		rospy.wait_for_service(joint_str + "_pos_pid_set_gains")
		pid_set_gains = rospy.ServiceProxy(joint_str + "_pos_pid_set_gains", PIDSetGains)
		try:
			set_gains_resp = pid_set_gains(kp_pos,kd_pos,ki_pos)

			# confirm response via sum
			if set_gains_resp.sum == int(kp_pos+kd_pos+ki_pos):
				print("pid gain checksum confirmed")
			else:
				print("Invalid response from PIDSetGains service")
				sys.exit(1)
		except rospy.ServiceException as set_gains_exception:
			print(str(set_gains_exception))

		rospy.wait_for_service(joint_str + "_pos_pid_compute")
		pid_compute = rospy.ServiceProxy(joint_str + "_pos_pid_compute", PIDCompute)

	# set up velocity pid service proxies
	if CONTROL_APPROACH == 2 or CONTROL_APPROACH == 3:
		rospy.wait_for_service(joint_str + "_vel_pid_set_gains")
		pid_set_gains = rospy.ServiceProxy(joint_str + "_vel_pid_set_gains", PIDSetGains)
		try:
			set_gains_resp = pid_set_gains(kp_vel,kd_vel,ki_vel)

			# confirm response via sum
			if set_gains_resp.sum == int(kp_vel+kd_vel+ki_vel):
				print("pid gain checksum confirmed")
			else:
				print("Invalid response from PIDSetGains service")
				sys.exit(1)
		except rospy.ServiceException as set_gains_exception:
			print(str(set_gains_exception))

		rospy.wait_for_service(joint_str + "_vel_pid_compute")
		pid_compute = rospy.ServiceProxy(joint_str + "_vel_pid_compute", PIDCompute)


	# set up publisher to write desired PWM to each motor (subscriber established in motor/read_write_node)
	set_motor_pwm_pub = rospy.Publisher('set_motor_pwm', SetMotorPWM, queue_size=10)

	if CONTROL_APPROACH == 1 or CONTROL_APPROACH == 3:
		set_motor_position_pub = rospy.Publisher('set_motor_position', SetMotorPosition, queue_size=10)
		target_start_position = SetMotorPosition()

		time.sleep(1)
		target_start_position.id = joint_id_dict[joint_str]
		target_start_position.position = 450
		set_motor_position_pub.publish(target_start_position)

	rate = rospy.Rate(10) # Hz
	# rospy.spin()
	time.sleep(1)

	if CONTROL_APPROACH == 1 or CONTROL_APPROACH == 3:
		# print current position of the motor and prompt user for desired position
		get_motor_position_resp = get_motor_position(joint_id_dict[joint_str])
		print("current motor position (encoder space): " + str(get_motor_position_resp.position))

		desired_motor_position = int(input("enter desired motor position (12-bit):\n"))
		check_motor_position_bounds(desired_motor_position)
	elif CONTROL_APPROACH == 2:
		# prompt user for desired velocity
		desired_motor_velocity = int(input("enter desired motor velocity (10-bit):\n"))
		check_motor_position_bounds(desired_motor_velocity)
	

	while not rospy.is_shutdown():
		if CONTROL_APPROACH == 1:
			desired_motor_pwm.pwm = control_motor_position(desired_motor_position) + ff_pwm
		elif CONTROL_APPROACH == 2:
			desired_motor_pwm.pwm = control_motor_velocity(desired_motor_velocity) + ff_pwm
		elif CONTROL_APPROACH == 3:
			desired_motor_pwm.pwm = cascade_control(desired_motor_position) + ff_pwm

		set_motor_pwm_pub.publish(desired_motor_pwm)

		# sleep enough to maintain desired rate
		rate.sleep()

