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
from enum import Enum

class ControlApproach(Enum):
	POSITION = 1
	VELOCITY = 2
	CASCADE = 3

motor_controller = ControlApproach.POSITION

# change when switching motors to tune
joint_str = "joint1"
joint_id_dict = {"joint1": 0, "joint2": 1, "joint3": 2}

global actual_motor_position
global actual_motor_velocity

desired_motor_pwm = SetMotorPWM()
desired_motor_pwm.id = joint_id_dict[joint_str]

# change the gains and re-run node
# - joint1 position control gains
kp_pos1 = 2.0
kd_pos1 = 0.1
ki_pos1 = 0.0
ff_pwm1 = 0.0 # TODO: @Oph
joint1_pos_pid_gains = [kp_pos1, kd_pos1, ki_pos1]

# - joint2 position control gains
kp_pos2 = 4.0
kd_pos2 = 0.1
ki_pos2 = 0.0
ff_pwm2 = 0.0 # TODO: @Oph
joint2_pos_pid_gains = [kp_pos2, kd_pos2, ki_pos2]

# - joint3 position control gains
kp_pos3 = 1.0
kd_pos3 = 0.0
ki_pos3 = 0.0
ff_pwm3 = 0.0
joint3_pos_pid_gains = [kp_pos3, kd_pos3, ki_pos3]

joint_pos_pid_gains = [joint1_pos_pid_gains, joint2_pos_pid_gains, joint3_pos_pid_gains]

# - joint1 velocity control gains
kp_vel1 = 2.0
kd_vel1 = 0.1
ki_vel1 = 0.0
ff_pwm1 = 0.0 # TODO: @Oph
joint1_vel_pid_gains = [kp_vel1, kd_vel1, ki_vel1]

# - joint2 velocity control gains
kp_vel2 = 4.0
kd_vel2 = 0.1
ki_vel2 = 0.0
ff_pwm2 = 0.0 # TODO: @Oph
joint2_vel_pid_gains = [kp_vel2, kd_vel2, ki_vel2]

# - joint3 velocity control gains
kp_vel3 = 1.0
kd_vel3 = 0.0
ki_vel3 = 0.0
ff_pwm3 = 0.0
joint3_vel_pid_gains = [kp_vel3, kd_vel3, ki_vel3]

joint_vel_pid_gains = [joint1_vel_pid_gains, joint2_vel_pid_gains, joint3_vel_pid_gains]

global ff_pwm
ff_pwm = 0 # int

min_motor_position = 450
max_motor_position = 925

max_motor_velocity = 1023 # signed 10-bit int with units 0.229 rpm/bit


if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
	bag = rosbag.Bag('actual_position.bag', 'w')
elif motor_controller is ControlApproach.VELOCITY:
	bag = rosbag.Bag('actual_velocity.bag', 'w')
global bag_data
bag_data = Int32()


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


def cascade_control(desired_motor_position):
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

	# rospy.wait_for_service('get_motor_velocity')
	# get_motor_velocity = rospy.ServiceProxy('get_motor_velocity, GetMotorVelocity')

	# set up position pid service proxies
	if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
		rospy.wait_for_service(joint_str + "_pos_pid_set_gains")
		pid_set_gains = rospy.ServiceProxy(joint_str + "_pos_pid_set_gains", PIDSetGains)
		try:
			set_gains_resp = pid_set_gains(*joint_pos_pid_gains[joint_id_dict[joint_str]])

			# confirm response via sum
			if set_gains_resp.sum == int(sum(joint_pos_pid_gains[joint_id_dict[joint_str]])):
				print("pid gain checksum confirmed")
			else:
				print("Invalid response from PIDSetGains service")
				sys.exit(1)
		except rospy.ServiceException as set_gains_exception:
			print(str(set_gains_exception))

		rospy.wait_for_service(joint_str + "_pos_pid_compute")
		pid_compute = rospy.ServiceProxy(joint_str + "_pos_pid_compute", PIDCompute)

	# set up velocity pid service proxies
	if motor_controller is ControlApproach.VELOCITY or motor_controller is ControlApproach.CASCADE:
		rospy.wait_for_service(joint_str + "_vel_pid_set_gains")
		pid_set_gains = rospy.ServiceProxy(joint_str + "_vel_pid_set_gains", PIDSetGains)
		try:
			set_gains_resp = pid_set_gains(*joint_vel_pid_gains[joint_id_dict[joint_str]])

			# confirm response via sum
			if set_gains_resp.sum == int(sum(joint_vel_pid_gains[joint_id_dict[joint_str]])):
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

	if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
		set_motor_position_pub = rospy.Publisher('set_motor_position', SetMotorPosition, queue_size=10)
		target_start_position = SetMotorPosition()

		time.sleep(1)
		target_start_position.id = joint_id_dict[joint_str]
		target_start_position.position = 450
		set_motor_position_pub.publish(target_start_position)

	rate = rospy.Rate(10) # Hz
	# rospy.spin()
	time.sleep(1)

	if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
		# print current position of the motor and prompt user for desired position
		get_motor_position_resp = get_motor_position(joint_id_dict[joint_str])
		print("current motor position (encoder space): " + str(get_motor_position_resp.position))

		desired_motor_position = int(input("enter desired motor position (12-bit):\n"))
		check_motor_position_bounds(desired_motor_position)
	elif motor_controller is ControlApproach.VELOCITY:
		# prompt user for desired velocity
		desired_motor_velocity = int(input("enter desired motor velocity (10-bit):\n"))
		check_motor_position_bounds(desired_motor_velocity)
	

	while not rospy.is_shutdown():
		if motor_controller is ControlApproach.POSITION:
			desired_motor_pwm.pwm = control_motor_position(desired_motor_position) + ff_pwm
		elif motor_controller is ControlApproach.VELOCITY:
			desired_motor_pwm.pwm = control_motor_velocity(desired_motor_velocity) + ff_pwm
		elif motor_controller is ControlApproach.CASCADE:
			desired_motor_pwm.pwm = cascade_control(desired_motor_position) + ff_pwm

		set_motor_pwm_pub.publish(desired_motor_pwm)

		# sleep enough to maintain desired rate
		rate.sleep()

