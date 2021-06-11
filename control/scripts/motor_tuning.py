#!/usr/bin/env python3
import rospy
import rosbag
import sys
import signal
from std_msgs.msg import Int32, Float32
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
global pen_pos_up


desired_motor_pwm = BulkSetPWM()
desired_motor_pwm.id1 = 0
desired_motor_pwm.id2 = 1
desired_motor_pwm.id3 = 2
desired_motor_pwm.value1 = 0
desired_motor_pwm.value2 = 0
desired_motor_pwm.value3 = 0
# desired_motor_pwm.id = joint_id_dict[joint_str]

# Specify initial motor position
initial_motor_position = 2000
# change the gains and re-run node
# - joint1 position control gains
# kp_pos1 = 2.0
# kd_pos1 = 0.0
# - joint1 position control gains
kp1_u = 1.25
kd1_u = 0.05
ki1_u = 1.5
kp1_d = 6.0
kd1_d = 0.1
ki1_d = 0.0
ff_pwm1 = 0.0 # TODO: @Oph
joint1_pos_pid_gains = [kp1_u, kd1_u, ki1_u, kp1_d, kd1_d, ki1_d]


# - joint2 position control gains
kp2_u = 1.25
kd2_u = 0.05
ki2_u = 1.5
kp2_d = 4.5
kd2_d = 0.1
ki2_d = 2.0
ff_pwm2 = 0.0 # TODO: @Oph
joint2_pos_pid_gains = [kp2_u, kd2_u, ki2_u, kp2_d, kd2_d, ki2_d]

# kp_pos3 = 1.0
# kd_pos3 = 0.0
# ki_pos3 = 0.0
# ff_pwm3 = 0.0
# joint3_pos_pid_gains = [kp_pos3, kd_pos3, ki_pos3]

joint_pos_pid_gains = [joint1_pos_pid_gains, joint2_pos_pid_gains]

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

min_motor_position = 990
max_motor_position = 3525

max_motor_velocity = 1023 # signed 10-bit int with units 0.229 rpm/bit


if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
	bag = rosbag.Bag('actual_position.bag', 'w')
elif motor_controller is ControlApproach.VELOCITY:
	bag = rosbag.Bag('actual_velocity.bag', 'w')
global bag_data
bag_pos = Int32()
bag_error = Int32()
bag_pwm = Int32()

def pen_down():
	global pen_pos_up

	print('\nPEN DOWN...\n')
	pen_pos_up = False
	
	stop_motors()
	time.sleep(.1)

	pen_down_msg = SetMotorPosition()
	pen_down_msg.id = 2
	pen_down_msg.position = 1275
	set_motor_position_pub.publish(pen_down_msg)
	time.sleep(2)


def pen_up():
	global pen_pos_up

	print('\nPEN UP...\n')
	pen_pos_up = True
	
	stop_motors()
	time.sleep(.1)
	
	pen_up_msg = SetMotorPosition()
	pen_up_msg.id = 2
	pen_up_msg.position = 1050
	set_motor_position_pub.publish(pen_up_msg)
	time.sleep(2)

def stop_motors():
	# write zero pwm
	# print("stopping motors...")
	desired_motor_pwm.value1 = 0
	desired_motor_pwm.value2 = 0
	desired_motor_pwm.value3 = 0
	set_motor_pwm_pub.publish(desired_motor_pwm)

def control_motor_position(desired_motor_position):
	global ff_pwm
	global bag_data
	global actual_motor_position
	global pen_pos_up

	get_motor_position_resp = get_motor_position()
	if joint_id_dict[joint_str] == 0:
		actual_motor_position = get_motor_position_resp.value1
	elif joint_id_dict[joint_str] == 1:
		actual_motor_position = get_motor_position_resp.value2
	elif joint_id_dict[joint_str] == 2:
		actual_motor_position = get_motor_position_resp.value3

	print(f'actual motor position: {actual_motor_position}')

	position_error = desired_motor_position - actual_motor_position

	pid_compute_resp = pid_compute(position_error, pen_pos_up)

	bag_pos.data = actual_motor_position
	bag_error.data = position_error
	bag_pwm.data = pid_compute_resp.output

	bag.write('actual_encoder1', bag_pos)	
	bag.write('m1_pwm', bag_pwm)	
	bag.write('error_enc1', bag_error)

	return pid_compute_resp.output


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
	stop_motors()
	sys.exit(0)

if __name__ == '__main__':
	signal.signal(signal.SIGINT, process_kill_handler)

	rospy.init_node("motor_tuning", anonymous=True)

	# set up motor position and velocity service proxies (established in motor/read_write_node)
	rospy.wait_for_service('bulk_get_position')
	get_motor_position = rospy.ServiceProxy('bulk_get_position', BulkGet)

	set_motor_position_pub = rospy.Publisher('set_motor_position', SetMotorPosition, queue_size=10)
	# rospy.wait_for_service('get_motor_velocity')
	# get_motor_velocity = rospy.ServiceProxy('get_motor_velocity, GetMotorVelocity')
	
	# set up publisher to write desired PWM to each motor (subscriber established in motor/read_write_node)
	set_motor_pwm_pub = rospy.Publisher('bulk_set_pwm', BulkSetPWM, queue_size=10)
	pen_up()

	# set up position pid service proxies
	if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
		# rospy.wait_for_service(joint_str + "_pos_pid_set_gains")
		rospy.wait_for_service("pid_set_gains1")
		# pid_set_gains = rospy.ServiceProxy(joint_str + "_pos_pid_set_gains", PIDSetGains)
		pid_set_gains = rospy.ServiceProxy("pid_set_gains1", PIDSetGains)
		try:
			set_gains_resp = pid_set_gains(*joint_pos_pid_gains[joint_id_dict[joint_str]])

			# confirm response via sum
			if set_gains_resp.sum == int(sum(joint_pos_pid_gains[joint_id_dict[joint_str]])):
				print(f"pid gain checksum confirmed: {set_gains_resp.sum}")

			else:
				print("Invalid response from PIDSetGains service")
				sys.exit(1)
		except rospy.ServiceException as set_gains_exception:
			print(str(set_gains_exception))

		# rospy.wait_for_service(joint_str + "_pos_pid_compute")
		# pid_compute = rospy.ServiceProxy(joint_str + "_pos_pid_compute", PIDCompute)
		rospy.wait_for_service("pid_compute1")
		pid_compute = rospy.ServiceProxy("pid_compute1", PIDCompute)

	# set up velocity pid service proxies
	if motor_controller is ControlApproach.VELOCITY or motor_controller is ControlApproach.CASCADE:
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

	if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
		set_motor_position_pub = rospy.Publisher('set_motor_position', SetMotorPosition, queue_size=10)
		target_start_position = SetMotorPosition()

		time.sleep(1)
		target_start_position.id = joint_id_dict[joint_str]
		target_start_position.position = initial_motor_position
		set_motor_position_pub.publish(target_start_position)
		time.sleep(1)
		pen_down()

	rate = rospy.Rate(100) # Hz
	# rospy.spin()
	# time.sleep(1)

	if motor_controller is ControlApproach.POSITION or motor_controller is ControlApproach.CASCADE:
		# print current position of the motor and prompt user for desired position
		get_motor_position_resp = get_motor_position()
		if joint_id_dict[joint_str] == 0:
			print("current motor position (encoder space): " + str(get_motor_position_resp.value1))
		elif joint_id_dict[joint_str] == 1:
			print("current motor position (encoder space): " + str(get_motor_position_resp.value2))
		elif joint_id_dict[joint_str] == 2:
			print("current motor position (encoder space): " + str(get_motor_position_resp.value3))


		desired_motor_position = int(input("enter desired motor position (12-bit):\n"))
		check_motor_position_bounds(desired_motor_position)
	elif motor_controller is ControlApproach.VELOCITY:
		# prompt user for desired velocity
		desired_motor_velocity = int(input("enter desired motor velocity (10-bit):\n"))
		check_motor_position_bounds(desired_motor_velocity)
	
	while not rospy.is_shutdown():

		if motor_controller is ControlApproach.POSITION:
			if joint_id_dict[joint_str] == 0:
				desired_motor_pwm.value1 = control_motor_position(desired_motor_position) + ff_pwm
			elif joint_id_dict[joint_str] == 1:
				desired_motor_pwm.value2 = control_motor_position(desired_motor_position) + ff_pwm
			elif joint_id_dict[joint_str] == 2:
				desired_motor_pwm.value3 = control_motor_position(desired_motor_position) + ff_pwm

		elif motor_controller is ControlApproach.VELOCITY:
			if joint_id_dict[joint_str] == 0:
				desired_motor_pwm.value1 = control_motor_velocity(desired_motor_velocity) + ff_pwm
			elif joint_id_dict[joint_str] == 1:
				desired_motor_pwm.value2 = control_motor_velocity(desired_motor_velocity) + ff_pwm
			elif joint_id_dict[joint_str] == 2:
				desired_motor_pwm.value3 = control_motor_velocity(desired_motor_velocity) + ff_pwm

		elif motor_controller is ControlApproach.CASCADE:
			if joint_id_dict[joint_str] == 0:
				desired_motor_pwm.value1 = cascade_control(desired_motor_position) + ff_pwm
			elif joint_id_dict[joint_str] == 1:
				desired_motor_pwm.value2 = cascade_control(desired_motor_position) + ff_pwm
			elif joint_id_dict[joint_str] == 2:
				desired_motor_pwm.value3 = cascade_control(desired_motor_position) + ff_pwm

		print(desired_motor_pwm)
		set_motor_pwm_pub.publish(desired_motor_pwm)

		# sleep enough to maintain desired rate
		rate.sleep()

