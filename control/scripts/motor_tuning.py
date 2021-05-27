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

global actual_motor_position
global desired_motor_position
global desired_motor_pwm

# nodes to launch before rosrunning this node
# - motor read_write node that includes all serial port stuff

# change the gains and re-run node
kp = 1.0
kd = 0.0
ki = 0.0

# change when switching motors to tune
joint_str = "joint3"

# TODO: reassign these if necessary
joint_id_dict = {"joint1": 0, "joint2": 1, "joint3": 2}

# global variables to be updated
actual_motor_position = 0
desired_motor_position = 0
desired_motor_pwm = SetMotorPWM()
desired_motor_pwm.id = joint_id_dict[joint_str]

bag = rosbag.Bag('actual_position.bag', 'w')

def control_motor():
	global actual_motor_position
	global desired_motor_position
	global desired_motor_pwm

	get_motor_position_resp = get_motor_position(joint_id_dict[joint_str])
	actual_motor_position = get_motor_position_resp.position

	position_error = desired_motor_position-actual_motor_position

	pid_compute_resp = pid_compute(position_error)

	desired_motor_pwm.pwm = pid_compute_resp.output
	print("desired_motor_pwm: " + str(desired_motor_pwm.pwm))


def process_kill_handler(sig, frame):
	print("closing bag...")
	bag.close() # close rosbag on CTRL+C
	sys.exit(0)

if __name__ == '__main__':
	signal.signal(signal.SIGINT, process_kill_handler)

	rospy.init_node("motor_tuning", anonymous=True)

	# set up motor encoder service proxy (service established in motor/read_write_node)
	rospy.wait_for_service('get_position')
	get_motor_position = rospy.ServiceProxy('get_position', GetMotorPosition)

	# set up pid service proxies
	rospy.wait_for_service(joint_str + "_pid_set_gains")
	pid_set_gains = rospy.ServiceProxy(joint_str + "_pid_set_gains", PIDSetGains)
	try:
		set_gains_resp = pid_set_gains(kp,kd,ki)

		# confirm response via sum
		if set_gains_resp.sum == int(kp+kd+ki):
			print("pid gain checksum confirmed")
		else:
			print("Invalid response from PIDSetGains service")
			sys.exit(1)
	except rospy.ServiceException as set_gains_exception:
		print(str(set_gains_exception))

	rospy.wait_for_service(joint_str + "_pid_compute")
	pid_compute = rospy.ServiceProxy(joint_str + "_pid_compute", PIDCompute)

	# set up publisher to write desired PWM to each motor (subscriber established in motor/read_write_node)
	pwm_pub = rospy.Publisher('set_PWM', SetMotorPWM, queue_size=10)

	motor_writer = rospy.Publisher('set_position', SetPosition, queue_size=10)
	target_pos = SetPosition()

	time.sleep(1)
	target_pos.id = joint_id_dict[joint_str]
	target_pos.position = 450
	motor_writer.publish(target_pos)

	rate = rospy.Rate(10) # Hz
	# rospy.spin()
	time.sleep(1)

	# print current position of the motor and prompt user for desired position
	get_motor_position_resp = get_motor_position(joint_id_dict[joint_str])
	print("current motor position: " + str(get_motor_position_resp.position))
	
	desired_motor_position = int(input("enter desired motor position:\n"))

	if desired_motor_position < 450 or desired_motor_position > 925:
		print("desired position out of bounds")
		sys.exit(0)


	while not rospy.is_shutdown():
		control_motor()
		pwm_pub.publish(desired_motor_pwm)
		# rospy.loginfo(actual_motor_position)
		bag_data = Int32()
		bag_data.data = actual_motor_position
		bag.write('actual_motor_position', bag_data)

		# sleep enough to maintain desired rate
		rate.sleep()

