#!/usr/bin/env python3
import rospy
import rosbag
from std_msgs.msg import Float32
import sys
import signal
import math
import time
from control.srv import *
from motor.srv import *

# define global variables
global actual_motor_encs
actual_motor_encs = [0,0]

global desired_motor_pwm1 # int
global desired_motor_pwm2 # int
desired_motor_pwm1 = SetMotorPWM()
desired_motor_pwm2 = SetMotorPWM()
desired_motor_pwms = [desired_motor_pwm1, desired_motor_pwm2]

# define constant motor properties
# add these offsets to motor values to convert to joint space
joint1_offset = math.pi # rad
joint2_offset = math.pi # rad
joint_offsets = [joint1_offset, joint2_offset]

# define min and max joint positions in radians
joint1_min = -math.pi/2       # rad
joint1_max = math.pi/2 # rad
joint2_min = -2.28       # rad
joint2_max = 2.28 # rad
joint_limits = [[joint1_min,joint1_max], [joint2_min,joint2_max]]

# - joint1 position control gains
kp1 = 2.0
kd1 = 0.1
ki1 = 0.0
ff_pwm1 = 0.0 # TODO: @Oph
joint1_pid_gains = [kp1, kd1, ki1]

# - joint2 position control gains
kp2 = 4.0
kd2 = 0.1
ki2 = 0.0
ff_pwm2 = 0.0 # TODO: @Oph
joint2_pid_gains = [kp2, kd2, ki2]

# - joint3 position control gains
kp = 1.0
kd = 0.0
ki = 0.0
ff_pwm3 = 0.0
joint3_pid_gains = [kp3, kd3, ki3]

joint_pid_gains = [joint1_pid_gains, joint2_pid_gains, joint3_pid_gains]
joint_ffs = [ff_pwm1, ff_pwm2, ff_pwm3]

# bags
actual_xy_bag = rosbag.Bag("actual_xy.bag", 'w')

def clamp_joint_position(joint_idx, desired_joint_theta):
	lower_limit = joint_limits[joint_idx,0]
	upper_limit = joint_limits[joint_idx,1]

	if desired_joint_theta < lower_limit:
		desired_joint_theta = lower_limit
	elif desired_joint_theta > upper_limit:
		desired_joint_theta = upper_limit

	return desired_joint_theta


def control_motor_positions(desired_motor_encs):
	global actual_motor_encs

	for i in range(2):
		get_motor_position_resp = get_motor_position(i)
		actual_motor_encs[i] = get_motor_position_resp.position # in encoder space [0-4095]

		position_error = desired_motor_encs[i] - actual_motor_encs[i]

		pid_compute_resp = pid_compute(position_error)

		desired_motor_pwms[i].id = i
		desired_motor_pwms[i].pwm = pid_compute_resp.output + joint_ffs[i]


def compute_actual_xy():
	global actual_motor_encs

	actual_joint_thetas = []
	# convert from motor encoder space all the way back to operational space
	for i in range(2):
		# map motor position in encoder space back to radians
		actual_motor_theta = actual_motor_encs[i] / 4095 * (2*math.pi)

		# convert actual from motor space to joint space
		actual_joint_thetas.append(actual_motor_theta + joint_offsets[i])

	fk_compute_resp = fk_compute(*actual_joint_thetas)
	
	actual_x = Float32()
	actual_x.data = fk_compute_resp.x
	actual_xy_bag.write('actual_x', actual_x)

	actual_y = Float32()
	actual_y.data = fk_compute_resp.y
	actual_xy_bag.write('actual_y', actual_y)


def process_kill_handler(sig, frame):
	print("closing bags...")
	actual_xy_bag.close() # close rosbag on CTRL+C
	sys.exit(0)


if __name__ == '__main__':
	signal.signal(signal.SIGINT, process_kill_handler)

	rospy.init_node("mission_control", anonymous=True)

	# set up services/pubs/subs from highest to lowest level of control
	# - inverse kinematics
	rospy.wait_for_service("ik_compute")
	ik_compute = rospy.ServiceProxy("ik_compute", IKCompute)

	# - joint position pid control
	for i in range(3):
		rospy.wait_for_service(joint_names[i] + "_pid_set_gains")
		pid_set_gains = rospy.ServiceProxy(joint_names[i] + "_pid_set_gains", PIDSetGains)
		try:
			set_gains_resp = pid_set_gains(*joint_pid_dict[joint_names[i]])
			# confirm response via sum
			if set_gains_resp.sum == int(sum(joint_pid_dict[joint_names[i]]))
				print(joint_names[i] + ": pid gains checksum confirmed")
			else:
				print("Invalid response from PIDSetGains service")
				sys.exit(1)
		except rospy.ServiceException as set_gains_exception:
			print(str(set_gains_exception))

		rospy.wait_for_service(joint_names[i] + "_pid_compute")
		pid_compute = rospy.ServiceProxy(joint_names[i] + "_pid_compute", PIDCompute)


	# - get actual motor position via encoder
	rospy.wait_for_service("get_motor_position")
	get_motor_position = rospy.ServiceProxy("get_motor_position", GetMotorPosition)

	# - command desired motor pwm
	motor_pwm_pub = rospy.Publisher("set_motor_pwm", SetMotorPWM, queue_size=10)

	# set up services/pub/subs that are separate from control
	rospy.wait_for_service("fk_compute")
	fk_compute = rospy.ServiceProxy("fk_compute", FKCompute)

	rate = rospy.Rate(10) # Hz

	# prompt user for desired (x,y) end-effector position in mm
	# - TODO: replace this with trajectory_generation output
	desired_x = float(input("Enter desired x position (mm):\n"))
	desired_y = float(input("Enter desired y position (mm):\n"))

	# convert mm to m before running ik
	desired_x = desired_x / 1000.0
	desired_y = desired_y / 1000.0

	# compute corresponding desired joint positions in radians
	ik_compute_resp = ik_compute(desired_x, desired_y)
	desired_joint_thetas = [ik_compute_resp.theta1, ik_compute_resp.theta2]
	
	desired_motor_encs = []
	for i in range(2):
		# clamp desired joint position (note this is in joint space, not motor space)
		# - just an additional safety feature (ik should confirm (x,y) is reachable)
		desired_joint_thetas[i] = clamp_joint_position(i, desired_joint_thetas[i])

		# convert desired from joint space to motor space
		desired_motor_theta = (desired_joint_thetas[i] - joint_offsets[i])

		# map motor position in radians to encoder space before starting pid control
		desired_motor_encs.append(desired_motor_theta / (2*math.pi) * 4095)

	while not rospy.is_shutdown():
		control_motor_positions(desired_motor_encs)

		for i in range(2):
			motor_pwm_pub.publish(desired_motor_pwms[i])

			# time.sleep(?) # TODO: might need to sleep between subsequent publishes (hopefully not)
			# - look into batch publisher @Alex

		compute_actual_xy()

		rate.sleep()













