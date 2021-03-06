#!/usr/bin/env python3
import rospy
import rosbag
from std_msgs.msg import Float32, Int32, Bool
import sys
import signal
import math
import time
from control.srv import *
from motor.srv import *
from motor.msg import *
from trajectory_generation.msg import *
import matplotlib.pyplot as plt
from numpy.linalg import norm
from datetime import datetime

global ready_to_go
ready_to_go = False
global desired_trajs
desired_trajs =[]

# define global variables
global actual_motor_encs
actual_motor_encs = [0,0]
global actual_xy
actual_xy = [0.0,0.0]
global pen_up_xy_epsilon
global pen_down_xy_epsilon
pen_up_xy_epsilon = 0.002
pen_down_xy_epsilon = 0.004

global motor_pwm_pub
global desired_motor_pwm
desired_motor_pwm = BulkSetPWM()
desired_motor_pwm.id1 = 0
desired_motor_pwm.id2 = 1
desired_motor_pwm.id3 = 2

global pen_pos_up

joint_names = ["joint1", "joint2", "joint3"]

# define constant motor properties
# add these offsets to motor values to convert to joint space
joint1_offset = math.pi # rad
joint2_offset = math.pi + (2.08 / 180 * math.pi) # rad
joint_offsets = [joint1_offset, joint2_offset]

# define min and max joint positions in radians
joint1_min = -math.pi/2       # rad
joint1_max = math.pi/2 # rad
joint2_min = -2.28 - (2.08 / 180 * math.pi)       # rad
joint2_max = 2.28 - (2.08 / 180 * math.pi) # rad
joint_limits = [[joint1_min,joint1_max], [joint2_min,joint2_max]]

# - joint1 position control gains
kp1_u = 1.25
kd1_u = 0.05
ki1_u = 1.5
kp1_d = 4.5
kd1_d = 0.1
ki1_d = 2.0
ff_pwm1 = 0 # TODO: @Oph
joint1_pid_gains = [kp1_u, kd1_u, ki1_u, kp1_d, kd1_d, ki1_d]

# - joint2 position control gains
kp2_u = 1.25
kd2_u = 0.05
ki2_u = 1.5
kp2_d = 4.5
kd2_d = 0.1
ki2_d = 2.0
ff_pwm2 = 0 # TODO: @Oph
joint2_pid_gains = [kp2_u, kd2_u, ki2_u, kp2_d, kd2_d, ki2_d]


# # - joint3 position control gains
# kp3 = 1.0
# kd3 = 0.0
# ki3 = 0.0
# ff_pwm3 = 0.0
# joint3_pid_gains = [kp3, kd3, ki3]

joint_pid_gains = [joint1_pid_gains, joint2_pid_gains]
joint_ffs = [ff_pwm1, ff_pwm2]

# bags
date = str(datetime.now())
data_bag = rosbag.Bag(f"run_data_{date}.bag", 'w')

desired_x_bag = Float32()
desired_y_bag = Float32()
desired_theta1 = Float32()
desired_theta2 = Float32()
desired_enc1 = Float32()
desired_enc2 = Float32()
error_enc1 = Float32()
error_enc2 = Float32()
pen_down_pos = Bool()
actual_x = Float32()
actual_y = Float32()
actual_enc1 =Int32()
actual_enc2 = Int32()
m1_pwm = Int32()
m2_pwm = Int32()

def clamp_joint_position(joint_idx, desired_joint_theta):
	lower_limit = joint_limits[joint_idx][0]
	upper_limit = joint_limits[joint_idx][1]

	if desired_joint_theta < lower_limit:
		desired_joint_theta = lower_limit
	elif desired_joint_theta > upper_limit:
		desired_joint_theta = upper_limit

	return desired_joint_theta


def control_motor_positions(desired_motor_encs):
	global desired_motor_pwm
	# global desired_motor_pwm1
	# global desired_motor_pwm2
	global actual_motor_encs
	global pen_pos_up

	bulk_pos_response = get_motor_position()
	# in encoder space [0-4095]
	actual_motor_encs = [bulk_pos_response.value1, bulk_pos_response.value2, bulk_pos_response.value3]

	desired_motor_pwm.value3 = 0

	for i in range(2):
		if actual_motor_encs[i] != 0:
			pid_compute_error = desired_motor_encs[i] - actual_motor_encs[i]
			if i == 0:
					pid_compute_resp = pid_compute1(pid_compute_error, pen_pos_up)
					if pen_pos_up:
						desired_motor_pwm.value1 = int(pid_compute_resp.output)
					else:
						desired_motor_pwm.value1 = int(pid_compute_resp.output + (joint_ffs[i]>0)*joint_ffs[i] - (joint_ffs[i]<0)*joint_ffs[i])
			elif i == 1:
					pid_compute_resp = pid_compute2(pid_compute_error, pen_pos_up)
					if pen_pos_up:
						desired_motor_pwm.value2 = int(pid_compute_resp.output)
					else:
						desired_motor_pwm.value2 = int(pid_compute_resp.output + (joint_ffs[i]>0)*joint_ffs[i] - (joint_ffs[i]<0)*joint_ffs[i])


def compute_actual_xy():
	global actual_motor_encs
	global actual_xy

	actual_joint_thetas = []
	# convert from motor encoder space all the way back to operational space
	for i in range(2):
		# map motor position in encoder space back to radians
		actual_motor_theta = actual_motor_encs[i] / 4095 * (2*math.pi)

		# convert actual from motor space to joint space
		actual_joint_thetas.append(actual_motor_theta - joint_offsets[i])

	fk_compute_resp = fk_compute(*actual_joint_thetas)

	actual_xy[0] = fk_compute_resp.x
	actual_xy[1] = fk_compute_resp.y


def pen_down():
	global pen_pos_up

	print('\nPEN DOWN...\n')
	pen_pos_up = False
	
	stop_motors()
	time.sleep(.1)

	pen_down_msg = SetMotorPosition()
	pen_down_msg.id = 2
	pen_down_msg.position = 1265
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
	pen_up_msg.position = 1100
	set_motor_position_pub.publish(pen_up_msg)
	time.sleep(2)


def process_kill_handler(sig, frame):
	print("closing bags...")
	data_bag.close() # close rosbag on CTRL+C
	return_home()
	stop_motors()
	sys.exit(0)


def return_home():
	global home_pos
	global desired_motor_pwm

	pen_up()

	print('\nRETURNING HOME...\n')

	desired_motor_encs = [1350, 3500]

	print("desired enc1: " + str(desired_motor_encs[0]))
	print("desired enc2: " + str(desired_motor_encs[1]))

	while True:
		control_motor_positions(desired_motor_encs)
		motor_pwm_pub.publish(desired_motor_pwm)

		print("actual_motor_encs1: " + str(actual_motor_encs[0]) + " / actual_motor_encs2: " + str(actual_motor_encs[1]))
		print("desired_motor_pwm1: " + str(desired_motor_pwm.value1)+ " / desired_motor_pwn2: " + str(desired_motor_pwm.value2))

		if norm([des-act for des,act in zip(desired_motor_encs,actual_motor_encs)]) <= 20: break



def stop_motors():
	# write zero pwm
	# print("stopping motors...")
	desired_motor_pwm.value1 = 0
	desired_motor_pwm.value2 = 0
	desired_motor_pwm.value3 = 0
	motor_pwm_pub.publish(desired_motor_pwm)
	time.sleep(1)


def trajectory_received_callback(data):
	global ready_to_go
	global desired_trajs
	ready_to_go = True
	desired_trajs.append(data.trajectory1)
	desired_trajs.append(data.trajectory2)
	# desired_traj_x = 0.209
	# desired_traj_y_start = 0.079
	# traj_step = 0.01
	# desired_traj_xy = []
	# for i in range(10):
	# 	desired_traj_xy.append([desired_traj_x, desired_traj_y_start + i*traj_step])	


if __name__ == '__main__':
	signal.signal(signal.SIGINT, process_kill_handler)

	rospy.init_node("mission_control", anonymous=True)

	rospy.Subscriber("desired_number_trajectory", Trajectory2D, trajectory_received_callback)
	# set up services/pubs/subs from highest to lowest level of control
	# - inverse kinematics
	rospy.wait_for_service("ik_compute")
	ik_compute = rospy.ServiceProxy("ik_compute", IKCompute)

	# - joint position pid control
	for i in range(1,3):
		rospy.wait_for_service("pid_set_gains" + str(i))
		pid_set_gains = rospy.ServiceProxy("pid_set_gains" + str(i), PIDSetGains)
		try:
			set_gains_resp = pid_set_gains(*joint_pid_gains[i-1])
			# confirm response via sum
			if set_gains_resp.sum == int(sum(joint_pid_gains[i-1])):
				print(f"{joint_names[i-1]}: pid gains checksum confirmed: {set_gains_resp}")
			else:
				print("Invalid response from PIDSetGains service")
				sys.exit(1)
		except rospy.ServiceException as set_gains_exception:
			print(str(set_gains_exception))

		rospy.wait_for_service("pid_compute" + str(i))
		if i == 1: pid_compute1 = rospy.ServiceProxy("pid_compute" + str(i), PIDCompute)
		elif i ==2: pid_compute2 = rospy.ServiceProxy("pid_compute" + str(i), PIDCompute)

	# - get actual motor position via encoder
	rospy.wait_for_service("bulk_get_position")
	get_motor_position = rospy.ServiceProxy('bulk_get_position', BulkGet)
	
	# - command desired motor pwm
	motor_pwm_pub = rospy.Publisher('bulk_set_pwm', BulkSetPWM, queue_size=10)
	
	# set up services/pub/subs that are separate from control
	rospy.wait_for_service("fk_compute")
	fk_compute = rospy.ServiceProxy("fk_compute", FKCompute)
	
	set_motor_position_pub = rospy.Publisher('set_motor_position', SetMotorPosition, queue_size=10) #add set motor position
	
	rate = rospy.Rate(60) # Hz

	pen_up()

	while not rospy.is_shutdown():
		# loop through desired trajectory instead of prompting user
		if ready_to_go:

			for i in range(len(desired_trajs)):

				desired_traj_xy = desired_trajs[i]
				#plot path
				# xs = []
				# ys = []
				# for j in range(len(desired_traj_xy)):
				# 	xs.append(desired_traj_xy[j].x)
				# 	ys.append(desired_traj_xy[j].y)

				# fig, ax = plt.subplots()
				# ax.plot(xs, ys, marker='o')
				# plt.show()



				for j in range(len(desired_traj_xy)):

					desired_x = desired_traj_xy[j].x
					desired_y = desired_traj_xy[j].y

					print("desired x: " + str(desired_x))
					print("desired y: " + str(desired_y))
					desired_x_bag.data = desired_x
					desired_y_bag.data = desired_y

					data_bag.write('desired_point', desired_traj_xy[j])
					data_bag.write('desired_x', desired_x_bag)
					data_bag.write('desired_y', desired_y_bag)

					# compute corresponding desired joint positions in radians
					ik_compute_resp = ik_compute(desired_x, desired_y)
					if not ik_compute_resp.success:
						stop_motors()
						sys.exit(0)

					desired_joint_thetas = [ik_compute_resp.theta1, ik_compute_resp.theta2]

					print("desired theta1: " + str(desired_joint_thetas[0]))
					print("desired theta2: " + str(desired_joint_thetas[1]))
					desired_theta1.data = desired_joint_thetas[0]
					desired_theta2.data = desired_joint_thetas[1]
					data_bag.write('desired_theta1',desired_theta1)
					data_bag.write('desired_theta2',desired_theta2)

					desired_motor_encs = []
					for k in range(2):
						# clamp desired joint position (note this is in joint space, not motor space)
						# - just an additional safety feature (ik should confirm (x,y) is reachable)
						desired_joint_thetas[k] = clamp_joint_position(k, desired_joint_thetas[k])

						# convert desired from joint space to motor space
						desired_motor_theta = (desired_joint_thetas[k] + joint_offsets[k])

						# map motor position in radians to encoder space before starting pid control
						desired_motor_encs.append(desired_motor_theta / (2*math.pi) * 4095)

					print("desired enc1: " + str(desired_motor_encs[0]))
					print("desired enc2: " + str(desired_motor_encs[1]))
					desired_enc1.data = desired_motor_encs[0]
					desired_enc2.data = desired_motor_encs[1]
					data_bag.write('desired_encoder1',desired_enc1)
					data_bag.write('desired_encoder2',desired_enc2)

					counter = 0
					while True:
						control_motor_positions(desired_motor_encs)

						motor_pwm_pub.publish(desired_motor_pwm)

						compute_actual_xy()
						x_err = abs(desired_x - actual_xy[0])
						y_err = abs(desired_y - actual_xy[1])
						# print("x error: " + str(x_err))
						# print("y error: " + str(y_err))
						print(f"error: {norm([x_err,y_err])}")
						print("actual_motor_encs1: " + str(actual_motor_encs[0]) + " / actual_motor_encs2: " + str(actual_motor_encs[1]))
						print("desired_motor_pwm1: " + str(desired_motor_pwm.value1)+ " / desired_motor_pwn2: " + str(desired_motor_pwm.value2))

						actual_x.data = actual_xy[0]
						actual_y.data = actual_xy[1]
						actual_enc1.data = actual_motor_encs[0]
						actual_enc2.data = actual_motor_encs[1]
						m1_pwm.data = desired_motor_pwm.value1
						m2_pwm.data = desired_motor_pwm.value2
						error_enc1.data = actual_motor_encs[0] - desired_motor_encs[0]
						error_enc2.data = actual_motor_encs[1] - desired_motor_encs[1]
						pen_down_pos.data = not pen_pos_up

						data_bag.write('actual_x', actual_x)
						data_bag.write('actual_y', actual_y)
						data_bag.write('actual_encoder1', actual_enc1)
						data_bag.write('actual_encoder2', actual_enc2)
						data_bag.write('m1_pwm', m1_pwm)
						data_bag.write('m2_pwm', m2_pwm)
						data_bag.write('error_enc1', error_enc1)
						data_bag.write('error_enc2', error_enc2)
						data_bag.write('pen_down_pos', pen_down_pos)

						if pen_pos_up and counter >= 3:
							if norm([x_err,y_err]) <= pen_up_xy_epsilon: break
						elif pen_pos_up and counter < 3:
							if norm([x_err,y_err]) <= pen_up_xy_epsilon: counter = counter + 1
							else: counter = 0
						elif not pen_pos_up:
							if norm([x_err,y_err]) <= pen_down_xy_epsilon: break
						else:
							print('this should never print')

					rate.sleep()

					#pen placed down after reaching the first point
					if j == 0:
						# stop_motors()
						# time.sleep(1)
						pen_down()
				pen_up()
			
			print("TRAJECTORY COMPLETE")
			#pen_up()
			data_bag.close()
			return_home()
			stop_motors()
			sys.exit(0)

		else:
			pass
				
		rate.sleep()













