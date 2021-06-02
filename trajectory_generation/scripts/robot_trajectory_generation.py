#!/usr/bin/env python3
import rospy # Python library for ROS
from geometry_msgs.msg import Point
import numpy as np
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(currentdir)
sys.path.append(parentdir)
from include import workspace_conversions
from trajectory_generation.msg import *

num_traj_files = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"]

global traj_pub
traj_to_pub = Trajectory2D()  
global ready_to_pub
ready_to_pub = False

manual_correction_x = 0.0035
manual_correction_y = 0.0035

base_frame_offset_x_mm = 0
base_frame_offset_y_mm = 146

global sparsify_threshold
sparsify_threshold = 0.002

traj_width_pixel = 100
traj_height_pixel = 250
traj_width_global = 50 #workspace_conversions.convert_pixels_to_mm(traj_width_pixel)
# traj_height_global  = workspace_conversions.convert_pixels_to_mm(traj_height_pixel)

def callback(data):
	global ready_to_pub

	if data.x != 0.0: # only take action after SPACEBAR click
		# prompt user for desired number
		num_to_write = int(input("Enter double-digit number\n"))
		first_digit = int(num_to_write/10)
		second_digit = num_to_write%10
		
		first_num_traj = np.loadtxt("/home/alex/catkin_ws/src/TA-Bot/trajectory_generation/scripts/" + num_traj_files[first_digit] + ".txt")
		second_num_traj = np.loadtxt("/home/alex/catkin_ws/src/TA-Bot/trajectory_generation/scripts/" + num_traj_files[second_digit] + ".txt")

		# - convert paper corner coords from pixels to mm
		paper_fiducial_global_x = workspace_conversions.convert_pixels_to_mm(data.x)
		paper_fiducial_global_y = workspace_conversions.convert_pixels_to_mm(data.y) + base_frame_offset_y_mm

		print("paper fiducial global x: " + str(paper_fiducial_global_x))
		print("paper fiducial global y: " + str(paper_fiducial_global_y))

		num_trajs = [first_num_traj, second_num_traj]
		full_traj_1 = []
		full_traj_2 = []

		for i in range(2):
			# everything from now on is in global fram
			# - offset paper corner to start pos for first number
			traj_start_corner_x = paper_fiducial_global_x - (2-i)*traj_width_global
			traj_start_corner_y = paper_fiducial_global_y

			print("digit " + str(i) + " traj_start_corner_x" + str(traj_start_corner_x))
			print("digit " + str(i) + " traj_start_corner_y" + str(traj_start_corner_y))

			# - add desired number trajectory global coords starting at previous offset
			# -- loop through each trajectory point and offset based on pixel frame orientation and
			#    append to trajectory msg
			for local_point in num_trajs[i]:
				global_point = Point()
				global_point.x = (traj_start_corner_x + local_point[0]) / 1000.0
				global_point.y = (traj_start_corner_y - local_point[1]) / 1000.0

				global_point.x += manual_correction_x
				global_point.y -= manual_correction_y
				
				if i == 0:
					full_traj_1.append(global_point)
				elif i == 1:
					full_traj_2.append(global_point)

		sparse_1 = [full_traj_1[0]]
		sparse_2 = [full_traj_2[0]]
		#if the distance between current global point and previous global point is greater than the sparsify threshold...
		for point in full_traj_1: # we have to start with the second point (aka not zero)
			
			if np.linalg.norm([point.x-sparse_1[-1].x, point.y-sparse_1[-1].y])>sparsify_threshold:
				#...then append the point
				sparse_1.append(point)

		for point in full_traj_2: # we have to start with the second point (aka not zero)
		
			if np.linalg.norm([point.x-sparse_2[-1].x, point.y-sparse_2[-1].y])>sparsify_threshold:
				#...then append the point
				sparse_2.append(point)

		
		traj_to_pub.trajectory1 = sparse_1
		traj_to_pub.trajectory2 = sparse_2

		ready_to_pub = True


def generate_trajectory():
	global ready_to_pub

	rate = rospy.Rate(60) # 10Hz

	while not rospy.is_shutdown():
		if ready_to_pub:
			traj_pub.publish(traj_to_pub)
			ready_to_pub = False # only pub once

		# sleep enough to maintain desired rate
		rate.sleep()


if __name__ == '__main__':
	rospy.init_node('robot_trajectory_generation', anonymous=True)
	rospy.Subscriber('/perception/paper_fiducial_pixel_coords', Point, callback, queue_size=10)
	traj_pub = rospy.Publisher("desired_number_trajectory", Trajectory2D, queue_size=10)

	try:
		generate_trajectory()
	except rospy.ROSInterruptException:
		pass