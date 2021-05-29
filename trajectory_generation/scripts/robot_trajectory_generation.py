#!/usr/bin/env python3
import rospy # Python library for ROS
from geometry_msgs.msg import Point
import numpy as np
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from include import workspace_conversions

trajectory = Trajectory2D()

traj_width_pixel = 200
traj_height_pixel = 250
traj_width_global = workspace_conversions.convert_pixels_to_mm(traj_width_pixel)
traj_height_global = workspace_conversions.convert_pixels_to_mm(traj_height_pixel)


def callback(data):
	if data.x != 0.0: # only take action after SPACEBAR click
		# - convert paper corner coords from pixels to mm
		paper_fiducial_global_x = workspace_conversions.convert_pixels_to_mm(data.x)
		paper_fiducial_global_y = workspace_conversions.convert_pixels_to_mm(data.y)
		# disregard theta for now

		# everything from now on is in global frame
		# - offset paper corner to start pos for first number
		traj_start = Point()
		# TODO: update offset sign with change in base frame orientation
		traj_start_corner.x = paper_fiducial_global_x - traj_width_global
		traj_start_corner.y = paper_fiducial_global_y

		# - add desired number trajectory global coords starting at previous offset
		# -- loop through each trajectory point and offset based on pixel frame orientation and
		#    append to trajectory msg

def generate_trajectory():
	rate = rospy.Rate(10) # 10Hz

	while not rospy.is_shutdown():
		# pub.publish()

		# sleep enough to maintain desired rate
		rate.sleep()


if __name__ == '__main__':
	rospy.init_node('robot_trajectory_generation', anonymous=True)
	rospy.Subscriber('/perception/paper_fiducial_pixel_coords', Point, callback, queue_size=1)
	# TODO: add subscriber from mission control for desired numbers to draw
	# pub = rospy.Publisher

	try:
		generate_trajectory()	
	except rospy.ROSInterruptException:
		pass