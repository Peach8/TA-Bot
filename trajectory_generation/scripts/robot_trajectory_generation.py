#!/usr/bin/env python3
import rospy # Python library for ROS
from geometry_msgs.msg import Pose2D
import numpy as np
import sys
sys.path.insert(0, './include/')
from include import workspace_conversions


paper_corner_global_pose = Pose2D() # mm coords of paper corner

def callback(data):
	if data.x != 0.0: # only take action after SPACEBAR click
		# - convert paper corner coords from pixels to mm
		paper_corner_global_pose.x = workspace_conversions.convert_pixels_to_mm(data.x)
		paper_corner_global_pose.y = workspace_conversions.convert_pixels_to_mm(data.y)
		# disregard theta for now

		# - offset paper corner to start pos for first number

		# - transform desired number trajectory to mm (TODO: save each offline)
		#   and apply each to previous offset result


def generate_trajectory():
	rate = rospy.Rate(10) # 10Hz

	while not rospy.is_shutdown():
		# pub.publish()

		# sleep enough to maintain desired rate
		rate.sleep()


if __name__ == '__main__':
	rospy.init_node('robot_trajectory_generation', anonymous=True)
	rospy.Subscriber('/perception/paper_corner_pixel_coords', Pose2D, callback, queue_size=1)
	# TODO: add subscriber from mission control for desired numbers to draw
	# pub = rospy.Publisher

	try:
		generate_trajectory()	
	except rospy.ROSInterruptException:
		pass