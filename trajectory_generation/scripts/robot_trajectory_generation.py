#!/usr/bin/env python3
import rospy # Python library for ROS
from geometry_msgs.msg import Pose2D
import numpy as np
import sys
sys.path.insert(0, './include/')
from include import workspace_conversions

def callback(data):
	if data.x != 0.0:
		rospy.loginfo(workspace_conversions.convert_pixels_to_mm(data.x))

def generate_trajectory():
	rate = rospy.Rate(10) # 10Hz

	while not rospy.is_shutdown():
		# pub.publish()

		# sleep enough to maintain desired rate
		rate.sleep()


if __name__ == '__main__':
	rospy.init_node('robot_trajectory_generation', anonymous=True)
	rospy.Subscriber('/perception/paper_corner_pixel_coords', Pose2D, callback, queue_size=1)
	rospy.spin()

	# pub = rospy.Publisher

	try:
		generate_trajectory()	
	except rospy.ROSInterruptException:
		pass