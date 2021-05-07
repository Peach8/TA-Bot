#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

def callback(data):
	# used to convert between ROS and OpenCV images
	br = CvBridge()

	# convert ROS image to OpenCV image
	current_frame = br.imgmsg_to_cv2(data)
	# make copy for post-processing
	img = current_frame.copy()

	## detect robot base and corner of paper

	cv2.imshow('feature_detection', img)

	## after click SPACEBAR, save base frame and paper corner pixel coords for conversion
	key = cv2.waitKey(1)
	if key%256 == 32: # SPACEBAR
		rospy.loginfo('detected features') # save detected features


def detect_features():
	rospy.init_node('workspace_feature_detection', anonymous=True)
	rospy.Subscriber('/perception/video_frames', Image, callback)

	#pub.publish
	rospy.spin()

if __name__ == '__main__':
	pub = rospy.Publisher('/perception/paper_corner_robot_frame', Pose2D, callback)

	try:
		detect_features()
	except rospy.ROSInterruptException:
		pass