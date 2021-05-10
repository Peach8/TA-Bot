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

	## Detect robot base via green fiducial sticker
	# - define hsv range corresponding to color green
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	green_mask = cv2.inRange(hsv, (40, 30, 30), (70, 255, 255))
	
	# - generate image that is black except for detected green regions
	imask = green_mask>0
	green = np.zeros_like(img, np.uint8)
	green[imask] = img[imask]
	
	# - use largest detected contour to define circle around green region
	gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY) # contour detection requries grayscale
	contours,_ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	areas = [cv2.contourArea(c) for c in contours]
	sorted_areas = np.sort(areas)
	cnt = contours[areas.index(sorted_areas[-1])] # save the biggest contour

	# - build circle based on largest detected contour and minimium enclosing circle
	(x,y),radius = cv2.minEnclosingCircle(cnt)
	center = (int(x), int(y))
	radius = int(radius)
	# - draw result on top of original raw image
	cv2.circle(img, center, radius, (0,0,255), 2)

	## Detect top-right paper corner via Harris
	


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