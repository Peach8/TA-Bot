#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Point
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np


fiducials_detected = False
area_thresh = 100 # TODO: tune on 80/20 setup
paper_fiducial_pixel_pose = Point() # data to be published (pixel coords of paper corner)

def callback(data):
	# used to convert between ROS and OpenCV images
	br = CvBridge()

	# convert ROS image to OpenCV image
	current_frame = br.imgmsg_to_cv2(data)
	# make copy for fiducial detection
	img = current_frame.copy()

	## Detect robot base and top-right corner via green fiducial sticker
	# - define hsv range corresponding to color green
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	green_mask = cv2.inRange(hsv, (40, 30, 30), (70, 255, 255))
	
	# - generate image that is black except for detected green regions
	imask = green_mask>0
	green = np.zeros_like(img, np.uint8)
	green[imask] = img[imask]
	
	# - use two largest detected contours to define circle around green regions
	gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY) # contour detection requries grayscale
	contours,_ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	areas = [cv2.contourArea(c) for c in contours]
	sorted_areas = np.sort(areas)
	large_areas = sorted_areas[sorted_areas > area_thresh]
	if large_areas.size:
		if len(large_areas) == 2:
			fiducials_detected = True

			cnt1 = contours[areas.index(large_areas[-1])] # save largest contour
			cnt2 = contours[areas.index(large_areas[-2])] # save 2nd largest contour

			# - build minimum enclosing circles based on the detected contours
			(x1,y1),radius1 = cv2.minEnclosingCircle(cnt1)
			(x2,y2),radius2 = cv2.minEnclosingCircle(cnt2)
			radius1 = int(radius1)
			radius2 = int(radius2)
		
			# - differentiate robot base fiducial from paper  fiducial
			if x1>x2: # TODO: change condition based on 80/20 setup
				paper_coords = (int(x1), int(y1))
				base_coords = (int(x2), int(y2))
			else:
				paper_coords = (int(x2), int(y2))
				base_coords = (int(x1), int(y1))

			# - draw results on top of original raw image
			cv2.circle(img, base_coords, radius1, (255,0,0), 2) # base in blue
			cv2.circle(img, paper_coords, radius2, (0,0,255), 2) # paper fiducial in red

			# - compute pixel coords of paper  fiducial st robot base fiducial is origin
			# -- this computation aligns with the base frame origin in our link diagram
			paper_offset_coords = (paper_coords[0]-base_coords[0], base_coords[1]-paper_coords[1])
		else:
			fiducials_detected = False
	else:
		fiducials_detected = False

	## Display final results
	cv2.imshow('feature_detection', img)

	paper_fiducial_pixel_pose.x = 0
	paper_fiducial_pixel_pose.y = 0

	## after click SPACEBAR, save base frame and paper fiducial pixel coords for conversion
	key = cv2.waitKey(1)
	if key%256 == 32: # SPACEBAR
		if fiducials_detected:
			# package Point
			paper_fiducial_pixel_pose.x = paper_offset_coords[0]
			paper_fiducial_pixel_pose.y = paper_offset_coords[1]


def detect_features():
	rate = rospy.Rate(10) # 10Hz

	while not rospy.is_shutdown():
		pub.publish(paper_fiducial_pixel_pose)

		# sleep enough to maintain desired rate
		rate.sleep()


if __name__ == '__main__':
	rospy.init_node('workspace_feature_detection', anonymous=True)
	rospy.Subscriber('/perception/video_frames', Image, callback, queue_size=1)
	pub = rospy.Publisher('/perception/paper_fiducial_pixel_coords', Point, queue_size=1)

	try:
		detect_features()	
	except rospy.ROSInterruptException:
		pass