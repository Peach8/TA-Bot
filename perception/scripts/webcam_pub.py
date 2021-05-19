#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
  
def publish_message():
 
	pub = rospy.Publisher('/perception/video_frames', Image, queue_size=10)
	 
	rospy.init_node('video_pub_py', anonymous=True)
	 
	rate = rospy.Rate(10) # 10Hz
	 
	cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
	 
	# used to convert between ROS and OpenCV images
	br = CvBridge()
 
	while not rospy.is_shutdown():
		ret, frame = cap.read()
		 
		if ret == True:
			pub.publish(br.cv2_to_imgmsg(frame))
             
		# sleep enough to maintain the desired rate
		rate.sleep()
         
if __name__ == '__main__':
	try:
		publish_message()
	except rospy.ROSInterruptException:
		pass