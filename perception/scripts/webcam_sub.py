#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
 
def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  # Make copy for post-processing
  img = current_frame.copy()

  # Convert to gray-scale before running corner-detection
  gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY);
  gray = np.float32(gray)
  # TODO: tune these threshold values for our specific setup
  dst = cv2.cornerHarris(gray,2,3,0.04)

  # Dilate just to help with displaying results
  dst = cv2.dilate(dst,None)
  img[dst>0.01*dst.max()] = [0,0,255]

  # Display image
  cv2.imshow("corners", img)
   
  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()