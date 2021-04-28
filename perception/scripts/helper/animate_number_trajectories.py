#!/usr/bin/env python3
import sys
import cv2 # OpenCV library
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from PIL import Image


def animate_trajectories():
	traj = np.loadtxt("./trajectory_coords/zero.txt", dtype=int)
	width = 200
	height = 250

	gray = 255*np.ones((height,width))

	# initialize video writer
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	fps = 50
	out = cv2.VideoWriter('zero.avi', fourcc, fps, (width,height))

	for point in traj:
		gray[point[1], point[0]] = 0
		gray3 = cv2.merge([gray,gray,gray])		
		u8 = gray3.astype(np.uint8)
		out.write(u8)

	out.release()

if __name__ == '__main__':
	animate_trajectories()