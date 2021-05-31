#!/usr/bin/env python3
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
from include import workspace_conversions
import numpy as np

zero_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/zero.txt')
one_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/one.txt')
two_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/two.txt')
three_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/three.txt')
four_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/four.txt')
five_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/five.txt')
six_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/six.txt')
seven_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/seven.txt')
eight_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/eight.txt')
nine_pixel_coords = np.loadtxt('./number_trajectory_pixel_coords/nine.txt')

pixel_coords_arrs = [zero_pixel_coords, one_pixel_coords, two_pixel_coords, \
					 three_pixel_coords, four_pixel_coords, five_pixel_coords, \
					 six_pixel_coords, seven_pixel_coords, eight_pixel_coords, nine_pixel_coords]

global_coords_arrs = []
for pixel_arr in pixel_coords_arrs:
	global_coords_arrs.append(workspace_conversions.convert_num_traj_pixels_to_mm(pixel_arr))

np.savetxt('./number_trajectory_global_coords/zero.txt', global_coords_arrs[0])
np.savetxt('./number_trajectory_global_coords/one.txt', global_coords_arrs[1])
np.savetxt('./number_trajectory_global_coords/two.txt', global_coords_arrs[2])
np.savetxt('./number_trajectory_global_coords/three.txt', global_coords_arrs[3])
np.savetxt('./number_trajectory_global_coords/four.txt', global_coords_arrs[4])
np.savetxt('./number_trajectory_global_coords/five.txt', global_coords_arrs[5])
np.savetxt('./number_trajectory_global_coords/six.txt', global_coords_arrs[6])
np.savetxt('./number_trajectory_global_coords/seven.txt', global_coords_arrs[7])
np.savetxt('./number_trajectory_global_coords/eight.txt', global_coords_arrs[8])
np.savetxt('./number_trajectory_global_coords/nine.txt', global_coords_arrs[9])