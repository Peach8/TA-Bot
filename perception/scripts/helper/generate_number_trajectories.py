#!/usr/bin/env python3
import sys
import cv2 # OpenCV library
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from sklearn.neighbors import NearestNeighbors
import networkx as nx


def generate_trajectories():

	img = cv2.imread('./number_images/zero/zero_2.jpg')

	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	thresh = 50
	thresh, out = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)

	cv2.imshow(" ", out)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	# find pixel coordinates for black line
	coords = np.column_stack(np.where(out < thresh))
	coords[:, [0, 1]] = coords[:, [1, 0]]

	# resort coordinates based on nearest neighbor to help piece together trajectory
	clf = NearestNeighbors(2).fit(coords)
	G = clf.kneighbors_graph()

	T = nx.from_scipy_sparse_matrix(G)

	order = list(nx.dfs_preorder_nodes(T,0))	

	x = coords[:,0]
	y = coords[:,1]

	xx = x[order]
	yy = y[order]

	coords = np.c_[xx,yy]

	paths = [list(nx.dfs_preorder_nodes(T, i)) for i in range(len(coords))]

	mindist = np.inf
	minidx = 0

	for i in range(len(coords)):
		p = paths[i]
		ordered = coords[p]
		# deifine cost based on euclidean distance
		cost = (((ordered[:-1] - ordered[1:])**2).sum(1)).sum()
		if cost < mindist:
			mindist = cost
			minidx = i	

	opt_order = paths[minidx]

	xx = x[opt_order]
	yy = y[opt_order]

	traj = np.c_[xx,yy]

	print(traj)


if __name__ == '__main__':
	generate_trajectories()