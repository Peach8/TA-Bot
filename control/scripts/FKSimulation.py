import math
import matplotlib.pyplot as plt 

# TODO: update geometry params
L1 = 1
L2 = 1

# Thetas
number_q = 10

q_start = 0 # may have to include the motor offset
q_end = math.pi/2 #this will be our desired theta

q1 = []
q2 = []

for i in range(0,number_q):
	tmp = q_start + i*(q_end - q_start)/(number_q-1)
	q1.append(tmp)
	q2.append(tmp)

	#Initialize Coordinates
	x0 = 0
	y0 = 0 

	# Loop Creating 100 values
	ct = 1
	for t1 in q1:
		for t2 in q2:
			x1 = L1 * math.cos(t1)
			y1 = L1 * math.sin(t1)

			x2 = x1 + L2 * math.cos(t2)
			y2 = y1 + L2 * math.sin(t2)

			filename = str(ct) + '.png'
			ct = ct+1
			plt.figure
			plt.plot([x0,x1], [y0,y1])
			plt.plot([x1,x2], [y1,y2])
			plt.xlim([0,2])
			plt.ylim([0,2])
			plt.savefig(filename)