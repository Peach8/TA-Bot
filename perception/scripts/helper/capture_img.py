#!/usr/bin/env python3

# open webcam feed and capture image on SPACEBAR click
import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

while True:
	ret, frame = cap.read()
	cv2.imshow("webcam", frame)

	key = cv2.waitKey(1)
	if key%256 == 27: # ESC
		break
	elif key%256 == 32: # SPACEBAR
		cv2.imwrite("./capture.jpg", frame)

cap.release()
cv2.destroyAllWindows()
