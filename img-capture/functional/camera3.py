import cv2
import os
import time

directory = '/home/robot/Pictures/left'
prev_frame = None
i = 0

c = cv2.VideoCapture(2)
c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

os.chdir(directory)



while True:
	ret, frame = c.read()
	if prev_frame is None or not (frame == prev_frame).all():		
		name = os.popen("echo -n `date +\"%Y-%m-%d-%H:%M:%S%N\"`").read()
		filename = str(name) + "-L" + str(i) + ".jpg"
		cv2.imwrite(filename, frame)
		print(filename)
		time.sleep(0.5)
		i = i + 1
	prev_frame = frame.copy()
