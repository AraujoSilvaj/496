import cv2
import os
import time

directory = '/496/images'

c = cv2.VideoCapture(0)
c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

os.chdir(directory)

i = 0
ret, prev_frame = c.read()

while True:
	ret, frame = c.read()
	#set images to same color
	prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
	current_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	#absolute difference between the two frames
	abs_diff = cv2.absdiff(prev_gray, current_gray)

	#set a threshold for the absolute difference
	threshold = 30
	_, thresholded_diff = cv2.threshold(abs_diff, threshold, 255, cv2.THRESH_BINARY)

	if cv2.countNonZero(thresholded_diff) > 10000:
		print(f"different frame --> {cv2.countNonZero(thresholded_diff)}")
		name = os.popen("echo -n `date +\"%Y-%m-%d-%H:%M:%S%N\"`").read()
		filename = str(name) + "-" + str(i) + ".jpg"
		cv2.imwrite(filename, frame)
		print(filename)
		i = i + 1
	else:
		print(f'Same frame -->{cv2.countNonZero(thresholded_diff)}')
	time.sleep(0.5)
	prev_frame = frame
