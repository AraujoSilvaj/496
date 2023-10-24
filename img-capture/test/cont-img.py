import cv2
import os

directory = '/home/robot/Pictures/test_images3'
cap = cv2.VideoCapture(0)
i = 1;

while cap.isOpened():
	ret, frame = cap.read()

	os.chdir(directory)

	filename = str(i) + '.jpg'
	i = i + 1

	cv2.imwrite(filename, frame)

	key = cv2.waitKey(1000)
	if key == 27:
		break
cap.release()
