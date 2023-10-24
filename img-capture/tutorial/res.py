import cv2
import os

directory = '/home/robot/Pictures/resolution_test'

def take_photo():
	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
	ret, frame = cap.read()
	os.chdir(directory)
	cv2.imwrite('FHD.jpg', frame)
	cap.release()

take_photo()
