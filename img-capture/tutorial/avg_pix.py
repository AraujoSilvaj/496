import cv2
import numpy as np
import mahotas
from pylab import imshow, show
import time

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
print(ret)

vid = frame[:, :, 0]
imshow(vid)
#show()
mean = vid.mean()
start = time.time()

for i in range(100):
	ret, frame = cap.read()
	vid = frame[:, :, 0]
	print(vid.mean())
	print(vid.shape)
	print(i)

elapsed = time.time() - start
print(elapsed/100)
cap.release()


