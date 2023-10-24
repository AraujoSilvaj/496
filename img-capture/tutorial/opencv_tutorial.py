#import opencv fro computer vision stuff
import cv2

#---------------------------------access the cam------------------------------
'''
cap = cv2.VideoCapture(0)
#get a frame from the capture device
ret, frame = cap.read()

#check if camera is there
print(ret)

#release capture back into the wild
cap.release()
'''
#---------------------------------take an image-------------------------------
'''
def take_photo():
	cap = cv2.VideoCapture(0)
	ret, frame = cap.read()
	cv2.imwrite('cap-tutorial_1.png', frame)
	cap.release()

take_photo()
'''

#--------------------------rendering in real time-----------------------------
cap = cv2.VideoCapture(0)

#loop through every frame until webcam is closed
while cap.isOpened():
	ret, frame = cap.read()

	#show image
	cv2.imshow('webcam', frame)

	#checks whether q has been hit and stops the loop
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()
