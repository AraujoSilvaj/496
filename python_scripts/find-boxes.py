import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from keras.preprocessing import image
from keras.applications.vgg16 import preprocess_input, decode_predictions
import time

labelling_model = load_model("../nn-files/model-2023-12-24-13_00-320x240-55549.keras")
predictor_model = load_model("../nn-files/model-detector-2023-12-24-13_01-320x240-55549.keras")
#img_path = "../img-capture/parking-lot-right-2023-11-10-214.jpg"
#img = image.load_img(img_path, target_size=(240, 320))

c = cv2.VideoCapture(0)
c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
c.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
c.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


def showBox(im, boxes):
  # Box is:
	print(boxes)
	(height, width, depth) = im.shape
	image = im.copy()
	for box in boxes:
    # A box is a triple (x,y, height), but box may be a few, so loop over all
    		for bi in range(int(len(box)/3)):
      			[x, y, box_height] = box[(bi*3):((bi+1)*3)]
      			image = cv2.rectangle(image,
          		(int((x- 0.05) * width), int((y - box_height/2)* height)),
          		(int((x + 0.05) * width), int((y + box_height/2) * height)),
          		(255,0,0), 1)

	return image

count = 0

while(True):
	ret, img = c.read()
	#img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	#img_arr = image.img_to_array(img)
	img_arr= np.asarray(img).astype("float32")	
	img_arr /= 255.0
	img_arr = np.expand_dims(img_arr, axis=0)	
	#img_arr = preprocess_input(img_arr)
	predictions = predictor_model.predict(img_arr)	

	print("Prediction:", predictions)
	
	if (predictions > 0.5):
		boxes = labelling_model.predict(img_arr)
		label_box = showBox(img, boxes)
	else:
		label_box = img

#x, y, height = predictions[0]	
	cv2.imwrite(f"image{count}.jpg",label_box)
	count += count
	time.sleep(.2)
	





#print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
