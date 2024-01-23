import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from keras.preprocessing import image
from keras.applications.vgg16 import preprocess_input, decode_predictions

labelling_model = load_model("../nn-files/model-2023-12-24-13_00-320x240-55549.keras")
predictor_model = load_model("../nn-files/model-detector-2023-12-24-13_01-320x240-55549.keras")
#img_path = "../img-capture/parking-lot-right-2023-11-10-214.jpg"
#img = image.load_img(img_path, target_size=(240, 320))

c = cv2.VideoCapture(0)
c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
c.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
c.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while(True):
	ret, img = c.read()
	img_arr = image.img_to_array(img)
	img_arr = np.expand_dims(img_arr, axis=0)
	img_arr = preprocess_input(img_arr)
	predictions = predictor_model.predict(img_arr)

	print(predictions)
	if (predictions == 1):
		boxes = labelling_model.predict(img_arr)
#x, y, height = predictions[0]
		print(boxes)

#print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
