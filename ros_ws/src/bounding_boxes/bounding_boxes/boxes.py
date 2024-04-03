import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from keras.preprocessing import image
from keras.applications.vgg16 import preprocess_input, decode_predictions
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
#import ros2py

labelling_model = load_model("/496/nn-files/model-2024-01-29-00_32-320x240-59671.keras")
predictor_model = load_model("/496/nn-files/model-detector-2023-12-24-13_01-320x240-55549.keras")
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

def nn_predictions():
	boxes = np.array([])
	ret, img = c.read()
	#img= cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	#img_arr = image.img_to_array(img)
	img_arr= np.asarray(img).astype("float16")  # THIS MAY NEED TO CHANGE	
	img_arr /= 255.0
	img_arr = np.expand_dims(img_arr, axis=0)	
	#img_arr = preprocess_input(img_arr)
	predictions = predictor_model.predict(img_arr)	

	 #print("Prediction:", predictions)
	
	if (predictions > 0.5):
		boxes = labelling_model.predict(img_arr)
		labeled_box = showBox(img, boxes)
		return (boxes[0], labeled_box)

	else:
                # No buckets found
		return (boxes, img)
	
	#box1 = boxes.tolist()

	return boxes

	#x, y, height = predictions[0]	
	'''cv2.imwrite(f"image{count}.jpg",label_box)
	count += count
	time.sleep(.2)'''




#print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
	#msg = nn_predictions()
        self.publisher_ = self.create_publisher(Float32MultiArray, 'boxes', 1)
        self.im_publisher_ = self.create_publisher(Image, 'label_boxes', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        (predict, image) = nn_predictions() 

        # TODO publish image as a topic
        # processes image data and converts to ros 2 message
        msg = Image()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'ANI717'
        msg.height = np.shape(image)[0]
        msg.width = np.shape(image)[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = np.shape(image)[2] * np.shape(image)[1]
        msg.data = np.array(image).tobytes()
        self.im_publisher_.publish(msg)
                
        # Publish the bounding boxes
        msg = Float32MultiArray()
        temp = predict.astype('float16').tolist()
        msg.data = list(map(float, temp))

        #print(msg.data)
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
