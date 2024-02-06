import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class BoxSubscriber(Node):

	def __init__(self):
		super().__init__('box_subscriber')
		self.subscription = self.create_subscription(
			Float32MultiArray,
			'boxes',
			self.listener_callback,
			10)
		self.subscription
		
	def listener_callback(self, msg):
		data = list(msg.data)
		
		for i in range(length(data):
			
		
		print(list(msg.data))
		

def main(args=None):
	rclpy.init(args=args)
	
	box_subscriber = BoxSubscriber()
	
	rclpy.spin(box_subscriber)
	
	
	
	
	box_subscriber.destroy_node()
	rclpy.shutdown()
	
	
if __name__ == '__main__':
	main()
	

