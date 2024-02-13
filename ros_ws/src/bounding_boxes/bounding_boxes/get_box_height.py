import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

distance_list = []
angle_list = []

num_readings = 100  # number of readings in the laser scan, within our field of view
laser_frequency = 4 # measured in Hz, will need to update if using GPU instead of CPU

class BoxSubscriber(Node):

	def __init__(self):
		super().__init__('box_subscriber')
		self.subscription = self.create_subscription(
			Float32MultiArray,
			'boxes',
			self.listener_callback,
			10)
		#self.subscription
		
	def listener_callback(self, msg):
		
		global distance_list, angle_list
		
		data = list(msg.data)
		distances = pixel_to_distance(data)
		angles = x_to_rads(data)
		print('distances: ', distances)
		print('angles: ', angles)
		

#TODO
# range_min,max units -> same as distances 'measured'
# intensities = device specific, discarded for now
# ranges = measured distances
class LaserScanPublisher(Node):
	
	def __init__(self):
		super().__init__('laser_scan_publisher')
		self.publisher = self.create_publisher(LaserScan, 'b_scan', 1)
		timer_period = 0.5 
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.count = 0
		print('Initialized LaserScan Publisher')
		
	def timer_callback(self):
		print('timer callback')
		current_time = self.get_clock().now().to_msg()
		print('current time: ', current_time)
		scan = LaserScan()
		scan.header.stamp = current_time
		scan.header.frame_id = 'laser_frame'
		scan.angle_min = -0.681
		scan.angle_max = 0.681
		scan.angle_increment = 3.14/num_readings
		scan.time_increment = (1.0 / laser_frequency) / num_readings
		scan.range_min = 2.0  # feet
		scan.range_max = 60.0 # feet
		
		scan.ranges = []
		scan.intensities = [1] * num_readings  # Filling with constant value 1
		
		for i in range(0, num_readings):
			scan.ranges.append(1.0*self.count)		#fake data
			
		self.publisher.publish(scan)
		print("Laserscan:")
		print(scan)
		
		self.count +=1


def main(args=None):
	rclpy.init(args=args)
	
	box_subscriber = BoxSubscriber()
	laser_scan_publisher = LaserScanPublisher()
	
	rclpy.spin(box_subscriber)
	#rclpy.spin(laser_scan_publisher)
	
	box_subscriber.destroy_node()
	laser_scan_publisher.destroy_node()
	rclpy.shutdown()	

def pixel_to_distance(lst):
	heights = lst[2::3]
	if heights:
		distance_list = [calculate_distance(height) for height in heights]
		print(distance_list)
		return distance_list
	else:
		return []

def calculate_distance(h):
	return 0.6 / np.tan(h * 1.02 / 2)
	
def x_to_rads(lst):
	x_coords = lst[0::3]
	if x_coords:
		angle_list = [calculate_angle(x) for x in x_coords]
		print(angle_list)
		return(angle_list)
	else:
		return []
		
def calculate_angle(x):
	return 1.3628*x - 0.681

if __name__ == '__main__':
	main()
	

