import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

distance_list = []
angle_list = []

num_readings = 50  # number of readings in the laser scan, within our field of view
laser_frequency = 1 # measured in Hz, will need to update if using GPU instead of CPU

class BoxSubscriber(Node):

	def __init__(self):
		super().__init__('box_subscriber')
		self.subscription = self.create_subscription(
			Float32MultiArray,
			'boxes',
			self.listener_callback,
			50)
		self.publisher = self.create_publisher(LaserScan, 'b_scan', 1)
		#self.subscription
	
	
# range_min,max units -> needs to be meters for SLAM
# intensities = device specific, discarded for now
# ranges = measured distances
	
	def listener_callback(self, msg):
		
		global distance_list, angle_list
		
		data = list(msg.data)
		distances = pixel_to_distance(data)
		angles = x_to_rads(data)
		#print('distances: ', distances)
		#print('angles: ', angles)
		current_time = self.get_clock().now().to_msg()
		scan = LaserScan()
		scan.header.stamp = current_time
		scan.header.frame_id = 'laser_frame'
		scan.angle_min = -0.681
		scan.angle_max = 0.681
		scan.angle_increment = 1.362/num_readings
		scan.time_increment = (1.0 / laser_frequency) / num_readings
		scan.range_min = 0.5  # meters
		scan.range_max = 100.0 # meters
		
		scan.ranges = [scan.range_max] * num_readings
		scan.intensities = [1.0] * num_readings  # Filling with constant value 1
		

			
		for i in range(len(angles)):
			angle = angles[i]
			# Find the corresponding distance for each angle and append to scan.ranges
			if scan.angle_min <= angle <= scan.angle_max:
				# Calculate the index corresponding to the angle in scan.ranges
				index = int((angle - scan.angle_min) / scan.angle_increment)
				# Ensure the index is within bounds
				index = min(max(0, index), num_readings - 1)
				print("Angle: ",angle)
				print("Distance: ", distances[i])
				print("Index: ", index)
				scan.ranges[index] = distances[i]
				'''
        			# Use the index to set the corresponding distance
				if index < len(distances):
					scan.ranges[index] = distances[i]
				else:
					# Set to max range if index is out of bounds
					scan.ranges[index] = scan.range_max  
					'''

		self.publisher.publish(scan)
		print("Laserscan: ")
		print(scan.ranges)
		
def main(args=None):
	rclpy.init(args=args)

	box_subscriber = BoxSubscriber()
	rclpy.spin(box_subscriber)
	
	box_subscriber.destroy_node()
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
	return (0.3683 / np.tan(h * 1.02 / 2)) / 3.281 # convert feet to meters for SLAM
	
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
	

