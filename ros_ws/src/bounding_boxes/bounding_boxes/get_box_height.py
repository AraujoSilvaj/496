import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

total_h_avg = []
total_x_avg = []
final_h_avg = []
final_x_avg = []

num_readings = 100
laser_frequency = 40

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
		global total_h_avg, total_x_avg
		global final_h_avg, final_x_avg
		data = list(msg.data)
		x_avg, h_avg = get_averages(data)
		print("height avg: ", h_avg)
		'''
		if (len(total_h_avg) < 10):
			total_h_avg.append(h_avg)
			print(total_h_avg)	
		
		if (len(total_h_avg) == 10):
			height_avg = sum(total_h_avg)/len(total_h_avg)
			print("Total Height average:", height_avg)
			total_h_avg = []
			final_h_avg.append(height_avg)
			print(final_h_avg)
			if(len(final_h_avg) == 10):
				h = sum(final_h_avg)/len(final_h_avg)
				print("Final height average:", h)
				final_h_avg = []
		
		if (len(total_x_avg) < 10):
			total_x_avg.append(x_avg)	
		
		if (len(total_x_avg) == 10):
			x_avg = sum(total_x_avg)/len(total_x_avg)
			#print("Total X coord average:", x_avg)
			total_x_avg = []
			final_x_avg.append(x_avg)
			if(len(final_x_avg) == 10):
				x = sum(final_x_avg)/len(final_x_avg)
				print("Final x average:", x)
				final_x_avg = []'''

#TODO get actual angle of vision
# convert actual angle to rads
# 0 is at center of camera
# convert x coord to 
# convert bb height to distance
# update num_readings and laser_frequency
# range_min,max units -> 
# intensities = 
# ranges = 
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
		scan.angle_min = -1.57
		scan.angle_max = 1.57
		scan.angle_increment = 3.14/num_readings
		scan.time_increment = (1.0 / laser_frequency) / num_readings
		scan.range_min = 0.0
		scan.range_max = 100.0
		
		scan.ranges = []
		scan.intensities = []
		
		for i in range(0, num_readings):
			scan.ranges.append(1.0*self.count)		#fake data
			scan.intensities.append(1)		#fake data
			
		self.publisher.publish(scan)
		print("Laserscan:")
		print(scan)
		
		self.count +=1


def main(args=None):
	rclpy.init(args=args)
	
	box_subscriber = BoxSubscriber()
	laser_scan_publisher = LaserScanPublisher()
	
	#rclpy.spin(box_subscriber)
	rclpy.spin(laser_scan_publisher)
	
	
	
	
	box_subscriber.destroy_node()
	laser_scan_publisher.destroy_node()
	rclpy.shutdown()
	

def get_averages(lst):
	heights = lst[2::3]
	x_coords = lst[0::3]
	
	if heights:
		h_avg = sum(heights) / len(heights)
	
	else:
		return None
	
	if x_coords:
		x_avg = sum(x_coords) / len(x_coords)
		
	
		
	return x_avg, h_avg	
	
if __name__ == '__main__':
	main()
	

