import numpy as np
from math import sqrt, pi
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __repr__(self):
        return f"Point({self.x}, {self.y})"
        
class Vector:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        
    def __repr__(self):
        return f"Vector({self.x}, {self.y})"  
        
    def magnitude(self):
        return sqrt(self.x**2 + self.y**2)
    
    def normalize(self):
        mag = self.magnitude()
        return Vector(self.x/mag, self.y/mag)
        
class Ray:
    def __init__(self, point, direction):
        self.point = point
        self.direction = direction.normalize()
        
    def __repr__(self):
        return f"Ray({self.point}, {self.direction})"
        
class Circle:
    def __init__(self, center):
        self.center = center
        self.radius = 0.152 # bucket radius in meters
        
    def intersect_ray(self, ray):
        U = Vector(self.center.x - ray.point.x, self.center.y - ray.point.y)
        U1 = ray.direction.x * U.x + ray.direction.y * U.y
        U2 = Vector(U.x - U1 * ray.direction.x, U.y - U1 * ray.direction.y)
        d = U2.magnitude()

        if d > self.radius:
            # No Intersection
            return None

        m = np.sqrt(self.radius**2 - d**2)

        P1 = Point(ray.point.x + U1 * ray.direction.x + m * ray.direction.x,
            ray.point.y + U1 * ray.direction.y + m * ray.direction.y)
        P2 = Point(ray.point.x + U1 * ray.direction.x - m * ray.direction.x,
            ray.point.y + U1 * ray.direction.y - m * ray.direction.y)

        # Return the closest intersection point
        dist1 = ((P1.x - ray.point.x)**2 + (P1.y - ray.point.y)**2)**0.5
        dist2 = ((P2.x - ray.point.x)**2 + (P2.y - ray.point.y)**2)**0.5

        if dist1 < dist2:
            return P1
        else:
            return P2

    def __repr__(self):
        return f"Circle({self.center}, {self.radius})"

distance_list = []
angle_list = []

num_readings = 100  # number of readings in the laser scan, within our field of view
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
		
		buckets = []
		for i in range(len(angles)):
		    angle = angles[i]
		    distance = distances[i]
		    x = distance * np.cos(angle)
		    y = distance * np.sin(angle)
		    buckets.append(Circle(Point(x,y)))
		
		current_time = self.get_clock().now().to_msg()
		scan = LaserScan()
		scan.header.stamp = current_time
		scan.header.frame_id = 'laser_frame'
		scan.angle_min = -0.681
		scan.angle_max = 0.681
		scan.angle_increment = 1.362/num_readings
		scan.time_increment = (1.0 / laser_frequency) / num_readings
		scan.range_min = 0.1  # meters
		scan.range_max = 30.0 # meters
		
		scan.ranges = [scan.range_max - 0.1] * num_readings
		scan.intensities = [1.0] * num_readings  # Filling with constant value 1
		

        # Cast rays and populate scan.ranges with intersection points			
		for i in range(num_readings):
			angle = scan.angle_min + i * scan.angle_increment
			ray = Ray(Point(0,0), Vector(np.cos(angle), np.sin(angle)))
			distance = scan.range_max - 0.1
			for bucket in buckets:
				#print(ray)
				intersection_point = bucket.intersect_ray(ray)
				if intersection_point:
					distance = sqrt((intersection_point.x - ray.point.x)**2 + (intersection_point.y - ray.point.y)**2)
			
			scan.ranges[i] = distance

		self.publisher.publish(scan)
		#print("Laserscan: ")
		#print(scan.ranges)
		
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
		#print(distance_list)
		return distance_list
	else:
		return []

def calculate_distance(h):
	return (0.18415 / np.tan(h * 1.02 / 2)) # in meters for SLAM
	
def x_to_rads(lst):
	x_coords = lst[0::3]
	if x_coords:
		angle_list = [calculate_angle(x) for x in x_coords]
		#print(angle_list)
		return(angle_list)
	else:
		return []
		
def calculate_angle(x):
	return 1.3628*x - 0.681

if __name__ == '__main__':
	main()
	
