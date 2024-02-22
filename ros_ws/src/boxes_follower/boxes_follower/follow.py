import rclpy
import time 
import numpy as np
#import random
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

alpha = 0.9
beta = 1.0 - alpha


class AckermannPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.tget_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
class BoxSubscriber(Node): 
    def __init__(self): 
        super().__init__('box_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, 'boxes', self.listener_callback, 10)
        self.curr = 0 
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
   
    def listener_callback(self, msg): 
        global alpha, beta
        #self.get_logger().info('Fron Bounding_boxes: "%s"' % msg.data)
        
        curr_dist = self.curr
        new_dist, idx = pixel_to_max_distance(msg.data)
        #print(new_height)   
        avg_dist = (alpha * curr_dist) + (beta * new_dist)
        self.curr = avg_dist
        
        x_coords = msg.data[0::3]
        x = x_coords[idx]
        
        print(avg_dist, new_dist, x)
        	
   	
def pixel_to_max_distance(lst):
    heights = lst[2::3]
    if heights:
        distance_list = [calculate_distance(height) for height in heights]
        #print(distance_list)
        max_dist = max(distance_list)
        idx = distance_list.index(max_dist)
                
        return max_dist, idx
    else:
        return []

def calculate_distance(h):
    return (0.18415 / np.tan(h * 1.02 / 2)) # convert feet to meters for SLAM4
    
def main(args=None):
    rclpy.init(args=args)
    
    box_subscriber = BoxSubscriber()
    
    rclpy.spin(box_subscriber)
    
    box_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
