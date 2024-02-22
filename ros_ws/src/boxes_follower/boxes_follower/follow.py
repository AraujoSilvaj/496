import rclpy
import time 
#import random
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
class BoxSubscriber(Node): 
    def __init__(self): 
    self.seubscription = self.create_subscription(Float32MultiArray, boxes, self.listener_callback, 10)
    
    def listener_callback(self, msg): 
        self.get_logger().info('' % msg.data)
