import rclpy
import time 
#import random
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped


#count = 0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if (self.i < 30): #driving straight
            msg.drive.steering_angle = 0.0  
            msg.drive.speed = 0.25  
            msg.drive.acceleration = 0.5
            print("Drive Forward")
        #elif (self.i < 30): #turning right
            #msg.drive.steering_angle = -0.26 
            #msg.drive.speed = 0.25  
            #msg.drive.acceleration = 0.5
            #print("Drive Forward")
        elif (self.i > 30):
            #self.i = 0
            msg.drive.steering_angle = 0.15  
            msg.drive.speed = 0.0  
            msg.drive.acceleration = 0.5
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing Ackermann Drive Command: "%s"' % msg)
        self.i += 1
        
        

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

