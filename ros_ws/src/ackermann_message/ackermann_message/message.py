import rclpy
import time 
#import random
from rclpy.node import Node

from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        if (self.i < 9): # short driving straight 
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.1
            print("Drive Forward 6.75 meter (22 ft)")
        elif (self.i < 15): #turning left
            msg.drive.steering_angle = 0.164
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (self.i < 35): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (self.i < 41): #turning left
            msg.drive.steering_angle = 0.164
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (self.i < 56): #med drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 11.25m (37 ft)")
        elif (self.i < 62): #turning left
            msg.drive.steering_angle = 0.164
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (self.i < 82): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (self.i < 88): #turning left
            msg.drive.steering_angle = 0.164
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (self.i < 92): # finish line
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.1
            print("Drive Forward 3 meter (10 ft)")
        
        
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing Ackermann Drive Command: "%s"' % msg)
        self.i += 1
       
def main(args=None):
    rclpy.init(args=args)

    ackermann_publisher = AckermannPublisher()

    rclpy.spin(ackermann_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        
