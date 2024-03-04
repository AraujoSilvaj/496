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
        
        ### TODO
        ## RECALIBRATE SERVO MIN MAX FOR FULL RANGE OF MOTION
        
        ## WIGGLE
        
        if (2 < self.i < 6): # short driving straight 
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 0.5
            msg.drive.acceleration = 0.5
            print("WIGGLE")
        elif (8 < self.i < 12): # short driving back
            msg.drive.steering_angle = 0.0
            msg.drive.speed = -0.5
            msg.drive.acceleration = 0.5
            print("WIGGLE")
        elif (12 < self.i < 17): # turn left (dont move)
            msg.drive.steering_angle = 0.2
            msg.drive.speed = 0.0
            msg.drive.acceleration = 0.5
            print("WIGGLE")
        elif (17 < self.i < 22): # turn right (dont move)
            msg.drive.steering_angle = -0.2
            msg.drive.speed = 0.0
            msg.drive.acceleration = 0.5
            print("WIGGLE")
            
        
        #elif (22 < self.i < 27): # driving 2 meter for test
            #msg.drive.steering_angle = 0.0
            #msg.drive.speed = 2.0
            #msg.drive.acceleration = 1.0
            #print("TEST: DRIVE FORWARD 2 METERS")    
            
           
        ## Drive Course
        elif (22 < self.i < 31): # DRIVE STRAIGHT
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.1
            print("Drive Forward 6.75 meters (22 ft)")
            
        elif (31 <= self.i < 37): #turning left
            msg.drive.steering_angle = 0.175
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
            
        elif (37 <= self.i < 57): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif ( 57 <= self.i < 63): #turning left
            msg.drive.steering_angle = 0.164
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (63 <= self.i < 78): #med drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 11.25m (37 ft)")
        elif (78 <= self.i < 84): #turning left
            msg.drive.steering_angle = 0.164
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (84 <= self.i < 104): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (104 <= self.i < 110): #turning left
            msg.drive.steering_angle = 0.164
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (110 <= self.i < 116): # finish line
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.1
            print("Drive Forward 3 meter (10 ft)")
            print("YOU MADE IT TO THE FINISH LINE")
        """
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

        
