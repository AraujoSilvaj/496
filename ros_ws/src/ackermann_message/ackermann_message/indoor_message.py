import rclpy
import time 
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

#button = True

class AckermannPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        #self.subscription = self.create_subscription(Bool, 'button_state', self.listener_callback, 10)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def listener_callback(self, button_state):
        #global button
        if  not button_state.data:
            self.i = 0
        #return

    def timer_callback(self):
        #global button
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        ### TODO
        ## RECALIBRATE SERVO MIN MAX FOR FULL RANGE OF MOTION
        
        if (self.i == 0):
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 0.0
            msg.drive.acceleration = 0.0
            print("STOP")
        
        ## WIGGLE
        elif (2 < self.i < 6): # short driving straight 
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
        elif (22 < self.i < 32): # DRIVE STRAIGHT
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.1
            print("Drive Forward 6.75 meters (22 ft)")
            
        elif (32 <= self.i < 42): #turning left
            msg.drive.steering_angle = 0.1725
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
            
        elif (42 <= self.i < 52): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (52 <= self.i < 62): #turning left
            msg.drive.steering_angle = 0.175
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (62 <= self.i < 72): #med drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 11.25m (37 ft)")
        elif (72 <= self.i < 82): #turning left
            msg.drive.steering_angle = 0.175
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (82 <= self.i < 92): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (92 <= self.i < 102): #turning left
            msg.drive.steering_angle = 0.1705
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (102 <= self.i < 112): # finish line
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.1
            print("Drive Forward 3 meter (10 ft)")
            print("YOU MADE IT TO THE FINISH LINE")
        
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing Ackermann Drive Command: "%s"' % msg)
        self.i += 1
       
def main(args=None):
    rclpy.init(args=args)
    ackermann_publisher = AckermannPublisher()
    rclpy.spin(ackermann_publisher)
    ackermann_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
