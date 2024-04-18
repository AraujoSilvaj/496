import rclpy
import time 
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.subscription = self.create_subscription(Bool, 'button_state', self.listener_callback, 10)
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
       
        
        if (self.i == 0):
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 0.0
            msg.drive.acceleration = 0.0
            print("STOP")   
           
        ## Drive Course
        elif (20 < self.i < 32): # DRIVE STRAIGHT
            msg.drive.steering_angle = -0.0325
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.005
            print("Drive Forward 6.75 meters (22 ft)")
            
        elif (32 <= self.i < 38): #turning left
            msg.drive.steering_angle = 0.165
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
            
        elif (38 <= self.i < 60): #long drive straight
            msg.drive.steering_angle = -0.01
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")

        elif (60 <= self.i < 66): #turning left
            msg.drive.steering_angle = 0.15
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")

        elif (66 <= self.i < 81): #med drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 11.25m (37 ft)")

        elif (81 <= self.i < 87): #turning left
            msg.drive.steering_angle = 0.115
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")

        elif (87 <= self.i < 111): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (111 <= self.i < 117): #turning left
            msg.drive.steering_angle = 0.16
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (117 <= self.i < 135): # finish line
            msg.drive.steering_angle = -0.0325
            msg.drive.speed = 3.0
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
