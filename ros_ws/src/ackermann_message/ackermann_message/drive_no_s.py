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
            #print("STOP")
        
        ## Drive Course
        elif (2 < self.i < 12): # DRIVE STRAIGHT
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.004
            #print("Drive Forward 6.75 meters (22 ft)")
            
        elif (11 <= self.i < 16): #turning left
            msg.drive.steering_angle = 0.23
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            #print("First Left Turn")
            
        elif (16 <= self.i < 24): #Drive through Hoop
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            #print("Drive through hoop")
            
        elif (24 <= self.i < 26): #turning right after hoop
            msg.drive.steering_angle = -0.1875
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            #print("Turning Right after hoop")
            
        elif (26 <= self.i < 37): #Driving straight after hoop
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            #print("Driving straight after hoop") 
              
        elif (37 <= self.i < 42): #turn left second corner
            msg.drive.steering_angle = 0.178
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            #print("turn left second corner")
        
        elif (42 <= self.i < 57): #drive straight ramp
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            #print("Drive Straight Ramp")
            
        elif (57 <= self.i < 62): #turn left 3rd corner
            msg.drive.steering_angle = 0.169
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            #print("turn left 3rd corner")
            
        elif (62 <= self.i < 85): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            #print("Long Drive Straight")
            
        elif (85 <= self.i < 90): #turn left 4th corner
            msg.drive.steering_angle = 0.182
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            #print("turn left 4th corner")
            
        elif (90 <= self.i < 98): #finish line
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 3.0  
            msg.drive.acceleration = 0.5
            #print("Finish Line")
        
        """
        elif (104 <= self.i < 111): #turning left
            msg.drive.steering_angle = 0.1705
            msg.drive.speed = 3.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (111 <= self.i < 122): # finish line
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
    ackermann_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
