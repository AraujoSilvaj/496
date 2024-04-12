import rclpy
import time 
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
import math

class AckermannPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        #self.subscription = self.create_subscription(Bool, 'button_state', self.button_callback, 10)
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.turning = False
        self.map_index = 0
    
    def button_callback(self, button_state):
        if  not button_state.data:
            self.i = 0

    def imu_callback(self, imu_msg):
        #Extract the yaw angle from the imu message
        quaternion = imu_msg. orientation
        self.current_yaw = 2 * math.atan2(quaternion.z, quaternion.w)
        
        # Unwrap the yaw angle to handle sign changes
        if self.current_yaw < 0:
            self.current_yaw += 2 * math.pi
        
        # If we're not currently turning, set the target yaw to 90 degrees
        #if not self.turning:
            #self.target_yaw = self.current_yaw + 157 # 90 degrees in radians
            

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        print(f"Current Yaw: {self.current_yaw}")
        print(f"Target Yaw: {self.target_yaw}")
        print(f"Map Index: {self.map_index}")
        
        if (self.i == 0):
            msg.drive.steering_angle = 1.0
            msg.drive.speed = 0.0
            msg.drive.acceleration = 0.0
            print("STOP")
            
        
        elif (1 < self.i < 14 and (self.map_index == 0 or self.map_index == 2)): # DRIVE STRAIGHT
            msg.drive.steering_angle = -0.0525
            msg.drive.speed = 0.5
            msg.drive.acceleration = 0.1

        elif (self.i == 14 and (self.map_index == 0 or self.map_index == 2)): # First Turn
            self.turning = True
            if self.target_yaw >= 2 * math.pi:
                self.target_yaw -= 2 * math.pi
            if self.target_yaw < 0:
                self.target_yaw += 2 * math.pi
            self.target_yaw = self.current_yaw + 1.57
            
        elif (1 < self.i < 9 and (self.map_index == 1 or self.map_index == 3)): # DRIVE STRAIGHT
            msg.drive.steering_angle = -0.0525
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.1

        elif (self.i == 9 and (self.map_index == 1 or self.map_index == 3)): # Second Turn
            self.turning = True
            if self.target_yaw >= 2 * math.pi:
                self.target_yaw -= 2 * math.pi
            if self.target_yaw < 0:
                self.target_yaw += 2 * math.pi
            self.target_yaw = self.current_yaw + 1.57
            
            
        elif self.turning:
#           Turn the vehicle until the current yaw is within 0.1 radians of the target yaw
            if abs(self.current_yaw - self.target_yaw) > 0.1:
                msg.drive.steering_angle = 0.2 if self.current_yaw < self.target_yaw else -0.2
                msg.drive.speed = 1.0
                msg.drive.acceleration = 0.5
            else:
                self.turning = False
                self.map_index += 1
                self.i = 1
                self.target_yaw += 1.57
                print("FInished Turning")
            
        """   
        ## Drive Course
        elif (22 < self.i < 32): # DRIVE STRAIGHT
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.1
            print("Drive Forward 6.75 meters (22 ft)")
            
        elif (32 <= self.i < 40): #turning left
            msg.drive.steering_angle = 0.1725
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
            
        elif (40 <= self.i < 50): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (50 <= self.i < 58): #turning left
            msg.drive.steering_angle = 0.175
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (58 <= self.i < 68): #med drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 11.25m (37 ft)")
        elif (68 <= self.i < 76): #turning left
            msg.drive.steering_angle = 0.175
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (76 <= self.i < 86): #long drive straight
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0  
            msg.drive.acceleration = 0.5
            print("Drive Forward 15m (49.5 ft)")
        elif (86 <= self.i < 94): #turning left
            msg.drive.steering_angle = 0.1705
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            print("Turn Left")
        elif (94 <= self.i < 99): # finish line
            msg.drive.steering_angle = 0.0
            msg.drive.speed = 1.0
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
