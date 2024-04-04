import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math
import time

class odomNavigation(Node):

    def __init__(self):
        super().__init__('odom_navigation')
        self.subscriber = self.create_subscription(Odometry, 'odometry/filtered', self.pose_callback, 10)
        self.stop_sub = self.create_subscription(Bool, 'stop_robot', self.stop_callback, 10)
        self.subscription = self.create_subscription(Bool, 'button_state', self.button_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)
        self.waypoint_sub = self.create_subscription(PoseStamped, 'goal_pose', self.waypoint_callback, 10)
        self.current_waypoint = None
        self.button_state = False
        
        # Create a timer to publish an initial Ackermann command
        #self.timer = self.create_timer(1.0, self.publish_initial_cmd)
        
    def publish_initial_cmd(self):
        time.sleep(10)
        # Publish an initial Ackermann command to start the robot's movement
        initial_cmd = AckermannDriveStamped()
        initial_cmd.drive.speed = 1.0  # Set the initial speed
        initial_cmd.drive.acceleration = 0.1
        initial_cmd.drive.steering_angle = 0.0
        self.publisher.publish(initial_cmd)
        print("published initial movement")

        # Stop the timer after publishing the initial command
        self.timer.cancel()    

    def button_callback(self, msg):
        self.button_state = msg.data
        if  not msg.data:
            self.publish_ackermann_cmd(0.0, 0.0, 0.0)
    
    def stop_callback(self,msg):
        if msg.data:
            stop_cmd = AckermannDriveStamped()
            stop_cmd.drive.speed = 0.0
            stop_cmd.drive.acceleration = 0.0
            stop_cmd.drive.steering_angle = 0.0
            self.publisher.publish(stop_cmd)
            print("published stop movement command")
    
    def waypoint_callback(self, msg):
        self.current_waypoint = msg.pose
        
    def pose_callback(self, msg):
        acceleration = 1.0
        desired_velocity = 1.0
        
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular
        
        if self.current_waypoint is not None:       
            desired_steering_angle = self.calculate_steering_angle(position, orientation, self.current_waypoint)
            
            if self.button_state:
                self.publish_ackermann_cmd(desired_velocity, acceleration, desired_steering_angle)
       
    def calculate_distance(self, position1, position2):
        dx = position1.x -position2.x
        dy = position1.y - position2.y
        return math.sqrt(dx**2 + dy**2)
     
    def calculate_steering_angle(self, position, orientation, waypoint):
        
        max_steering_angle = 0.25 # Maximum steering angle in radians
        
        # calculate the vector from the current position to the waypoint
        waypoint_vector = [waypoint.position.x - position.x, waypoint.position.y - position.y]
        
        # calculate the angle between the robot's heading and the waypoint vector
        robot_heading = 2 * math.atan2(orientation.z, orientation.w)
        waypoint_angle = math.atan2(waypoint_vector[1], waypoint_vector[0])
        angle_diff = waypoint_angle - robot_heading
        
        #print("waypoint vector: ", waypoint_vector)
        #print("robot heading: ", robot_heading)
        #print("angle dif: ", angle_diff)
        
        # normalize the angle difference to [-pi,pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        ## calculate the desired steering angle using the pure pursuit algorithm
        #lookahead_distance = 5.0 # adjust this value as needed
        #desired_steering_angle = math.atan2(2.0 * math.sin(angle_diff) * lookahead_distance, math.sqrt(1 + (2 * math.cos(angle_diff) * lookahead_distance)**2))
        
        # trying a simpler approach to find steering angle
        # go to goal approach
        desired_steering_angle = max(-max_steering_angle, min(max_steering_angle, angle_diff))
        
        #print("normalized angle dif: ", angle_diff)
        #print("Desired Steering Angle: ", desired_steering_angle)
        #print()
        
        return desired_steering_angle
        
    def publish_ackermann_cmd(self, velocity, acceleration, steering_angle):
        #Create an AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.acceleration = acceleration
        drive_msg.drive.steering_angle = steering_angle

        # Publish the Ackermann drive command
        self.publisher.publish(drive_msg)
        
        
def main(args= None):
    rclpy.init(args=args)
    odom_navigation = odomNavigation()
    rclpy.spin(odom_navigation)
    odom_navigation.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
