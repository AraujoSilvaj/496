import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math

class Waypoints(Node):
    
    def __init__(self):
        super().__init__('waypoints')
        self.subscriber = self.create_subscription(Odometry, 'odometry/filtered', self.pose_callback, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.waypoints = self.load_waypoints('/496/ros_ws/src/navigation/small_square.txt')
        self.stop_pub = self.create_publisher(Bool, 'stop_robot', 10)
        self.subscription = self.create_subscription(Bool, 'button_state', self.button_callback, 10)
        self.waypoint_index = 0
    
    def button_callback(self, msg):
        if  not msg.data:
            self.waypoint_index = 0
        
    def pose_callback(self, msg):
        min_dist = 0.1 # minimum distance to waypoint in meters
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular
        
        # Compare current pose with the next waypoint
        if self.waypoint_index < len(self.waypoints):
            next_waypoint = self.waypoints[self.waypoint_index]
            distance_to_waypoint = self.calculate_distance(position, next_waypoint.position)
            
            #print("-------distance to waypoint------")
            #print(distance_to_waypoint)
            #print("-------------------------------")
            #print("Waypoint index: ", self.waypoint_index)
            #print("Waypoint: ", next_waypoint)
            #print()
            
            waypoint_msg = PoseStamped()
            waypoint_msg.pose = next_waypoint
            waypoint_msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
            self.waypoint_pub.publish(waypoint_msg)
            
            if distance_to_waypoint < min_dist:
                self.waypoint_index += 1
                
        else:
            # reached the final waypoint (start line), stop the robot
            print("Reached the final waypoint. Stopping robot")
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)

        
    def load_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as file:
            for line in file:
                values = [float(value) for value in line.strip().split(',') if value]
                x, y, z, w = values
                quat = Quaternion(x=0.0, y=0.0, z=z, w=w)
                waypoints.append(Pose(position=Point(x=x, y=y, z=0.0), orientation=quat))
        return waypoints
    
    def calculate_distance(self, position1, position2):
        dx = position1.x -position2.x
        dy = position1.y - position2.y
        return math.sqrt(dx**2 + dy**2)
        
def main(args=None):
    rclpy.init(args=args)
    waypoints = Waypoints()
    rclpy.spin(waypoints)
    waypoints.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
