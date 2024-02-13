import rclpy
from rclpy.node import Node
import serial 

from sensor_msgs.msg import Imu
class ImuPublisher(Node): 
    def __init__(self): 
        super.publisher_ = self.create_publisher(Imu, 'imu-pub', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Imu()
        imu = Serial('/dev/ttyIMU', 9600)
        msg.data = imu.readline()
        self.publisher_.publish(msg)
        self.getlogger().info('Publishing: "%s"' % msg.data)

def main(args=None): 
	rclpy.init(args=args)
	
	imu_publihser = ImuPublisher()
	
	rclpy.spin(imu_publisher)
	
	imu_publisher.destroy_node()
	rclpy.shutdown()
