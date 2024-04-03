import rclpy 
from rclpy.node import Node 
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import array
import numpy as np

class OccupancyGridPublisher(Node): 
    def __init__(self):
        super().__init__('occupancy_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'occupancy_publisher', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_grid)
        self.grid = self.generate_grid()
        
    def generate_grid(self):
        pgm_file =  open('/496/ros_ws/src/rob_lowe_launch/include/course.pgm', 'rb')
        assert pgm_file.readline() == b'P5\n'
        (width, height) = [int(i) for i in pgm_file.readline().split()]
        depth = int(pgm_file.readline())
        assert depth <= 255 
        
        '''raster = []
        for x in range(height): 
            row = []
            for y in range(width): 
                row.append(ord(pgm_file.read(1)))
            raster.append(row)'''
            
        raster = []
        for i in range(width * height): 
            value = ord(pgm_file.read(1))
            scaled_value = int((value/2.55)) 
            raster.append(scaled_value)
            
        raster_array = array.array('b', raster)
            
        header_msg = Header()
        header_msg.frame_id = "map"
        
        return OccupancyGrid(header=header_msg, info=MapMetaData(width=width, height=height, resolution=1.0), data=raster_array)
        
    def publish_grid(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.grid)
        
def main(args=None): 
    rclpy.init(args=args)
    
    occupancy_publisher = OccupancyGridPublisher()
    rclpy.spin(occupancy_publisher)
    
    occupancy_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__': 
    main()
