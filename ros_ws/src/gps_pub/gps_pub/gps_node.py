import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#import serial

class GPSListener(Node):

    def __init__(self):
        super().__init__('gps_listener')

        # Initialize the GPS serial connection (adjust '/dev/ttyUSB0' as needed)
        # need to add serial 
        #self.serial_port = serial.Serial('/dev/tty', baudrate=9600, timeout=1)

        # Create a publisher to publish GPS data
        self.publisher_ = self.create_publisher(String, 'gps_data', 10)

        # Set a timer to periodically read from the GPS sensor
        self.timer = self.create_timer(1.0, self.read_gps_data)

    def read_gps_data(self):
        self.get_logger().info(f'Published GPS Data')
        # if self.serial_port.in_waiting > 0:
        #     gps_data = self.serial_port.readline().decode('utf-8', errors='replace').strip()
        #     msg = String()
        #     msg.data = gps_data
        #     self.publisher_.publish(msg)
        #     self.get_logger().info(f'Published GPS Data: {gps_data}')

def main(args=None):
    rclpy.init(args=args)
    gps_listener = GPSListener()
    rclpy.spin(gps_listener)
    gps_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()