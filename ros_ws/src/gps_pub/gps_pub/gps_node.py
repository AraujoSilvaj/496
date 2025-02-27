import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class GPSListener(Node):

    def __init__(self):
        super().__init__('gps_listener')
        
        # Adjust baudrate if your GPS uses something other than 9600
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

        # Create a publisher to publish GPS data
        self.publisher_ = self.create_publisher(String, 'gps_data', 10)

        # Set a timer to read from the GPS sensor every second
        self.timer = self.create_timer(1.0, self.read_gps_data)

        self.get_logger().info('GPSListener node started. Listening on /dev/ttyACM0...')

    def read_gps_data(self):
        # Check if there is data waiting in the buffer
        if self.serial_port.in_waiting > 0:
            # Read a line (NMEA sentence) from the GPS
            raw_data = self.serial_port.readline().decode('utf-8', errors='replace').strip()

            # Publish the raw NMEA data
            msg = String()
            msg.data = raw_data
            self.publisher_.publish(msg)

            self.get_logger().info(f'Published GPS Data: {raw_data}')

def main(args=None):
    rclpy.init(args=args)
    gps_listener = GPSListener()
    rclpy.spin(gps_listener)
    gps_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
