import rclpy
from rclpy.node import Node
from VescState.msg import voltage_input

class VescVoltagePublisher(Node):

	def __init__(self):
		super().__init__('vesc_voltage_publisher')
		
		self.vesc_voltage_publisher = self.create_publisher(VescMsg, 'vesc_voltage', 10)
		
		self.timer = self.create_timer(1.0, self.publish_voltage)
		
	def publish_voltage(self):
		msg = VescMsg()
		
		msg.voltage_input = 48 # replace with actual voltage
		
		self.vesc_voltage_publisher.publish(msg)
		print(msg)
		

def main(args=None):
	rclpy.init(args=args)
	vesc_voltage_publisher = VescVoltagePublisher()
	rclpy.spin(vesc_voltage_publisher)
	vesc_voltage_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
