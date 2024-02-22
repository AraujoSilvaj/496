import rclpy
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64

class VescVoltagePublisher(Node):

	def __init__(self):
		super().__init__('vesc_voltage_publisher')
		self.subscription = self.create_subscription(VescStateStamped, 'sensors/core',self.vesc_state_callback,10)
		self.voltage_publisher = self.create_publisher(Float64, 'input_voltage', 10)
		self.battery_percentage_publisher = self.create_publisher(Float64,'battery_percentage',10)
		
		self.min_voltage = 11.1
		self.max_voltage = 12.6
		
		
	def vesc_state_callback(self,msg):
		input_voltage = msg.state.voltage_input
		voltage_msg = Float64()
		voltage_msg.data = input_voltage
		self.voltage_publisher.publish(voltage_msg)
		
		# calculate battery percentage
		battery_percentage = ((input_voltage - self.min_voltage) / (self.max_voltage - self.min_voltage)) * 100
		
		battery_percentage_msg = Float64()
		battery_percentage_msg.data = battery_percentage
		self.battery_percentage_publisher.publish(battery_percentage_msg)
		
		print(f'Input Voltage: {input_voltage}V	Battery Percent: {battery_percentage}%')
	
		

def main(args=None):
	rclpy.init(args=args)
	vesc_voltage_publisher = VescVoltagePublisher()
	rclpy.spin(vesc_voltage_publisher)
	vesc_voltage_publisher.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
