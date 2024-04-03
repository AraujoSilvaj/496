import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
#import keyboard


class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')
        self.publisher_ = self.create_publisher(Bool, 'button_state', 10)
        self.button_pin = 33  # GPIO pin where the button is connected
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #self.key = 'space'
        self.timer_ = self.create_timer(0.99, self.publish_button_state)

    def publish_button_state(self):
        button_state = bool(GPIO.input(self.button_pin))
        #button_state = GPIO.input(self.button_pin)
        #button_state = keyboard.is_pressed(self.key)
        msg = Bool()
        msg.data = button_state
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing button state: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    button_publisher = ButtonPublisher()
    rclpy.spin(button_publisher)
    button_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

