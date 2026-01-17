import rclpy
from rclpy.node import Node
from gpiozero import LED

class BlinkNode(Node):
    def __init__(self):
        super().__init__('led_blinker')
        
        # Setup GPIO 17 (Physical Pin 11)
        self.led = LED(17)
        self.timer_period = 1.0  # Seconds
        
        # Create a ROS timer that triggers every 1 second
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Blink Node has started on GPIO 17')

    def timer_callback(self):
        # Toggle the LED state
        self.led.toggle()
        
        # Log the status to the ROS terminal
        status = "ON" if self.led.is_lit else "OFF"
        self.get_logger().info(f'LED is now: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = BlinkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
