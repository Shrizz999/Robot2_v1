import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# --- PIN CONFIGURATION (BCM Numbering) ---
# Left Motor
LEFT_ENA = 13  # PWM Speed (Physical Pin 33)
LEFT_IN1 = 17  # Direction (Physical Pin 11)
LEFT_IN2 = 27  # Direction (Physical Pin 13)

# Right Motor
RIGHT_IN3 = 22 # Direction (Physical Pin 15)
RIGHT_IN4 = 10 # Direction (Physical Pin 19)
RIGHT_ENB = 19 # PWM Speed (Physical Pin 35)

class DirectMotorDriver(Node):
    def __init__(self):
        super().__init__('direct_motor_driver')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Setup GPIO
        try:
            self.pwm_left = PWMOutputDevice(LEFT_ENA)
            self.pwm_right = PWMOutputDevice(RIGHT_ENB)
            self.dir_left_1 = DigitalOutputDevice(LEFT_IN1)
            self.dir_left_2 = DigitalOutputDevice(LEFT_IN2)
            self.dir_right_1 = DigitalOutputDevice(RIGHT_IN3)
            self.dir_right_2 = DigitalOutputDevice(RIGHT_IN4)
            self.get_logger().info('GPIO Initialized on RPi 5')
        except Exception as e:
            self.get_logger().error(f'Failed to init GPIO: {e}')

        self.wheel_base = 0.25
        self.max_speed = 0.5

    def set_left_motor(self, speed):
        # --- STALL PROTECTION LOGIC ---
        # If speed is tiny, stop completely to prevent whining
        if abs(speed) < 0.05:
            speed = 0.0
        # If speed is requested but too low to move, BOOST it to 0.3 (30%)
        elif abs(speed) < 0.6:
            speed = 0.6 if speed > 0 else -0.6

        # Apply to GPIO
        if speed > 0:
            self.dir_left_1.on()
            self.dir_left_2.off()
            self.pwm_left.value = min(speed, 1.0)
        elif speed < 0:
            self.dir_left_1.off()
            self.dir_left_2.on()
            self.pwm_left.value = min(abs(speed), 1.0)
        else:
            self.dir_left_1.off()
            self.dir_left_2.off()
            self.pwm_left.value = 0

    def set_right_motor(self, speed):
        # --- STALL PROTECTION LOGIC ---
        # If speed is tiny, stop completely
        if abs(speed) < 0.05:
            speed = 0.0
        # If speed is requested but too low to move, BOOST it to 0.3 (30%)
        elif abs(speed) < 0.6:
            speed = 0.6 if speed > 0 else -0.6

        # Apply to GPIO
        if speed > 0:
            self.dir_right_1.on()
            self.dir_right_2.off()
            self.pwm_right.value = min(speed, 1.0)
        elif speed < 0:
            self.dir_right_1.off()
            self.dir_right_2.on()
            self.pwm_right.value = min(abs(speed), 1.0)
        else:
            self.dir_right_1.off()
            self.dir_right_2.off()
            self.pwm_right.value = 0

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)

        left_pwm = left_vel / self.max_speed
        right_pwm = right_vel / self.max_speed

        self.get_logger().info(f"CMD: x={linear_x:.2f} | PWM: L={left_pwm:.2f}, R={right_pwm:.2f}")

        self.set_left_motor(left_pwm)
        self.set_right_motor(right_pwm)

def main(args=None):
    rclpy.init(args=args)
    node = DirectMotorDriver()
    rclpy.spin(node)
    
    node.pwm_left.close()
    node.pwm_right.close()
    node.dir_left_1.close()
    node.dir_left_2.close()
    node.dir_right_1.close()
    node.dir_right_2.close()
    
    node.destroy_node()
    rclpy.shutdown()
