#!/usr/bin/env python3
"""
IMU Node - MPU6050 IMU driver
Path: ~/ros2_ws/src/my_bot_nav/my_bot_nav/imu_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus2
import math
import time

MPU_ADDR = 0x68

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # I2C Setup with retry
        self.bus = None
        for attempt in range(5):
            try:
                self.bus = smbus2.SMBus(1)
                # Wake up MPU6050 (sleeps by default)
                self.bus.write_byte_data(MPU_ADDR, 0x6B, 0)
                self.get_logger().info(f"MPU6050 connected (attempt {attempt+1})")
                break
            except Exception as e:
                self.get_logger().warn(f"MPU6050 init failed: {e}")
                time.sleep(1.0)
        
        if self.bus is None:
            self.get_logger().error("CRITICAL: Could not connect to MPU6050")
            return
        
        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Calibration
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0
        self.accel_offset_x = 0.0
        self.accel_offset_y = 0.0
        self.accel_offset_z = 0.0
        
        self.calibrate()
        
        # Timer (50 Hz for IMU)
        self.create_timer(0.02, self.publish_imu)
        
        self.get_logger().info("IMU Node Started")

    def read_word_2c(self, addr):
        """Read 16-bit signed value from two registers"""
        high = self.bus.read_byte_data(MPU_ADDR, addr)
        low = self.bus.read_byte_data(MPU_ADDR, addr + 1)
        val = (high << 8) | low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def read_gyro(self):
        """Read gyroscope (rad/s)"""
        try:
            gx = self.read_word_2c(0x43) / 131.0 * (math.pi / 180.0)
            gy = self.read_word_2c(0x45) / 131.0 * (math.pi / 180.0)
            gz = self.read_word_2c(0x47) / 131.0 * (math.pi / 180.0)
            return gx, gy, gz
        except:
            return 0.0, 0.0, 0.0

    def read_accel(self):
        """Read accelerometer (m/s²)"""
        try:
            ax = self.read_word_2c(0x3B) / 16384.0 * 9.81
            ay = self.read_word_2c(0x3D) / 16384.0 * 9.81
            az = self.read_word_2c(0x3F) / 16384.0 * 9.81
            return ax, ay, az
        except:
            return 0.0, 0.0, 0.0

    def calibrate(self):
        """Calibrate IMU - Robot must be stationary!"""
        self.get_logger().info("Calibrating IMU... KEEP ROBOT STILL!")
        
        gyro_samples = []
        accel_samples = []
        
        for i in range(100):
            gx, gy, gz = self.read_gyro()
            ax, ay, az = self.read_accel()
            
            gyro_samples.append((gx, gy, gz))
            accel_samples.append((ax, ay, az))
            time.sleep(0.01)
        
        # Calculate offsets
        self.gyro_offset_x = sum(s[0] for s in gyro_samples) / len(gyro_samples)
        self.gyro_offset_y = sum(s[1] for s in gyro_samples) / len(gyro_samples)
        self.gyro_offset_z = sum(s[2] for s in gyro_samples) / len(gyro_samples)
        
        self.accel_offset_x = sum(s[0] for s in accel_samples) / len(accel_samples)
        self.accel_offset_y = sum(s[1] for s in accel_samples) / len(accel_samples)
        self.accel_offset_z = sum(s[2] for s in accel_samples) / len(accel_samples) - 9.81
        
        self.get_logger().info(f"Gyro offsets: {self.gyro_offset_x:.4f}, "
                              f"{self.gyro_offset_y:.4f}, {self.gyro_offset_z:.4f}")
        self.get_logger().info("Calibration complete!")

    def publish_imu(self):
        # Read sensors
        gx, gy, gz = self.read_gyro()
        ax, ay, az = self.read_accel()
        
        # Apply calibration
        gx -= self.gyro_offset_x
        gy -= self.gyro_offset_y
        gz -= self.gyro_offset_z
        ax -= self.accel_offset_x
        ay -= self.accel_offset_y
        az -= self.accel_offset_z
        
        # Create message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Orientation (not provided by MPU6050)
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1
        
        # Angular velocity
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025
        ]
        
        # Linear acceleration
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
