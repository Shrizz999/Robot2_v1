#!/usr/bin/env python3
"""
Wheel Odometry Node - Encoder-based velocity estimation
Path: ~/ros2_ws/src/my_bot_nav/my_bot_nav/wheel_odometry_node.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import smbus2
import math
import time

# AS5600 Magnetic Encoder
AS5600_ADDR = 0x36
TICKS_PER_METER = 18000.0  # ⚠️ CALIBRATE: drive 1m, count ticks, update this

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # I2C Setup with retry
        self.bus = None
        for attempt in range(5):
            try:
                self.bus = smbus2.SMBus(1)
                self.get_logger().info(f"AS5600 connected (attempt {attempt+1})")
                break
            except Exception as e:
                self.get_logger().warn(f"I2C init failed: {e}")
                time.sleep(1.0)
        
        if self.bus is None:
            self.get_logger().error("CRITICAL: Could not connect to AS5600")
            return
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_angle = self.read_as5600_raw()
        self.last_time = self.get_clock().now()
        
        # Parameters
        self.wheel_base = 0.25  # meters between wheels
        
        # Timer (20 Hz)
        self.create_timer(0.05, self.update)
        
        self.get_logger().info("Wheel Odometry Node Started")

    def read_as5600_raw(self):
        """Read raw angle from AS5600 (0-4095)"""
        try:
            high = self.bus.read_byte_data(AS5600_ADDR, 0x0C)
            low = self.bus.read_byte_data(AS5600_ADDR, 0x0D)
            return (high << 8) | low
        except:
            return 0

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt < 0.001:  # Avoid division by zero
            return
        
        self.last_time = current_time
        
        # Read encoder
        curr_angle = self.read_as5600_raw()
        delta_ticks = curr_angle - self.prev_angle
        
        # Handle 12-bit wrapping
        if delta_ticks < -2048:
            delta_ticks += 4096
        if delta_ticks > 2048:
            delta_ticks -= 4096
        
        self.prev_angle = curr_angle
        
        # Convert to distance
        distance = delta_ticks / TICKS_PER_METER
        
        # Calculate velocity
        linear_vel = distance / dt
        
        # Dead reckoning position (for TF only)
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        
        # Prepare messages
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        
        # Velocity in body frame
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Covariance matrices
        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.5
        ]
        
        odom_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e6
        ]
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
