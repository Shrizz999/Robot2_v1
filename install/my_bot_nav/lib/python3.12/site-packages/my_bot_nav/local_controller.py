#!/usr/bin/env python3
"""
Local Controller - DWA-based obstacle avoidance and path following
Replaces the unsafe pure pursuit controller
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import math
import numpy as np

class LocalController(Node):
    def __init__(self):
        super().__init__('local_controller')
        
        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State
        self.current_path = None
        self.current_scan = None
        self.current_velocity = Twist()
        
        # Parameters
        self.declare_parameter('max_vel_x', 0.4)
        self.declare_parameter('min_vel_x', 0.05)
        self.declare_parameter('max_vel_theta', 1.5)
        self.declare_parameter('acc_lim_x', 0.5)
        self.declare_parameter('acc_lim_theta', 1.0)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_distance', 0.3)
        self.declare_parameter('control_frequency', 10.0)
        
        # Control timer
        freq = self.get_parameter('control_frequency').value
        self.create_timer(1.0 / freq, self.control_loop)
        
        self.get_logger().info('Local Controller initialized')
    
    def path_callback(self, msg):
        """Store new path"""
        self.current_path = msg.poses
        self.get_logger().info(f'Received new path with {len(self.current_path)} points')
    
    def scan_callback(self, msg):
        """Store latest laser scan"""
        self.current_scan = msg
    
    def odom_callback(self, msg):
        """Store current velocity for acceleration limiting"""
        self.current_velocity = msg.twist.twist
    
    def get_robot_pose(self):
        """Get current robot pose in map frame"""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Extract yaw from quaternion
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (x, y, yaw)
        except Exception as e:
            return None
    
    def find_lookahead_point(self, robot_pose):
        """Find lookahead point on path"""
        if not self.current_path:
            return None
        
        rx, ry, _ = robot_pose
        lookahead = self.get_parameter('lookahead_distance').value
        
        # Find furthest point within lookahead distance
        target = None
        for pose in self.current_path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - rx)**2 + (py - ry)**2)
            
            if dist <= lookahead:
                target = (px, py)
        
        # If no point found, use last point
        if target is None and self.current_path:
            last = self.current_path[-1].pose.position
            target = (last.x, last.y)
        
        return target
    
    def check_collision(self, vel_x, vel_theta, dt=1.0):
        """Predict if trajectory will collide with obstacle"""
        if self.current_scan is None:
            return False
        
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return False
        
        rx, ry, ryaw = robot_pose
        
        # Simulate trajectory
        num_steps = 10
        for step in range(num_steps):
            t = (step + 1) * dt / num_steps
            
            # Predict position
            if abs(vel_theta) < 0.01:
                # Straight line
                px = rx + vel_x * t * math.cos(ryaw)
                py = ry + vel_x * t * math.sin(ryaw)
            else:
                # Arc
                radius = vel_x / vel_theta
                px = rx + radius * (math.sin(ryaw + vel_theta * t) - math.sin(ryaw))
                py = ry - radius * (math.cos(ryaw + vel_theta * t) - math.cos(ryaw))
            
            # Check against laser scan
            for i, r in enumerate(self.current_scan.ranges):
                if r < self.current_scan.range_min or r > self.current_scan.range_max:
                    continue
                
                angle = self.current_scan.angle_min + i * self.current_scan.angle_increment
                ox = rx + r * math.cos(ryaw + angle)
                oy = ry + r * math.sin(ryaw + angle)
                
                dist = math.sqrt((px - ox)**2 + (py - oy)**2)
                if dist < self.get_parameter('obstacle_distance').value:
                    return True
        
        return False
    
    def compute_velocity_command(self):
        """DWA-inspired velocity computation"""
        robot_pose = self.get_robot_pose()
        if robot_pose is None or not self.current_path:
            return Twist()
        
        rx, ry, ryaw = robot_pose
        
        # Check if goal reached
        goal = self.current_path[-1].pose.position
        dist_to_goal = math.sqrt((goal.x - rx)**2 + (goal.y - ry)**2)
        
        if dist_to_goal < self.get_parameter('goal_tolerance').value:
            self.get_logger().info('Goal reached!')
            self.current_path = None
            return Twist()
        
        # Find target point
        target = self.find_lookahead_point(robot_pose)
        if target is None:
            return Twist()
        
        tx, ty = target
        
        # Calculate desired heading
        angle_to_target = math.atan2(ty - ry, tx - rx)
        angular_error = angle_to_target - ryaw
        
        # Normalize angle
        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi
        
        # Compute velocities
        max_vel_x = self.get_parameter('max_vel_x').value
        min_vel_x = self.get_parameter('min_vel_x').value
        max_vel_theta = self.get_parameter('max_vel_theta').value
        
        # Linear velocity (reduce when turning)
        vel_x = max_vel_x * (1.0 - abs(angular_error) / math.pi)
        vel_x = max(min_vel_x, vel_x)
        
        # Angular velocity (P-controller)
        vel_theta = 2.0 * angular_error
        vel_theta = max(-max_vel_theta, min(max_vel_theta, vel_theta))
        
        # Apply acceleration limits
        acc_lim_x = self.get_parameter('acc_lim_x').value
        acc_lim_theta = self.get_parameter('acc_lim_theta').value
        dt = 0.1  # Assume 10Hz
        
        max_delta_v = acc_lim_x * dt
        max_delta_w = acc_lim_theta * dt
        
        vel_x = max(self.current_velocity.linear.x - max_delta_v,
                    min(self.current_velocity.linear.x + max_delta_v, vel_x))
        vel_theta = max(self.current_velocity.angular.z - max_delta_w,
                       min(self.current_velocity.angular.z + max_delta_w, vel_theta))
        
        # Collision checking
        if self.check_collision(vel_x, vel_theta):
            self.get_logger().warn('Collision predicted! Stopping...')
            vel_x = 0.0
            vel_theta = 0.0
        
        # Create command
        cmd = Twist()
        cmd.linear.x = vel_x
        cmd.angular.z = vel_theta
        
        return cmd
    
    def control_loop(self):
        """Main control loop"""
        cmd = self.compute_velocity_command()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LocalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
