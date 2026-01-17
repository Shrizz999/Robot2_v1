import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_path = []
        self.lookahead_distance = 0.3 # meters
        self.goal_tolerance = 0.1 # meters
        self.speed = 0.4 # m/s

        # Control Loop Timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.current_path = msg.poses
        self.get_logger().info(f'Received path with {len(self.current_path)} points')

    def get_yaw_from_tf(self, trans):
        q = trans.transform.rotation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.current_path:
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            ryaw = self.get_yaw_from_tf(trans)
        except Exception:
            return

        # 1. Find the lookahead point
        target_point = None
        for pose in self.current_path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - rx)**2 + (py - ry)**2)
            
            # Find first point outside lookahead radius
            if dist > self.lookahead_distance:
                target_point = (px, py)
                break
        
        # If no point found, we might be near end, pick last point
        if target_point is None:
            last = self.current_path[-1].pose.position
            dist_to_goal = math.sqrt((last.x - rx)**2 + (last.y - ry)**2)
            if dist_to_goal < self.goal_tolerance:
                # Goal Reached
                self.stop_robot()
                self.current_path = [] # Clear path
                self.get_logger().info("Goal Reached!")
                return
            target_point = (last.x, last.y)

        # 2. Calculate Control
        tx, ty = target_point
        angle_to_target = math.atan2(ty - ry, tx - rx)
        angular_error = angle_to_target - ryaw

        # Normalize angle to -pi to pi
        while angular_error > math.pi: angular_error -= 2*math.pi
        while angular_error < -math.pi: angular_error += 2*math.pi

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = 1.5 * angular_error # P-Controller for rotation

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
