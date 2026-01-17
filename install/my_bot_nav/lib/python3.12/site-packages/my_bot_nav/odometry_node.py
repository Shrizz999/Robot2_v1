import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import smbus2
import math
import time

# --- SETTINGS ---
MPU_ADDR = 0x68
AS5600_ADDR = 0x36

# *** CRITICAL TUNING *** # How many encoder "ticks" = 1 meter of travel?
# 1. Drive robot exactly 1 meter.
# 2. Check how many ticks it counted.
# Start with 18000 as a guess.
TICKS_PER_METER = 18000.0 

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # 1. Setup I2C
        # 1. Setup I2C with RETRY
        self.bus = None
        for attempt in range(5):
            try:
                self.bus = smbus2.SMBus(1)
                # Wake up MPU6050 (It sleeps by default)
                self.bus.write_byte_data(MPU_ADDR, 0x6B, 0)
                self.get_logger().info(f"Sensors Connected on attempt {attempt+1}!")
                break
            except Exception as e:
                self.get_logger().warn(f"Sensor init failed (attempt {attempt+1}): {e}")
                time.sleep(1.0)

        if self.bus is None:
            self.get_logger().error("CRITICAL: Could not connect to sensors after 5 tries.")
            return # Stop here

        # 2. Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 3. State Variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()
        self.prev_angle = self.read_as5600_raw()

        # Calibration for Gyro Drift
        self.gyro_offset = 0.0
        self.calibrate_gyro()

        # 4. Timer (Run at 20 Hz)
        self.create_timer(0.05, self.update)

    def read_as5600_raw(self):
        try:
            # Read registers 0x0C and 0x0D (Raw Angle 0-4095)
            high = self.bus.read_byte_data(AS5600_ADDR, 0x0C)
            low = self.bus.read_byte_data(AS5600_ADDR, 0x0D)
            return (high << 8) | low
        except:
            return 0

    def read_gyro_z(self):
        try:
            # Read Gyro Z registers 0x47 and 0x48
            high = self.bus.read_byte_data(MPU_ADDR, 0x47)
            low = self.bus.read_byte_data(MPU_ADDR, 0x48)
            val = (high << 8) | low
            # Convert to signed 16-bit
            if val > 32768: val -= 65536
            # Scale for degrees/sec (Sensitivity 131) -> Convert to Radians
            return (val / 131.0) * (math.pi / 180.0)
        except:
            return 0.0

    def calibrate_gyro(self):
        self.get_logger().info("Calibrating Gyro... KEEP ROBOT STILL!")
        total = 0
        for _ in range(50):
            total += self.read_gyro_z()
            time.sleep(0.02)
        self.gyro_offset = total / 50.0
        self.get_logger().info("Calibration Done.")

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- SENSOR FUSION ---

        # 1. Get Distance (AS5600)
        curr_angle = self.read_as5600_raw()
        delta_ticks = curr_angle - self.prev_angle

        # Handle 12-bit wrapping (0 -> 4095)
        # If it jumped from 0 to 4090, it went backward
        if delta_ticks < -2048: delta_ticks += 4096
        # If it jumped from 4090 to 0, it went forward
        if delta_ticks > 2048:  delta_ticks -= 4096

        self.prev_angle = curr_angle

        # Convert ticks to meters
        distance = delta_ticks / TICKS_PER_METER

        # 2. Get Rotation (MPU6050)
        gyro_z = self.read_gyro_z() - self.gyro_offset
        self.th += gyro_z * dt  # Integrate velocity to get angle

        # 3. Calculate Position (Odometry)
        self.x += distance * math.cos(self.th)
        self.y += distance * math.sin(self.th)

        # --- PUBLISH ---
        q = self.euler_to_quaternion(0, 0, self.th)

        # A. Publish TF (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # B. Publish Topic (/odom)
        o = Odometry()
        o.header = t.header
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(o)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qz = math.sin(yaw/2)
        qw = math.cos(yaw/2)
        return [0.0, 0.0, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
