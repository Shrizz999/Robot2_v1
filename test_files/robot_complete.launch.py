import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. SENSORS: YDLidar X2 Configuration ---
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='lidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': '/dev/ydlidar',
            'frame_id': 'laser_frame',
            'baudrate': 115200,      # X2 Specific
            'lidar_type': 1,         # Triangle
            'sample_rate': 3,
            'angle_min': -180.0,
            'angle_max': 180.0,
            'range_min': 0.1,
            'range_max': 12.0,
            'frequency': 10.0,
            'auto_reconnect': True,
            'reversion': True,
            'inverted': True,
            'isSingleChannel': True
        }]
    )

    # --- 2. SENSORS: I2C GPIO (MPU6050 + AS5600) ---
    imu_node = Node(
        package='ros2_mpu6050_driver', # Ensure this package is built
        executable='mpu6050_driver',
        name='imu_node',
        output='screen',
        parameters=[{'i2c_address': 0x68}]
    )

    encoder_node = Node(
        package='as5600_driver',       # The package we created previously
        executable='encoder_node',
        name='encoder_node',
        output='screen'
    )

    # --- 3. SYSTEM: Transforms (TF) ---
    # This connects the 'base' of your robot to the 'laser' so RViz understands geometry
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'laser_frame']
    )

    # --- 4. VISUALIZATION: RViz ---
    # We load a specific config file so you don't have to setup displays every time
    rviz_config_path = os.path.expanduser('~/ros2_ws/src/ydlidar_ros2_driver/config/my_robot.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        lidar_node,
        imu_node,
        encoder_node,
        tf_node,
        rviz_node
    ])
