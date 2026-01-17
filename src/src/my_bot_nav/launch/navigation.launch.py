import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. YDLiDAR Node (The Eyes)
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'ignore_array': '',
            'baudrate': 115200,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 3,
            'abnormal_check_count': 4,
            'resolution_fixed': True,
            'reversion': False,
            'inverted': True,
            'auto_reconnect': True,
            'isSingleChannel': True,
            'intensity': False,
            'support_motor_dtr': True,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 10.0,
            'range_min': 0.4, 
            'frequency': 7.0,
            'invalid_range_is_inf': False
        }]
    )

    # 2. Real Odometry Node (The Inner Ear - Encoder/Gyro)
    # REPLACES the old static_tf_pub_odom
    odom_node = Node(
        package='my_bot_nav',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # 3. Static TF Publishers (Robot Geometry)
    tf_footprint_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # 4. SLAM Toolbox (The Mapper)
    slam_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
                ])
            )
        ]
    )

    # 5. Global Planner (The Brain)
    planner_node = Node(
        package='my_bot_nav',
        executable='a_star_planner',
        name='custom_planner',
        output='screen'
    )

    # 6. Path Follower (The Nerves)
    follower_node = Node(
        package='my_bot_nav',
        executable='path_follower',
        name='custom_follower',
        output='screen'
    )

    # NOTE: Motor Driver is intentionally MISSING. 
    # It is launched by the script to avoid GPIO lockups.

    return LaunchDescription([
        ydlidar_node,
        odom_node,       # <--- REAL SENSORS
        tf_footprint_node,
        tf_laser_node,
        slam_launch,
        planner_node,
        follower_node
    ])
