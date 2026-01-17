import os
from glob import glob
from setuptools import setup

package_name = 'my_bot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ---------------------------------------------------------
        # THIS WAS MISSING: Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ---------------------------------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robovyu',
    maintainer_email='robovyu@todo.todo',
    description='Custom Navigation Stack',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_star_planner = my_bot_nav.a_star_planner:main',
            'path_follower = my_bot_nav.path_follower:main',
            # 'serial_motor_driver = ...',  <-- You can remove/comment the old one
            'direct_motor_driver = my_bot_nav.direct_motor_driver:main', 
            'odometry_node = my_bot_nav.odometry_node:main', 
        ],
    },

)
