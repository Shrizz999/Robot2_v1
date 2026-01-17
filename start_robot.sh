#!/bin/bash

# 1. Setup Env
export PYTHONUNBUFFERED=1
cd /home/robovyu/nav_ws
sleep 5

# 2. Source ROS
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3. Fix GPIO for RPi 5
export GPIOZERO_PIN_FACTORY=lgpio
# Fix I2C Speed for Sensors
sudo dtparam i2c_arm_baudrate=10000

# 4. START BRAINS + SENSORS (Background)
# This loads Lidar, SLAM, and your NEW Odometry Node
ros2 launch my_bot_nav navigation.launch.py &
LAUNCH_PID=$!

# 5. Wait for sensors to calibrate
sleep 15

# 6. Safety Check (Kill any zombies)
pkill -f direct_motor_driver
sleep 1

# 7. START MUSCLES (Foreground)
# This triggers the "Startup Wiggle"
ros2 run my_bot_nav direct_motor_driver

wait $LAUNCH_PID
