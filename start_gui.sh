#!/bin/bash

# 1. Wait for Desktop to load
sleep 10

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash
source /home/robovyu/nav_ws/install/setup.bash

# 3. Launch RViz with your config
# We point directly to the config file path you asked for
ros2 run rviz2 rviz2 -d /home/robovyu/bot_config.rviz
