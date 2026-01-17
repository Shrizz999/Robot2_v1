# Complete Navigation System - Installation Guide

## System Architecture Overview

```
Hardware Sensors → Sensor Nodes → EKF Fusion → SLAM → Navigation Stack → Motor Control
```

### TF Tree (FIXED)
```
map → odom → base_link → base_laser
                      → imu_link
```

---

## Prerequisites

### 1. Install ROS 2 Jazzy Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-msgs \
  python3-smbus2 \
  python3-numpy
```

### 2. Install GPIO Libraries (Raspberry Pi)

```bash
sudo apt install -y python3-gpiozero python3-lgpio
```

### 3. Install YDLidar Driver

```bash
cd ~/ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver
```

---

## Package Structure

Create the following directory structure:

```
my_bot_nav/
├── my_bot_nav/
│   ├── __init__.py
│   ├── wheel_odometry_node.py
│   ├── imu_node.py
│   ├── a_star_planner.py
│   ├── local_controller.py
│   └── direct_motor_driver.py
├── launch/
│   └── navigation.launch.py
├── config/
│   ├── ekf.yaml
│   ├── planner.yaml
│   └── controller.yaml
├── resource/
│   └── my_bot_nav
├── setup.py
└── package.xml
```

---

## Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select my_bot_nav
source install/setup.bash
```

---

## Hardware Calibration

### 1. Calibrate Encoder Scaling

Drive the robot **exactly 1 meter** and record encoder ticks:

```bash
ros2 run my_bot_nav wheel_odometry_node
# Watch the tick counts, then measure actual distance
```

Update `TICKS_PER_METER` in `wheel_odometry_node.py`:

```python
TICKS_PER_METER = 18000.0  # Adjust based on your measurement
```

### 2. Calibrate IMU

Keep robot **completely still** during startup for automatic calibration.

### 3. Verify TF Tree

```bash
ros2 run tf2_tools view_frames
# Should show: map → odom → base_link → {base_laser, imu_link}
```

---

## Launch Instructions

### Method 1: Single Launch File (Recommended)

```bash
# Terminal 1: Launch navigation stack
ros2 launch my_bot_nav navigation.launch.py

# Terminal 2: Launch motor driver (separate to avoid GPIO conflicts)
ros2 run my_bot_nav direct_motor_driver
```

### Method 2: Manual Launch (Debugging)

```bash
# Terminal 1: Sensors
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node

# Terminal 2: Wheel Odometry
ros2 run my_bot_nav wheel_odometry_node

# Terminal 3: IMU
ros2 run my_bot_nav imu_node

# Terminal 4: EKF
ros2 run robot_localization ekf_node --ros-args --params-file config/ekf.yaml

# Terminal 5: SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 6: Planner
ros2 run my_bot_nav a_star_planner

# Terminal 7: Controller
ros2 run my_bot_nav local_controller

# Terminal 8: Motor Driver
ros2 run my_bot_nav direct_motor_driver
```

---

## Usage

### 1. Visualize in RViz

```bash
ros2 run rviz2 rviz2
```

Add displays:
- **TF** - Verify frame tree
- **LaserScan** - Topic: `/scan`
- **Map** - Topic: `/map`
- **Path** - Topic: `/plan`
- **Odometry** - Topic: `/odometry/filtered`

### 2. Set Navigation Goal

In RViz:
1. Click "2D Goal Pose" button
2. Click on map to set goal
3. Robot will automatically plan and navigate

Or via command line:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"
```

---

## Troubleshooting

### Issue: "No TF from map to base_link"

**Fix:** Wait 10-15 seconds for SLAM to initialize.

```bash
# Check TF
ros2 run tf2_ros tf2_echo map base_link
```

---

### Issue: "Robot spins in place"

**Cause:** Encoder or IMU not publishing.

```bash
# Check topics
ros2 topic hz /wheel/odometry
ros2 topic hz /imu/data

# Check EKF output
ros2 topic echo /odometry/filtered
```

---

### Issue: "Path found but robot doesn't move"

**Check local controller:**

```bash
ros2 topic echo /cmd_vel
# Should show non-zero velocities

ros2 topic echo /scan
# Verify LiDAR is working
```

---

### Issue: "Robot drives into obstacles"

**Adjust safety parameters in `config/controller.yaml`:**

```yaml
obstacle_distance: 0.5  # Increase clearance
max_vel_x: 0.3          # Reduce speed
```

---

### Issue: "Path constantly replanning"

**Tune planner parameters in `config/planner.yaml`:**

```yaml
replanning_rate: 0.5          # Reduce frequency
path_validity_distance: 0.5   # Check less ahead
```

---

## Key Improvements from Original Code

### ✅ Fixed Issues

1. **TF Tree**: Corrected to `odom → base_link` (removed base_footprint)
2. **IMU Integration**: Added proper `sensor_msgs/Imu` publisher
3. **Sensor Fusion**: Implemented robot_localization EKF
4. **A* Safety**: Fixed unknown space handling (was treating -1 as free)
5. **Dynamic Replanning**: Added automatic path validation
6. **Motor Control**: Removed dangerous PWM jumps, added smooth acceleration
7. **Obstacle Avoidance**: Replaced pure pursuit with DWA-based local controller
8. **Covariance**: Added proper uncertainty matrices to all odometry messages

### 🆕 New Features

1. **Dynamic Replanning** - Automatically replans when obstacles detected
2. **Collision Prediction** - Checks trajectory safety before moving
3. **Smooth Acceleration** - Prevents wheel slip and odometry errors
4. **Recovery Behaviors** - Stops when collision imminent
5. **Parameter Files** - Easy tuning without code changes

---

## Performance Tuning

### For Faster Navigation (Open Areas)

```yaml
# controller.yaml
max_vel_x: 0.6
lookahead_distance: 0.7
```

### For Tighter Spaces

```yaml
# controller.yaml
max_vel_x: 0.3
obstacle_distance: 0.4
lookahead_distance: 0.4
```

### For Better Mapping

```yaml
# Slow down during exploration
max_vel_x: 0.2
```

---

## Next Steps

1. **Tune EKF** - Adjust covariance matrices in `ekf.yaml` for your sensors
2. **Add Recovery Behaviors** - Implement rotate-in-place, back-up routines
3. **Optimize A*** - Add costmap integration for better obstacle handling
4. **Add Nav2** - Replace custom controller with Nav2 stack for production use

---

## System Status Checklist

Before navigation:

- [ ] All sensors publishing (check with `ros2 topic list`)
- [ ] TF tree complete (`ros2 run tf2_tools view_frames`)
- [ ] EKF fusing data (`ros2 topic echo /odometry/filtered`)
- [ ] SLAM building map (`ros2 topic echo /map`)
- [ ] Static TFs published (laser, IMU)
- [ ] Motor driver responding (`ros2 topic pub /cmd_vel ...`)

---

## Contact

For issues specific to this implementation, check:
- TF tree: `ros2 run tf2_tools view_frames`
- Topic rates: `ros2 topic hz <topic_name>`
- Node status: `ros2 node list`
- Logs: `ros2 run rqt_console rqt_console`
