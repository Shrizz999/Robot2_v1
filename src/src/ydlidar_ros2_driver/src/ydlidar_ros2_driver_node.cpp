/*
 * YDLIDAR ROS2 DRIVER
 *
 * Copyright 2015 - 2023 EAI TEAM
 * http://www.ydlidar.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "CYdLidar.h"
#include "ydlidar_config.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

using namespace ydlidar;

bool should_run = true;

void sig_handler(int signo) {
  should_run = false;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  
  // Create the node
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  CYdLidar laser;

  // ----------------------------------------------------------------------
  // DECLARE PARAMETERS (Connecting YAML to C++)
  // ----------------------------------------------------------------------
  std::string str_optvalue;
  node->declare_parameter("port", "/dev/ttyUSB0");
  node->get_parameter("port", str_optvalue);
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  node->declare_parameter("ignore_array", "");
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id;
  node->declare_parameter("frame_id", "laser_frame");
  node->get_parameter("frame_id", frame_id);

  int int_optvalue;
  node->declare_parameter("baudrate", 115200);
  node->get_parameter("baudrate", int_optvalue);
  laser.setlidaropt(LidarPropSerialBaudrate, &int_optvalue, sizeof(int));

  node->declare_parameter("lidar_type", 1);
  node->get_parameter("lidar_type", int_optvalue);
  laser.setlidaropt(LidarPropLidarType, &int_optvalue, sizeof(int));

  node->declare_parameter("device_type", 0);
  node->get_parameter("device_type", int_optvalue);
  laser.setlidaropt(LidarPropDeviceType, &int_optvalue, sizeof(int));

  node->declare_parameter("sample_rate", 3);
  node->get_parameter("sample_rate", int_optvalue);
  laser.setlidaropt(LidarPropSampleRate, &int_optvalue, sizeof(int));

  node->declare_parameter("abnormal_check_count", 4);
  node->get_parameter("abnormal_check_count", int_optvalue);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &int_optvalue, sizeof(int));

  bool bool_optvalue;
  node->declare_parameter("resolution_fixed", true);
  node->get_parameter("resolution_fixed", bool_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &bool_optvalue, sizeof(bool));

  node->declare_parameter("reversion", true);
  node->get_parameter("reversion", bool_optvalue);
  laser.setlidaropt(LidarPropReversion, &bool_optvalue, sizeof(bool));

  node->declare_parameter("inverted", true);
  node->get_parameter("inverted", bool_optvalue);
  laser.setlidaropt(LidarPropInverted, &bool_optvalue, sizeof(bool));

  node->declare_parameter("auto_reconnect", true);
  node->get_parameter("auto_reconnect", bool_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &bool_optvalue, sizeof(bool));

  node->declare_parameter("isSingleChannel", false);
  node->get_parameter("isSingleChannel", bool_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &bool_optvalue, sizeof(bool));

  node->declare_parameter("intensity", false);
  node->get_parameter("intensity", bool_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &bool_optvalue, sizeof(bool));

  node->declare_parameter("support_motor_dtr", false);
  node->get_parameter("support_motor_dtr", bool_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_optvalue, sizeof(bool));
  
  // --- THE FIX IS BELOW ---
  // We added ", false" to give it a default value
  node->declare_parameter("m3_mode", false); 
  // ------------------------

  float float_optvalue;
  node->declare_parameter("angle_max", 180.0f);
  node->get_parameter("angle_max", float_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &float_optvalue, sizeof(float));

  node->declare_parameter("angle_min", -180.0f);
  node->get_parameter("angle_min", float_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &float_optvalue, sizeof(float));

  node->declare_parameter("range_max", 16.0f);
  node->get_parameter("range_max", float_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &float_optvalue, sizeof(float));

  node->declare_parameter("range_min", 0.08f);
  node->get_parameter("range_min", float_optvalue);
  laser.setlidaropt(LidarPropMinRange, &float_optvalue, sizeof(float));

  node->declare_parameter("frequency", 10.0f);
  node->get_parameter("frequency", float_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &float_optvalue, sizeof(float));

  node->declare_parameter("invalid_range_is_inf", false);
  bool invalid_range_is_inf = false;
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  // ----------------------------------------------------------------------
  // INITIALIZE LIDAR
  // ----------------------------------------------------------------------
  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }

  // Publisher
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  
  // Signal Handler
  signal(SIGINT, sig_handler);

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now scanning...");

  // ----------------------------------------------------------------------
  // MAIN LOOP
  // ----------------------------------------------------------------------
  while (ret && rclcpp::ok() && should_run) {
    LaserScan scan;

    if (laser.doProcessSimple(scan)) {
      auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;

      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          float range = scan.points[i].range;
          if (range < scan.config.min_range || range > scan.config.max_range) {
             range = invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0;
          }
          scan_msg->ranges[index] = range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }
      scan_pub->publish(std::move(scan_msg));
    } else {
      RCLCPP_WARN(node->get_logger(), "Failed to get scan data");
    }
    
    rclcpp::spin_some(node);
  }

  // Cleanup
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}
