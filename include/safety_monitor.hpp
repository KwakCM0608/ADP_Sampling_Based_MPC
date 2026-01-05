#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

class SafetyMonitor : public rclcpp::Node {
public:
  SafetyMonitor();

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr danger_pub_;

  double danger_threshold_;
};

