#include "safety_monitor.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

SafetyMonitor::SafetyMonitor()
: Node("safety_monitor")
{
  // 35cm 안쪽이면 위험
  declare_parameter("threshold", 0.35);
  danger_threshold_ = get_parameter("threshold").as_double();

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    10,
    std::bind(&SafetyMonitor::scan_callback, this, std::placeholders::_1));

  danger_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/trigger", 10);

  RCLCPP_INFO(get_logger(), "[SafetyMonitor] Ready. threshold=%.3f m", danger_threshold_);
}

void SafetyMonitor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int n = msg->ranges.size();
  if (n == 0) return;

  int center = n / 2;
  int span = n / 12; // ±15도 정도

  double min_front = std::numeric_limits<double>::infinity();

  for (int i = center - span; i <= center + span; ++i) {
    if (i < 0 || i >= n) continue;

    float r = msg->ranges[i];

    if (!std::isfinite(r)) continue;
    if (r <= msg->range_min) continue;
    if (r >= msg->range_max) continue;

    min_front = std::min(min_front, static_cast<double>(r));
  }

  std_msgs::msg::Bool out;

  if (!std::isfinite(min_front)) {
    out.data = false;
  } else {
    out.data = (min_front < danger_threshold_);
  }

  if (out.data) {
    RCLCPP_WARN(get_logger(), "[SafetyMonitor] DANGER! min_front=%.3f m (th=%.3f)",
                min_front, danger_threshold_);
  } else {
    RCLCPP_DEBUG(get_logger(), "[SafetyMonitor] SAFE. min_front=%.3f m (th=%.3f)",
                 min_front, danger_threshold_);
  }

  danger_pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

