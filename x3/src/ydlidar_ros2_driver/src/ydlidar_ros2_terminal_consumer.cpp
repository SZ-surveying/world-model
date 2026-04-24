#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace {

double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

bool is_valid_range(float value, float range_min, float range_max) {
  return std::isfinite(value) && value >= range_min && value <= range_max;
}

bool angle_in_sector(double angle_deg, double center_deg, double half_width_deg) {
  double diff = std::fmod(angle_deg - center_deg + 540.0, 360.0) - 180.0;
  return std::abs(diff) <= half_width_deg;
}

class YDLidarTerminalConsumer : public rclcpp::Node {
 public:
  YDLidarTerminalConsumer()
      : Node("ydlidar_ros2_terminal_consumer") {
    front_half_width_deg_ = declare_parameter("front_half_width_deg", 15.0);
    side_half_width_deg_ = declare_parameter("side_half_width_deg", 20.0);
    rear_half_width_deg_ = declare_parameter("rear_half_width_deg", 20.0);

    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&YDLidarTerminalConsumer::handle_scan, this, std::placeholders::_1));
  }

 private:
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (scan->ranges.empty()) {
      RCLCPP_WARN(get_logger(), "Received an empty scan on frame %s",
                  scan->header.frame_id.c_str());
      return;
    }

    size_t valid_count = 0;
    float min_range = std::numeric_limits<float>::infinity();
    double min_angle_deg = 0.0;
    float front_min = std::numeric_limits<float>::infinity();
    float left_min = std::numeric_limits<float>::infinity();
    float right_min = std::numeric_limits<float>::infinity();
    float rear_min = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const float range = scan->ranges[i];
      if (!is_valid_range(range, scan->range_min, scan->range_max)) {
        continue;
      }

      ++valid_count;
      const double angle_deg = rad2deg(scan->angle_min + scan->angle_increment * i);

      if (range < min_range) {
        min_range = range;
        min_angle_deg = angle_deg;
      }

      if (angle_in_sector(angle_deg, 0.0, front_half_width_deg_)) {
        front_min = std::min(front_min, range);
      }
      if (angle_in_sector(angle_deg, 90.0, side_half_width_deg_)) {
        left_min = std::min(left_min, range);
      }
      if (angle_in_sector(angle_deg, -90.0, side_half_width_deg_)) {
        right_min = std::min(right_min, range);
      }
      if (angle_in_sector(angle_deg, 180.0, rear_half_width_deg_) ||
          angle_in_sector(angle_deg, -180.0, rear_half_width_deg_)) {
        rear_min = std::min(rear_min, range);
      }
    }

    const double angle_min_deg = rad2deg(scan->angle_min);
    const double angle_max_deg = rad2deg(scan->angle_max);

    if (valid_count == 0) {
      RCLCPP_WARN(get_logger(),
                  "scan frame=%s valid_points=0 angle=[%.1f, %.1f] deg",
                  scan->header.frame_id.c_str(), angle_min_deg, angle_max_deg);
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "scan frame=%s points=%zu valid=%zu angle=[%.1f, %.1f] deg front=%.3f left=%.3f right=%.3f rear=%.3f nearest=%.3f m @ %.1f deg",
        scan->header.frame_id.c_str(), scan->ranges.size(), valid_count,
        angle_min_deg, angle_max_deg, finite_or_nan(front_min), finite_or_nan(left_min),
        finite_or_nan(right_min), finite_or_nan(rear_min), min_range, min_angle_deg);
  }

  float finite_or_nan(float value) const {
    return std::isfinite(value) ? value : std::numeric_limits<float>::quiet_NaN();
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  double front_half_width_deg_;
  double side_half_width_deg_;
  double rear_half_width_deg_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YDLidarTerminalConsumer>());
  rclcpp::shutdown();
  return 0;
}
