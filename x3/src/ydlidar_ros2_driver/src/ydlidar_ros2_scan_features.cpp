#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ydlidar_interfaces/msg/scan_features.hpp"

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

float finite_or_nan(float value) {
  return std::isfinite(value) ? value : std::numeric_limits<float>::quiet_NaN();
}

class YDLidarScanFeaturesNode : public rclcpp::Node {
 public:
  YDLidarScanFeaturesNode() : Node("ydlidar_ros2_scan_features") {
    front_half_width_deg_ = declare_parameter("front_half_width_deg", 15.0);
    side_half_width_deg_ = declare_parameter("side_half_width_deg", 20.0);
    rear_half_width_deg_ = declare_parameter("rear_half_width_deg", 20.0);

    features_pub_ = create_publisher<ydlidar_interfaces::msg::ScanFeatures>(
        "scan_features", rclcpp::SensorDataQoS());
    nearest_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "scan_nearest_point", rclcpp::SensorDataQoS());

    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&YDLidarScanFeaturesNode::handle_scan, this, std::placeholders::_1));
  }

 private:
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    float front_min = std::numeric_limits<float>::infinity();
    float left_min = std::numeric_limits<float>::infinity();
    float right_min = std::numeric_limits<float>::infinity();
    float rear_min = std::numeric_limits<float>::infinity();
    float nearest_range = std::numeric_limits<float>::infinity();
    double nearest_angle = 0.0;
    float nearest_x = std::numeric_limits<float>::quiet_NaN();
    float nearest_y = std::numeric_limits<float>::quiet_NaN();
    size_t valid_count = 0;

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float range = scan->ranges[i];
      if (!is_valid_range(range, scan->range_min, scan->range_max)) {
        continue;
      }

      ++valid_count;
      double angle = scan->angle_min + scan->angle_increment * i;
      double angle_deg = rad2deg(angle);

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

      if (range < nearest_range) {
        nearest_range = range;
        nearest_angle = angle_deg;
        nearest_x = static_cast<float>(range * std::cos(angle));
        nearest_y = static_cast<float>(range * std::sin(angle));
      }
    }

    ydlidar_interfaces::msg::ScanFeatures features_msg;
    features_msg.header = scan->header;
    features_msg.front_min = finite_or_nan(front_min);
    features_msg.left_min = finite_or_nan(left_min);
    features_msg.right_min = finite_or_nan(right_min);
    features_msg.rear_min = finite_or_nan(rear_min);
    features_msg.nearest_range = finite_or_nan(nearest_range);
    features_msg.nearest_angle_deg = static_cast<float>(nearest_angle);
    features_msg.nearest_point.x = nearest_x;
    features_msg.nearest_point.y = nearest_y;
    features_msg.nearest_point.z = 0.0;
    features_msg.valid_count = static_cast<uint32_t>(valid_count);
    features_msg.total_count = static_cast<uint32_t>(scan->ranges.size());
    features_pub_->publish(features_msg);

    if (std::isfinite(nearest_range)) {
      geometry_msgs::msg::PointStamped nearest_msg;
      nearest_msg.header = scan->header;
      nearest_msg.point.x = nearest_x;
      nearest_msg.point.y = nearest_y;
      nearest_msg.point.z = 0.0;
      nearest_pub_->publish(nearest_msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<ydlidar_interfaces::msg::ScanFeatures>::SharedPtr features_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_pub_;
  double front_half_width_deg_;
  double side_half_width_deg_;
  double rear_half_width_deg_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YDLidarScanFeaturesNode>());
  rclcpp::shutdown();
  return 0;
}
