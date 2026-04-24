#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ydlidar_interfaces/msg/scan_features.hpp"

namespace {

bool finite_positive(float value) {
  return std::isfinite(value) && value > 0.0f;
}

class YDLidarRuleAvoidanceNode : public rclcpp::Node {
 public:
  YDLidarRuleAvoidanceNode() : Node("ydlidar_ros2_rule_avoidance") {
    stop_distance_ = declare_parameter("stop_distance", 0.50);
    avoid_distance_ = declare_parameter("avoid_distance", 0.90);
    side_safe_distance_ = declare_parameter("side_safe_distance", 0.60);
    forward_speed_ = declare_parameter("forward_speed", 0.20);
    slow_speed_ = declare_parameter("slow_speed", 0.08);
    turn_speed_ = declare_parameter("turn_speed", 0.60);
    steer_gain_ = declare_parameter("steer_gain", 0.80);

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "avoidance/cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("avoidance/state", 10);

    subscription_ = create_subscription<ydlidar_interfaces::msg::ScanFeatures>(
        "scan_features", rclcpp::SensorDataQoS(),
        std::bind(&YDLidarRuleAvoidanceNode::handle_features, this, std::placeholders::_1));
  }

 private:
  void handle_features(const ydlidar_interfaces::msg::ScanFeatures::SharedPtr msg) {
    const float front = msg->front_min;
    const float left = msg->left_min;
    const float right = msg->right_min;
    const float nearest = msg->nearest_range;
    const float valid_count = static_cast<float>(msg->valid_count);

    if (!finite_positive(valid_count) || !finite_positive(nearest)) {
      publish_command(0.0, 0.0, "no_valid_scan");
      return;
    }

    double linear_x = 0.0;
    double angular_z = 0.0;
    std::string state = "hold";

    const bool front_valid = finite_positive(front);
    const bool left_valid = finite_positive(left);
    const bool right_valid = finite_positive(right);

    const double safer_turn =
        choose_turn_direction(left_valid ? left : std::numeric_limits<float>::quiet_NaN(),
                              right_valid ? right : std::numeric_limits<float>::quiet_NaN());

    if (!front_valid) {
      state = "front_unknown_stop";
    } else if (front < stop_distance_) {
      linear_x = 0.0;
      angular_z = safer_turn * turn_speed_;
      state = "front_blocked_turn";
    } else if (front < avoid_distance_) {
      linear_x = slow_speed_;
      angular_z = safer_turn * turn_speed_;
      state = "avoid_turn";
    } else {
      linear_x = forward_speed_;
      angular_z = compute_steering(left, right);
      state = "forward";

      if ((left_valid && left < side_safe_distance_) ||
          (right_valid && right < side_safe_distance_)) {
        state = "forward_steer";
      }
    }

    publish_command(linear_x, angular_z, state);
  }

  double choose_turn_direction(float left, float right) const {
    const bool left_valid = finite_positive(left);
    const bool right_valid = finite_positive(right);

    if (left_valid && right_valid) {
      return left >= right ? 1.0 : -1.0;
    }
    if (left_valid) {
      return 1.0;
    }
    if (right_valid) {
      return -1.0;
    }
    return 1.0;
  }

  double compute_steering(float left, float right) const {
    const bool left_valid = finite_positive(left);
    const bool right_valid = finite_positive(right);

    if (!left_valid || !right_valid) {
      return 0.0;
    }

    const double diff = static_cast<double>(left - right);
    const double denom = std::max(static_cast<double>(left + right), 1e-3);
    double steering = steer_gain_ * diff / denom;
    steering = std::clamp(steering, -turn_speed_, turn_speed_);
    return steering;
  }

  void publish_command(double linear_x, double angular_z, const std::string &state) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = linear_x;
    cmd.twist.angular.z = angular_z;
    cmd_pub_->publish(cmd);

    std_msgs::msg::String state_msg;
    state_msg.data = state;
    state_pub_->publish(state_msg);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "state=%s cmd.linear.x=%.3f cmd.angular.z=%.3f",
        state.c_str(), linear_x, angular_z);
  }

  rclcpp::Subscription<ydlidar_interfaces::msg::ScanFeatures>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  double stop_distance_;
  double avoid_distance_;
  double side_safe_distance_;
  double forward_speed_;
  double slow_speed_;
  double turn_speed_;
  double steer_gain_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YDLidarRuleAvoidanceNode>());
  rclcpp::shutdown();
  return 0;
}
