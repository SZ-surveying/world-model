#include <chrono>
#include <exception>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class ExternalNavBridgeNode : public rclcpp::Node {
 public:
  ExternalNavBridgeNode() : Node("external_nav_bridge_node") {
    odom_timeout_ms_ = declare_parameter("odom_timeout_ms", 500);
    imu_timeout_ms_ = declare_parameter("imu_timeout_ms", 500);
    height_timeout_ms_ = declare_parameter("height_timeout_ms", 500);
    require_imu_for_output_ = declare_parameter("require_imu_for_output", false);
    require_height_for_output_ =
        declare_parameter("require_height_for_output", false);
    output_frame_id_ =
        declare_parameter<std::string>("output_frame_id", "external_nav");
    output_child_frame_id_ =
        declare_parameter<std::string>("output_child_frame_id", "base_link");
    ap_tf_topic_ = declare_parameter<std::string>("ap_tf_topic", "/ap/tf");
    ap_tf_parent_frame_ =
        declare_parameter<std::string>("ap_tf_parent_frame", "odom");
    ap_tf_child_frame_ =
        declare_parameter<std::string>("ap_tf_child_frame", "base_link");
    expected_odom_frame_id_ =
        declare_parameter<std::string>("expected_odom_frame_id", "odom");
    expected_odom_child_frame_id_ =
        declare_parameter<std::string>("expected_odom_child_frame_id", "base_link");
    min_odom_rate_hz_ = declare_parameter("min_odom_rate_hz", 4.0);
    max_height_covariance_ = declare_parameter("max_height_covariance", 4.0);
    height_topic_ =
        declare_parameter<std::string>("height_topic", "/height/estimate");
    coordinate_mode_ =
        declare_parameter<std::string>("coordinate_mode", "pass_through_enu_flu");

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/external_nav/status", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/external_nav/odom", 10);
    ap_tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(ap_tf_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ExternalNavBridgeNode::handle_odom, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&ExternalNavBridgeNode::handle_imu, this, std::placeholders::_1));

    height_sub_ = create_subscription<std_msgs::msg::String>(
        height_topic_, 10,
        std::bind(&ExternalNavBridgeNode::handle_height, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        500ms, std::bind(&ExternalNavBridgeNode::publish_status, this));

    RCLCPP_INFO(get_logger(),
                "external_nav_bridge started; waiting for /odom%s%s coordinate_mode=%s",
                require_imu_for_output_ ? " and /imu/data" : "",
                require_height_for_output_ ? " and /height/estimate" : "",
                coordinate_mode_.c_str());
  }

 private:
  struct HeightEstimate {
    double z{0.0};
    double vz{0.0};
    double covariance{0.0};
    std::string source_type;
  };

  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto stamp = now();
    if (last_odom_) {
      const double delta_sec = (stamp - last_odom_time_).seconds();
      if (delta_sec > 0.0) {
        if (odom_rate_hz_ <= 0.0) {
          odom_rate_hz_ = 1.0 / delta_sec;
        } else {
          odom_rate_hz_ = 0.8 * odom_rate_hz_ + 0.2 * (1.0 / delta_sec);
        }
      }
    }
    last_odom_ = msg;
    last_odom_time_ = stamp;
  }

  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    last_imu_ = msg;
    last_imu_time_ = now();
  }

  void handle_height(const std_msgs::msg::String::SharedPtr msg) {
    last_height_raw_ = msg;
    last_height_time_ = now();
    last_height_ = parse_height_estimate(msg->data);
  }

  void publish_status() {
    const auto stamp = now();
    const double odom_age_ms = age_ms(stamp, last_odom_time_, last_odom_);
    const double imu_age_ms = age_ms(stamp, last_imu_time_, last_imu_);
    const double height_age_ms =
        age_ms(stamp, last_height_time_, last_height_raw_);
    const bool odom_fresh = last_odom_ && odom_age_ms >= 0.0 &&
                            odom_age_ms < static_cast<double>(odom_timeout_ms_);
    const bool imu_ok =
        last_imu_ && imu_age_ms >= 0.0 &&
        imu_age_ms < static_cast<double>(imu_timeout_ms_);
    const bool frame_ok = odom_frame_ok();
    const bool rate_ok = odom_rate_hz_ >= min_odom_rate_hz_;
    const bool odom_ok = odom_fresh && frame_ok && rate_ok;
    const bool height_fresh =
        last_height_raw_ && height_age_ms >= 0.0 &&
        height_age_ms < static_cast<double>(height_timeout_ms_);
    const bool height_parse_ok = last_height_.has_value();
    const bool height_covariance_ok =
        height_parse_ok && last_height_->covariance >= 0.0 &&
        last_height_->covariance <= max_height_covariance_;
    const bool height_ok =
        height_fresh && height_parse_ok && height_covariance_ok;

    const bool ready = odom_ok && (!require_imu_for_output_ || imu_ok) &&
                       (!require_height_for_output_ || height_ok);

    if (ready) {
      publish_external_nav_odom();
    }

    std_msgs::msg::String status;
    status.data =
        build_status(odom_fresh, frame_ok, rate_ok, imu_ok, height_fresh,
                     height_parse_ok, height_covariance_ok, ready, odom_age_ms,
                     imu_age_ms, height_age_ms);
    status_pub_->publish(status);
  }

  void publish_external_nav_odom() {
    if (!last_odom_) {
      return;
    }

    nav_msgs::msg::Odometry out = *last_odom_;
    out.header.stamp = now();
    out.header.frame_id = output_frame_id_;
    out.child_frame_id = output_child_frame_id_;
    odom_pub_->publish(out);

    tf2_msgs::msg::TFMessage ap_tf_msg;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = out.header.stamp;
    transform.header.frame_id = ap_tf_parent_frame_;
    transform.child_frame_id = ap_tf_child_frame_;
    transform.transform.translation.x = out.pose.pose.position.x;
    transform.transform.translation.y = out.pose.pose.position.y;
    transform.transform.translation.z = out.pose.pose.position.z;
    transform.transform.rotation = out.pose.pose.orientation;
    ap_tf_msg.transforms.push_back(transform);
    ap_tf_pub_->publish(ap_tf_msg);
  }

  double age_ms(const rclcpp::Time &now_stamp, const rclcpp::Time &last_stamp,
                const std::shared_ptr<void const> &msg) const {
    if (!msg) {
      return -1.0;
    }
    return static_cast<double>((now_stamp - last_stamp).nanoseconds()) / 1000000.0;
  }

  bool odom_frame_ok() const {
    if (!last_odom_) {
      return false;
    }
    return last_odom_->header.frame_id == expected_odom_frame_id_ &&
           last_odom_->child_frame_id == expected_odom_child_frame_id_;
  }

  std::optional<double> json_number(const std::string &data,
                                    const std::string &key) const {
    const std::string quoted_key = "\"" + key + "\"";
    const auto key_pos = data.find(quoted_key);
    if (key_pos == std::string::npos) {
      return std::nullopt;
    }
    const auto colon_pos = data.find(":", key_pos + quoted_key.size());
    if (colon_pos == std::string::npos) {
      return std::nullopt;
    }
    const auto start = data.find_first_of("-0123456789.", colon_pos + 1);
    if (start == std::string::npos) {
      return std::nullopt;
    }
    const auto end =
        data.find_first_not_of("-0123456789.eE+", start);
    try {
      return std::stod(data.substr(start, end - start));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  std::optional<std::string> json_string(const std::string &data,
                                         const std::string &key) const {
    const std::string quoted_key = "\"" + key + "\"";
    const auto key_pos = data.find(quoted_key);
    if (key_pos == std::string::npos) {
      return std::nullopt;
    }
    const auto colon_pos = data.find(":", key_pos + quoted_key.size());
    if (colon_pos == std::string::npos) {
      return std::nullopt;
    }
    const auto first_quote = data.find("\"", colon_pos + 1);
    if (first_quote == std::string::npos) {
      return std::nullopt;
    }
    const auto second_quote = data.find("\"", first_quote + 1);
    if (second_quote == std::string::npos) {
      return std::nullopt;
    }
    return data.substr(first_quote + 1, second_quote - first_quote - 1);
  }

  std::optional<HeightEstimate> parse_height_estimate(
      const std::string &data) const {
    const auto z = json_number(data, "z");
    const auto vz = json_number(data, "vz");
    const auto covariance = json_number(data, "covariance");
    const auto source_type = json_string(data, "source_type");
    if (!z || !vz || !covariance || !source_type || source_type->empty()) {
      return std::nullopt;
    }
    return HeightEstimate{*z, *vz, *covariance, *source_type};
  }

  std::string state_for(bool odom_fresh, bool frame_ok, bool rate_ok, bool imu_ok,
                        bool height_fresh, bool height_parse_ok,
                        bool height_covariance_ok, bool ready) const {
    if (ready) {
      return "healthy";
    }
    if (!last_odom_) {
      return "waiting_for_odom";
    }
    if (!odom_fresh) {
      return "timeout";
    }
    if (!frame_ok) {
      return "invalid_frame";
    }
    if (!rate_ok) {
      return "low_rate";
    }
    if (require_imu_for_output_ && !imu_ok) {
      return "waiting_for_imu";
    }
    if (require_height_for_output_ && !last_height_raw_) {
      return "waiting_for_height";
    }
    if (require_height_for_output_ && !height_fresh) {
      return "height_timeout";
    }
    if (require_height_for_output_ && !height_parse_ok) {
      return "invalid_height";
    }
    if (require_height_for_output_ && !height_covariance_ok) {
      return "height_covariance_high";
    }
    return "not_ready";
  }

  std::string build_status(bool odom_fresh, bool frame_ok, bool rate_ok,
                           bool imu_ok, bool height_fresh, bool height_parse_ok,
                           bool height_covariance_ok, bool ready,
                           double odom_age_ms, double imu_age_ms,
                           double height_age_ms) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{";
    oss << "\"state\":\""
        << state_for(odom_fresh, frame_ok, rate_ok, imu_ok, height_fresh,
                     height_parse_ok, height_covariance_ok, ready)
        << "\",";
    oss << "\"ready\":" << (ready ? "true" : "false") << ",";
    oss << "\"odom\":{";
    oss << "\"present\":" << (last_odom_ ? "true" : "false") << ",";
    oss << "\"fresh\":" << (odom_fresh ? "true" : "false") << ",";
    oss << "\"frame_ok\":" << (frame_ok ? "true" : "false") << ",";
    oss << "\"rate_ok\":" << (rate_ok ? "true" : "false") << ",";
    oss << "\"age_ms\":" << odom_age_ms << ",";
    oss << "\"rate_hz\":" << odom_rate_hz_ << ",";
    oss << "\"frame_id\":\"" << (last_odom_ ? last_odom_->header.frame_id : "") << "\",";
    oss << "\"child_frame_id\":\"" << (last_odom_ ? last_odom_->child_frame_id : "")
        << "\"},";
    oss << "\"imu\":{";
    oss << "\"required\":" << (require_imu_for_output_ ? "true" : "false") << ",";
    oss << "\"present\":" << (last_imu_ ? "true" : "false") << ",";
    oss << "\"fresh\":" << (imu_ok ? "true" : "false") << ",";
    oss << "\"age_ms\":" << imu_age_ms << "},";
    oss << "\"height\":{";
    oss << "\"required\":" << (require_height_for_output_ ? "true" : "false")
        << ",";
    oss << "\"present\":" << (last_height_raw_ ? "true" : "false") << ",";
    oss << "\"fresh\":" << (height_fresh ? "true" : "false") << ",";
    oss << "\"parse_ok\":" << (height_parse_ok ? "true" : "false") << ",";
    oss << "\"covariance_ok\":"
        << (height_covariance_ok ? "true" : "false") << ",";
    oss << "\"age_ms\":" << height_age_ms << ",";
    oss << "\"topic\":\"" << height_topic_ << "\",";
    oss << "\"max_covariance\":" << max_height_covariance_ << ",";
    oss << "\"source_type\":\""
        << (last_height_ ? last_height_->source_type : "") << "\",";
    oss << "\"z\":" << (last_height_ ? last_height_->z : 0.0) << ",";
    oss << "\"vz\":" << (last_height_ ? last_height_->vz : 0.0) << ",";
    oss << "\"covariance\":"
        << (last_height_ ? last_height_->covariance : 0.0) << "},";
    oss << "\"output\":{";
    oss << "\"odom_topic\":\"/external_nav/odom\",";
    oss << "\"ap_tf_topic\":\"" << ap_tf_topic_ << "\",";
    oss << "\"coordinate_mode\":\"" << coordinate_mode_ << "\"}";
    oss << "}";
    return oss.str();
  }

  int odom_timeout_ms_;
  int imu_timeout_ms_;
  int height_timeout_ms_;
  bool require_imu_for_output_;
  bool require_height_for_output_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  std::string ap_tf_topic_;
  std::string ap_tf_parent_frame_;
  std::string ap_tf_child_frame_;
  std::string expected_odom_frame_id_;
  std::string expected_odom_child_frame_id_;
  double min_odom_rate_hz_;
  double odom_rate_hz_{0.0};
  double max_height_covariance_;
  std::string height_topic_;
  std::string coordinate_mode_;

  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  std_msgs::msg::String::SharedPtr last_height_raw_;
  std::optional<HeightEstimate> last_height_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_height_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr height_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr ap_tf_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExternalNavBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
