#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class CartographerIndoorNode : public rclcpp::Node {
 public:
  CartographerIndoorNode() : Node("cartographer_indoor_node") {
    scan_timeout_ms_ = declare_parameter("scan_timeout_ms", 500);
    imu_timeout_ms_ = declare_parameter("imu_timeout_ms", 500);
    tf_timeout_ms_ = declare_parameter("tf_timeout_ms", 500);
    publish_placeholder_odom_ =
        declare_parameter("publish_placeholder_odom", false);
    odom_source_mode_ =
        declare_parameter<std::string>("odom_source_mode", "tf");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ =
        declare_parameter<std::string>("base_frame_id", "base_link");
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/cartographer/status", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&CartographerIndoorNode::handle_scan, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&CartographerIndoorNode::handle_imu, this, std::placeholders::_1));

    if (odom_source_mode_ == "tf") {
      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
          tf_topic_, 10,
          std::bind(&CartographerIndoorNode::handle_tf, this, std::placeholders::_1));
    } else if (odom_source_mode_ != "placeholder") {
      RCLCPP_WARN(get_logger(),
                  "unsupported odom_source_mode '%s', using placeholder mode",
                  odom_source_mode_.c_str());
      odom_source_mode_ = "placeholder";
      publish_placeholder_odom_ = true;
    }

    timer_ = create_wall_timer(
        500ms, std::bind(&CartographerIndoorNode::publish_status, this));

    RCLCPP_INFO(get_logger(),
                "cartographer_indoor started; mode=%s tf_topic=%s odom_frame=%s base_frame=%s",
                odom_source_mode_.c_str(), tf_topic_.c_str(), odom_frame_id_.c_str(),
                base_frame_id_.c_str());
  }

 private:
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
    last_scan_time_ = now();
    ++scan_count_;
  }

  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    last_imu_ = msg;
    last_imu_time_ = now();
    ++imu_count_;
  }

  void handle_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto &transform : msg->transforms) {
      if (transform.header.frame_id == odom_frame_id_ &&
          transform.child_frame_id == base_frame_id_) {
        last_odom_transform_ =
            std::make_shared<geometry_msgs::msg::TransformStamped>(transform);
        last_tf_time_ = now();
        ++tf_count_;
        publish_odom_from_transform(transform);
      }
    }
  }

  void publish_status() {
    const bool scan_ok = has_recent(last_scan_time_, scan_timeout_ms_, last_scan_);
    const bool imu_ok = has_recent(last_imu_time_, imu_timeout_ms_, last_imu_);
    const bool tf_ok = has_recent(last_tf_time_, tf_timeout_ms_, last_odom_transform_);
    const bool publishing_placeholder =
        publish_placeholder_odom_ || odom_source_mode_ == "placeholder";
    const bool odom_ready = publishing_placeholder || (odom_source_mode_ == "tf" && tf_ok);

    std_msgs::msg::String status;
    if (!scan_ok && !imu_ok) {
      status.data = build_status("waiting_for_scan_and_imu", false, scan_ok, imu_ok, tf_ok);
    } else if (!scan_ok) {
      status.data = build_status("waiting_for_scan", false, scan_ok, imu_ok, tf_ok);
    } else if (!imu_ok) {
      status.data = build_status("waiting_for_imu", false, scan_ok, imu_ok, tf_ok);
    } else if (publishing_placeholder) {
      publish_placeholder_odom();
      status.data = build_status("publishing_placeholder_odom", true, scan_ok, imu_ok, tf_ok);
    } else if (odom_source_mode_ == "tf") {
      status.data = build_status(tf_ok ? "publishing_tf_backed_odom"
                                       : "waiting_for_cartographer_tf",
                                 odom_ready, scan_ok, imu_ok, tf_ok);
    } else {
      status.data = build_status("waiting_for_cartographer_backend", false, scan_ok,
                                 imu_ok, tf_ok);
    }

    status_pub_->publish(status);
  }

  template <typename SharedPtrT>
  bool has_recent(const rclcpp::Time &stamp, int timeout_ms,
                  const SharedPtrT &msg) const {
    if (!msg) {
      return false;
    }

    return (now() - stamp).nanoseconds() <
           static_cast<int64_t>(timeout_ms) * 1000000LL;
  }

  double age_ms(const rclcpp::Time &stamp, const std::shared_ptr<void const> &msg) const {
    if (!msg) {
      return -1.0;
    }
    return static_cast<double>((now() - stamp).nanoseconds()) / 1000000.0;
  }

  static std::string json_escape(const std::string &value) {
    std::ostringstream escaped;
    for (const char ch : value) {
      switch (ch) {
        case '"':
          escaped << "\\\"";
          break;
        case '\\':
          escaped << "\\\\";
          break;
        case '\n':
          escaped << "\\n";
          break;
        default:
          escaped << ch;
          break;
      }
    }
    return escaped.str();
  }

  std::string build_status(const std::string &state, bool ready, bool scan_ok,
                           bool imu_ok, bool tf_ok) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "{";
    oss << "\"state\":\"" << json_escape(state) << "\",";
    oss << "\"ready\":" << (ready ? "true" : "false") << ",";
    oss << "\"mode\":\"" << json_escape(odom_source_mode_) << "\",";
    oss << "\"scan\":{";
    oss << "\"present\":" << (last_scan_ ? "true" : "false") << ",";
    oss << "\"fresh\":" << (scan_ok ? "true" : "false") << ",";
    oss << "\"age_ms\":" << age_ms(last_scan_time_, last_scan_) << ",";
    oss << "\"count\":" << scan_count_ << "},";
    oss << "\"imu\":{";
    oss << "\"present\":" << (last_imu_ ? "true" : "false") << ",";
    oss << "\"fresh\":" << (imu_ok ? "true" : "false") << ",";
    oss << "\"age_ms\":" << age_ms(last_imu_time_, last_imu_) << ",";
    oss << "\"count\":" << imu_count_ << "},";
    oss << "\"tf\":{";
    oss << "\"present\":" << (last_odom_transform_ ? "true" : "false") << ",";
    oss << "\"fresh\":" << (tf_ok ? "true" : "false") << ",";
    oss << "\"age_ms\":" << age_ms(last_tf_time_, last_odom_transform_) << ",";
    oss << "\"count\":" << tf_count_ << ",";
    oss << "\"topic\":\"" << json_escape(tf_topic_) << "\",";
    oss << "\"frame_id\":\"" << json_escape(odom_frame_id_) << "\",";
    oss << "\"child_frame_id\":\"" << json_escape(base_frame_id_) << "\"},";
    oss << "\"output\":{";
    oss << "\"odom_topic\":\"/odom\",";
    oss << "\"status_topic\":\"/cartographer/status\",";
    oss << "\"odom_count\":" << odom_count_ << ",";
    oss << "\"last_x\":" << last_odom_x_ << ",";
    oss << "\"last_y\":" << last_odom_y_ << ",";
    oss << "\"last_yaw_z\":" << last_odom_yaw_z_ << "}";
    oss << "}";
    return oss.str();
  }

  void publish_odom_from_transform(
      const geometry_msgs::msg::TransformStamped &transform) {
    nav_msgs::msg::Odometry odom;
    odom.header = transform.header;
    odom.child_frame_id = transform.child_frame_id;
    odom.pose.pose.position.x = transform.transform.translation.x;
    odom.pose.pose.position.y = transform.transform.translation.y;
    odom.pose.pose.position.z = transform.transform.translation.z;
    odom.pose.pose.orientation = transform.transform.rotation;

    // Cartographer TF wiring provides pose first; twist is left zero until a
    // dedicated velocity source is added.
    odom.pose.covariance[0] = 0.05;
    odom.pose.covariance[7] = 0.05;
    odom.pose.covariance[35] = 0.1;
    odom.twist.covariance[0] = 0.5;
    odom.twist.covariance[7] = 0.5;
    odom.twist.covariance[35] = 1.0;

    if (last_imu_) {
      odom.twist.twist.angular = last_imu_->angular_velocity;
    }

    last_odom_x_ = odom.pose.pose.position.x;
    last_odom_y_ = odom.pose.pose.position.y;
    last_odom_yaw_z_ = odom.pose.pose.orientation.z;
    ++odom_count_;
    odom_pub_->publish(odom);
  }

  void publish_placeholder_odom() {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;

    if (last_imu_) {
      odom.pose.pose.orientation = last_imu_->orientation;
      odom.twist.twist.angular = last_imu_->angular_velocity;
    } else {
      odom.pose.pose.orientation.w = 1.0;
    }

    odom.pose.covariance[0] = 0.25;
    odom.pose.covariance[7] = 0.25;
    odom.pose.covariance[35] = 0.5;
    odom.twist.covariance[0] = 0.5;
    odom.twist.covariance[7] = 0.5;
    odom.twist.covariance[35] = 1.0;

    last_odom_x_ = odom.pose.pose.position.x;
    last_odom_y_ = odom.pose.pose.position.y;
    last_odom_yaw_z_ = odom.pose.pose.orientation.z;
    ++odom_count_;
    odom_pub_->publish(odom);
  }

  int scan_timeout_ms_;
  int imu_timeout_ms_;
  int tf_timeout_ms_;
  bool publish_placeholder_odom_;
  std::string odom_source_mode_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string tf_topic_;
  uint64_t scan_count_{0};
  uint64_t imu_count_{0};
  uint64_t tf_count_{0};
  uint64_t odom_count_{0};
  double last_odom_x_{0.0};
  double last_odom_y_{0.0};
  double last_odom_yaw_z_{0.0};

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  geometry_msgs::msg::TransformStamped::SharedPtr last_odom_transform_;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tf_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartographerIndoorNode>());
  rclcpp::shutdown();
  return 0;
}
