#include <memory>
#include <string>
#include <algorithm>
#include <chrono>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class FailsafeManagerNode : public rclcpp::Node
{
public:
  FailsafeManagerNode()
  : Node("failsafe_manager_node"),
    status_topic_(declare_parameter<std::string>("status_topic", "fcu_link/status")),
    input_target_topic_(declare_parameter<std::string>("input_target_topic", "target_point")),
    output_target_topic_(declare_parameter<std::string>("output_target_topic", "target_point_safe")),
    safe_frame_id_(declare_parameter<std::string>("safe_frame_id", "delta_base_link")),
    safe_x_(declare_parameter<double>("safe_x", 0.0)),
    safe_y_(declare_parameter<double>("safe_y", 0.0)),
    safe_z_(declare_parameter<double>("safe_z", 0.25)),
    input_timeout_sec_(declare_parameter<double>("input_timeout_sec", 0.4)),
    output_rate_hz_(declare_parameter<double>("output_rate_hz", 20.0))
  {
    input_timeout_sec_ = std::max(0.01, input_timeout_sec_);
    output_rate_hz_ = std::max(1.0, output_rate_hz_);

    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, 10,
      std::bind(&FailsafeManagerNode::statusCallback, this, std::placeholders::_1));

    input_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      input_target_topic_, 20,
      std::bind(&FailsafeManagerNode::inputTargetCallback, this, std::placeholders::_1));

    output_target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(output_target_topic_, 20);
    last_input_time_ = now();

    const auto timer_period = std::chrono::duration<double>(1.0 / output_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&FailsafeManagerNode::publishOutput, this));

    RCLCPP_INFO(
      get_logger(),
      "failsafe_manager started: status_topic=%s input=%s output=%s input_timeout=%.3fs output_rate=%.1fHz",
      status_topic_.c_str(), input_target_topic_.c_str(), output_target_topic_.c_str(),
      input_timeout_sec_, output_rate_hz_);
  }

private:
  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (link_status_ != msg->data) {
      RCLCPP_INFO(get_logger(), "failsafe link status -> %s", msg->data.c_str());
    }
    link_status_ = msg->data;
  }

  void inputTargetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    last_input_target_ = *msg;
    last_input_time_ = now();
    has_input_target_ = true;
  }

  void publishOutput()
  {
    const bool link_ok = (link_status_ == "OK");
    const bool has_fresh_target = has_input_target_ && ((now() - last_input_time_).seconds() <= input_timeout_sec_);

    if (link_ok && has_fresh_target) {
      auto out = last_input_target_;
      out.header.stamp = now();
      output_target_pub_->publish(out);
      return;
    }

    geometry_msgs::msg::PointStamped safe_msg;
    safe_msg.header.stamp = now();
    safe_msg.header.frame_id = safe_frame_id_;
    safe_msg.point.x = safe_x_;
    safe_msg.point.y = safe_y_;
    safe_msg.point.z = safe_z_;
    output_target_pub_->publish(safe_msg);
  }

  std::string status_topic_;
  std::string input_target_topic_;
  std::string output_target_topic_;
  std::string safe_frame_id_;
  double safe_x_;
  double safe_y_;
  double safe_z_;
  double input_timeout_sec_;
  double output_rate_hz_;

  std::string link_status_{"WAIT_FCU"};
  geometry_msgs::msg::PointStamped last_input_target_;
  rclcpp::Time last_input_time_;
  bool has_input_target_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr input_target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr output_target_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FailsafeManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
