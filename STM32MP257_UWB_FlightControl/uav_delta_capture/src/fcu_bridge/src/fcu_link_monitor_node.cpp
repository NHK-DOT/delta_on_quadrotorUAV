#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class FcuLinkMonitorNode : public rclcpp::Node
{
public:
  FcuLinkMonitorNode()
  : Node("fcu_link_monitor_node"),
    local_pose_topic_(declare_parameter<std::string>("local_pose_topic", "/mavros/local_position/pose")),
    status_topic_(declare_parameter<std::string>("status_topic", "fcu_link/status")),
    timeout_sec_(declare_parameter<double>("timeout_sec", 0.5)),
    check_rate_hz_(declare_parameter<double>("check_rate_hz", 5.0)),
    lost_threshold_count_(declare_parameter<int>("lost_threshold_count", 2)),
    ok_threshold_count_(declare_parameter<int>("ok_threshold_count", 2))
  {
    timeout_sec_ = std::max(0.01, timeout_sec_);
    check_rate_hz_ = std::max(1.0, check_rate_hz_);
    lost_threshold_count_ = std::max(1, lost_threshold_count_);
    ok_threshold_count_ = std::max(1, ok_threshold_count_);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      local_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FcuLinkMonitorNode::poseCallback, this, std::placeholders::_1));

    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    const auto timer_period = std::chrono::duration<double>(1.0 / check_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&FcuLinkMonitorNode::timerCallback, this));

    last_pose_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "fcu_link_monitor started: local_pose_topic=%s status_topic=%s timeout=%.3fs lost_threshold=%d ok_threshold=%d",
      local_pose_topic_.c_str(), status_topic_.c_str(), timeout_sec_,
      lost_threshold_count_, ok_threshold_count_);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {
    last_pose_time_ = now();
    has_pose_ = true;
  }

  void publishStatus(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    last_status_ = status;
    RCLCPP_INFO(get_logger(), "FCU link status -> %s", status.c_str());
  }

  void timerCallback()
  {
    if (!has_pose_) {
      lost_count_ = 0;
      ok_count_ = 0;
      publishStatus("WAIT_FCU");
      return;
    }

    const double dt = (now() - last_pose_time_).seconds();
    if (dt > timeout_sec_) {
      lost_count_++;
      ok_count_ = 0;
      if (lost_count_ >= lost_threshold_count_) {
        publishStatus("LOST");
      }
      return;
    }

    ok_count_++;
    lost_count_ = 0;
    if (ok_count_ >= ok_threshold_count_) {
      publishStatus("OK");
      return;
    }

    if (last_status_ == "INIT") {
      publishStatus("WAIT_FCU");
    }
  }

  std::string local_pose_topic_;
  std::string status_topic_;
  double timeout_sec_;
  double check_rate_hz_;
  int lost_threshold_count_;
  int ok_threshold_count_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_pose_time_;
  bool has_pose_{false};
  int lost_count_{0};
  int ok_count_{0};
  std::string last_status_{"INIT"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FcuLinkMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
