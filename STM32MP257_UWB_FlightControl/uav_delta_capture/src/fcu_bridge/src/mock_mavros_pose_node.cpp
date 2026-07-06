#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class MockMavrosPoseNode : public rclcpp::Node
{
public:
  MockMavrosPoseNode()
  : Node("mock_mavros_pose_node"),
    topic_name_(declare_parameter<std::string>("topic_name", "/mavros/local_position/pose")),
    frame_id_(declare_parameter<std::string>("frame_id", "map")),
    trajectory_mode_(declare_parameter<std::string>("trajectory_mode", "hover")),
    publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 30.0)),
    origin_x_(declare_parameter<double>("origin_x", 0.0)),
    origin_y_(declare_parameter<double>("origin_y", 0.0)),
    origin_z_(declare_parameter<double>("origin_z", 1.0)),
    amplitude_x_(declare_parameter<double>("amplitude_x", 0.6)),
    amplitude_y_(declare_parameter<double>("amplitude_y", 0.6)),
    amplitude_z_(declare_parameter<double>("amplitude_z", 0.2)),
    circle_period_sec_(declare_parameter<double>("circle_period_sec", 12.0)),
    line_period_sec_(declare_parameter<double>("line_period_sec", 8.0)),
    yaw_rate_rad_s_(declare_parameter<double>("yaw_rate_rad_s", 0.25))
  {
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    circle_period_sec_ = std::max(0.1, circle_period_sec_);
    line_period_sec_ = std::max(0.1, line_period_sec_);

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_name_, rclcpp::SensorDataQoS());

    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&MockMavrosPoseNode::timerCallback, this));

    start_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "mock_mavros_pose_node started: topic=%s frame_id=%s mode=%s rate=%.2fHz",
      topic_name_.c_str(), frame_id_.c_str(), trajectory_mode_.c_str(), publish_rate_hz_);
  }

private:
  void timerCallback()
  {
    const double t = (now() - start_time_).seconds();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    if (trajectory_mode_ == "circle") {
      const double w = 2.0 * M_PI / circle_period_sec_;
      msg.pose.position.x = origin_x_ + amplitude_x_ * std::cos(w * t);
      msg.pose.position.y = origin_y_ + amplitude_y_ * std::sin(w * t);
      msg.pose.position.z = origin_z_ + amplitude_z_ * std::sin(0.5 * w * t);
    } else if (trajectory_mode_ == "line") {
      const double w = 2.0 * M_PI / line_period_sec_;
      msg.pose.position.x = origin_x_ + amplitude_x_ * std::sin(w * t);
      msg.pose.position.y = origin_y_;
      msg.pose.position.z = origin_z_ + amplitude_z_ * std::sin(0.5 * w * t);
    } else {
      msg.pose.position.x = origin_x_;
      msg.pose.position.y = origin_y_;
      msg.pose.position.z = origin_z_;
    }

    const double yaw = yaw_rate_rad_s_ * t;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    msg.pose.orientation = tf2::toMsg(q);

    pub_->publish(msg);
  }

  std::string topic_name_;
  std::string frame_id_;
  std::string trajectory_mode_;

  double publish_rate_hz_;
  double origin_x_;
  double origin_y_;
  double origin_z_;
  double amplitude_x_;
  double amplitude_y_;
  double amplitude_z_;
  double circle_period_sec_;
  double line_period_sec_;
  double yaw_rate_rad_s_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MockMavrosPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
