#include "delta_kinematics/delta_kinematics.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace delta_kinematics
{

DeltaKinematicsNode::DeltaKinematicsNode()
: Node("delta_kinematics_node"),
  l1_(declare_parameter<double>("L1", 100.0)),
  l2_(declare_parameter<double>("L2", 150.0)),
  r_base_(declare_parameter<double>("R", 55.0)),
  r_eff_(declare_parameter<double>("r", 20.0)),
  target_topic_(declare_parameter<std::string>("target_topic", "target_point")),
  solver_(l1_, l2_, r_base_, r_eff_),
  latest_target_(Eigen::Vector3d::Zero()),
  has_target_(false)
{
  joint_msg_.data.resize(3, 0.0);

  target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    target_topic_, 10,
    std::bind(&DeltaKinematicsNode::targetCallback, this, std::placeholders::_1));

  joint_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("joint_angles", 10);

  publish_timer_ = create_wall_timer(20ms, std::bind(&DeltaKinematicsNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "DeltaKinematicsNode started: L1=%.2f, L2=%.2f, R=%.2f, r=%.2f, target_topic=%s",
    l1_, l2_, r_base_, r_eff_, target_topic_.c_str());
}

void DeltaKinematicsNode::targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  latest_target_.x() = msg->point.x;
  latest_target_.y() = msg->point.y;
  latest_target_.z() = msg->point.z;
  has_target_ = true;

  RCLCPP_DEBUG(
    get_logger(),
    "Received target_point: [%.3f, %.3f, %.3f] frame=%s",
    latest_target_.x(), latest_target_.y(), latest_target_.z(), msg->header.frame_id.c_str());
}

void DeltaKinematicsNode::timerCallback()
{
  if (!has_target_) {
    return;
  }

  Eigen::Vector3d joints_rad;
  if (!solver_.inverseKinematics(latest_target_, joints_rad)) {
    RCLCPP_DEBUG(get_logger(), "IK failed for current target.");
    return;
  }

  joint_msg_.data[0] = joints_rad.x();
  joint_msg_.data[1] = joints_rad.y();
  joint_msg_.data[2] = joints_rad.z();
  joint_pub_->publish(joint_msg_);

  RCLCPP_DEBUG(
    get_logger(),
    "Published joint_angles(rad): [%.4f, %.4f, %.4f]",
    joint_msg_.data[0], joint_msg_.data[1], joint_msg_.data[2]);
}

}  // namespace delta_kinematics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<delta_kinematics::DeltaKinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
