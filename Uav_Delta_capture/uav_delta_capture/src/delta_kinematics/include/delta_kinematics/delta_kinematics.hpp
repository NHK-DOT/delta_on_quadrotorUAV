#ifndef DELTA_KINEMATICS__DELTA_KINEMATICS_HPP_
#define DELTA_KINEMATICS__DELTA_KINEMATICS_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

namespace delta_kinematics
{

class DeltaKinematics
{
public:
  DeltaKinematics(double l1, double l2, double base_radius, double effector_radius);

  void setGeometry(double l1, double l2, double base_radius, double effector_radius);

  bool forwardKinematics(const Eigen::Vector3d & joint_angles_rad, Eigen::Vector3d & position) const;
  bool inverseKinematics(const Eigen::Vector3d & position, Eigen::Vector3d & joint_angles_rad) const;

private:
  double l1_;
  double l2_;
  double r_base_;
  double r_eff_;
};

class DeltaKinematicsNode : public rclcpp::Node
{
public:
  DeltaKinematicsNode();

private:
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void timerCallback();

  double l1_;
  double l2_;
  double r_base_;
  double r_eff_;
  std::string target_topic_;

  DeltaKinematics solver_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  Eigen::Vector3d latest_target_;
  bool has_target_;

  std_msgs::msg::Float64MultiArray joint_msg_;
};

}  // namespace delta_kinematics

#endif  // DELTA_KINEMATICS__DELTA_KINEMATICS_HPP_
