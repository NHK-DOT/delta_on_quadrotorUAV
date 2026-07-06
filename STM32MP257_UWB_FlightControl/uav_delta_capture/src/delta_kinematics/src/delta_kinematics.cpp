#include "delta_kinematics/delta_kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace delta_kinematics
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

DeltaKinematics::DeltaKinematics(double l1, double l2, double base_radius, double effector_radius)
: l1_(l1), l2_(l2), r_base_(base_radius), r_eff_(effector_radius)
{
}

void DeltaKinematics::setGeometry(double l1, double l2, double base_radius, double effector_radius)
{
  l1_ = l1;
  l2_ = l2;
  r_base_ = base_radius;
  r_eff_ = effector_radius;
}

bool DeltaKinematics::inverseKinematics(
  const Eigen::Vector3d & position,
  Eigen::Vector3d & joint_angles_rad) const
{
  if (l1_ <= 0.0 || l2_ <= 0.0) {
    return false;
  }

  static const std::array<double, 3> arm_angles = {0.0, 2.0 * kPi / 3.0, -2.0 * kPi / 3.0};

  for (std::size_t i = 0; i < arm_angles.size(); ++i) {
    const double phi = arm_angles[i];
    Eigen::Matrix2d rot;
    rot << std::cos(phi), std::sin(phi),
      -std::sin(phi), std::cos(phi);

    const Eigen::Vector2d p_xy_local = rot * position.head<2>();
    const double x_local = p_xy_local.x() + (r_base_ - r_eff_);
    const double z_local = position.z();

    const double d = std::sqrt(x_local * x_local + z_local * z_local);
    if (d > (l1_ + l2_) || d < std::abs(l1_ - l2_)) {
      return false;
    }

    const double cos_beta = std::clamp((l1_ * l1_ + d * d - l2_ * l2_) / (2.0 * l1_ * d), -1.0, 1.0);
    const double beta = std::acos(cos_beta);
    const double alpha = std::atan2(-z_local, x_local);

    joint_angles_rad(static_cast<Eigen::Index>(i)) = alpha + beta;
  }

  return true;
}

bool DeltaKinematics::forwardKinematics(
  const Eigen::Vector3d & joint_angles_rad,
  Eigen::Vector3d & position) const
{
  if (l1_ <= 0.0 || l2_ <= 0.0) {
    return false;
  }

  static const std::array<double, 3> arm_angles = {0.0, 2.0 * kPi / 3.0, -2.0 * kPi / 3.0};
  Eigen::Vector3d estimate = Eigen::Vector3d::Zero();

  for (std::size_t i = 0; i < arm_angles.size(); ++i) {
    const double theta = joint_angles_rad(static_cast<Eigen::Index>(i));
    const double phi = arm_angles[i];

    const double x_local = (r_base_ - r_eff_) + l1_ * std::cos(theta);
    const double z_local = -l1_ * std::sin(theta) - l2_;

    Eigen::Matrix2d inv_rot;
    inv_rot << std::cos(phi), -std::sin(phi),
      std::sin(phi), std::cos(phi);

    const Eigen::Vector2d xy_global = inv_rot * Eigen::Vector2d(x_local, 0.0);
    estimate.x() += xy_global.x();
    estimate.y() += xy_global.y();
    estimate.z() += z_local;
  }

  position = estimate / 3.0;
  return true;
}

}  // namespace delta_kinematics
