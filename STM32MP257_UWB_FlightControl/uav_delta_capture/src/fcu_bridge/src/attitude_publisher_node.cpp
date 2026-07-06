/**
 * @file attitude_publisher_node.cpp
 * @brief 从飞控提取姿态角，发布给其他节点使用
 *
 * 订阅：
 *   - /mavros/local_position/pose (PoseStamped)  — 飞控本地位姿
 *   - /mavros/imu/data (Imu)                      — IMU 原始数据
 *
 * 发布：
 *   - fcu/local_attitude (Vector3Stamped)  — 本地位姿的 roll/pitch/yaw
 *   - fcu/imu_attitude (Vector3Stamped)    — IMU 的 roll/pitch/yaw
 *
 * 功能：
 *   1. 从 PoseStamped 提取 roll/pitch/yaw，发布为 Vector3Stamped
 *   2. 从 Imu 提取 roll/pitch/yaw，发布为 Vector3Stamped
 *   3. 每秒打印一次诊断日志
 *
 * 设计原则：
 *   - 只做姿态提取，不做任何控制逻辑
 *   - 不依赖视觉、UWB 等其他模块
 *   - 可以独立测试：ros2 run fcu_bridge attitude_publisher
 */

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class AttitudePublisherNode : public rclcpp::Node
{
public:
  AttitudePublisherNode()
  : Node("attitude_publisher_node"),
    local_pose_topic_(declare_parameter<std::string>("local_pose_topic", "/mavros/local_position/pose")),
    imu_topic_(declare_parameter<std::string>("imu_topic", "/mavros/imu/data")),
    local_attitude_topic_(declare_parameter<std::string>("local_attitude_topic", "fcu/local_attitude")),
    imu_attitude_topic_(declare_parameter<std::string>("imu_attitude_topic", "fcu/imu_attitude"))
  {
    const auto best_effort = rclcpp::SensorDataQoS();

    // 订阅飞控位姿
    local_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      local_pose_topic_, best_effort,
      std::bind(&AttitudePublisherNode::localPoseCallback, this, std::placeholders::_1));

    // 订阅 IMU
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, best_effort,
      std::bind(&AttitudePublisherNode::imuCallback, this, std::placeholders::_1));

    // 发布姿态
    local_attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      local_attitude_topic_, 20);
    imu_attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      imu_attitude_topic_, 50);

    // 诊断定时器
    diag_timer_ = create_wall_timer(1000ms, std::bind(&AttitudePublisherNode::diagCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "attitude_publisher started: pose=%s imu=%s",
      local_pose_topic_.c_str(), imu_topic_.c_str());
  }

private:
  // ── 四元数 → roll/pitch/yaw ──────────────────────────────────────────

  static geometry_msgs::msg::Vector3Stamped quaternionToRpy(
    const geometry_msgs::msg::Quaternion & q,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
  {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf_q.normalize();

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.vector.x = roll;
    msg.vector.y = pitch;
    msg.vector.z = yaw;
    return msg;
  }

  // ── 回调函数 ─────────────────────────────────────────────────────────

  void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 提取姿态角并发布
    latest_local_attitude_ = quaternionToRpy(
      msg->pose.orientation, msg->header.stamp, msg->header.frame_id);
    local_attitude_pub_->publish(latest_local_attitude_);

    // 保存位姿用于诊断
    latest_local_pose_ = *msg;
    has_local_pose_ = true;

    RCLCPP_DEBUG(
      get_logger(),
      "local_attitude: roll=%.3f pitch=%.3f yaw=%.3f",
      latest_local_attitude_.vector.x,
      latest_local_attitude_.vector.y,
      latest_local_attitude_.vector.z);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 提取姿态角并发布
    latest_imu_attitude_ = quaternionToRpy(
      msg->orientation, msg->header.stamp, msg->header.frame_id);
    imu_attitude_pub_->publish(latest_imu_attitude_);

    has_imu_ = true;

    RCLCPP_DEBUG(
      get_logger(),
      "imu_attitude: roll=%.3f pitch=%.3f yaw=%.3f",
      latest_imu_attitude_.vector.x,
      latest_imu_attitude_.vector.y,
      latest_imu_attitude_.vector.z);
  }

  void diagCallback()
  {
    if (!has_local_pose_) {
      RCLCPP_INFO(get_logger(), "waiting %s ...", local_pose_topic_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "altitude=%.3f rpy(rad)=[%.3f, %.3f, %.3f] imu=%s",
      latest_local_pose_.pose.position.z,
      latest_local_attitude_.vector.x,
      latest_local_attitude_.vector.y,
      latest_local_attitude_.vector.z,
      has_imu_ ? "OK" : "WAIT");
  }

  // ── 参数 ─────────────────────────────────────────────────────────────

  std::string local_pose_topic_;
  std::string imu_topic_;
  std::string local_attitude_topic_;
  std::string imu_attitude_topic_;

  // ── 订阅/发布 ────────────────────────────────────────────────────────

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr local_attitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_attitude_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // ── 状态 ─────────────────────────────────────────────────────────────

  geometry_msgs::msg::PoseStamped latest_local_pose_;
  geometry_msgs::msg::Vector3Stamped latest_local_attitude_;
  geometry_msgs::msg::Vector3Stamped latest_imu_attitude_;
  bool has_local_pose_{false};
  bool has_imu_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttitudePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
