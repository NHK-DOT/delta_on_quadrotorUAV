/**
 * @file vision_transform_node.cpp
 * @brief 视觉像素偏移 → 机械臂坐标系目标点
 *
 * 订阅：
 *   - vision/target_offset (PointStamped)  — 视觉目标像素偏移（相机坐标系）
 *
 * 发布：
 *   - target_point (PointStamped)  — 变换后的目标点（机械臂坐标系 delta_base_link）
 *
 * 功能：
 *   1. 接收视觉模块发布的像素偏移（相机坐标系）
 *   2. 用 TF2 变换到机械臂坐标系（delta_base_link）
 *   3. 发布变换后的目标点，供机械臂运动学节点使用
 *
 * 设计原则：
 *   - 只做坐标变换，不做任何控制逻辑
 *   - 不依赖飞控、UWB 等其他模块
 *   - 可以独立测试：ros2 run vision_bridge vision_transform
 */

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class VisionTransformNode : public rclcpp::Node
{
public:
  VisionTransformNode()
  : Node("vision_transform_node"),
    camera_frame_(declare_parameter<std::string>("camera_frame", "camera_optical_frame")),
    target_frame_(declare_parameter<std::string>("target_frame", "delta_base_link")),
    vision_offset_topic_(declare_parameter<std::string>("vision_offset_topic", "vision/target_offset")),
    target_point_topic_(declare_parameter<std::string>("target_point_topic", "target_point")),
    tf_timeout_sec_(declare_parameter<double>("tf_timeout_sec", 0.05)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 订阅视觉偏移
    vision_offset_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      vision_offset_topic_, 20,
      std::bind(&VisionTransformNode::visionOffsetCallback, this, std::placeholders::_1));

    // 发布目标点
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(target_point_topic_, 20);

    RCLCPP_INFO(
      get_logger(),
      "vision_transform started: input=%s output=%s camera_frame=%s target_frame=%s",
      vision_offset_topic_.c_str(), target_point_topic_.c_str(),
      camera_frame_.c_str(), target_frame_.c_str());
  }

private:
  // ── 视觉偏移回调 ─────────────────────────────────────────────────────

  void visionOffsetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // 构造输入点（相机坐标系）
    geometry_msgs::msg::PointStamped in_camera;
    in_camera.header = msg->header;
    in_camera.point = msg->point;

    // 如果没有 frame_id，使用默认的相机坐标系
    if (in_camera.header.frame_id.empty()) {
      in_camera.header.frame_id = camera_frame_;
    }

    // TF2 变换：相机坐标系 → 机械臂坐标系
    try {
      auto in_arm = tf_buffer_.transform(
        in_camera, target_frame_, tf2::durationFromSec(tf_timeout_sec_));

      target_pub_->publish(in_arm);

      RCLCPP_DEBUG(
        get_logger(),
        "transformed: camera[%.3f, %.3f, %.3f] → %s[%.3f, %.3f, %.3f]",
        msg->point.x, msg->point.y, msg->point.z,
        target_frame_.c_str(),
        in_arm.point.x, in_arm.point.y, in_arm.point.z);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF transform failed (%s → %s): %s",
        in_camera.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  // ── 参数 ─────────────────────────────────────────────────────────────

  std::string camera_frame_;
  std::string target_frame_;
  std::string vision_offset_topic_;
  std::string target_point_topic_;
  double tf_timeout_sec_;

  // ── TF2 ──────────────────────────────────────────────────────────────

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ── 订阅/发布 ────────────────────────────────────────────────────────

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vision_offset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionTransformNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
