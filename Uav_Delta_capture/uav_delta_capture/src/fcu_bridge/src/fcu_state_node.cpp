#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "uav_delta_msgs/msg/fcu_state.hpp"

using namespace std::chrono_literals;

class FcuStateNode : public rclcpp::Node
{
public:
  FcuStateNode()
  : Node("fcu_state_node"),
    state_topic_(declare_parameter<std::string>("state_topic", "/mavros/state")),
    battery_topic_(declare_parameter<std::string>("battery_topic", "/mavros/battery")),
    gps_topic_(declare_parameter<std::string>("gps_topic", "/mavros/gpsstatus/gps")),
    ekf_topic_(declare_parameter<std::string>("ekf_topic", "/mavros/estimator_status")),
    pose_topic_(declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose")),
    output_topic_(declare_parameter<std::string>("output_topic", "fcu_state")),
    publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 10.0)),
    use_mock_(declare_parameter<bool>("use_mock", false)),
    mock_mode_(declare_parameter<std::string>("mock_mode", "GUIDED")),
    mock_armed_(declare_parameter<bool>("mock_armed", false)),
    mock_altitude_(declare_parameter<double>("mock_altitude", 1.0))
  {
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);

    pub_ = create_publisher<uav_delta_msgs::msg::FcuState>(output_topic_, 10);

    if (!use_mock_) {
      const auto best_effort = rclcpp::SensorDataQoS();

      state_sub_ = create_subscription<mavros_msgs::msg::State>(
        state_topic_, best_effort,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
          last_state_ = msg;
          has_state_ = true;
        });

      battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
        battery_topic_, best_effort,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
          last_battery_ = msg;
          has_battery_ = true;
        });

      gps_sub_ = create_subscription<mavros_msgs::msg::GPSRAW>(
        gps_topic_, best_effort,
        [this](const mavros_msgs::msg::GPSRAW::SharedPtr msg) {
          last_gps_ = msg;
          has_gps_ = true;
        });

      ekf_sub_ = create_subscription<mavros_msgs::msg::EstimatorStatus>(
        ekf_topic_, best_effort,
        [this](const mavros_msgs::msg::EstimatorStatus::SharedPtr msg) {
          last_ekf_ = msg;
          has_ekf_ = true;
        });

      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, best_effort,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          last_pose_ = msg;
          has_pose_ = true;
        });
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FcuStateNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "fcu_state_node started: output=%s rate=%.1fHz mock=%s",
      output_topic_.c_str(), publish_rate_hz_, use_mock_ ? "true" : "false");
  }

private:
  void timerCallback()
  {
    uav_delta_msgs::msg::FcuState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "fcu";

    if (use_mock_) {
      fillMockState(msg);
    } else {
      fillRealState(msg);
    }

    pub_->publish(msg);
  }

  void fillMockState(uav_delta_msgs::msg::FcuState & msg)
  {
    msg.connected = true;
    msg.mode = mock_mode_;
    msg.armed = mock_armed_;
    msg.manual_input = false;

    msg.voltage = 12.6f;
    msg.remaining = 0.85f;
    msg.current = 2.0f;

    msg.fix_type = 0;  // no GPS for optical flow mode
    msg.satellites_visible = 0;

    msg.attitude_status = true;
    msg.vel_horiz_status = true;
    msg.vel_vert_status = true;
    msg.pos_horiz_status = true;
    msg.estimator_ok = true;

    msg.local_x = 0.0f;
    msg.local_y = 0.0f;
    msg.local_z = static_cast<float>(mock_altitude_);
    msg.roll = 0.0f;
    msg.pitch = 0.0f;
    msg.yaw = 0.0f;
  }

  void fillRealState(uav_delta_msgs::msg::FcuState & msg)
  {
    // State (connection, mode, armed)
    if (has_state_ && last_state_) {
      msg.connected = last_state_->connected;
      msg.mode = last_state_->mode;
      msg.armed = last_state_->armed;
    } else {
      msg.connected = false;
      msg.mode = "";
      msg.armed = false;
    }
    msg.manual_input = false;  // TODO: check manual_control topic if needed

    // Battery
    if (has_battery_ && last_battery_) {
      msg.voltage = last_battery_->voltage;
      msg.remaining = last_battery_->percentage;
      msg.current = last_battery_->current;
    } else {
      msg.voltage = 0.0f;
      msg.remaining = 0.0f;
      msg.current = 0.0f;
    }

    // GPS
    if (has_gps_ && last_gps_) {
      msg.fix_type = last_gps_->fix_type;
      msg.satellites_visible = last_gps_->satellites_visible;
    } else {
      msg.fix_type = 0;
      msg.satellites_visible = 0;
    }

    // Estimator
    if (has_ekf_ && last_ekf_) {
      msg.attitude_status = last_ekf_->attitude_status_flag;
      msg.vel_horiz_status = last_ekf_->velocity_horiz_status_flag;
      msg.vel_vert_status = last_ekf_->velocity_vert_status_flag;
      msg.pos_horiz_status = last_ekf_->pos_horiz_abs_status_flag;
      // pos_horiz_abs 需要 GPS/外部绝对定位, 室内光流/UWB 方案永远为 false,
      // 不纳入 estimator_ok 判定, 否则室内无法解锁
      msg.estimator_ok = msg.attitude_status && msg.vel_horiz_status &&
                         msg.vel_vert_status;
    } else {
      msg.attitude_status = false;
      msg.vel_horiz_status = false;
      msg.vel_vert_status = false;
      msg.pos_horiz_status = false;
      msg.estimator_ok = false;
    }

    // Local position
    if (has_pose_ && last_pose_) {
      msg.local_x = static_cast<float>(last_pose_->pose.position.x);
      msg.local_y = static_cast<float>(last_pose_->pose.position.y);
      msg.local_z = static_cast<float>(last_pose_->pose.position.z);

      double roll, pitch, yaw;
      tf2::Quaternion q(
        last_pose_->pose.orientation.x,
        last_pose_->pose.orientation.y,
        last_pose_->pose.orientation.z,
        last_pose_->pose.orientation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      msg.roll = static_cast<float>(roll);
      msg.pitch = static_cast<float>(pitch);
      msg.yaw = static_cast<float>(yaw);
    } else {
      msg.local_x = 0.0f;
      msg.local_y = 0.0f;
      msg.local_z = 0.0f;
      msg.roll = 0.0f;
      msg.pitch = 0.0f;
      msg.yaw = 0.0f;
    }
  }

  // Parameters
  std::string state_topic_;
  std::string battery_topic_;
  std::string gps_topic_;
  std::string ekf_topic_;
  std::string pose_topic_;
  std::string output_topic_;
  double publish_rate_hz_;
  bool use_mock_;
  std::string mock_mode_;
  bool mock_armed_;
  double mock_altitude_;

  // Subscribers
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr gps_sub_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr ekf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publisher
  rclcpp::Publisher<uav_delta_msgs::msg::FcuState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Cached messages
  mavros_msgs::msg::State::SharedPtr last_state_;
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_;
  mavros_msgs::msg::GPSRAW::SharedPtr last_gps_;
  mavros_msgs::msg::EstimatorStatus::SharedPtr last_ekf_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_pose_;

  bool has_state_{false};
  bool has_battery_{false};
  bool has_gps_{false};
  bool has_ekf_{false};
  bool has_pose_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FcuStateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
