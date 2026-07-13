#include <chrono>
#include <cmath>
#include <cctype>
#include <condition_variable>
#include <functional>
#include <set>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "uav_delta_msgs/msg/fcu_state.hpp"
#include "uav_delta_msgs/srv/flight_command.hpp"

using namespace std::chrono_literals;

class FlightCommanderNode : public rclcpp::Node
{
public:
  FlightCommanderNode()
  : Node("flight_commander_node"),
    service_topic_(declare_parameter<std::string>("service_topic", "flight_command")),
    fcu_state_topic_(declare_parameter<std::string>("fcu_state_topic", "fcu_state")),
    cmd_vel_topic_(declare_parameter<std::string>("cmd_vel_topic", "cmd_vel")),
    mavros_arm_topic_(declare_parameter<std::string>("mavros_arm_topic", "/mavros/cmd/arming")),
    mavros_set_mode_topic_(declare_parameter<std::string>("mavros_set_mode_topic", "/mavros/set_mode")),
    mavros_vel_topic_(declare_parameter<std::string>("mavros_vel_topic", "/mavros/setpoint_velocity/cmd_vel")),
    mavros_takeoff_topic_(declare_parameter<std::string>("mavros_takeoff_topic", "/mavros/cmd/takeoff")),
    takeoff_target_clearance_m_(declare_parameter<double>("takeoff_target_clearance_m", 0.2)),
    vel_forward_rate_hz_(declare_parameter<double>("vel_forward_rate_hz", 20.0)),
    vel_timeout_sec_(declare_parameter<double>("vel_timeout_sec", 0.5)),
    use_mock_(declare_parameter<bool>("use_mock", false)),
    max_vel_xy_(declare_parameter<double>("max_vel_xy", 1.0)),
    max_vel_z_(declare_parameter<double>("max_vel_z", 0.5)),
    skip_ekf_check_(declare_parameter<bool>("skip_ekf_check", false)),
    auto_vel_modes_(parseModes(declare_parameter<std::string>("auto_vel_modes", "GUIDED")))
  {
    vel_forward_rate_hz_ = std::max(1.0, vel_forward_rate_hz_);
    vel_timeout_sec_ = std::max(0.05, vel_timeout_sec_);
    max_vel_xy_ = std::max(0.1, max_vel_xy_);
    max_vel_z_ = std::max(0.1, max_vel_z_);
    takeoff_target_clearance_m_ = std::max(0.05, takeoff_target_clearance_m_);
    last_vel_time_ = now();

    // Use a reentrant callback group so async service calls from within
    // the service callback don't deadlock with the executor.
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    srv_ = create_service<uav_delta_msgs::srv::FlightCommand>(
      service_topic_,
      std::bind(&FlightCommanderNode::commandCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_default,
      cb_group_);

    // FCU state subscription
    state_sub_ = create_subscription<uav_delta_msgs::msg::FcuState>(
      fcu_state_topic_, 10,
      [this](const uav_delta_msgs::msg::FcuState::SharedPtr msg) {
        last_state_ = msg;
        has_state_ = true;
      });

    // Velocity command subscription
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_, 20,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        last_vel_ = msg;
        has_vel_ = true;
        last_vel_time_ = now();
      });

    // Velocity publisher (to MAVROS)
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(mavros_vel_topic_, 20);

    // MAVROS service clients (share the reentrant callback group)
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>(
      mavros_arm_topic_, rmw_qos_profile_default, cb_group_);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(
      mavros_set_mode_topic_, rmw_qos_profile_default, cb_group_);
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>(
      mavros_takeoff_topic_, rmw_qos_profile_default, cb_group_);

    // Timer for republishing velocity
    const auto vel_period = std::chrono::duration<double>(1.0 / vel_forward_rate_hz_);
    vel_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(vel_period),
      std::bind(&FlightCommanderNode::velTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "flight_commander_node started: service=%s mock=%s max_vel_xy=%.1f max_vel_z=%.1f vel_timeout=%.2fs skip_ekf=%s",
      service_topic_.c_str(), use_mock_ ? "true" : "false", max_vel_xy_, max_vel_z_,
      vel_timeout_sec_, skip_ekf_check_ ? "true" : "false");
  }

private:
  void commandCallback(
    const std::shared_ptr<uav_delta_msgs::srv::FlightCommand::Request> request,
    std::shared_ptr<uav_delta_msgs::srv::FlightCommand::Response> response)
  {
    const int cmd = request->command;

    if (use_mock_) {
      response->success = true;
      response->message = "mock: command " + std::to_string(cmd) + " accepted";
      RCLCPP_INFO(get_logger(), "Mock command %d accepted", cmd);
      return;
    }

    switch (cmd) {
      case uav_delta_msgs::srv::FlightCommand::Request::CMD_ARM:
        handleArm(true, response);
        break;
      case uav_delta_msgs::srv::FlightCommand::Request::CMD_DISARM:
        handleArm(false, response);
        break;
      case uav_delta_msgs::srv::FlightCommand::Request::CMD_MODE_GUIDED:
        handleSetMode("GUIDED", response);
        break;
      case uav_delta_msgs::srv::FlightCommand::Request::CMD_MODE_LOITER:
        handleSetMode("LOITER", response);
        break;
      case uav_delta_msgs::srv::FlightCommand::Request::CMD_MODE_LAND:
        handleSetMode("LAND", response);
        break;
      case uav_delta_msgs::srv::FlightCommand::Request::CMD_TAKEOFF:
        handleTakeoff(request->param, response);
        break;
      default:
        response->success = false;
        response->message = "unknown command: " + std::to_string(cmd);
        RCLCPP_WARN(get_logger(), "Unknown command: %d", cmd);
        break;
    }
  }

  void handleArm(
    bool arm,
    std::shared_ptr<uav_delta_msgs::srv::FlightCommand::Response> response)
  {
    if (!has_state_ || !last_state_->connected) {
      response->success = false;
      response->message = "FCU not connected";
      RCLCPP_WARN(get_logger(), "Arm rejected: FCU not connected");
      return;
    }

    if (arm && !skip_ekf_check_ && !last_state_->estimator_ok) {
      response->success = false;
      response->message = "EKF not ready, cannot arm";
      RCLCPP_WARN(get_logger(), "Arm rejected: EKF not ok");
      return;
    }

    if (!arm_client_->service_is_ready()) {
      response->success = false;
      response->message = "MAVROS arming service not available";
      RCLCPP_ERROR(get_logger(), "Arming service not ready");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;

    arm_client_->async_send_request(req,
      [this, arm, response, &mtx, &cv, &done](
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
      {
        auto result = future.get();
        // 判定以飞控的 MAV_RESULT 为准: result->result == 0 (MAV_RESULT_ACCEPTED) 才算成功。
        // 实测样本: 成功 success=True/result=0; 失败 success=False/result=4(FAILED)。
        // 不能用 result->result != 0 判成功 (那会把 FAILED=4 误判为接受)。
        const bool accepted = (result->result == 0);
        response->success = accepted;
        response->message = accepted ? (arm ? "armed" : "disarmed")
                                     : "arm/disarm rejected by FCU (result=" + std::to_string(result->result) + ")";
        RCLCPP_INFO(get_logger(), "Arm(%s) raw: success=%d result=%u -> %s",
                    arm ? "true" : "false",
                    static_cast<int>(result->success),
                    static_cast<unsigned>(result->result),
                    response->message.c_str());
        {
          std::lock_guard<std::mutex> lk(mtx);
          done = true;
        }
        cv.notify_one();
      });

    std::unique_lock<std::mutex> lk(mtx);
    if (!cv.wait_for(lk, 5s, [&done]{ return done; })) {
      response->success = false;
      response->message = "arming service call failed (timeout)";
      RCLCPP_ERROR(get_logger(), "Arming service call timeout");
    }
  }

  void handleSetMode(
    const std::string & mode,
    std::shared_ptr<uav_delta_msgs::srv::FlightCommand::Response> response)
  {
    if (!has_state_ || !last_state_->connected) {
      response->success = false;
      response->message = "FCU not connected";
      RCLCPP_WARN(get_logger(), "SetMode rejected: FCU not connected");
      return;
    }

    if (!set_mode_client_->service_is_ready()) {
      response->success = false;
      response->message = "MAVROS set_mode service not available";
      RCLCPP_ERROR(get_logger(), "SetMode service not ready");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;

    set_mode_client_->async_send_request(req,
      [this, mode, response, &mtx, &cv, &done](
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
      {
        auto result = future.get();
        response->success = result->mode_sent;
        response->message = result->mode_sent ? ("mode -> " + mode) : "mode change rejected";
        RCLCPP_INFO(get_logger(), "SetMode(%s) result: %s", mode.c_str(), response->message.c_str());
        {
          std::lock_guard<std::mutex> lk(mtx);
          done = true;
        }
        cv.notify_one();
      });

    std::unique_lock<std::mutex> lk(mtx);
    if (!cv.wait_for(lk, 5s, [&done]{ return done; })) {
      response->success = false;
      response->message = "set_mode service call failed (timeout)";
      RCLCPP_ERROR(get_logger(), "SetMode service call timeout");
    }
  }

  void handleTakeoff(
    float altitude,
    std::shared_ptr<uav_delta_msgs::srv::FlightCommand::Response> response)
  {
    if (!has_state_ || !last_state_->connected) {
      response->success = false;
      response->message = "FCU not connected";
      return;
    }

    if (!last_state_->armed) {
      response->success = false;
      response->message = "must arm before takeoff";
      return;
    }

    if (altitude <= 0.0f) {
      altitude = 1.0f;  // default 1m
    }

    float target_altitude = altitude;
    float current_local_z = 0.0f;
    if (has_state_ && last_state_) {
      current_local_z = std::max(0.0f, last_state_->local_z);
      target_altitude = std::max(
        target_altitude,
        current_local_z + static_cast<float>(takeoff_target_clearance_m_));
    }

    // Switch to GUIDED mode (fire-and-forget, don't block)
    if (set_mode_client_->service_is_ready()) {
      auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      mode_req->custom_mode = "GUIDED";
      set_mode_client_->async_send_request(mode_req,
        [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture) {});
    }

    // Call MAVROS takeoff service (MAV_CMD_NAV_TAKEOFF = 22)
    if (!takeoff_client_->service_is_ready()) {
      response->success = false;
      response->message = "MAVROS takeoff service not available";
      RCLCPP_ERROR(get_logger(), "Takeoff service not ready");
      return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->min_pitch = 0.0f;
    req->yaw = 0.0f;
    req->latitude = 0.0;
    req->longitude = 0.0;
    req->altitude = target_altitude;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;

    takeoff_client_->async_send_request(req,
      [this, altitude, target_altitude, current_local_z, response, &mtx, &cv, &done](
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future)
      {
        auto result = future.get();
        response->success = result->success;
        response->message = result->success
          ? ("takeoff to " + std::to_string(altitude) + "m accepted (service_target=" +
            std::to_string(target_altitude) + "m)")
          : "takeoff command rejected by FCU";
        RCLCPP_INFO(
          get_logger(), "Takeoff(request=%.1fm service_target=%.2fm current_local_z=%.2fm) result: %s",
          altitude, target_altitude, current_local_z, response->message.c_str());
        {
          std::lock_guard<std::mutex> lk(mtx);
          done = true;
        }
        cv.notify_one();
      });

    std::unique_lock<std::mutex> lk(mtx);
    if (!cv.wait_for(lk, 5s, [&done]{ return done; })) {
      response->success = false;
      response->message = "takeoff service call failed (timeout)";
      RCLCPP_ERROR(get_logger(), "Takeoff service call timeout");
    }

    takeoff_altitude_ = target_altitude;
  }

  void velTimerCallback()
  {
    if (!has_vel_ || !last_vel_) {
      return;
    }

    if (use_mock_) {
      return;
    }

    auto clamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
    clamped->header.stamp = now();
    clamped->header.frame_id = last_vel_->header.frame_id.empty() ? "body" : last_vel_->header.frame_id;

    const double age = (now() - last_vel_time_).seconds();
    if (!velocityForwardingAllowed()) {
      clamped->twist.linear.x = 0.0;
      clamped->twist.linear.y = 0.0;
      clamped->twist.linear.z = 0.0;
      clamped->twist.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Velocity forwarding blocked by FCU mode/state, publishing zero setpoint");
    } else if (age > vel_timeout_sec_) {
      clamped->twist.linear.x = 0.0;
      clamped->twist.linear.y = 0.0;
      clamped->twist.linear.z = 0.0;
      clamped->twist.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Velocity command stale (age=%.2fs), publishing zero setpoint", age);
    } else {
      clamped->twist.linear.x = clamp(last_vel_->twist.linear.x, -max_vel_xy_, max_vel_xy_);
      clamped->twist.linear.y = clamp(last_vel_->twist.linear.y, -max_vel_xy_, max_vel_xy_);
      clamped->twist.linear.z = clamp(last_vel_->twist.linear.z, -max_vel_z_, max_vel_z_);
      clamped->twist.angular.z = last_vel_->twist.angular.z;
    }

    vel_pub_->publish(*clamped);
  }

  bool velocityForwardingAllowed() const
  {
    if (!has_state_ || !last_state_) {
      return false;
    }
    if (!last_state_->armed) {
      return false;
    }
    return auto_vel_modes_.count(toUpper(last_state_->mode)) > 0;
  }

  static std::string toUpper(std::string value)
  {
    for (char & c : value) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return value;
  }

  static std::set<std::string> parseModes(const std::string & text)
  {
    std::set<std::string> modes;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
      std::string cleaned;
      for (char c : item) {
        if (!std::isspace(static_cast<unsigned char>(c))) {
          cleaned.push_back(c);
        }
      }
      if (!cleaned.empty()) {
        modes.insert(toUpper(cleaned));
      }
    }
    if (modes.empty()) {
      modes.insert("GUIDED");
    }
    return modes;
  }

  static double clamp(double val, double lo, double hi)
  {
    return std::max(lo, std::min(hi, val));
  }

  // Parameters
  std::string service_topic_;
  std::string fcu_state_topic_;
  std::string cmd_vel_topic_;
  std::string mavros_arm_topic_;
  std::string mavros_set_mode_topic_;
  std::string mavros_vel_topic_;
  std::string mavros_takeoff_topic_;
  double takeoff_target_clearance_m_;
  double vel_forward_rate_hz_;
  double vel_timeout_sec_;
  bool use_mock_;
  double max_vel_xy_;
  double max_vel_z_;
  bool skip_ekf_check_;
  std::set<std::string> auto_vel_modes_;

  // Callback group (reentrant to allow nested async service calls)
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Subscribers
  rclcpp::Subscription<uav_delta_msgs::msg::FcuState>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

  // Service server
  rclcpp::Service<uav_delta_msgs::srv::FlightCommand>::SharedPtr srv_;

  // MAVROS service clients
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr vel_timer_;

  // Cached state
  uav_delta_msgs::msg::FcuState::SharedPtr last_state_;
  geometry_msgs::msg::TwistStamped::SharedPtr last_vel_;
  bool has_state_{false};
  bool has_vel_{false};
  rclcpp::Time last_vel_time_{0, 0, RCL_ROS_TIME};
  float takeoff_altitude_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlightCommanderNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
