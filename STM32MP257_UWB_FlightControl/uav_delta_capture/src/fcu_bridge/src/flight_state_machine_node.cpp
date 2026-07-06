#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class FlightStateMachineNode : public rclcpp::Node
{
public:
  // ── 飞行状态 ─────────────────────────────────────────────────────

  enum class Phase {
    INIT,
    WAIT_FCU,
    TRACKING,
    RECOVERING,
    FAILSAFE
  };

  FlightStateMachineNode()
  : Node("flight_state_machine_node"),
    status_topic_(declare_parameter<std::string>("status_topic", "fcu_link/status")),
    state_topic_(declare_parameter<std::string>("state_topic", "uav_bridge/flight_state")),
    reset_topic_(declare_parameter<std::string>("reset_topic", "uav_bridge/flight_reset")),
    publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 5.0)),
    recovery_timeout_(declare_parameter<double>("recovery_timeout", 3.0))
  {
    publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
    recovery_timeout_ = std::max(0.5, recovery_timeout_);

    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, 10,
      std::bind(&FlightStateMachineNode::statusCallback, this, std::placeholders::_1));

    reset_sub_ = create_subscription<std_msgs::msg::String>(
      reset_topic_, 10,
      std::bind(&FlightStateMachineNode::resetCallback, this, std::placeholders::_1));

    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    const auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&FlightStateMachineNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "flight_state_machine started: status=%s state=%s reset=%s timeout=%.1fs",
      status_topic_.c_str(), state_topic_.c_str(), reset_topic_.c_str(), recovery_timeout_);
  }

private:
  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    last_link_status_ = msg->data;
  }

  void resetCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "RESET" && phase_ == Phase::FAILSAFE) {
      RCLCPP_INFO(get_logger(), "Received RESET, FAILSAFE -> WAIT_FCU");
      phase_ = Phase::WAIT_FCU;
    }
  }

  void timerCallback()
  {
    switch (phase_) {
      case Phase::INIT:
        phase_ = Phase::WAIT_FCU;
        break;

      case Phase::WAIT_FCU:
        if (last_link_status_ == "OK") {
          phase_ = Phase::TRACKING;
        }
        break;

      case Phase::TRACKING:
        if (last_link_status_ == "LOST") {
          phase_ = Phase::RECOVERING;
          recovery_start_time_ = now();
          RCLCPP_WARN(get_logger(), "Link LOST, entering RECOVERING (timeout=%.1fs)", recovery_timeout_);
        }
        break;

      case Phase::RECOVERING: {
        double elapsed = (now() - recovery_start_time_).seconds();
        if (last_link_status_ == "OK") {
          RCLCPP_INFO(get_logger(), "Link recovered after %.1fs", elapsed);
          phase_ = Phase::TRACKING;
        } else if (elapsed >= recovery_timeout_) {
          RCLCPP_ERROR(get_logger(), "Recovery timeout (%.1fs), entering FAILSAFE", elapsed);
          phase_ = Phase::FAILSAFE;
          recovery_count_++;
        }
        break;
      }

      case Phase::FAILSAFE:
        // 锁存状态，等待外部 RESET
        break;
    }

    // 发布状态变化
    std::string state_str = phaseToString(phase_);
    if (state_str != last_published_state_) {
      RCLCPP_INFO(get_logger(), "flight state: %s -> %s", last_published_state_.c_str(), state_str.c_str());
      last_published_state_ = state_str;
    }

    std_msgs::msg::String msg;
    msg.data = state_str;
    state_pub_->publish(msg);
  }

  static const char * phaseToString(Phase p)
  {
    switch (p) {
      case Phase::INIT: return "INIT";
      case Phase::WAIT_FCU: return "WAIT_FCU";
      case Phase::TRACKING: return "TRACKING";
      case Phase::RECOVERING: return "RECOVERING";
      case Phase::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  // ── 参数 ─────────────────────────────────────────────────────────

  std::string status_topic_;
  std::string state_topic_;
  std::string reset_topic_;
  double publish_rate_hz_;
  double recovery_timeout_;

  // ── 状态 ─────────────────────────────────────────────────────────

  std::string last_link_status_{"WAIT_FCU"};
  Phase phase_{Phase::INIT};
  std::string last_published_state_{"INIT"};
  rclcpp::Time recovery_start_time_{0, 0, RCL_ROS_TIME};
  int recovery_count_{0};

  // ── ROS 接口 ─────────────────────────────────────────────────────

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlightStateMachineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
