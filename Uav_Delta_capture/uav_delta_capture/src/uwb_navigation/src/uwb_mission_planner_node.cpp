/**
 * @file uwb_mission_planner_node.cpp
 * @brief UWB 导航任务规划：路径规划状态机 + 3D 坐标计算 + 速度控制
 *
 * 订阅：
 *   - uwb_aoa/data (UwbAoa)           — UWB AOA 数据（来自 uwb_aoa_driver）
 *   - fcu_state (FcuState)             — 飞控状态（来自 fcu_state_node）
 *   - fcu_link/status (String)         — FCU 链路状态（来自 fcu_link_monitor）
 *
 * 发布：
 *   - cmd_vel (TwistStamped)           — 速度指令（给 flight_commander）
 *   - uwb_mission/state (String)       — 任务状态
 *   - uwb_mission/event (String)       — 任务事件
 *
 * 服务：
 *   - flight_command (FlightCommand)   — 飞行命令（解锁、起飞、降落等）
 *
 * 功能：
 *   1. 路径规划状态机：IDLE → TAKEOFF → HOVER_TAKEOFF → MOVE_ABOVE → HOVER_ABOVE → DESCEND → HOVER_FINAL → DONE
 *   2. 3D 坐标计算：利用 UWB AOA 的距离、方位角、仰角计算三维坐标
 *   3. 速度控制：平滑的比例控制，越近越慢
 *
 * 设计原则：
 *   - 只做路径规划和控制，不做数据采集
 *   - 依赖 uwb_aoa_driver 提供 UWB 数据
 *   - 依赖 fcu_bridge 提供飞控状态
 *   - 可以独立测试：ros2 run uwb_navigation uwb_mission_planner
 *
 * 路径规划：
 *   起飞 → 悬停 → 水平飞到 A 上方 → 悬停 → 垂直下降 → 近距离悬停
 *
 *   1. TAKEOFF: 起飞到指定高度（默认 1.5m）
 *   2. HOVER_TAKEOFF: 悬停稳定（等待高度稳定）
 *   3. MOVE_ABOVE: 水平移动到 A 正上方（利用方位角控制，保持高度）
 *   4. HOVER_ABOVE: 在 A 上方悬停稳定
 *   5. DESCEND: 垂直下降到近距离（默认 0.5m）
 *   6. HOVER_FINAL: 最终悬停（等待机械臂接管）
 *   7. DONE: 任务完成
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "uav_delta_msgs/msg/fcu_state.hpp"
#include "uav_delta_msgs/msg/uwb_aoa.hpp"
#include "uav_delta_msgs/srv/flight_command.hpp"

using namespace std::chrono_literals;

class UwbMissionPlannerNode : public rclcpp::Node
{
public:
  // ── 任务阶段 ─────────────────────────────────────────────────────

  enum class Phase {
    IDLE,            // 空闲
    ARMING,          // 解锁中
    TAKEOFF,         // 起飞
    HOVER_TAKEOFF,   // 起飞后悬停
    MOVE_ABOVE,      // 水平移动到 A 上方
    HOVER_ABOVE,     // 在 A 上方悬停
    DESCEND,         // 垂直下降
    HOVER_FINAL,     // 最终悬停
    DONE,            // 任务完成
    FAILSAFE         // 故障保护
  };

  UwbMissionPlannerNode()
  : Node("uwb_mission_planner_node"),
    // 话题参数
    uwb_aoa_topic_(declare_parameter<std::string>("uwb_aoa_topic", "uwb_aoa/data")),
    fcu_state_topic_(declare_parameter<std::string>("fcu_state_topic", "fcu_state")),
    fcu_link_topic_(declare_parameter<std::string>("fcu_link_topic", "fcu_link/status")),
    cmd_vel_topic_(declare_parameter<std::string>("cmd_vel_topic", "cmd_vel")),
    mission_state_topic_(declare_parameter<std::string>("mission_state_topic", "uwb_mission/state")),
    mission_event_topic_(declare_parameter<std::string>("mission_event_topic", "uwb_mission/event")),
    flight_command_service_(declare_parameter<std::string>("flight_command_service", "flight_command")),
    // 路径参数
    takeoff_altitude_(declare_parameter<double>("takeoff_altitude", 1.5)),
    descend_altitude_(declare_parameter<double>("descend_altitude", 0.5)),
    altitude_tolerance_(declare_parameter<double>("altitude_tolerance", 0.15)),
    // 控制参数
    kp_horizontal_(declare_parameter<double>("kp_horizontal", 0.4)),
    kp_vertical_(declare_parameter<double>("kp_vertical", 0.3)),
    ki_horizontal_(declare_parameter<double>("ki_horizontal", 0.0)),
    ki_vertical_(declare_parameter<double>("ki_vertical", 0.0)),
    kd_horizontal_(declare_parameter<double>("kd_horizontal", 0.0)),
    kd_vertical_(declare_parameter<double>("kd_vertical", 0.0)),
    integral_limit_(declare_parameter<double>("integral_limit", 1.0)),
    lp_filter_alpha_(declare_parameter<double>("lp_filter_alpha", 0.3)),
    max_vel_xy_(declare_parameter<double>("max_vel_xy", 0.5)),
    max_vel_z_(declare_parameter<double>("max_vel_z", 0.3)),
    max_accel_xy_(declare_parameter<double>("max_accel_xy", 0.3)),
    max_accel_z_(declare_parameter<double>("max_accel_z", 0.2)),
    // 死区参数
    horizontal_deadband_(declare_parameter<double>("horizontal_deadband", 0.15)),
    azimuth_deadband_(declare_parameter<double>("azimuth_deadband", 3.0)),
    // 稳定检测
    hover_stable_time_(declare_parameter<double>("hover_stable_time", 2.0)),
    control_rate_hz_(declare_parameter<double>("control_rate_hz", 20.0)),
    // 安全参数
    uwb_signal_timeout_(declare_parameter<double>("uwb_signal_timeout", 3.0)),
    low_battery_pct_(declare_parameter<double>("low_battery_pct", 20.0))
  {
    // 限制参数范围
    takeoff_altitude_ = std::max(0.5, takeoff_altitude_);
    descend_altitude_ = std::max(0.2, descend_altitude_);
    kp_horizontal_ = std::max(0.01, kp_horizontal_);
    kp_vertical_ = std::max(0.01, kp_vertical_);
    ki_horizontal_ = std::max(0.0, ki_horizontal_);
    ki_vertical_ = std::max(0.0, ki_vertical_);
    kd_horizontal_ = std::max(0.0, kd_horizontal_);
    kd_vertical_ = std::max(0.0, kd_vertical_);
    integral_limit_ = std::max(0.1, integral_limit_);
    lp_filter_alpha_ = std::clamp(lp_filter_alpha_, 0.01, 0.99);
    max_vel_xy_ = std::max(0.1, max_vel_xy_);
    max_vel_z_ = std::max(0.1, max_vel_z_);

    // 创建回调组
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 订阅 UWB AOA 数据
    uwb_sub_ = create_subscription<uav_delta_msgs::msg::UwbAoa>(
      uwb_aoa_topic_, 10,
      [this](const uav_delta_msgs::msg::UwbAoa::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        last_uwb_ = msg;
        has_uwb_ = true;
        last_uwb_time_ = now();
      });

    // 订阅飞控状态
    fcu_sub_ = create_subscription<uav_delta_msgs::msg::FcuState>(
      fcu_state_topic_, 10,
      [this](const uav_delta_msgs::msg::FcuState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        last_fcu_ = msg;
        has_fcu_ = true;
      });

    // 订阅 FCU 链路状态
    link_sub_ = create_subscription<std_msgs::msg::String>(
      fcu_link_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(data_mtx_);
        last_link_status_ = msg->data;
      });

    // 发布速度指令
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 20);

    // 发布任务状态
    state_pub_ = create_publisher<std_msgs::msg::String>(mission_state_topic_, 10);
    event_pub_ = create_publisher<std_msgs::msg::String>(mission_event_topic_, 10);

    // 创建飞行命令服务客户端
    flight_cmd_client_ = create_client<uav_delta_msgs::srv::FlightCommand>(
      flight_command_service_, rmw_qos_profile_default, cb_group_);

    // 控制定时器
    const auto ctrl_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    ctrl_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ctrl_period),
      std::bind(&UwbMissionPlannerNode::controlCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "uwb_mission_planner started: takeoff=%.1fm descend=%.1fm kp_h=%.2f kp_v=%.2f",
      takeoff_altitude_, descend_altitude_, kp_horizontal_, kp_vertical_);
  }

private:
  // ── 控制回调（主循环）────────────────────────────────────────────

  void controlCallback()
  {
    switch (phase_) {
      case Phase::IDLE:           tickIdle(); break;
      case Phase::ARMING:         tickArming(); break;
      case Phase::TAKEOFF:        tickTakeoff(); break;
      case Phase::HOVER_TAKEOFF:  tickHoverTakeoff(); break;
      case Phase::MOVE_ABOVE:     tickMoveAbove(); break;
      case Phase::HOVER_ABOVE:    tickHoverAbove(); break;
      case Phase::DESCEND:        tickDescend(); break;
      case Phase::HOVER_FINAL:    tickHoverFinal(); break;
      case Phase::DONE:           break;
      case Phase::FAILSAFE:       tickFailsafe(); break;
    }

    publishState();
  }

  // ── 阶段处理函数 ─────────────────────────────────────────────────

  void tickIdle()
  {
    // 等待外部触发（通过服务或话题）
    // 这里可以添加自动启动逻辑
  }

  void tickArming()
  {
    if (arming_in_progress_) return;

    arming_in_progress_ = true;
    callFlightCommand(
      uav_delta_msgs::srv::FlightCommand::Request::CMD_ARM, 0.0f,
      [this](bool success, const std::string & msg) {
        arming_in_progress_ = false;
        if (success) {
          RCLCPP_INFO(get_logger(), "Arm successful");
          publishEvent("armed");
          transitionTo(Phase::TAKEOFF);
        } else {
          RCLCPP_WARN(get_logger(), "Arm failed: %s, retrying...", msg.c_str());
        }
      });
  }

  void tickTakeoff()
  {
    if (takeoff_in_progress_) return;

    takeoff_in_progress_ = true;
    callFlightCommand(
      uav_delta_msgs::srv::FlightCommand::Request::CMD_TAKEOFF,
      static_cast<float>(takeoff_altitude_),
      [this](bool success, const std::string & msg) {
        takeoff_in_progress_ = false;
        if (success) {
          RCLCPP_INFO(get_logger(), "Takeoff command accepted (%.1fm)", takeoff_altitude_);
          publishEvent("takeoff_accepted");
          transitionTo(Phase::HOVER_TAKEOFF);
        } else {
          RCLCPP_ERROR(get_logger(), "Takeoff failed: %s", msg.c_str());
          transitionTo(Phase::FAILSAFE);
        }
      });
  }

  void tickHoverTakeoff()
  {
    if (!checkFcuAndLink()) return;

    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!has_fcu_) return;

    float alt = last_fcu_->local_z;
    if (std::fabs(alt - takeoff_altitude_) <= altitude_tolerance_) {
      // 高度到达，开始稳定计时
      if (!hover_start_time_.has_value()) {
        hover_start_time_ = now();
        RCLCPP_INFO(get_logger(), "Altitude reached: %.2fm, stabilizing...", alt);
      } else {
        double stable_duration = (now() - hover_start_time_.value()).seconds();
        if (stable_duration >= hover_stable_time_) {
          RCLCPP_INFO(get_logger(), "Hover stabilized, moving above target");
          publishEvent("hover_stabilized");
          hover_start_time_.reset();
          transitionTo(Phase::MOVE_ABOVE);
        }
      }
    } else {
      hover_start_time_.reset();
    }
  }

  void tickMoveAbove()
  {
    if (!checkFcuAndLink()) return;

    UwbData uwb;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      if (!has_uwb_ || !uwb_data_valid()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for UWB data...");
        return;
      }
      uwb = getLatestUwb();
    }

    // 水平控制：利用方位角控制，目标是 azimuth = 0（在正前方）
    // PID 控制
    double dt = 1.0 / control_rate_hz_;
    double alpha = lp_filter_alpha_;

    float az_rad = uwb.azimuth_deg * M_PI / 180.0;
    float horizontal_dist = uwb.distance_m * std::cos(az_rad);

    // 误差计算
    double error_x = -az_rad;  // 方位角误差
    double error_y = -horizontal_dist;  // 距离误差

    // 低通滤波
    filtered_error_x_ = alpha * error_x + (1.0 - alpha) * filtered_error_x_;
    filtered_error_y_ = alpha * error_y + (1.0 - alpha) * filtered_error_y_;

    // 积分项（含抗饱和）
    integral_x_ += error_x * dt;
    integral_y_ += error_y * dt;
    integral_x_ = std::clamp(integral_x_, -integral_limit_, integral_limit_);
    integral_y_ = std::clamp(integral_y_, -integral_limit_, integral_limit_);

    // 微分项
    double derivative_x = (filtered_error_x_ - prev_error_x_) / dt;
    double derivative_y = (filtered_error_y_ - prev_error_y_) / dt;

    // 保存误差
    prev_error_x_ = filtered_error_x_;
    prev_error_y_ = filtered_error_y_;

    // PID 输出
    float vx = 0.0f, vy = 0.0f;
    if (std::abs(uwb.azimuth_deg) > azimuth_deadband_) {
      vx = -kp_horizontal_ * error_x - ki_horizontal_ * integral_x_ - kd_horizontal_ * derivative_x;
      vx = std::clamp(vx, static_cast<float>(-max_vel_xy_), static_cast<float>(max_vel_xy_));
    }

    if (horizontal_dist > horizontal_deadband_) {
      vy = -kp_horizontal_ * error_y - ki_horizontal_ * integral_y_ - kd_horizontal_ * derivative_y;
      vy = std::clamp(vy, static_cast<float>(-max_vel_xy_), static_cast<float>(max_vel_xy_));
    }

    // 高度保持 - PID 控制
    float vz = 0.0f;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      if (has_fcu_) {
        double error_z = takeoff_altitude_ - last_fcu_->local_z;

        // 低通滤波
        filtered_error_z_ = alpha * error_z + (1.0 - alpha) * filtered_error_z_;

        // 积分
        integral_z_ += error_z * dt;
        integral_z_ = std::clamp(integral_z_, -integral_limit_, integral_limit_);

        // 微分
        double derivative_z = (filtered_error_z_ - prev_error_z_) / dt;
        prev_error_z_ = filtered_error_z_;

        vz = kp_vertical_ * error_z + ki_vertical_ * integral_z_ + kd_vertical_ * derivative_z;
        vz = std::clamp(vz, static_cast<float>(-max_vel_z_), static_cast<float>(max_vel_z_));
      }
    }

    // 检查是否到达（方位角接近 0，距离接近 0）
    if (std::abs(uwb.azimuth_deg) < azimuth_deadband_ && horizontal_dist < horizontal_deadband_) {
      if (!hover_start_time_.has_value()) {
        hover_start_time_ = now();
        RCLCPP_INFO(get_logger(), "Above target, stabilizing...");
      } else {
        double stable_duration = (now() - hover_start_time_.value()).seconds();
        if (stable_duration >= hover_stable_time_) {
          RCLCPP_INFO(get_logger(), "Above target stabilized, descending");
          publishEvent("above_target_reached");
          hover_start_time_.reset();
          transitionTo(Phase::DESCEND);
        }
      }
    } else {
      hover_start_time_.reset();
    }

    publishVelocity(vx, vy, vz);
  }

  void tickHoverAbove()
  {
    // 这个阶段实际上在 tickMoveAbove 中处理了
    // 如果需要更精确的悬停控制，可以在这里实现
    transitionTo(Phase::DESCEND);
  }

  void tickDescend()
  {
    if (!checkFcuAndLink()) return;

    UwbData uwb;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      if (!has_uwb_ || !uwb_data_valid()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for UWB data...");
        return;
      }
      uwb = getLatestUwb();
    }

    // 水平保持：保持在 A 正上方 - PID 控制（下降时增益减半）
    double dt = 1.0 / control_rate_hz_;
    double alpha = lp_filter_alpha_;

    float az_rad = uwb.azimuth_deg * M_PI / 180.0;
    float horizontal_dist = uwb.distance_m * std::cos(az_rad);

    // 误差计算
    double error_x = -az_rad;
    double error_y = -horizontal_dist;

    // 低通滤波
    filtered_error_x_ = alpha * error_x + (1.0 - alpha) * filtered_error_x_;
    filtered_error_y_ = alpha * error_y + (1.0 - alpha) * filtered_error_y_;

    // 积分项
    integral_x_ += error_x * dt;
    integral_y_ += error_y * dt;
    integral_x_ = std::clamp(integral_x_, -integral_limit_, integral_limit_);
    integral_y_ = std::clamp(integral_y_, -integral_limit_, integral_limit_);

    // 微分项
    double derivative_x = (filtered_error_x_ - prev_error_x_) / dt;
    double derivative_y = (filtered_error_y_ - prev_error_y_) / dt;
    prev_error_x_ = filtered_error_x_;
    prev_error_y_ = filtered_error_y_;

    // PID 输出（下降时水平修正减半）
    float vx = 0.0f, vy = 0.0f;
    if (std::abs(uwb.azimuth_deg) > azimuth_deadband_) {
      vx = (-kp_horizontal_ * error_x - ki_horizontal_ * integral_x_ - kd_horizontal_ * derivative_x) * 0.5f;
      vx = std::clamp(vx, static_cast<float>(-max_vel_xy_ * 0.5f), static_cast<float>(max_vel_xy_ * 0.5f));
    }

    if (horizontal_dist > horizontal_deadband_) {
      vy = (-kp_horizontal_ * error_y - ki_horizontal_ * integral_y_ - kd_horizontal_ * derivative_y) * 0.5f;
      vy = std::clamp(vy, static_cast<float>(-max_vel_xy_ * 0.5f), static_cast<float>(max_vel_xy_ * 0.5f));
    }

    // 垂直下降 - PID 控制
    float vz = 0.0f;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      if (has_fcu_) {
        float current_alt = last_fcu_->local_z;
        float target_alt = descend_altitude_;

        // 如果仰角可用，利用仰角计算更精确的高度
        if (uwb.elevation_deg != 0) {
          float el_rad = uwb.elevation_deg * M_PI / 180.0;
          float height_from_uwb = uwb.distance_m * std::sin(el_rad);
          target_alt = std::max(static_cast<float>(descend_altitude_), height_from_uwb);
        }

        double error_z = target_alt - current_alt;

        // 低通滤波
        filtered_error_z_ = alpha * error_z + (1.0 - alpha) * filtered_error_z_;

        // 积分
        integral_z_ += error_z * dt;
        integral_z_ = std::clamp(integral_z_, -integral_limit_, integral_limit_);

        // 微分
        double derivative_z = (filtered_error_z_ - prev_error_z_) / dt;
        prev_error_z_ = filtered_error_z_;

        if (std::abs(error_z) > altitude_tolerance_) {
          vz = kp_vertical_ * error_z + ki_vertical_ * integral_z_ + kd_vertical_ * derivative_z;
          vz = std::clamp(vz, static_cast<float>(-max_vel_z_), 0.0f);
        }
      }
    }

    // 检查是否到达最终高度
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      if (has_fcu_) {
        float current_alt = last_fcu_->local_z;
        if (std::fabs(current_alt - descend_altitude_) <= altitude_tolerance_) {
          if (!hover_start_time_.has_value()) {
            hover_start_time_ = now();
            RCLCPP_INFO(get_logger(), "Final altitude reached: %.2fm, stabilizing...", current_alt);
          } else {
            double stable_duration = (now() - hover_start_time_.value()).seconds();
            if (stable_duration >= hover_stable_time_) {
              RCLCPP_INFO(get_logger(), "Final hover stabilized, waiting for arm");
              publishEvent("final_hover_reached");
              hover_start_time_.reset();
              transitionTo(Phase::HOVER_FINAL);
            }
          }
        } else {
          hover_start_time_.reset();
        }
      }
    }

    publishVelocity(vx, vy, vz);
  }

  void tickHoverFinal()
  {
    // 保持悬停，等待机械臂接管
    publishVelocity(0.0f, 0.0f, 0.0f);

    // 这里可以添加与机械臂的交互逻辑
    // 比如订阅机械臂状态，当机械臂完成抓取后触发降落
  }

  void tickFailsafe()
  {
    // 故障保护：降落
    publishVelocity(0.0f, 0.0f, 0.0f);

    if (!failsafe_land_sent_) {
      failsafe_land_sent_ = true;
      callFlightCommand(
        uav_delta_msgs::srv::FlightCommand::Request::CMD_MODE_LAND, 0.0f,
        [this](bool success, const std::string & msg) {
          if (success) {
            RCLCPP_INFO(get_logger(), "Failsafe LAND mode set");
            publishEvent("failsafe_landing");
          } else {
            RCLCPP_ERROR(get_logger(), "Failsafe LAND failed: %s", msg.c_str());
          }
        });
    }
  }

  // ── 辅助函数 ─────────────────────────────────────────────────────

  void transitionTo(Phase new_phase)
  {
    RCLCPP_INFO(get_logger(), "Phase: %s → %s", phaseToString(phase_), phaseToString(new_phase));
    phase_ = new_phase;

    // 阶段切换时清零 PID 状态
    integral_x_ = 0.0;
    integral_y_ = 0.0;
    integral_z_ = 0.0;
    prev_error_x_ = 0.0;
    prev_error_y_ = 0.0;
    prev_error_z_ = 0.0;
    filtered_error_x_ = 0.0;
    filtered_error_y_ = 0.0;
    filtered_error_z_ = 0.0;
  }

  static const char * phaseToString(Phase phase)
  {
    switch (phase) {
      case Phase::IDLE: return "IDLE";
      case Phase::ARMING: return "ARMING";
      case Phase::TAKEOFF: return "TAKEOFF";
      case Phase::HOVER_TAKEOFF: return "HOVER_TAKEOFF";
      case Phase::MOVE_ABOVE: return "MOVE_ABOVE";
      case Phase::HOVER_ABOVE: return "HOVER_ABOVE";
      case Phase::DESCEND: return "DESCEND";
      case Phase::HOVER_FINAL: return "HOVER_FINAL";
      case Phase::DONE: return "DONE";
      case Phase::FAILSAFE: return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  bool checkFcuAndLink()
  {
    std::lock_guard<std::mutex> lk(data_mtx_);

    if (!has_fcu_ || !last_fcu_->connected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for FCU...");
      return false;
    }

    if (last_link_status_ == "LOST") {
      RCLCPP_ERROR(get_logger(), "FCU link lost!");
      transitionTo(Phase::FAILSAFE);
      return false;
    }

    if (last_fcu_->remaining < low_battery_pct_ / 100.0f) {
      RCLCPP_ERROR(get_logger(), "Low battery: %.0f%%", last_fcu_->remaining * 100);
      transitionTo(Phase::FAILSAFE);
      return false;
    }

    return true;
  }

  bool uwb_data_valid()
  {
    double age = (now() - last_uwb_time_).seconds();
    return age <= uwb_signal_timeout_ && last_uwb_->signal_valid;
  }

  struct UwbData {
    float distance_m;
    float azimuth_deg;
    float elevation_deg;
  };

  UwbData getLatestUwb()
  {
    return {
      last_uwb_->distance_m,
      last_uwb_->azimuth_deg,
      last_uwb_->elevation_deg
    };
  }

  void publishVelocity(float vx, float vy, float vz)
  {
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = now();
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    vel_pub_->publish(msg);
  }

  void publishState()
  {
    auto msg = std_msgs::msg::String();
    msg.data = phaseToString(phase_);
    state_pub_->publish(msg);
  }

  void publishEvent(const std::string & event)
  {
    auto msg = std_msgs::msg::String();
    msg.data = event;
    event_pub_->publish(msg);
  }

  void callFlightCommand(
    int command, float param,
    std::function<void(bool, const std::string &)> callback)
  {
    if (!flight_cmd_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Flight command service not ready");
      callback(false, "service not ready");
      return;
    }

    auto request = std::make_shared<uav_delta_msgs::srv::FlightCommand::Request>();
    request->command = command;
    request->param = param;

    flight_cmd_client_->async_send_request(
      request,
      [this, callback](rclcpp::Client<uav_delta_msgs::srv::FlightCommand>::SharedFuture future) {
        auto result = future.get();
        callback(result->success, result->message);
      });
  }

  // ── 参数 ─────────────────────────────────────────────────────────

  std::string uwb_aoa_topic_;
  std::string fcu_state_topic_;
  std::string fcu_link_topic_;
  std::string cmd_vel_topic_;
  std::string mission_state_topic_;
  std::string mission_event_topic_;
  std::string flight_command_service_;

  double takeoff_altitude_;
  double descend_altitude_;
  double altitude_tolerance_;

  double kp_horizontal_;
  double kp_vertical_;
  double ki_horizontal_;
  double ki_vertical_;
  double kd_horizontal_;
  double kd_vertical_;
  double integral_limit_;
  double lp_filter_alpha_;
  double max_vel_xy_;
  double max_vel_z_;
  double max_accel_xy_;
  double max_accel_z_;

  double horizontal_deadband_;
  double azimuth_deadband_;

  double hover_stable_time_;
  double control_rate_hz_;

  double uwb_signal_timeout_;
  double low_battery_pct_;

  // PID 状态
  double integral_x_{0.0};
  double integral_y_{0.0};
  double integral_z_{0.0};
  double prev_error_x_{0.0};
  double prev_error_y_{0.0};
  double prev_error_z_{0.0};
  double filtered_error_x_{0.0};
  double filtered_error_y_{0.0};
  double filtered_error_z_{0.0};

  // 状态 ─────────────────────────────────────────────────────────

  Phase phase_{Phase::IDLE};
  std::optional<rclcpp::Time> hover_start_time_;
  bool arming_in_progress_{false};
  bool takeoff_in_progress_{false};
  bool failsafe_land_sent_{false};

  // ── 数据 ─────────────────────────────────────────────────────────

  std::mutex data_mtx_;
  uav_delta_msgs::msg::UwbAoa::SharedPtr last_uwb_;
  uav_delta_msgs::msg::FcuState::SharedPtr last_fcu_;
  std::string last_link_status_;
  bool has_uwb_{false};
  bool has_fcu_{false};
  rclcpp::Time last_uwb_time_{0, 0, RCL_ROS_TIME};

  // ── ROS 接口 ─────────────────────────────────────────────────────

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<uav_delta_msgs::msg::UwbAoa>::SharedPtr uwb_sub_;
  rclcpp::Subscription<uav_delta_msgs::msg::FcuState>::SharedPtr fcu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr link_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Client<uav_delta_msgs::srv::FlightCommand>::SharedPtr flight_cmd_client_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UwbMissionPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}