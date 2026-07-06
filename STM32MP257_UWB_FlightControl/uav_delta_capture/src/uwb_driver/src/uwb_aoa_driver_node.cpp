/**
 * @file uwb_aoa_driver_node.cpp
 * @brief UWB AOA 数据驱动：串口采集、协议解析、卡尔曼滤波
 *
 * 订阅：
 *   - 无（直接从串口读取）
 *
 * 发布：
 *   - uwb_aoa/data (UwbAoa)  — 解析后的 UWB AOA 数据（带卡尔曼滤波）
 *
 * 功能：
 *   1. 从串口读取 UWB AOA 模组的二进制数据
 *   2. 解析协议帧（0x2001 命令）
 *   3. 卡尔曼滤波平滑距离和方位角
 *   4. 发布 UwbAoa 消息
 *
 * 设计原则：
 *   - 只做数据采集和预处理，不做任何控制逻辑
 *   - 不依赖飞控、视觉等其他模块
 *   - 可以独立测试：ros2 run uwb_driver uwb_aoa_driver
 *
 * 协议格式（来自 ALX-AOA-FIT 规格书）：
 *   - 帧长度：37 字节
 *   - 命令码：0x2001（位置数据）
 *   - 数据：距离(cm)、方位角(°)、仰角(°)
 *   - 校验：异或校验
 */

#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "uav_delta_msgs/msg/uwb_aoa.hpp"

using namespace std::chrono_literals;

// ── 1D 卡尔曼滤波器 ──────────────────────────────────────────────────

class KalmanFilter1D
{
public:
  KalmanFilter1D(float Q, float R) : Q_(Q), R_(R) {}

  void reset(float initial)
  {
    x_ = initial;
    P_ = 1.0f;
    initialized_ = true;
  }

  float update(float measured, float dt)
  {
    if (!initialized_) {
      reset(measured);
      return x_;
    }
    // Predict
    P_ += Q_ * dt;
    // Update
    float S = P_ + R_;
    float K = P_ / S;
    x_ += K * (measured - x_);
    P_ = (1.0f - K) * P_;
    return x_;
  }

  float value() const { return x_; }
  bool isInitialized() const { return initialized_; }

private:
  float Q_, R_;
  float x_{0}, P_{1};
  bool initialized_{false};
};

// ── UWB AOA 帧常量 ──────────────────────────────────────────────────

static constexpr size_t AOA_FRAME_LEN = 37;
static constexpr uint16_t AOA_CMD_POSITION = 0x2001;

// 字节序转换（大端 → 小端）
static uint16_t swapU16(uint16_t v)
{
  return static_cast<uint16_t>((v << 8) | (v >> 8));
}

static uint32_t swapU32(uint32_t v)
{
  return ((v & 0x000000FFu) << 24) |
         ((v & 0x0000FF00u) << 8) |
         ((v & 0x00FF0000u) >> 8) |
         ((v & 0xFF000000u) >> 24);
}

static int16_t swapS16(int16_t v)
{
  return static_cast<int16_t>(swapU16(static_cast<uint16_t>(v)));
}

// 解析后的 AOA 帧
struct AoaFrame {
  uint32_t anchor_id;
  uint32_t tag_id;
  uint32_t distance_cm;   // 厘米
  int16_t azimuth_deg;    // 度，有符号
  int16_t elevation_deg;  // 度，有符号
  uint16_t tag_status;
  uint16_t batch_sn;
};

// ── 主节点 ──────────────────────────────────────────────────────────

class UwbAoaDriverNode : public rclcpp::Node
{
public:
  UwbAoaDriverNode()
  : Node("uwb_aoa_driver_node"),
    serial_port_(declare_parameter<std::string>("serial_port", "/dev/ttyUSB0")),
    serial_baud_(declare_parameter<int>("serial_baud", 115200)),
    uwb_frame_id_(declare_parameter<std::string>("uwb_frame_id", "uwb")),
    uwb_aoa_topic_(declare_parameter<std::string>("uwb_aoa_topic", "uwb_aoa/data")),
    signal_loss_timeout_sec_(declare_parameter<double>("signal_loss_timeout_sec", 0.2)),
    max_range_m_(declare_parameter<double>("max_range_m", 15.0)),
    status_rate_hz_(declare_parameter<double>("status_rate_hz", 10.0)),
    kalman_Q_(declare_parameter<double>("kalman_Q", 0.1)),
    kalman_R_(declare_parameter<double>("kalman_R", 0.1)),
    kf_distance_(kalman_Q_, kalman_R_),
    kf_azimuth_(kalman_Q_, kalman_R_)
  {
    // 发布 UWB AOA 数据
    aoa_pub_ = create_publisher<uav_delta_msgs::msg::UwbAoa>(uwb_aoa_topic_, 10);

    // 打开串口
    serial_fd_ = openSerial(serial_port_, serial_baud_);
    if (serial_fd_ < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to open serial port %s — running in IDLE (no UWB data)",
        serial_port_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Serial port %s opened (fd=%d)", serial_port_.c_str(), serial_fd_);
      io_thread_ = std::thread(&UwbAoaDriverNode::ioLoop, this);
    }

    // 状态发布定时器
    const auto status_period = std::chrono::duration<double>(1.0 / status_rate_hz_);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
      std::bind(&UwbAoaDriverNode::statusCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "uwb_aoa_driver started: port=%s baud=%d kalman_Q=%.2f kalman_R=%.2f",
      serial_port_.c_str(), serial_baud_, kalman_Q_, kalman_R_);
  }

  ~UwbAoaDriverNode() override
  {
    running_ = false;
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }
  }

private:
  // ── 串口 I/O 线程（二进制帧解析）──────────────────────────────────

  void ioLoop()
  {
    uint8_t buf[256];
    std::vector<uint8_t> accum;

    while (running_) {
      int n = ::read(serial_fd_, buf, sizeof(buf));
      if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
          continue;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read error: %s", strerror(errno));
        std::this_thread::sleep_for(10ms);
        continue;
      }

      accum.insert(accum.end(), buf, buf + n);

      // 扫描累积缓冲区，寻找完整帧
      while (accum.size() >= AOA_FRAME_LEN) {
        size_t frame_start = findFrameStart(accum);
        if (frame_start == std::string::npos) {
          if (accum.size() > AOA_FRAME_LEN) {
            accum.erase(accum.begin(), accum.end() - (AOA_FRAME_LEN - 1));
          }
          break;
        }

        if (frame_start > 0) {
          accum.erase(accum.begin(), accum.begin() + frame_start);
        }

        if (accum.size() < AOA_FRAME_LEN) {
          break;
        }

        // 验证异或校验
        uint8_t xor_val = 0;
        for (size_t i = 0; i < AOA_FRAME_LEN - 1; i++) {
          xor_val ^= accum[i];
        }
        uint8_t expected_xor = accum[AOA_FRAME_LEN - 1];

        if (xor_val != expected_xor) {
          accum.erase(accum.begin());
          continue;
        }

        // 解析帧
        AoaFrame frame;
        if (parseFrame(accum.data(), frame)) {
          processFrame(frame);
        }

        accum.erase(accum.begin(), accum.begin() + AOA_FRAME_LEN);
      }
    }
  }

  // 查找帧起始（通过命令码 0x2001）
  static size_t findFrameStart(const std::vector<uint8_t> & buf)
  {
    for (size_t i = 0; i + AOA_FRAME_LEN <= buf.size(); i++) {
      uint16_t cmd = static_cast<uint16_t>((buf[i + 8] << 8) | buf[i + 9]);
      if (cmd == AOA_CMD_POSITION) {
        return i;
      }
    }
    return std::string::npos;
  }

  // 解析二进制帧
  bool parseFrame(const uint8_t * data, AoaFrame & frame)
  {
    uint16_t cmd = static_cast<uint16_t>((data[8] << 8) | data[9]);
    if (cmd != AOA_CMD_POSITION) {
      return false;
    }

    memcpy(&frame.anchor_id, &data[12], 4);
    frame.anchor_id = swapU32(frame.anchor_id);

    memcpy(&frame.tag_id, &data[16], 4);
    frame.tag_id = swapU32(frame.tag_id);

    memcpy(&frame.distance_cm, &data[20], 4);
    frame.distance_cm = swapU32(frame.distance_cm);

    memcpy(&frame.azimuth_deg, &data[24], 2);
    frame.azimuth_deg = swapS16(frame.azimuth_deg);

    memcpy(&frame.elevation_deg, &data[26], 2);
    frame.elevation_deg = swapS16(frame.elevation_deg);

    memcpy(&frame.tag_status, &data[28], 2);
    frame.tag_status = swapU16(frame.tag_status);

    memcpy(&frame.batch_sn, &data[30], 2);
    frame.batch_sn = swapU16(frame.batch_sn);

    return true;
  }

  // 处理解析后的帧（卡尔曼滤波）
  void processFrame(const AoaFrame & frame)
  {
    float dist_m = static_cast<float>(frame.distance_cm) * 0.01f;
    float az_deg = static_cast<float>(frame.azimuth_deg);
    float el_deg = static_cast<float>(frame.elevation_deg);

    // 卡尔曼滤波
    auto t = now();
    float dt = 0.05f;  // 默认 ~20Hz
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      if (last_frame_time_.nanoseconds() > 0) {
        dt = static_cast<float>((t - last_frame_time_).seconds());
        if (dt <= 0.0f || dt > 1.0f) dt = 0.05f;
      }
      last_frame_time_ = t;

      kf_distance_.update(dist_m, dt);
      kf_azimuth_.update(az_deg, dt);

      latest_.distance_m = kf_distance_.value();
      latest_.azimuth_deg = kf_azimuth_.value();
      latest_.elevation_deg = el_deg;
      latest_.anchor_id = frame.anchor_id;
      latest_.tag_id = frame.tag_id;
      latest_.tag_status = frame.tag_status;
      latest_.time = t;
      has_data_ = true;
    }
  }

  // ── 状态发布回调 ─────────────────────────────────────────────────

  void statusCallback()
  {
    AoaData aoa;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      if (has_data_) {
        aoa = latest_;
        double age = (now() - aoa.time).seconds();
        valid = (age <= signal_loss_timeout_sec_);
      }
    }

    auto msg = uav_delta_msgs::msg::UwbAoa();
    msg.header.stamp = now();
    msg.header.frame_id = uwb_frame_id_;
    msg.distance_m = aoa.distance_m;
    msg.azimuth_deg = aoa.azimuth_deg;
    msg.elevation_deg = aoa.elevation_deg;
    msg.anchor_id = aoa.anchor_id;
    msg.tag_id = aoa.tag_id;
    msg.tag_status = aoa.tag_status;
    msg.quality = valid ? 1.0f : 0.0f;
    msg.signal_valid = valid;

    aoa_pub_->publish(msg);
  }

  // ── 工具函数 ─────────────────────────────────────────────────────

  int openSerial(const std::string & port, int baud)
  {
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      return -1;
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
      ::close(fd);
      return -1;
    }

    speed_t b = B115200;
    switch (baud) {
      case 9600: b = B9600; break;
      case 38400: b = B38400; break;
      case 57600: b = B57600; break;
      case 115200: b = B115200; break;
      default: b = B115200; break;
    }
    cfsetispeed(&tty, b);
    cfsetospeed(&tty, b);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      ::close(fd);
      return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
  }

  // ── 数据结构 ─────────────────────────────────────────────────────

  struct AoaData {
    float distance_m{0};
    float azimuth_deg{0};
    float elevation_deg{0};
    uint32_t anchor_id{0};
    uint32_t tag_id{0};
    uint16_t tag_status{0};
    rclcpp::Time time{0, 0, RCL_ROS_TIME};
  };

  // ── 参数 ─────────────────────────────────────────────────────────

  std::string serial_port_;
  int serial_baud_;
  std::string uwb_frame_id_;
  std::string uwb_aoa_topic_;
  double signal_loss_timeout_sec_;
  double max_range_m_;
  double status_rate_hz_;
  double kalman_Q_;
  double kalman_R_;

  // ── 串口 ─────────────────────────────────────────────────────────

  int serial_fd_{-1};
  std::thread io_thread_;
  std::atomic<bool> running_{true};

  // ── 卡尔曼滤波器 ─────────────────────────────────────────────────

  KalmanFilter1D kf_distance_;
  KalmanFilter1D kf_azimuth_;

  // ── 数据（受 data_mtx_ 保护）────────────────────────────────────

  std::mutex data_mtx_;
  AoaData latest_;
  bool has_data_{false};
  rclcpp::Time last_frame_time_{0, 0, RCL_ROS_TIME};

  // ── 发布/定时器 ─────────────────────────────────────────────────

  rclcpp::Publisher<uav_delta_msgs::msg::UwbAoa>::SharedPtr aoa_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UwbAoaDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
